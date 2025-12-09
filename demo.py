import math
import numpy as np
import pybullet as p
from utils import load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, wait_if_gui, load_pybullet

ENV_JSON   = 'environment1.json'  
ROBOT_URDF = 'mybot.urdf'

np.random.seed(1)

STATE_NAMES  = ('x','y','theta','u','v','w')
STATE_LIMITS = {
    'x': (-4.75, 4.75), 'y': (-4.75, 4.75), 'theta': (-math.pi, math.pi),
    'u': (-0.5, 0.5), 'v': (-0.5, 0.5), 'w': (-0.3, 0.3),
}

MASS  = 5.0
INERT = 5.0
DT    = 0.15

GOAL_BIAS       = 0.12   
CONNECT_HOPS    = 120    
GOAL_CONNECT_HOPS = 120    
MAX_ITERS       = 50000

ROBOT_R   = 0.30
Z_CENTER  = 0.20
Z_HEIGHT  = 0.30

def wrap_theta(t):
    while t < -math.pi: t += 2*math.pi
    while t >  math.pi: t -= 2*math.pi
    return t

def theta_diff(t1, t2):
    d = t1 - t2
    while d < -math.pi: d += 2*math.pi
    while d >  math.pi: d -= 2*math.pi
    return abs(d)

def taskspace(cfg): return (cfg[0], cfg[1], cfg[2])

def limit_check(cfg):
    for i,k in enumerate(STATE_NAMES):
        if i == 2:  
            continue
        lo,hi = STATE_LIMITS[k]
        if not (lo <= cfg[i] <= hi): return False
    return True

def distance(a,b):
    w = [4,4,3,2,2,1]
    acc = 0.0
    for i in range(6):
        acc += w[i]*((theta_diff(b[i],a[i])**2) if i==2 else (b[i]-a[i])**2)
    return math.sqrt(acc)

def achieve(c1, c2):
    dx = abs(c2[0]-c1[0]); dy = abs(c2[1]-c1[1])
    if math.hypot(dx,dy) >= 0.125: return False
    if theta_diff(c2[2], c1[2])   >= 0.15:  return False
    if math.hypot(c2[3]-c1[3], c2[4]-c1[4]) >= 0.20: return False
    if abs(c2[5]-c1[5])          >= 0.20:   return False
    return True

def dynamics(now_state, inputs):
    x,y,theta,u,v,w = now_state
    fx,fy,tau = inputs
    ax = fx / MASS
    ay = fy / MASS
    alpha = tau / INERT

    u  = u + ax * DT
    v  = v + ay * DT
    w  = w + alpha * DT
    x2 = x + (u * math.cos(theta) - v * math.sin(theta)) * DT
    y2 = y + (u * math.sin(theta) + v * math.cos(theta)) * DT
    t2 = wrap_theta(theta + w * DT)
    return (x2, y2, t2, u, v, w)

def primitives_18():
    return [
        ( 1.0,  0.0,  0.0), ( 0.0,  0.0, -1.0), ( 0.0,  0.0,  1.0), (-1.0,  0.0,  0.0),
        ( 0.0,  1.0,  0.0), ( 0.0, -1.0,  0.0), ( 1.0,  1.0,  0.0), ( 1.0, -1.0,  0.0),
        (-1.0,  1.0,  0.0), (-1.0, -1.0,  0.0), ( 1.0,  0.0,  1.0), ( 1.0,  0.0, -1.0),
        (-1.0,  0.0,  1.0), (-1.0,  0.0, -1.0), ( 0.0,  1.0,  1.0), ( 0.0,  1.0, -1.0),
        ( 0.0, -1.0,  1.0), ( 0.0, -1.0, -1.0)
    ]

def best_step(q_from, q_towards, prims):
    best=None; best_d=float('inf')
    for u in prims:
        qn = dynamics(q_from, u)
        d  = distance(qn, q_towards)
        if d < best_d:
            best_d = d; best = qn
    return best

def make_pose_coll_checker(obstacles_dict, radius=ROBOT_R, z_center=Z_CENTER, height=Z_HEIGHT, margin=0.0):
    body_ids = list(obstacles_dict.values())
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    probe = p.createMultiBody(0, col, -1, [0,0,-10], [0,0,0,1])

    def coll(qxyz):
        x,y,_ = qxyz
        p.resetBasePositionAndOrientation(probe, [x,y,z_center], [0,0,0,1])
        for bid in body_ids:
            if p.getClosestPoints(bid, probe, margin):
                return True
        return False
    return coll

def sample_goal_biased(goal_config, p_goal):
    if np.random.rand() <= p_goal:
        return goal_config
    q = []
    for k in STATE_NAMES:
        lo,hi = STATE_LIMITS[k]
        q.append(float(np.random.uniform(lo,hi)))
    q[2] = wrap_theta(q[2])
    return tuple(q)

def nearest(nodes, q_rand):
    ds = [distance(n, q_rand) for n in nodes]
    return nodes[int(np.argmin(ds))]

def kd_rrt_connect(start_config, goal_config, coll_pose_fn, draw_edge=None):
    T = [start_config]
    parent = {start_config: None}
    prims = primitives_18()

    def backtrace(tip):
        out=[]; cur=tip
        while cur is not None:
            out.append(taskspace(cur)); cur=parent[cur]
        return list(reversed(out))

    for it in range(1, MAX_ITERS+1):

        q_rand = sample_goal_biased(goal_config, GOAL_BIAS)
        if not limit_check(q_rand) or coll_pose_fn(taskspace(q_rand)):
            continue

        q_near = nearest(T, q_rand)
        hops = 0
        q_tip = q_near
        while hops < CONNECT_HOPS:
            if coll_pose_fn(taskspace(q_tip)) or not limit_check(q_tip): break
            if achieve(q_tip, q_rand): break

            q_new = best_step(q_tip, q_rand, prims)
            if coll_pose_fn(taskspace(q_new)) or not limit_check(q_new): break
            if q_new in parent: break

            parent[q_new] = q_tip
            T.append(q_new)
            if draw_edge:
                a=taskspace(q_tip); b=taskspace(q_new)
                draw_edge(a,b,1.0,(1,1,1,0.55))
            q_tip = q_new
            hops += 1

            if achieve(q_tip, goal_config):
                path = backtrace(q_tip)
                for a,b in zip(path[:-1], path[1:]):
                    p.addUserDebugLine([a[0],a[1],0.08],[b[0],b[1],0.08],[0,1,0,1],3.0)
                return path

        if distance(q_tip, goal_config) < 1.2:   # proximity threshold to attempt goal-connect
            qc = q_tip
            for _ in range(GOAL_CONNECT_HOPS):
                if achieve(qc, goal_config):
                    path = backtrace(qc)
                    for a,b in zip(path[:-1], path[1:]):
                        p.addUserDebugLine([a[0],a[1],0.08],[b[0],b[1],0.08],[0,1,0,1],3.0)
                    return path
                qn = best_step(qc, goal_config, prims)
                if coll_pose_fn(taskspace(qn)) or not limit_check(qn):
                    break
                if qn in parent: break
                parent[qn] = qc
                T.append(qn)
                if draw_edge:
                    a=taskspace(qc); b=taskspace(qn)
                    draw_edge(a,b,1.2,(1,1,0,0.7))  
                qc = qn

        if it % 800 == 0:
            dmin = min(distance(n, goal_config) for n in T)
            print(f"[KD-RRT] it={it}, |T|={len(T)}, dmin≈{dmin:.3f}")

    print("[KD-RRT] Failed to find a path.")
    return []

def main():
    print("The demo will take about five minutes to run")
    connect(use_gui=True)
    _, obstacles = load_env(ENV_JSON)
    p.resetDebugVisualizerCamera(
        cameraDistance=8.0,    
        cameraYaw=90,          
        cameraPitch=-89.9,       
        cameraTargetPosition=[0, 0, 0]  
    )
    coll_pose_fn = make_pose_coll_checker(obstacles)

    start_cfg = (-4.45,  4.45, 0.0,        0.0, 0.0, 0.0)
    goal_cfg  = ( 4.45, -4.45,-math.pi/2,  0.0, 0.0, 0.0)

    draw_sphere_marker((goal_cfg[0], goal_cfg[1], 0.10), 0.15, (0,1,0,1))
    draw_sphere_marker((start_cfg[0], start_cfg[1], 0.10), 0.15, (1,0,0,1))

    def draw_edge(a,b, width=1.0, rgba=(1,1,1,0.45)):
        p.addUserDebugLine([a[0],a[1],0.06],[b[0],b[1],0.06], rgba, width)

    path = kd_rrt_connect(start_cfg, goal_cfg, coll_pose_fn, draw_edge=draw_edge)
    if not path:
        wait_if_gui(); disconnect(); return

    robot = load_pybullet(ROBOT_URDF)
    base_joints = [0,1,2]

    for a,b in zip(path[:-1], path[1:]):
        p.addUserDebugLine([a[0], a[1], 0.10], [b[0], b[1], 0.10], [0,1,0], 6.0)

    joint_path = [(q[0], q[1], q[2]) for q in path]
    execute_trajectory(robot, base_joints, joint_path, sleep=0.10)

    wait_if_gui()
    disconnect()


if __name__ == '__main__':
    main()
