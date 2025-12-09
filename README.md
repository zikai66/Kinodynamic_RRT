# KD-RRT Hovercraft Demo

This repo contains a **Kinodynamic RRT** demo in PyBullet for a planar hovercraft
(controls in `x`, `y`, and yaw). It builds a search tree, connects aggressively
toward the goal, and then plays the planned trajectory on a simple robot model.

Grading runners will execute:
```bash
./install.sh
python3 demo.py
```

The video demonstration can be found 