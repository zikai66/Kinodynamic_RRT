# KD-RRT Hovercraft Demo

This repo contains a **Kinodynamic RRT** demo in PyBullet for a planar hovercraft
(controls in `x`, `y`, and yaw). It builds a search tree, connects aggressively
toward the goal, and then plays the planned trajectory on a simple robot model.

1. Downlaod the code by excecuting the following command in bash:

```bash
git clone https://github.com/zikai66/Kinodynamic_RRT.git
```

2. Run the following command to build and activate the environment 
```bash
chmod +x install.sh
./install.sh
source .venv/bin/activate
```

3. Run the Demo
```bash
python3 demo.py
```

## Demo Video
Watch the project demo on YouTube: [Demo Video](https://www.youtube.com/watch?v=iivorhKRokE)

## Report
Report(PDF) can be found in final_report folder
