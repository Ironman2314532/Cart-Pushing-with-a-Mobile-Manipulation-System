# Cart Pushing with a Mobile Manipulation System

This project explores motion planning for a mobile manipulation robot that pushes a cart through a **dynamic 2D environment**. The system combines a **non-holonomic mobile base** with a **holonomic cart**, requiring motion planners that respect complex kinodynamic constraints.

## Objective

To evaluate and compare the performance of the following motion planning algorithms:
- **A\*** (Baseline)
- **ADA\***
- **Bi2RRT\***

Each planner computes collision-free trajectories from start to goal while accounting for the coupled dynamics of the robot and the cart in a changing environment.

## System Overview

- **Mobile Base**: Differential-drive robot (non-holonomic)
- **Cart**: Holonomic with independent planar movement
- **Planning Constraints**: Collision avoidance, actuator limits, coupled dynamics
- **Environment**: Dynamic 2D space with moving and static obstacles


## How to Run

Make sure Python 3 and the following packages are installed:
- `numpy`
- `pygame`
- `matplotlib`

Run each planner using:

```bash
# A*
python A-star/Move_Robot.py

# ADA*
python ADA_star/Move_Robot.py

# Bi2RRT*
python BI_RRT_star/Move_Robot.py
```

## License
[MIT](https://choosealicense.com/licenses/mit/)


