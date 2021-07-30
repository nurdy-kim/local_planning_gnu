# Local Planning



### Usage (pygame)
---

1. Clone this repository

    ```
    git clone https://github.com/nurdy-kim/local_planning_gnu
    cd local_planning_gnu
    git checkout pygame
    ```


2. Requirements

    1. Framework
    ```
    python3 3.8 or later
    gym
    pyyaml
    numba
    numpy
    ```

   2. F1TENTH Gym Environment
    
    to install f1tenth_gym, follow the [Quickstart](https://github.com/f1tenth/f1tenth_gym) at f1tenth/f1tenth_gym


3. Run
    1. Quickstart
    ```
    python3 sim.py
    ```
    2. Choose Planner
    ```
    # Import class from planner/ folder.
    from planner.fgm_stech import FGM as FGM_STECH
    from planner.fgm_gnu import FGM as FGM_GNU
    from planner.odg_pf import ODGPF
    
    ...
    
    # choose you want 
    # planner = FGM_GNU(conf)
    # planner = FGM_STECH(conf)
    planner = ODGPF(conf)
    ```


4. Parameters
 
   If you want to change the parameters, you need modify `params.yaml`

    - `wpt_path` - **real** path of waypoint file (you **MUST** change!)
    - `wpt_delimeter` - delimiter of csv file
    - `max_speed` - Max speed 
    - `min_speed` - Min speed
    - `rate` - provoid particular rate for a loop (Hz)
    - `robot_length` - length of wheelbase
    
    - `robot_scale` - width of car
    - `mass` - mass of car
    - `mu` - surface friction coefficient
    - `pi` - π
    - `g`- Gravitational acceleration 
    
5. 
