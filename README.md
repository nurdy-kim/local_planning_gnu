# Local Planning

### Usage
---

1. Clone this repository

    ```
    cd ~/catkin_ws/src # ros workspace etc) f1tenth_ws
    git clone https://github.com/nurdy-kim/local_planning_gnu
    ```

2. Build Catkin Workspace

    ```
    cd ..
    catkin_make
    source devel/setup.bash
    ```
 
 3. Run
 
    ```
    roslaunch local_planning_gnu [LAUNCH_FILE]
    ```
  
    - `car_fgm_gnu.launch` - Run FGM_GNU on F1TENTH Racecar system
    - `car_fgm_pp.launch` - Run FGM_PurePursuit on F1TENTH Racecar system
    - `car_fgm_seoultech.launch` - Run FGM_SEOULTECH on F1TENTH Racecar system
    - `car_odg_pf.launch` - Run ODG-PF on F1TENTH Racecar system
    - `car_odg_pp.launch` - Run ODG-PurePursuit on F1TENTH Racecar system
    
    
    - `sim_fgm_gnu.launch` - Run FGM_GNU on F1TENTH Gym ROS simulator system
    - `sim_fgm_pp.launch` - Run FGM_PurePursuit on F1TENTH Gym ROS simulator system
    - `sim_fgm_seoultech.launch` - Run FGM_SEOULTECH on F1TENTH Gym ROS simulator system
    - `sim_odg_pf.launch` - Run ODG-PF on F1TENTH Gym ROS simulator system
    - `sim_odg_pp.launch` - Run ODG-PurePursuit on F1TENTH Gym ROS simulator system

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
