import gym
import time
import yaml
import numpy as np
from argparse import Namespace

from planner.fgm_stech import FGM as FGM_STECH
from planner.fgm_gnu import FGM as FGM_GNU
from planner.odg_pf import ODGPF
from planner.odg_gnu import ODGGNU
from planner.fgm_conv import GapFollower

if __name__ == '__main__':

    with open('sim_params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)


    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
    # print(info)
    env.render()

    # planner = FGM_GNU(conf)
    # planner = FGM_STECH(conf)
    # planner = ODGPF(conf)
    # planner = ODGGNU(conf)
    planner = GapFollower(conf)

    laptime = 0.0
    start = time.time()
    done_i = 1
    while not done:
        # print(f"speed: {obs['linear_vels_x'][0]}", end='\r')

        scan_data = obs['scans'][0]
        odom_data = {
            'x': obs['poses_x'][0],
            'y': obs['poses_y'][0],
            'theta': obs['poses_theta'][0],
            'linear_vel': obs['linear_vels_x'][0]
        }

        speed, steer = planner.driving(scan_data, odom_data)
        obs, step_reward, done, info = env.step(np.array([[steer, speed]]))
        laptime += step_reward
        env.render(mode='human')

        if done and not info['checkpoint_done'][0]:
            print(f"It seems collision... {done_i} times, Sim elapsed time: {laptime} \nReset Environment.")

            obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
            done_i += 1
            laptime = 0.0



    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)