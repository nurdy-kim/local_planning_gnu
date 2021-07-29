import gym
import time
import yaml
import numpy as np
from argparse import Namespace

from planner.fgm_stech import FGM as FGM_STECH

if __name__ == '__main__':

    with open('sim_params.yaml') as file:
        conf_dict = yaml.load(file, Loader=yaml.FullLoader)
    conf = Namespace(**conf_dict)

    env = gym.make('f110_gym:f110-v0', map=conf.map_path, map_ext=conf.map_ext, num_agents=1)
    obs, step_reward, done, info = env.reset(np.array([[conf.sx, conf.sy, conf.stheta]]))
    env.render()

    # planner = FGM_GNU(conf)
    planner = FGM_STECH(conf)

    laptime = 0.0
    start = time.time()

    while not done:
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

    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)