# ================================================================================ #
import math
import os
import numpy as np
import sys
import argparse
import time
import random
import scipy
import matplotlib.pyplot as plt
import tensorflow as tf
from mpi4py import MPI
# ================================================================================ #
from baselines.common.cmd_util import make_mujoco_env, mujoco_arg_parser
from baselines.common import tf_util as U
from baselines.common import set_global_seeds
from baselines import logger
import gym
from gym import spaces
from gym.utils import seeding
# ================================================================================ #
import deeprl
import pposgd_simple as pposgd
import mlp_policy
import timechecker
# ================================================================================ #

class Environment():
    def __init__(self):
        self.simulator = deeprl.Env('deeprl')

        self.num_timesteps = int(5E10)

        obs = self.simulator.GetStates()
        self.num_states = len(obs)
        
        self.controlHz = self.simulator.GetControlHz()
        self.simulationHz = self.simulator.GetSimulationHz()
        self.num_control_per_sim = self.simulationHz // self.controlHz
        print('Num control per sim: ',self.num_control_per_sim)
        self.simulation_timestep = self.simulator.GetTimeStep()

        self.observation_space = spaces.Box(
            -5.,5.,shape=(self.num_states,),dtype='float32')

        norm_upper_bound = self.simulator.GetNormUpperBound()
        norm_lower_bound = self.simulator.GetNormLowerBound()

        self.action_space = spaces.Box(
            low=np.array(norm_lower_bound),
            high=np.array(norm_upper_bound),dtype='float32')

        self.reward_range = (-float('inf'), float('inf'))

        self.time_step = self.simulator.GetTimeStep()
        self.elapsed_time = 0.0
        self.max_episode_time = 10.0
        self.seed = 0
        self.metadata = None
        self.spec = None
        self.scale = 1.0
        self.network_name = None
        # reward key 
        reward_dict = self.simulator.GetRewards()
        self.reward_keys = list(reward_dict.keys())
        print('Reward Type: ',self.reward_keys)

    def loadNN(self,path):
        self.network_name = path
        load_dir = '../../model/%s'%self.network_name
        self.pi = self.train(num_timesteps=1,seed=self.seed,model_path=load_dir,argtype='play')

    def get_action(self,state):
        action = np.array(self.pi.act(stochastic=False, ob=state)[0])
        return np.array(action,dtype=np.float32)

    def set_seed(self,seed,reward_scale=1.0):
        rank = MPI.COMM_WORLD.Get_rank()
        if rank == 0:
            logger.configure()
        else:
            logger.configure(format_strs=[])
        workerseed = seed + 10000 * MPI.COMM_WORLD.Get_rank()
        set_global_seeds(workerseed)
        logger_path = None if logger.get_dir() is None else os.path.join(logger.get_dir(), str(rank))
        self.seed = workerseed

        return

    def train(self,num_timesteps,seed,
        model_path=None,model_iter=None,argtype=None):
        sess = U.single_threaded_session()
        sess.__enter__()
        def policy_fn(name, ob_space, ac_space):
            return mlp_policy.MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
                    hid_size=128, num_hid_layers=3)
        self.set_seed(seed)
        pi = pposgd.learn(self, policy_fn,
                max_timesteps=num_timesteps,
                timesteps_per_actorbatch=512,
                clip_param=0.2,entcoeff=0.0,
                optim_epochs=10,optim_stepsize=1e-4,optim_batchsize=64, 
                gamma=0.99, 
                lam=0.95,
                schedule='linear',
                model_path=model_path,
                model_iter=model_iter,
                mode=argtype
            )
        return pi

    def step(self, param):
        elapsed_time = 0.0
        step_num = 0

        self.simulator.SetActions(np.array(param,dtype=np.float32),len(param))
        for i in range(self.num_control_per_sim):
            self.simulator.Step()
            elapsed_time += 1/self.simulationHz
            step_num += 1
        self.elapsed_time += elapsed_time
        obs = self.simulator.GetStates()
        reward = self.simulator.GetRewards()
        eoe = self.simulator.isEndOfEpisode()

        if eoe == 1:
            print('\teoe! - ',end='')
        if self.elapsed_time > self.max_episode_time:
            print('\ttime over!')

        eoe_result = eoe == 1 or self.elapsed_time > self.max_episode_time
        return obs, reward, eoe_result, step_num

    def reset(self,flag):
        self.elapsed_time = 0.0
        self.simulator.Reset()
        if flag == True: 
            self.simulator.UpdateRandomTargetVelocity()
        return self.simulator.GetStates()

def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--type',type=str)
    parser.add_argument('--model',type=str)

    args = parser.parse_args()

    print('===================================================')

    if args.type == "train":
        logger.configure()
        print('TRAINING START!')
        print('Model Name : ',args.model)
        env = Environment()
        env.network_name = 'train_%s'%timechecker.str_datetime()
        pi = env.train(num_timesteps=1E8,seed=8,argtype=args.type)

    if args.type =="retrain":
        logger.configure()
        print('KEEP TRAINING!')
        print('Model Name : ',args.model)
        env = Environment()
        env.network_name = args.model

        load_dir = '../model/%s'%env.network_name
        print(env.network_name)
        _,_,_,iteration = env.network_name.split("_",4)
        pi = env.train(num_timesteps=env.num_timesteps,seed=env.seed,model_path=load_dir,model_iter=int(iteration),argtype=args.type)


    if args.type == "play":
        print('NETWORK PLAY!')
        print('Network Name : ',args.model)
        env = Environment()
        env.network_name = args.model

        load_dir = '../model/%s'%env.network_name
        pi = env.train(num_timesteps=1,seed=env.seed,model_path=load_dir,argtype=args.type)
        env.reset(True)
        obs = env.simulator.GetStates(env.movement_mode)
        rew_list = []
        step_num = 0
        sum_rew= 0
        update_target_time = 0.0
        env.max_episode_time = 20.0
        env.simulator.UpdateRandomTargetVelocity() 
        while True:
            action = pi.act(stochastic=False, ob=obs)[0]
            obs, rew, eoe, eoe_condition, _ = env.step(action,True)
            print(step_num)
            for key in rew.keys():
                print(key, rew[key])
            if env.elapsed_time > env.max_episode_time:
                print('time over!')
                break
            if eoe:
                print('eoe!')
            if update_target_time > 10.0:
                env.simulator.UpdateRandomTargetVelocity() 
                update_target_time = 0.0
            step_num += 1
            update_target_time += 1/env.controlHz

if __name__ == '__main__':
    main()