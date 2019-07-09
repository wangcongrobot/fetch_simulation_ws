#! /usr/bin/env python
import numpy as np
import gym
import os
import sys
from arguments import get_args, Args
from mpi4py import MPI
from subprocess import CalledProcessError
from ddpg_agent import ddpg_agent
import random
import torch
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
import rospy

"""
train the agent, the MPI part code is copy from openai baselines(https://github.com/openai/baselines/blob/master/baselines/her)

"""


def get_env_params(env):
    obs = env.reset()
    rospy.logwarn(obs)

    # params = {'obs': obs['observation'].shape[0],
    #           'goal': obs['desired_goal'].shape[0],
    #           'action': env.action_space.shape[0],
    #           'action_max': env.action_space.high[0],
    #           }

    # close the environment
    params = {'obs': 1,
              'goal': 1,
              'action': 3,
              'action_max': 1.,
              }
    params['max_timesteps'] = 50
    return params


def launch(args):
    # create the ddpg_agent
    task_and_robot_environment_name = rospy.get_param(
        '/fetch/task_and_robot_environment_name')
    # to register our task env to openai env.
    # so that we don't care the output of this method for now.
    env = StartOpenAI_ROS_Environment(task_and_robot_environment_name)
    # env = gym.make(args.env_name)

    # set random seeds for reproduce
    env.seed(args.seed + MPI.COMM_WORLD.Get_rank())
    random.seed(args.seed + MPI.COMM_WORLD.Get_rank())
    np.random.seed(args.seed + MPI.COMM_WORLD.Get_rank())
    torch.manual_seed(args.seed + MPI.COMM_WORLD.Get_rank())
    if args.cuda:
        torch.cuda.manual_seed(args.seed + MPI.COMM_WORLD.Get_rank())
    # get the environment parameters
    env_params = get_env_params(env)
    # create the ddpg agent to interact with the environment
    ddpg_trainer = ddpg_agent(args, env, env_params)
    ddpg_trainer.learn()


if __name__ == '__main__':
    rospy.init_node('train_fetch_her',
                    anonymous=True, log_level=rospy.WARN)
    # take the configuration for the HER
    os.environ['OMP_NUM_THREADS'] = '1'
    os.environ['MKL_NUM_THREADS'] = '1'
    os.environ['IN_MPI'] = '1'
    # get the params
    args = Args()
    launch(args)
