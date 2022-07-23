#! /usr/bin/env python3

import rospy
from utils import range_finder as rf
# from algorithms.bug2 import BUG2
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tempfile import TemporaryFile
import torch.nn.functional as F
from cv_bridge import CvBridge
import torch.optim as optim
import torch.nn as nn
import numpy as np
import pickle
import torch
import yaml
import time
import cv2
import os

pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
lidar_g = None


class LinearDeepQNetwork(nn.Module):
    def __init__(self, lr, n_actions, input_dims):
        # run the constructor of the parent class.
        super(LinearDeepQNetwork, self).__init__()

        self.fc1 = nn.Linear(input_dims, 256)
        self.fc2 = nn.Linear(256, 256)
        self.dropout = nn.Dropout(0.2)
        self.fc3 = nn.Linear(256, n_actions)
        # choose gradient decent method for backward propergation
        self.optimizer = optim.Adam(self.parameters(), lr=lr)
        self.loss = nn.MSELoss()

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        # send the network to the device
        self.to(self.device)

    # forward propergation: activation function
    def forward(self, state):
        layer1 = F.relu(self.fc1(state))
        layer2 = F.relu(self.fc2(layer1))
        drop_out = self.dropout(layer2)
        out_actions = self.fc3(drop_out)
        return out_actions


def updateLidar(lidar):
    global lidar_g
    lidar_g = lidar


sub_scan = rospy.Subscriber('scan', LaserScan, updateLidar)

TURTLE = '003'
bridge = CvBridge()
state = None
font = cv2.FONT_HERSHEY_SIMPLEX
outfile = TemporaryFile()

# Hyper parameters
episodes = 50
max_steps = 50000
action_low = [-1.5, -0.1]
action_high = [1.5, 0.12]

# Loading configs from config.yaml
path = os.path.dirname(os.path.abspath(__file__))
with open(path + '/config.yml', 'r') as ymlfile:
    config = yaml.load(ymlfile, Loader=yaml.FullLoader)

env = input('Which environment are you running? [1 | 2 | l]:\n')
rospy.init_node(config['env_name'].replace('-', '_') + "_test_real")
real_ttb = rf.RealTtb(config, path, output=(640, 640))
bridge = CvBridge()


def getImage(image):
    global state
    image = bridge.compressed_imgmsg_to_cv2(image)
    size = image.shape
    frame = image[round(size[0] * 0.1):round(size[0] * 0.9), round(size[1] * 0.28):round(size[1] * 0.68)]
    angle = distance = None
    try:
        lidar = np.array(lidar_g.ranges)
        lidar = np.array([min(lidar[[i - 1, i, i + 1]]) for i in range(7, 361, 15)]).squeeze()
        angle, distance, frame = real_ttb.get_angle_distance(frame, lidar, green_magnitude=1.0)
        distance *= 2.
    except:
        pass

    if not angle is None and not distance is None:
        state = np.hstack([lidar, angle, distance])

    # out.write(frame)
    # Display the resulting frame
    #cv2.imshow('frame', frame)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    pass


sub_image = rospy.Subscriber('/usb_cam/compressed/image_right', CompressedImage, getImage, queue_size=1)

path_results = path + '/real_results'
if not os.path.exists(path_results):
    os.makedirs(path_results)

time.sleep(1)
algorithms_sel = np.array(['1', '2', '3', 'e', 'r'])
while True:
    algorithm = ""
    while not any(algorithm.lower() == algorithms_sel):
        print('Choose the algorithm or exit the test:')
        algorithm = input('1->DQN | 2->Double-DQN | 3->BUG2 | e->exit | r->reset\n')

    if algorithm.lower() == 'e':
        break
    if algorithm.lower() == 'r':
        real_ttb.cleanPath()
        continue

    if algorithm != '3':
        model = f"ddqn_st{env.upper()}_model_5k.pth" if algorithm.lower() == '2' else f"dqn_st{env.upper()}_model_5k.pth"
        model_fn = f"{path}/{model}"

        # Loading neural network model
        action_size = 5
        observation_space = 26
        actor = LinearDeepQNetwork(0, action_size, observation_space)

        try:
            actor.load_state_dict(torch.load(model_fn, map_location=config['device']))
        except:
            actor = torch.load(model_fn)
            actor.to(config['device'])
        actor.eval()
    # else:
        # b2 = BUG2()

    det = {0: -1.5, 1: -0.75, 2: 0, 3: 0.75, 4: 1.5}  # turning actions into angular vel
    local_episode = 0
    while local_episode < episodes:
        if local_episode == 0:
            quit = input("Press [Enter] to start the test or press [q] to quit...")
        else:
            quit = input("Press [Enter] to continue to the next episode or press [q] to quit...")
        if quit.lower() == 'q':
            break

        real_ttb.cleanPath()
        episode_reward = 0
        lidar_list = list()
        num_steps = 0
        local_episode += 1
        ep_start_time = time.time()
        done = False
        while True:
            start = time.time()
            print('Num steps:', num_steps)
            if state is not None:
                for s in range(len(state)):
                    if state[s] > 2.0:
                        state[s] = 2.0

            val_state = state
            state[:-2] *= state[:-2] * 2.2
            print('State:', state)

            if algorithm != '7':
                state = torch.tensor(state, dtype=torch.float).to(config['device'])  # turn to pytorch tensor
                actions = actor(state)  # pass to dqn, this is q value, same as self.model.forward(state)
                action = torch.argmax(actions).item()
            # else:
                # action = b2.get_action(state)

            print('Action:', action)
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.06
            vel_cmd.angular.z = det[action] * 0.65
            pub_cmd_vel.publish(vel_cmd)

            done = False
            reward = 0
            for i in range(len(val_state[:24])):
                if val_state[i] == 0.0:
                    val_state[i] = 1.0
            if val_state[-1] < 0.3:
                done = True
                reward = 20
            if 0.005 < min(val_state[0:24]) < 0.001:
                done = True
                reward = -200
            episode_reward += reward

            scan = val_state[0:24]
            # lidar_list.append(scan)

            print('Done:', done)
            if done or num_steps == max_steps:
                break
            else:
                num_steps += 1

            print('Step timing:', time.time() - start)
            fps = round(1 / (time.time() - start))
            print('FPS:', fps)
            time.sleep(0.15)

        # Log metrics
        episode_timing = time.time() - ep_start_time
        print(f"Agent: [Test] Episode: [{local_episode}/{episodes}] Reward: [{episode_reward}/20] "
              f"Steps: [{num_steps}/{max_steps}] Episode Timing: {round(episode_timing, 2)}s")

        # Save log file
        values = [episode_reward, episode_timing, local_episode, num_steps, real_ttb.pts, lidar_list]
        with open(path_results + '/{}_st{}_t{}'.format('dqn' if algorithm == '1' else 'ddqn', env, local_episode), "wb") as fp:
            pickle.dump(values, fp)
    # out.release()
    print('Episode done!')
