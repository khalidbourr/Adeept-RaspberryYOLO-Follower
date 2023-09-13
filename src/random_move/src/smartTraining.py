#!/usr/bin/env python3

import rospy
import random
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple
import sys

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
device = torch.device("cpu")


from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA



pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
led_pub = rospy.Publisher('/led', ColorRGBA, queue_size=10)





velX = 0
anguZ = 0
minRange = 2.5
obs_num = 3
acts_num = 10
total_rewards = []
number_of_steps = []
total_durations = []
turning_counter = 0

Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))
GAMMA = 0.99  
MAX_STEPS = 1000
NUM_EPISODES = 700




########################################################## Robot class ###################################################################################

class Robot:
    def __init__(self, num_states, num_actions):
        self.core = Core(num_states, num_actions)

    def update_q_function(self):
       self.core.replay()

    def get_action(self, state, episode):
        action = self.core.decide_action(state, episode)
        return action

    def memorize(self, state, action, state_next, reward):
        self.core.memory.push(state, action, state_next, reward)
        #print("State: {}, Action: {}, State Next: {}, Reward: {}".format(state, action, state_next, reward))

    def update_target_q_function(self):
        self.core.update_target_q_network()
        
    def save(self):
        self.core.save_main_q_network() 
        



#the Core class is used for reinforcement learning
class Core:

 # Constructor for initializing the class
    def __init__(self, num_states, num_actions):
        self.num_actions = num_actions
        self.memory = ReplayMemory(CAPACITY)
        n_in, n_mid, n_out = num_states, 64, num_actions
        self.main_q_network = Net(n_in, n_mid, n_out).to(device)
        self.target_q_network = Net(n_in, n_mid, n_out).to(device)
        self.optimizer = optim.Adam(self.main_q_network.parameters(), lr=0.0001)

# Method for implementing the experience replay
    def replay(self):
        if len(self.memory) < BATCH_SIZE:
            return
        self.batch, self.state_batch, self.action_batch, self.reward_batch, self.non_final_next_states = self.make_minibatch()
        self.expected_state_action_values = self.get_expected_state_action_values()
        self.update_main_q_network()
        
    def decide_action(self, state, episode):
        initial_epsilon = 1.0
        final_epsilon = 0.1
        decay_rate = 0.99
        epsilon = max(final_epsilon, initial_epsilon * (decay_rate ** episode))

        if epsilon <= np.random.uniform(0, 1):
            self.main_q_network.eval()
            with torch.no_grad():
                output = self.main_q_network(state).squeeze()
                action = output.max(0)[1].view(1, 1)
                

        else:
            action = torch.LongTensor(
                [[random.randrange(self.num_actions)]]) 
        return action
        
        
    def make_minibatch(self):
        transitions = self.memory.sample(BATCH_SIZE)
        batch = Transition(*zip(*transitions))
        state_batch = torch.cat(batch.state).view(-1, 3)  # Reshape into [64, 3] if each state is [1, 3]
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)
        non_final_next_states = torch.cat([s for s in batch.next_state if s is not None]).view(-1, 3)  



        return batch, state_batch, action_batch, reward_batch, non_final_next_states


    def get_expected_state_action_values(self):
        self.main_q_network.eval()
        self.target_q_network.eval()


        self.state_action_values = self.main_q_network(self.state_batch).gather(1, self.action_batch)

        non_final_mask = torch.BoolTensor(tuple(map(lambda s: s is not None, self.batch.next_state))).to(device)
        next_state_values = torch.zeros(BATCH_SIZE).to(device)
        a_m = torch.zeros(BATCH_SIZE).type(torch.LongTensor).to(device)

        a_m[non_final_mask] = self.main_q_network(self.non_final_next_states).detach().max(1)[1]
        a_m_non_final_next_states = a_m[non_final_mask].view(-1, 1)

        next_state_values[non_final_mask] = self.target_q_network(self.non_final_next_states).gather(1, a_m_non_final_next_states).detach().squeeze()

        expected_state_action_values = self.reward_batch + GAMMA * next_state_values

        return expected_state_action_values
       
        
    def update_main_q_network(self):
        self.main_q_network.train()
        loss = F.smooth_l1_loss(self.state_action_values,
		self.expected_state_action_values.unsqueeze(1))
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
    def update_target_q_network(self):
        self.target_q_network.load_state_dict(self.main_q_network.state_dict())
        
    def save_main_q_network(self):
        torch.save(self.main_q_network.state_dict(),'/home/bourr/Adeept_Neotic/adeept_ws/src/random_move/src/Trained/Main.pt')
        torch.save(self.target_q_network.state_dict(),'/home/vampiro/xkalim2_ws/src/neuronbot2/neuronbot2_gazebo/src/Target.pt')   
        
        
########################################################## Class of neural network ###################################################################################
                          
                          
class Net(nn.Module):

    def __init__(self, n_in, n_mid, n_out):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(n_in, n_mid)
        self.fc2 = nn.Linear(n_mid, n_mid)
        self.fc3 = nn.Linear(n_mid, n_mid)
        self.fc4 = nn.Linear(n_mid, n_out)

    def forward(self, x):
        h1 = F.relu(self.fc1(x))
        h2 = F.relu(self.fc2(h1))
        h3 = F.relu(self.fc3(h2))
        output = self.fc4(h3)
        return output

BATCH_SIZE = 64
CAPACITY = 10000      


class ReplayMemory:

    def __init__(self, CAPACITY):
        self.capacity = CAPACITY
        self.memory = []
        self.index = 0

    def push(self, state, action, state_next, reward):
        state_tensor = torch.tensor(state, dtype=torch.float32).to(device)
    
    # Convert reward to tensor
        reward_tensor = torch.tensor([reward], dtype=torch.float32).to(device)
    
    # Convert state_next to tensor only if it's not None
        state_next_tensor = torch.tensor(state_next, dtype=torch.float32).to(device) if state_next is not None else None
    
        if len(self.memory) < self.capacity:
            self.memory.append(None)

    # Store the tensors, not the original variables
        self.memory[self.index] = Transition(state_tensor, action, state_next_tensor, reward_tensor)

        self.index = (self.index + 1) % self.capacity



    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)
        

# Get range data from ultrasonic/range sensor
def range_callback(range_msg):
    global minRange
    minRange = range_msg.range




def get_robot_vel(data):
    global velX, anguZ
    velX = data.linear.x  
    anguZ= data.angular.z


def collision_detected():
    global minRange
    led_msg = ColorRGBA()
    led_msg.r = 1.0  # In ROS std_msgs/ColorRGBA, RGB values are from 0 to 1
    led_msg.g = 0.0
    led_msg.b = 0.0
    led_msg.a = 0.0
    if minRange < 0.2:
        led_pub.publish(led_msg)
        return True
    led_msg0 = ColorRGBA()
    led_msg0.r = 0.0  # In ROS std_msgs/ColorRGBA, RGB values are from 0 to 1
    led_msg0.g = 0.0
    led_msg0.b = 0.0
    led_msg0.a = 0.0  
    led_pub.publish(led_msg)      
    return False
    

def stop_robot():
    # Create a Twist message with zero velocities
    stop_twist = Twist()
    stop_twist.linear.x = 0.0
    stop_twist.angular.z = 0.0
    
    # Publish the message
    pub_vel.publish(stop_twist)
 

def simulate_mobile_robot(robot, NUM_EPISODES=100, MAX_STEPS=800):
    global velX, anguZ, minRange
    total_rewards = []
    total_durations = []
    action_to_vel_ang = {
        0: {'linear': -1.0, 'angular': 0.0},  # Move backward fast
        1: {'linear': 1.0, 'angular': 0.0},   # Move forward fast
        2: {'linear': 0.0, 'angular': 1.0},   # Turn left fast
	3: {'linear': 0.0, 'angular': -1.0},  # Turn right fast
        4: {'linear': 0.5, 'angular': 0.5},   # Move forward and turn left
        5: {'linear': 0.5, 'angular': -0.5},  # Move forward and turn right
	6: {'linear': -0.5, 'angular': 0.5},  # Move backward and turn left
        7: {'linear': -0.5, 'angular': -0.5}, # Move backward and turn right
	8: {'linear': 0.2, 'angular': 0.8},   # Slow forward and fast turn left
	9: {'linear': 0.8, 'angular': 0.2}    # Fast forward and slow turn left
    }

    
    for episode in range(NUM_EPISODES):
        state = torch.FloatTensor([velX, anguZ, minRange])
        episode_reward = 0
        episode_10_list = np.zeros(10)
        done = False
        stop_robot()
        input("Please replace the robot to its initial position and press Enter to continue...")

        for step in range(MAX_STEPS):
            action = robot.get_action(state, episode)
            action1 = int(action.item())
            twist = Twist()
            effect = action_to_vel_ang.get(action1, {'linear': 0.0, 'angular': 0.0})
            velX += effect['linear']
            anguZ += effect['angular']
            velX = np.clip(velX, -1, 1)
            anguZ = np.clip(anguZ, -1, 1)  
            print(action) 
            twist.linear.x = velX
            twist.angular.z = anguZ
            pub_vel.publish(twist)
            
            next_state = torch.FloatTensor([velX, anguZ, minRange])



            # Check for termination
            if collision_detected() or step == MAX_STEPS - 1:
                done = True

            if done:
                if collision_detected():
                    reward = torch.FloatTensor([-1000.0])
                    turning_counter = 0
                else:
                    reward = torch.FloatTensor([1000.0])
                    turning_counter = 0
                next_state = None
            else:
                if velX>0:  
                    reward = torch.FloatTensor([100.0])  # Add your other rewards here
                    turning_counter = 0
                elif anguZ!=0:
                    turning_counter +=1
                    if turning_counter > 3:
                        reward = torch.FloatTensor([-50.0])    
                    else: 
                        reward = torch.FloatTensor([20.0])    
                else: 
                    reward = torch.FloatTensor([-10.0])           

            robot.memorize(state, action, next_state, reward.item())
            robot.update_q_function()
            episode_reward += reward.item()
            state = next_state

            if done:
                episode_10_list = np.hstack((episode_10_list[1:], episode_reward))
                total_rewards.append(episode_reward)
                total_durations.append(step)
                
                if episode % 10 == 0:
                    robot.update_target_q_function()
                plot_durations(step)
                plot_rewards()
                break
            if rospy.is_shutdown():
                print("ROS is shutting down. Exiting the program.")
                sys.exit(0)  

       # plt.figure(figsize=(12, 6))
       # plt.subplot(1, 2, 1)
       # plt.title('Total Rewards')
       # plt.plot(total_rewards)
       # plt.subplot(1, 2, 2)
       # plt.title('Episode Duration')
       # plt.plot(total_durations)
       # plt.show()    
    
      
    
    
    
def plot_rewards():
    plt.figure(2)
    plt.clf()
    plt.title('Training')
    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.plot(total_rewards)
    plt.pause(0.001)
    
def plot_end_distances():
    plt.figure(3)
    plt.clf()
    plt.title('Training')
    plt.xlabel('Episode')
    plt.ylabel('End Distance to Goal')
    plt.plot(end_distances)
    plt.pause(0.001)    
    
def plot_data():
    fig, (ax1, ax2) = plt.subplots(1, 2)
    
    # Plot rewards
    ax1.plot(total_rewards)
    ax1.set_title("Episode Rewards")
    ax1.set_xlabel("Episode")
    ax1.set_ylabel("Total Reward")
    
    # Plot durations
    ax2.plot(total_durations)
    ax2.set_title("Episode Durations")
    ax2.set_xlabel("Episode")
    ax2.set_ylabel("Duration (in steps)")
    
    plt.show()    

def plot_collisions():
    plt.figure(4)
    plt.clf()
    plt.title('Training')
    plt.xlabel('Episode')
    plt.ylabel('Number of Collisions')
    plt.plot(num_collisions)
    plt.pause(0.001)

def plot_durations(step):
    plt.figure(2)
    plt.clf()
    number_of_steps.append(step)
    x = np.arange(0, len(number_of_steps))
    plt.title('Training')
    plt.xlabel('Episode')
    plt.ylabel('Duration')
    plt.plot(x, number_of_steps)

    plt.pause(0.001)



number_of_steps = []

if __name__ == '__main__':
    ######## initialize the ros Node #################
    rospy.init_node('training_node', anonymous=True)
    
    ############# subscribe to get the data from sensors #############
    rospy.Subscriber("/range", Range, range_callback)
    rospy.Subscriber("/cmd_vel", Twist, get_robot_vel)

    # State size representing: (linear_velocity, angular_velocity, distance_to_obstacle)
    num_states = 3  
    num_actions = 10
    robot = Robot(num_states, num_actions)

    try:
        simulate_mobile_robot(robot)
            
    except KeyboardInterrupt:            
        print("Training is shutting down")


    robot.save()
    x= np.range(0,len(number_of_steps))
    plt.plot(x,number_of_steps)
    plt.show()

      
           
