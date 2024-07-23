#!/usr/bin/python3

# Copyright (c) 2019, SCALE Lab, Brown University
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 


import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import datetime
from .scl_session import SCLSession as SCLGame
from .fpga_session import FPGASession as FPGAGame
# from torch.utils.tensorboard import SummaryWriter
# from setuptools._vendor.packaging.version import Version as LooseVersion

def log(message):
    print('[DRiLLS {:%Y-%m-%d %H:%M:%S}'.format(datetime.datetime.now()) + "] " + message)

class Normalizer():
    def __init__(self, num_inputs):
        self.num_inputs = num_inputs
        self.n = torch.zeros(num_inputs)
        self.mean = torch.zeros(num_inputs)
        self.mean_diff = torch.zeros(num_inputs)
        self.var = torch.zeros(num_inputs)

    def observe(self, x):
        x = torch.tensor(x, dtype=torch.float32)
        self.n += 1.
        last_mean = self.mean.clone()
        self.mean += (x - self.mean) / self.n
        self.mean_diff += (x - last_mean) * (x - self.mean)
        self.var = torch.clamp(self.mean_diff / self.n, min=1e-2, max=1e9)

    def normalize(self, inputs):
        inputs = torch.tensor(inputs, dtype=torch.float32)
        obs_std = torch.sqrt(self.var)
        return (inputs - self.mean) / obs_std
    
    def reset(self):
        self.n = torch.zeros(self.num_inputs)
        self.mean = torch.zeros(self.num_inputs)
        self.mean_diff = torch.zeros(self.num_inputs)
        self.var = torch.zeros(self.num_inputs)

class ActorCritic(nn.Module):
    def __init__(self, state_size, num_actions):
        super(ActorCritic, self).__init__()
        self.state_size = state_size
        self.num_actions = num_actions

        # Actor
        self.actor_fc1 = nn.Linear(state_size, 20)
        self.actor_fc2 = nn.Linear(20, 20)
        self.actor_fc3 = nn.Linear(20, num_actions)

        # Critic
        self.critic_fc1 = nn.Linear(state_size, 10)
        self.critic_fc2 = nn.Linear(10, 1)
        
        nn.init.xavier_uniform_(self.actor_fc1.weight)
        nn.init.xavier_uniform_(self.actor_fc2.weight)
        nn.init.xavier_uniform_(self.actor_fc3.weight)
        nn.init.xavier_uniform_(self.critic_fc1.weight)
        nn.init.xavier_uniform_(self.critic_fc2.weight)
        
        for layer in [self.actor_fc1, self.actor_fc2, self.actor_fc3, self.critic_fc1, self.critic_fc2]:
            nn.init.constant_(layer.bias, 0)


    def forward(self, x):
        # Actor
        a = torch.relu(self.actor_fc1(x))
        a = torch.relu(self.actor_fc2(a))
        action_probs = torch.softmax(self.actor_fc3(a), dim=-1)

        # Critic
        c = torch.relu(self.critic_fc1(x))
        state_values = self.critic_fc2(c)

        return action_probs, state_values

class A2C:
    def __init__(self, options, load_model=False, fpga_mapping=False):
        if fpga_mapping:
            self.game = FPGAGame(options)
        else:
            self.game = SCLGame(options)

        self.num_actions = self.game.action_space_length
        self.state_size = self.game.observation_space_size
        self.normalizer = Normalizer(self.state_size)

        self.model = ActorCritic(self.state_size, self.num_actions)
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.01)
        
        # model saving/restoring
        self.model_dir = options['model_dir']
        
        # tensorboard
        #self.writer = SummaryWriter()
        #self.global_step = 0

        if load_model:
            self.model.load_state_dict(torch.load(self.model_dir))
            log("Model restored.")
        else:
            self.model.train()

        self.gamma = 0.99

    def save_model(self):
        torch.save(self.model.state_dict(), self.model_dir)
        log("Model saved in path: %s" % str(self.model_dir))
        
        #self.writer.flush()

    def train_episode(self):
        state = self.game.reset()
        self.normalizer.reset()
        self.normalizer.observe(state)
        state = self.normalizer.normalize(state)
        done = False
        
        episode_states = []
        episode_actions = []
        episode_rewards = []
        
        while not done:
            log('Iteration: ' + str(self.game.iteration))
            state_tensor = torch.from_numpy(np.array(state).reshape(1, self.state_size)).float()
            action_probs, _ = self.model(state_tensor)
            action_probs = action_probs.detach().numpy()
            action = np.random.choice(range(action_probs.shape[1]), p=action_probs.ravel())
            new_state, reward, done, _ = self.game.step(action)
            
            # append this step
            episode_states.append(state.tolist())
            action_ = np.zeros(self.num_actions)
            action_[action] = 1
            episode_actions.append(action_.tolist())
            episode_rewards.append(reward)
            
            state = new_state
            self.normalizer.observe(state)
            state = self.normalizer.normalize(state)
            
            #self.global_step += 1
        
        # Now that we have run the episode, we use this data to train the agent
        discounted_episode_rewards = self.discount_and_normalize_rewards(episode_rewards)
        discounted_episode_rewards = torch.tensor(discounted_episode_rewards).float()


        episode_states = torch.tensor(episode_states).float()
        episode_actions = torch.tensor(episode_actions).float()
        episode_rewards = torch.tensor(episode_rewards).float()
        
        action_probs, state_values = self.model(episode_states)
        advantage = discounted_episode_rewards - state_values
        advantage = advantage.detach().unsqueeze(-1)
        # Critic loss
        critic_loss = (advantage.pow(2)).mean()

        # Actor loss
        action_log_probs = torch.log(action_probs)
        
       # print("action_log_probs shape:", action_log_probs.shape)
       # print("episode_actions shape:", episode_actions.shape)
       # print("advantage shape:", advantage.shape)
        
        actor_loss = -(action_log_probs * episode_actions * advantage.detach()).mean()

        # Total loss
        total_loss = critic_loss + actor_loss
        self.optimizer.zero_grad()
        total_loss.backward()
        self.optimizer.step()

        self.save_model()
       # print(episode_rewards)
       
       # self.writer.add_scalar('Loss/total_loss', total_loss.item(), self.global_step)
       # self.writer.add_scalar('Rewards/total_rewards', np.sum(episode_rewards.numpy()), self.global_step)
        
        return np.sum(episode_rewards.numpy())
    
    def discount_and_normalize_rewards(self, episode_rewards):
        discounted_episode_rewards = np.zeros_like(episode_rewards)
        cumulative = 0.0
        for i in reversed(range(len(episode_rewards))):
            cumulative = cumulative * self.gamma + episode_rewards[i]
            discounted_episode_rewards[i] = cumulative

        mean = np.mean(discounted_episode_rewards)
        std = np.std(discounted_episode_rewards)
    
        discounted_episode_rewards = (discounted_episode_rewards - mean) / std
    
        return discounted_episode_rewards

