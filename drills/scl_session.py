#!/usr/bin/python3

# Copyright (c) 2019, SCALE Lab, Brown University
# All rights reserved.

# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree. 

import os
import re
import datetime
import numpy as np
from subprocess import check_output
from .features import extract_features

def log(message):
    print('[DRiLLS {:%Y-%m-%d %H:%M:%S}'.format(datetime.datetime.now()) + "] " + message)

class SCLSession:
    """
    A class to represent a logic synthesis optimization session using ABC
    """
    def __init__(self, params):
        self.params = params

        self.action_space_length = len(self.params['optimizations'])
        self.observation_space_size = 9     # number of features

        self.iteration = 0
        self.episode = 0
        self.sequence = ['strash']
        self.delay, self.area, self.power = float('inf'), float('inf'), float('inf')

        self.best_known_area = (float('inf'), float('inf'), float('inf'), -1, -1)
        self.best_known_delay = (float('inf'), float('inf'), float('inf'), -1, -1)
        self.best_known_area_meets_constraint = (float('inf'), float('inf'), float('inf'), -1, -1)
        self.best_known_power = (float('inf'), float('inf'), float('inf'), -1, -1)
        self.initial_delay = float('inf')
        # logging
        self.log = None
    
    def __del__(self):
        if self.log:
            self.log.close()
    
    def reset(self):
        """
        resets the environment and returns the state
        """
        self.iteration = 0
        self.episode += 1
        self.delay, self.area, self.power = float('inf'), float('inf'), float('inf')
        self.sequence = ['strash']
        self.episode_dir = os.path.join(self.params['playground_dir'], str(self.episode))
        if not os.path.exists(self.episode_dir):
            os.makedirs(self.episode_dir)
        
        # logging
        log_file = os.path.join(self.episode_dir, 'log.csv')
        if self.log:
            self.log.close()
        self.log = open(log_file, 'w')
        self.log.write('iteration, optimization, area, delay, power, best_area, best_delay, best_power\n')

        state, _ = self._run()

        # logging
        self.log.write(', '.join([str(self.iteration), self.sequence[-1], str(self.area), str(self.delay), str(self.power)]) + '\n')
        self.log.flush()

        self.initial_delay = self.delay

        return state
    
    def step(self, optimization):
        """
        accepts optimization index and returns (new state, reward, done, info)
        """
        self.sequence.append(self.params['optimizations'][optimization])
        new_state, reward = self._run()

        # logging
        if self.area < self.best_known_area[0]:
            self.best_known_area = (self.area, self.delay, self.power, self.episode, self.iteration)
        if self.delay < self.best_known_delay[1]:
            self.best_known_delay = (self.area, self.delay, self.power, self.episode, self.iteration)
        if self.power < self.best_known_power[2]:
            self.best_known_power = (self.area, self.delay, self.power, self.episode, self.iteration)
        if self.delay <= self.params['mapping']['clock_period'] and self.area < self.best_known_area_meets_constraint[0]:
            self.best_known_area_meets_constraint = (self.area, self.delay, self.power, self.episode, self.iteration)
        self.log.write(', '.join([str(self.iteration), self.sequence[-1], str(self.area), str(self.delay), str(self.power)]) + ', ' +
            '; '.join(list(map(str, self.best_known_area))) + ', ' + 
            '; '.join(list(map(str, self.best_known_delay))) + ', ' + 
            '; '.join(list(map(str, self.best_known_power))) + '\n')
        self.log.flush()

        return new_state, reward, self.iteration == self.params['iterations'], None

    def _run(self):
        """
        run ABC on the given design file with the sequence of commands
        """
        self.iteration += 1
        output_design_file = os.path.join(self.episode_dir, str(self.iteration) + '.v')
        output_design_file_mapped = os.path.join(self.episode_dir, str(self.iteration) + '-mapped.v')
   
        abc_command = 'read ' + self.params['mapping']['library_file'] + '; '
        abc_command += 'read ' + self.params['design_file'] + '; '
        abc_command += ';'.join(self.sequence) + '; '
        abc_command += 'write ' + output_design_file + '; '
        # abc_command += 'map -D ' + str(self.params['mapping']['clock_period']) + '; '
        abc_command += 'map' + '; '
        abc_command += 'upsize' + '; '
        abc_command += 'write ' + output_design_file_mapped + '; '
        abc_command += 'topo; stime;'

        sta_cmdfile = 'test.tcl'
        # Modify tcl cmd file for sta
        with open(sta_cmdfile, "r") as file:
            lines = file.readlines()

        # Modify the third line
        for i, line in enumerate(lines):
            if line.strip().startswith("read_verilog"):
                # Split the line at 'read_verilog' and replace everything after it
                parts = line.split(' ', 1)
                if len(parts) > 1:
                    lines[i] = parts[0] + ' ' + output_design_file_mapped + '\n'
                break

        # Write the modified contents back to the file
        with open(sta_cmdfile, "w") as file:
            file.writelines(lines)

                # Read the content of the file
       
        try:
            proc1 = check_output([self.params['abc_binary'], '-c', abc_command])
            with open(output_design_file_mapped, "r") as file:
                lines = file.readlines()

            for i, line in enumerate(lines):
                stripped_line = line.lstrip()  # Remove leading whitespace
                if stripped_line.startswith("160nm"):
                    # Prepend "tech", and add two spaces at the start of the line
                    lines[i] = "  " + 'tech' + stripped_line

            # Write the modified content back to the file
            with open(output_design_file_mapped, "w") as file:
                file.writelines(lines)

            proc2 = check_output([self.params['sta_binary'], '-no_splash', '-exit', sta_cmdfile])
            #print(proc2)
            # get reward
            area = self._get_metrics(proc1)
            delay = self._get_delay(proc2)
            power = self._get_power(proc2)
            #print(power)
            reward = self._get_reward(delay, area, power)
            self.delay, self.area, self.power = delay, area, power
            # get new state of the circuit
            state = self._get_state(output_design_file)
            return state, reward
        except Exception as e:
            print(e)
            return None, None
        
    def _get_metrics(self, stats):
        """
        parse delay and area from the stats command of ABC
        """
        line = stats.decode("utf-8").split('\n')[-2].split(':')[-1].strip()
        
        ob = re.search(r'Delay *= *[0-9]+.?[0-9]*', line)
        delay = float(ob.group().split('=')[1].strip())
        
        ob = re.search(r'Area *= *[0-9]+.?[0-9]*', line)
        area = float(ob.group().split('=')[1].strip())

        return area

    def _get_delay(self, stats):
        """
        parse delay from the stats command of OpenSTA
        """
        line = stats.decode("utf-8")
        
        match = re.search(r'\s+([\d\.\-]+)\s+data arrival time',  line)
        
        if match:
            delay = float(match.group(1))
            #print(delay)
            return delay
        else:
            return None

    def _get_power(self, stats):
        """
        parse power information from the stats command of OpenSTA
        """
        line = stats.decode("utf-8")
        
        pattern = r"^Total\s+\S+\s+\S+\s+(\S+)"
        match = re.search(pattern, line, re.MULTILINE)

        if match:
            power = float(match.group(1)) # This captures the third number after 'Total'
            print("Extracted power:", power)
            return power
        else:
            print("No match found")
            return None

    def _get_reward(self, delay, area, power):
        # Calculate changes in area, delay, and power
        ''' Dynamic reward calculation
        delta_area = self.area - area
        delta_delay = self.delay - delay
        delta_power = self.power - power

        # Normalize changes
        norm_delta_area = delta_area / self.area if self.area else 0
        norm_delta_delay = delta_delay / self.delay if self.delay else 0
        norm_delta_power = delta_power / self.power if self.power else 0

        # Dynamic adjustment of weights based on the learning phase
        initial_phase = 0.5  # Initial phase is 50% of total episodes
        current_phase = self.episode / float(self.params['episodes'])

        if current_phase < initial_phase:
            w_area = 0
            w_delay = 0.6
            w_power = 0.4
        else:
            w_area = 0.3
            w_delay = 0.5
            w_power = 0.2

        # Calculate reward as a weighted sum of normalized area, delay, and power improvements
        reward = (w_area * norm_delta_area) + (w_delay * norm_delta_delay) + (w_power * norm_delta_power)
        '''
        '''
        constraint_met = True
        constraint_power = True

        optimization_improvement = 0    # (-1, 0, 1) <=> (worse, same, improvement)
        constraint_improvement = 0      # (-1, 0, 1) <=> (worse, same, improvement)
        power_improvement = 0

        # check optimizing parameter
        if area < self.area:
            optimization_improvement = 1
        elif area == self.area:
            optimization_improvement = 0
        else:
            optimization_improvement = -1
        
        # check constraint parameter
        if delay > self.params["mapping"]["clock_period"]:
            constraint_met = False
            if delay < self.delay:
                constraint_improvement = 1
            elif delay == self.delay:
                constraint_improvement = 0
            else:
                constraint_improvement = -1

        # check power parameter
        if power > 0.00133:
            constraint_power = False
            if power < self.power:
                power_improvement = 1
            elif delay == self.delay:
                power_improvement = 0
            else:
                power_improvement = -1

        # now calculate the reward

        return self._reward_table(constraint_power, constraint_met, constraint_improvement, optimization_improvement, power_improvement)
        '''
        delta_area = self.area - area
        delta_delay = self.delay - delay
        delta_power = self.power - power

        # Normalize changes
        norm_delta_area = delta_area / self.area if self.area else 0
        norm_delta_delay = delta_delay / self.delay if self.delay else 0
        norm_delta_power = delta_power / self.power if self.power else 0

        # Check improvement threshold for delay
        delay_improvement_threshold = 0.20  # 20% improvement
        # print((self.initial_delay - delay) / self.initial_delay)
        is_delay_improved = ((self.initial_delay - delay) / self.initial_delay) >= delay_improvement_threshold

        # Set weights based on whether the delay improvement threshold has been met
        if not is_delay_improved:
            # Focus solely on delay
            w_area = 0
            w_delay = 1
            w_power = 0
        else:
            # Focus on the product of delay and power once the delay is improved by 20%
            w_area = 0
            w_delay = 0.5
            w_power = 0.5

        # Calculate reward as a weighted sum of normalized area, delay, and power improvements
        reward = (w_area * norm_delta_area) + (w_delay * norm_delta_delay) + (w_power * norm_delta_power)
        return reward

    def _reward_table(self, constraint_power, constraint_met, contraint_improvement, optimization_improvement, power_improvement):
        return {
            True:{
                True: {
                    0: {
                        1: 4,
                        0: 0,
                        -1: -1
                    }
                },
                False: {
                    1: {
                        1: 3,
                        0: 2,
                        -1: 1
                    },
                    0: {
                        1: 2,
                        0: 0,
                        -1: -2
                    },
                    -1: {
                        1: -1,
                        0: -2,
                        -1: -3
                    }
                }
            },
            False: {
                True: {
                    0: {
                        1: 3,
                        0: 0,
                        -1: -1
                    }
                },
                False: {
                    1: {
                        1: 2,
                        0: 1,
                        -1: 0
                    },
                    0: {
                        1: 1,
                        0: 0,
                        -1: -1
                    },
                    -1: {
                        1: -1,
                        0: -2,
                        -1: -3
                    }
                }
            }
        }[constraint_power][constraint_met][contraint_improvement][optimization_improvement]

    def _get_state(self, design_file):
        return extract_features(design_file)
    
