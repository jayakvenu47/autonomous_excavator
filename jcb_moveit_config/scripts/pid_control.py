#! /usr/bin/env python3
import yaml
import os.path
from PID import PID


CURRENT_PATH = os.getcwd()
PACKAGE_PATH = os.path.abspath(os.path.join(CURRENT_PATH, os.pardir))
CONFIG_FOLDER_PATH = os.path.abspath(os.path.join(PACKAGE_PATH, "config"))
CONFIG_TEST_FILE = os.path.abspath(os.path.join(CURRENT_PATH, "config.txt"))
PID_CONFIG_PATH = os.path.abspath(os.path.join(CONFIG_FOLDER_PATH, "excavator_pid.yaml"))


class PidControlExcavator():
    def __init__(self, target_L=[5, 5, 5]):
        self.sample_time = 0.3
        configs = self.read_config(PID_CONFIG_PATH)
        self.pid_1 = PID(configs["Joint_1"]["p"], configs["Joint_1"]["i"], configs["Joint_1"]["d"])
        self.pid_2 = PID(configs["Joint_2"]["p"], configs["Joint_2"]["i"], configs["Joint_2"]["d"])
        self.pid_3 = PID(configs["Joint_3"]["p"], configs["Joint_3"]["i"], configs["Joint_3"]["d"])

        self.pid_1.SetPoint = target_L[0]
        self.pid_2.SetPoint = target_L[1]
        self.pid_3.SetPoint = target_L[2]

        self.pid_1.setSampleTime(self.sample_time)
        self.pid_2.setSampleTime(self.sample_time)
        self.pid_3.setSampleTime(self.sample_time)

    def read_config(self, pid_config_path = PID_CONFIG_PATH):
        default_pids = {'Joint_1': {'p': 10.0, 'i': 1, 'd': 1},
                        'Joint_2': {'p': 10, 'i': 1, 'd': 1},
                        'Joint_3': {'p': 10, 'i': 1, 'd': 1}}
        if not os.path.isfile(pid_config_path):
            print("There is no yaml file for configuring the PID values of the controller, please check")
            return default_pids
        with open(pid_config_path) as f:
            configs = yaml.load(f)
            return configs
            
        
        
    
    def read_sensor(self, file_path):
        feedback_values = list()
        if not os.path.isfile(file_path):
            return None
        with open (file_path, 'r') as f:
            datas = f.readline().split(',')
            if datas:
                for data in datas:
                    if data:
                        feedback_values.append(float(data))
                    else: 
                        return []
            return feedback_values

    def update_values(self, feedback_values):
        output = list()
        output.append(self.pid_1.update(feedback_values[0]))
        output.append(self.pid_2.update(feedback_values[1]))
        output.append(self.pid_3.update(feedback_values[2]))
        return output


if __name__=='__main__':
    # TO DO: make a yaml file to config PID
    pid_controller = PidControlExcavator()
    while 1:
        feedback_values = pid_controller.read_sensor(CONFIG_TEST_FILE)
        if not feedback_values:
            print("There is no feedback value from the sensor, please check the sensor")
            continue
        target = pid_controller.update_values(feedback_values)
        print(f"The target value we are going to send to the excavator is {target}")
        # send the target to the excavator
        # .......send to drive function........
