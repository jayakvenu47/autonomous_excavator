#! /usr/bin/env python3

import nidaqmx
import os.path
from PID import PID
from IPython.display import clear_output
from nidaqmx.constants import CurrentShuntResistorLocation


CURRENT_PATH = os.getcwd()

CONFIG_FOLDER_PATH = os.path.abspath(os.path.join(CURRENT_PATH, "config"))
CONFIG_TEST_FILE = os.path.abspath(os.path.joi n(CURRENT_PATH, "config.txt"))
PID_CONFIG_PATH = os.path.abspath(os.path.join(CONFIG_FOLDER_PATH, "excavator_pid.yaml"))
DATA_AI0_LIST = []
DATA_AI1_LIST = []
DATA_AI2_LIST = []


PID_CONFIG = {'Joint_1': {'p': 11, 'i': 0.05, 'd': 25},
              'Joint_2': {'p': 35, 'i': 0.001, 'd': 30},
              'Joint_3': {'p': 45, 'i': 0.001, 'd': 25}}


def get_last_average_values(name_0, name_1, name_2):
    global DATA_AI0_LIST
    global DATA_AI1_LIST
    global DATA_AI2_LIST
    if len(DATA_AI0_LIST) != len(DATA_AI1_LIST) or len(DATA_AI0_LIST) != len(DATA_AI2_LIST):
        print("We got different numbers of data from the sensors, and they are:")
        print(f"{DATA_AI0_LIST}, {DATA_AI1_LIST}, {DATA_AI2_LIST}")
        print("Clearing all the datas from the lists......")
        DATA_AI0_LIST = []
        DATA_AI1_LIST = []
        DATA_AI2_LIST = []
        print("Cleared the lists......")
    last_sensor_data_0 = read_sensor(name_0)
    last_sensor_data_1 = read_sensor(name_1)
    last_sensor_data_2 = read_sensor(name_2)
    if not last_sensor_data_0 or not last_sensor_data_1 or not last_sensor_data_2:
        return None
    if len(DATA_AI0_LIST) == 17:
        DATA_AI0_LIST.pop(0)
        DATA_AI1_LIST.pop(0)
        DATA_AI2_LIST.pop(0)
    DATA_AI0_LIST.append(last_sensor_data_0)
    DATA_AI1_LIST.append(last_sensor_data_1)
    DATA_AI2_LIST.append(last_sensor_data_2)
    sensor_data_average_0 = sum(DATA_AI0_LIST) / len(DATA_AI0_LIST)
    sensor_data_average_1 = sum(DATA_AI1_LIST) / len(DATA_AI1_LIST)
    sensor_data_average_2 = sum(DATA_AI2_LIST) / len(DATA_AI2_LIST)
    sensor_data_average = [sensor_data_average_0, sensor_data_average_1, sensor_data_average_2]
    return sensor_data_average


def read_sensor_test(file_path):
    feedback_values = []
    if not os.path.isfile(file_path):
        return None
    with open(file_path, 'r') as f:
        datas = f.readline().split(',')
        if datas:
            for data in datas:
                if data:
                    feedback_values.append(float(data))
                else:
                    return []
        return feedback_values


def read_sensor(port_name="Dev1/ai0"):
    with nidaqmx.Task() as task:
        clear_output(wait=True)
        task.ai_channels.add_ai_current_chan(port_name, min_val=-0.01, max_val=0.02,
                                             shunt_resistor_loc=CurrentShuntResistorLocation.EXTERNAL,
                                             ext_shunt_resistor_val=100.0)
        sensor_data = task.read()
        clear_output(wait=True)
    return sensor_data


def pid_control_excavator(tar_l_0=0.5, tar_l_1=0.3, tar_l_2=0.3):
    sample_time = 0.01
    configs = PID_CONFIG

    pid_1 = PID(configs["Joint_1"]["p"], configs["Joint_1"]["i"], configs["Joint_1"]["d"])
    pid_2 = PID(configs["Joint_2"]["p"], configs["Joint_2"]["i"], configs["Joint_2"]["d"])
    pid_3 = PID(configs["Joint_3"]["p"], configs["Joint_3"]["i"], configs["Joint_3"]["d"])

    pid_1.SetPoint = tar_l_0
    pid_2.SetPoint = tar_l_1
    pid_3.SetPoint = tar_l_2

    pid_1.setSampleTime(sample_time)
    pid_2.setSampleTime(sample_time)
    pid_3.setSampleTime(sample_time)
    return pid_1, pid_2, pid_3


def update_values(feedback_values, pid_1, pid_2, pid_3):
    output = list()
    output.append(pid_1.update(feedback_values[0]))
    output.append(pid_2.update(feedback_values[1]))
    output.append(pid_3.update(feedback_values[2]))
    return output


def main(target_l=[0.1, 0.1, 0.1], i=0):

    global pid_base, pid_upper, pid_bucket, u
    sensor_name_0 = "Dev1/ai0"
    sensor_name_1 = "Dev1/ai39"
    sensor_name_2 = "Dev1/ai2"
    if i == 0:
        pid_base, pid_upper, pid_bucket = pid_control_excavator(target_l[0], target_l[1], target_l[2])
    print(i)
    sensor_data_current = get_last_average_values(sensor_name_0, sensor_name_1, sensor_name_2)
    error_list = [target_l[0] - sensor_data_current[0],
                  target_l[1] - sensor_data_current[1],
                  target_l[2] - sensor_data_current[2]]
    if error_list[0] <= 0.0001 and error_list[1] <= 0.0001 and error_list[2] <= 0.0001:
        print('success')
        return [70, 30, 25]
    if not sensor_data_current:
        print(f"There is lack of feedback values from the sensors, please check the sensor")
        return [70, 30, 25]
    print("*******************************************************************************")
    print(f"The current mA of the cylinders are:\n {sensor_data_current}")
    target = update_values(sensor_data_current, pid_base, pid_upper, pid_bucket)
    deviation_motor = [target[i] - sensor_data_current[i] for i in range(len(target))]
    print(f"The target value we are going to send to the excavator are:\n {deviation_motor}")
    print("*******************************************************************************")

    multiplied_list = [(deviation_motor[0]*-5000),(deviation_motor[1]*-5000),(deviation_motor[2]*-3000)]
    # values for testing - slow movement
    # multiplied_list = [(deviation_motor[0] * 10), (deviation_motor[1] * 10), (deviation_motor[2] * 10)]
    return multiplied_list


"""
######################################Labview display functions below############################
######################################do not run along with other functions######################







def deviation_display(target_l=[0.1, 0.1, 0.1], i=0):
    #i=i-1
    global pid_base, pid_upper, pid_bucket, u
    sensor_name_0 = "Dev1/ai0"
    sensor_name_1 = "Dev1/ai39"
    sensor_name_2 = "Dev1/ai2"
    # mylist = target_l
    if i==0:
        #return [-2,-2,-1]
        # return mylist
        pid_base, pid_upper, pid_bucket = pid_control_excavator(target_l[0], target_l[1], target_l[2])
    #while 1:
    print(i)
    sensor_data_current = get_last_average_values(sensor_name_0, sensor_name_1, sensor_name_2)
    # error_list = [target_l[0] - sensor_data_current[0],
    #               target_l[1] - sensor_data_current[1],
    #               target_l[2] - sensor_data_current[2]]
    # if error_list[0] <= 0.0001 and error_list[1] <= 0.001 and error_list[2] <= 0.001:
    #     print('success')
    #     return [1000, 1000, 1000]

    if not sensor_data_current:
        print(f"There is lack of feedback values from the sensors, please check the sensor")
        return [50, 30, 25]
        #continue
    print("*******************************************************************************")
    print(f"The current mA of the cylinders are:\n {sensor_data_current}")
    target = update_values(sensor_data_current, pid_base, pid_upper, pid_bucket)
    deviation_motor = [target[i] - sensor_data_current[i] for i in range(len(target))]
    print(f"The target value we are going to send to the excavator are:\n {deviation_motor}")
    print("*******************************************************************************")

    return deviation_motor
"""


def read_sensor_1():
    with nidaqmx.Task() as task:
        clear_output(wait=True)
        task.ai_channels.add_ai_current_chan("Dev1/ai0", min_val=-0.01, max_val=0.02,
                                             shunt_resistor_loc=CurrentShuntResistorLocation.EXTERNAL,
                                             ext_shunt_resistor_val=100.0)
        task.ai_channels.add_ai_current_chan("Dev1/ai39", min_val=-0.01, max_val=0.02,
                                             shunt_resistor_loc=CurrentShuntResistorLocation.EXTERNAL,
                                             ext_shunt_resistor_val=100.0)
        task.ai_channels.add_ai_current_chan("Dev1/ai2", min_val=-0.01, max_val=0.02,
                                             shunt_resistor_loc=CurrentShuntResistorLocation.EXTERNAL,
                                             ext_shunt_resistor_val=100.0)
        sensor_data = task.read()
        clear_output(wait=True)
    return sensor_data


if __name__ == '__main__':
    while 1:
        main(0)
