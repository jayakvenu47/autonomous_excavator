import nidaqmx
from IPython.display import clear_output
from nidaqmx.constants import CurrentShuntResistorLocation
import socket
import pickle


DATA_AI0_LIST = []
DATA_AI1_LIST = []
DATA_AI2_LIST = []


def socket_conn():
    sensor_data_average=get_last_average_values()
    serverSocket = socket.socket()
    print("Server socket created")
    # Associate the server socket with the IP and Port
    ip = "192.168.0.50"
    port = 35491 #port_no
    serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    serverSocket.bind((ip, port))
    print("Server socket bound with with ip {} port {}".format(ip, port))
    serverSocket.listen()
    (clientConnection, clientAddress) = serverSocket.accept()
    print("Connected to client")
    data = pickle.dumps(sensor_data_average)
    clientConnection.send(data)

    try:
        serverSocket.shutdown()
        serverSocket.close()
        return 1
    except:
        return 2


#function to read sensor data - NIDAQMX
#external shunt=100 chosen to match sensor values with the previous project
def read_sensor(port_name="Dev1/ai0"):
    with nidaqmx.Task() as task:
        clear_output(wait=True)
        task.ai_channels.add_ai_current_chan(port_name, min_val=-0.01, max_val=0.02,
                                             shunt_resistor_loc=CurrentShuntResistorLocation.EXTERNAL,
                                             ext_shunt_resistor_val=100.0)
        sensor_data = task.read()
        clear_output(wait=True)
    return sensor_data

#computing moving_average
def get_last_average_values(name_0="Dev1/ai0", name_1="Dev1/ai39", name_2="Dev1/ai2"):
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

#length of list for calculating moving average=17
    while len(DATA_AI0_LIST) < 17:
        last_sensor_data_0 = read_sensor(name_0)
        last_sensor_data_1 = read_sensor(name_1)
        last_sensor_data_2 = read_sensor(name_2)
        if not last_sensor_data_0 or not last_sensor_data_1 or not last_sensor_data_2:
            return None
        DATA_AI0_LIST.append(last_sensor_data_0)
        DATA_AI1_LIST.append(last_sensor_data_1)
        DATA_AI2_LIST.append(last_sensor_data_2)
    sensor_data_average_0 = sum(DATA_AI0_LIST) / len(DATA_AI0_LIST)
    sensor_data_average_1 = sum(DATA_AI1_LIST) / len(DATA_AI1_LIST)
    sensor_data_average_2 = sum(DATA_AI2_LIST) / len(DATA_AI2_LIST)
    sensor_data_average = [sensor_data_average_0, sensor_data_average_1, sensor_data_average_2]
    return sensor_data_average


if __name__ == '__main__':
    socket_conn()