#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
import socket
import pickle
from datetime import datetime
from std_msgs.msg import Int32
import time

#Getting number of waypoints in the planned path
def callback_count(msg):
    print('Got number of list elements')
    global count
    count = msg.data

#Passing the list of waypoint to client(Labview) for execution
def callback_path(msg):
    print('Entered Callback')
    global serverSocket
    global clientConnection
    global i
    rospy.loginfo('Getting the joint values')
    joint_value = [msg.x, msg.y, msg.z]
    rospy.loginfo(f'Outputing the list of joint values {joint_value}')
    rospy.loginfo(f'i value is  {i}')
    rospy.loginfo(f'count value is  {count}')

    try:
        data = pickle.dumps(joint_value)
        clientConnection.send(data)
        i = i + 1

        if i == count:
            print('Sent all data!!!')
            time.sleep(0.3)
            msg1 = "end"
            msg1bytes = str.encode(msg1)
            print('Sending end message!')
            clientConnection.send(msg1bytes)
            print('Exiting from callback!')
            i=0
    except:
        print('Exception! Could not send any message! Try again !!!')



if __name__ == "__main__":
    i=0
    rospy.init_node("test")
    serverSocket = socket.socket()
    print("Server socket created")
    serverSocket.settimeout(10000)
    # Associate the server socket with the IP and Port
    ip = "192.168.0.51"
    port = 35496 #port_no
    serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    serverSocket.bind((ip, port))
    print("Server socket bound with with ip {} port {}".format(ip, port))
    serverSocket.listen()

    sub_2 = rospy.Subscriber('/planned_joint_values_count', Int32, callback_count)
    sub = rospy.Subscriber('/planned_joint_positions', Vector3, callback_path)

    while not rospy.is_shutdown():
        while True:
            (clientConnection, clientAddress) = serverSocket.accept()
            print("Connected to client")
