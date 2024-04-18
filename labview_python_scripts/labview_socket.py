
#Getting the planned path list from ROS
def importing():
    import socket
    import pickle
    global my_list
    my_list = list()
    socketObject = socket.socket()


    """
    socketObject.connect(("192.168.0.51", 35496))
    HTTPMessage = "GET / HTTP/1.1\r\nHost: localhost\r\n Connection: close\r\n\r\n"
    bytes = str.encode(HTTPMessage)
    socketObject.sendall(bytes)
    """


    i=0
    while (True):
        recvd_data = socketObject.recv(1024)
        if recvd_data==b"end":
            print('end')
            break
        data = pickle.loads(recvd_data)
        print(f'This is the {data}')
        print(i)
        my_list.append(data)
        i=i+1
    return True




def return_list(i):
    global my_list
    if i >= len(my_list):
        return [0.0102057, 0.0080406450, 0.005785561]           #sensor values for the arm to come down after reached the goal position
        #return my_list[i-1]                                    #in case of multiple plannings
    return my_list[i]



def count_itr():
    length = len(my_list)
    return (length-1)


#defined main for testing purposes
if __name__ == '__main__':
    importing()
    return_list(0)
