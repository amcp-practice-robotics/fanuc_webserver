from robot import Robot, init_func
from operator import le
from pickle import TRUE
from time import sleep
import numpy as np
import socket
import json

def forward(joint_list):
    robot = init_func()
    our_xyzwpr = robot.forward(joint_list).tolist()
    return our_xyzwpr

def inverse(world_list):
    robot = init_func()
    our_joints = robot.inverse(world_list)[0]
    return our_joints

def jpos_to_str(jpos_list):
    out = ""
    for i in range(0, 6):
        out += f"{int(jpos_list[i] * 1000)}\n"
    out += "100\n" #базовая скорость
    return out

def client_handler(HOST_FANUC, PORT_FANUC, move_dct, in_sock, logs):
    if logs:
        print(f'Connecting to FANUC\nIP: {HOST_FANUC}\nPort: {PORT_FANUC}')
    #while True:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST_FANUC, PORT_FANUC))
    except:
        if logs:
            print('trying to connect...')
        sock.close()
        return -1

    if 'joints' in move_dct.keys():
        jpos_str = jpos_to_str(move_dct['joints'])
    else:
        joints_from_solve = inverse(move_dct['world'])
        jpos_str = jpos_to_str(joints_from_solve)
    
    print(jpos_str.encode())
    sended = sock.send(jpos_str.encode())
    
    if sended:
        if logs:
            print('Package sended successful')
    else:
        if logs:
            print('Error package sending')
        sock.close()
        return -1

    data = sock.recv(1024).decode().strip()

    if data == 'not_reach':
        if logs:
            print('Not reach')
        sock.close()
        return -1
    if data == 'start':
        if logs:
            print('Package receive started')
    else:
        if logs:
            print('Error package recieving')
        sock.close()
        return -1
    print(data)
        
    big_data = ''
    while True:
        data = sock.recv(1024)
        data = data.decode().strip().replace(' .', '0.').replace('-.','-0.')
        if data.endswith('end'):
            if logs:
                print('Package receive ended')
            sock.close()
            return 1
        big_data += data
        pos = big_data.find('}{')
        if pos != -1:
            try:
                global_out_data = json.loads(big_data[:pos] + '}')
            except:
                two_packs = big_data[:pos] + '}'
                pos = two_packs.rfind('}\n{')
                print('twopacks: ' + two_packs + 'end')
                print('twopacks1: ' + two_packs[pos + 1:] + 'end')
                #t = two_packs.split('\n')
                global_out_data = json.loads(two_packs[pos + 1:])
            global_out_data['our_cart'] = forward(global_out_data['joints'])
            global_out_data['error_cart'] = np.linalg.norm(np.array(global_out_data['cart'][:3]) - global_out_data['our_cart'][:3], 1)
            global_out_data['error_ang'] = np.linalg.norm(np.array(global_out_data['cart'][3:]) - global_out_data['our_cart'][3:], 1)
            print(global_out_data)
            in_sock.send(json.dumps(global_out_data).encode())
            sleep(0.05)
            big_data = big_data[pos+1:]

if __name__ == "__main__":
    start = 'start\n'
    end = 'end\n'
 
    HOST_FANUC = '192.168.31.150' 
    #HOST_FANUC = '127.0.0.1'
    PORT_FANUC = 10000

    HOST_SITE = '127.0.0.1'
    PORT_SITE = 10004

    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((HOST_SITE, PORT_SITE))
            in_data = sock.recv(1024)
            move_dct = json.loads(in_data.decode().strip())
            sock.send(start.encode())
            client_handler(HOST_FANUC, PORT_FANUC, move_dct, sock, logs=1)
            sock.send(end.encode())
            sock.close()