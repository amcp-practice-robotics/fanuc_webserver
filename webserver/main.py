import json
from socket import socket
from time import sleep
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import socket
import threading
from robot import Robot, init_func
import numpy as np
    
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

HOST = '127.0.0.1'
PORT = 10004
coords = ''
coordsSend = '' 
isConnected = False

@app.route('/')
def index():
    return render_template('index.html', enumerate=enumerate)

@socketio.on('connect')
def user_connected():
    pass

@socketio.event
def getcoords(message):
    emit('getcoords', coords)
    emit('status', json.dumps({"status": isConnected}))

@socketio.event
def sendcoords(coords_dct):
    global coordsSend
    with app.app_context():
        coordsSend = coords_dct

def runSocket():
    global coords, coordsSend, isConnected
    tmp = coordsSend
    with app.app_context():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            while True:
                conn, addr = s.accept()
                with conn:
                    isConnected = True
                    while tmp == coordsSend:
                        sleep(0.5)
                    print(coordsSend)
                    conn.sendall(coordsSend.encode())
                    data = conn.recv(1024).decode().strip()
                    if data == 'start':
                        while data != 'end':
                            data = conn.recv(1024).decode().strip()
                            if data == 'end':
                                print('end receive package')
                                break
                            coords = data
                    tmp = coordsSend
                    isConnected = False
                    conn.close()

if __name__ == '__main__':
    server = threading.Thread(target=runSocket)
    server.start()
    socketio.run(app)