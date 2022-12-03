from robot import Robot, init_func
import json
import math
import numpy as np
import sqlalchemy as db

cr_tbl = """CREATE TABLE IF NOT EXISTS logs ( 
		            id INTEGER PRIMARY KEY AUTOINCREMENT,
	                j1 FLOAT, j2 FLOAT, j3 FLOAT, 
	                j4 FLOAT, j5 FLOAT, j6 FLOAT,
	                fanuc_x FLOAT, fanuc_y FLOAT, 
		            fanuc_z FLOAT, fanuc_w FLOAT, 
		            fanuc_p FLOAT, fanuc_r FLOAT,
			        x FLOAT, y FLOAT, z FLOAT, 
        			w FLOAT, p FLOAT, r FLOAT);"""

ins_logs = """INSERT INTO logs (j1, j2, j3, j4, j5, j6, 
                                fanuc_x, fanuc_y, fanuc_z, 
                                fanuc_w, fanuc_p, fanuc_r, 
                                x, y, z, w, p, r) VALUES 
                                (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)"""

engine = db.create_engine('sqlite:///../logs/log.db')
engine.connect()
engine.execute(cr_tbl)

robot = init_func()

with open('../logs/log.txt', 'r') as file:
    for i in file.readlines():
        i = i[:-1]
        logs = json.loads(i)
        for i in range(1, 7):
            logs['j' + str(i)] = math.degrees(logs['j' + str(i)])
        clog = [logs['j1'], logs['j2'], logs['j3'],
                logs['j4'], logs['j5'], logs['j6']]
        fanuc_xyzwpr = [logs['x'], logs['y'], logs['z'],
                        logs['w'], logs['p'], logs['r']]
        our_xyzwpr = robot.forward(clog).tolist()
        #print(clog, '\nf', fanuc_xyzwpr, '\no', our_xyzwpr)
        engine.execute(ins_logs, clog+fanuc_xyzwpr+our_xyzwpr)

#outs = engine.execute('SELECT * FROM "logs"')
#print(outs.fetchall())








