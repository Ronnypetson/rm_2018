# -*- coding: utf-8 -*-
import vrep, time, math,localization
import numpy as np
import matplotlib.pyplot as plt


server_IP = "127.0.0.1"
server_port = 25000
nome_sensor = []
handle_sensores = []

detect = np.zeros(16)
noDetectionDist = 4.0
#maxDetectionDist=0.1

RAIO_ROBO = 0.455/2
ang_ultrassom = [90, 50, 30, 10, -10, -30, -50, -90]

reference = 0.5
Kp = 10
min_speed = 1


#---------------------Conecta no servidor---------------------------------


clientID = vrep.simxStart(server_IP, server_port, True, True, 2000, 5)

if (clientID!=-1):
	print ("Servidor Conectado!")
	

#------------------------------Inicializa Sensores ----------------------------
	for i in range(0,16):
		nome_sensor.append("Pioneer_p3dx_ultrasonicSensor" + str(i+1))

		res, handle = vrep.simxGetObjectHandle(clientID, nome_sensor[i], vrep.simx_opmode_oneshot_wait)

		if(res != vrep.simx_return_ok):
			print (nome_sensor[i] + " nao conectado")
		else:
			print (nome_sensor[i] + " conectado")
			handle_sensores.append(handle)
			
		
#------------------------------Inicializa Motores ----------------------------
	resLeft, handle_motor_esq = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
	if(resLeft != vrep.simx_return_ok):
		print("Motor Esquerdo : Handle nao encontrado!")
	else:
		print("Motor Esquerdo: Conectado")

	resRight, handle_motor_dir = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)
	if(resRight != vrep.simx_return_ok):
		print("Motor Direito: Handle nao encontrado!")
	else:
		print("Motor Direito: Conectado")

#------------------------------Inicializa Robo ----------------------------

	resRobo, handle_robo = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_oneshot_wait)


else:
	print("Servidor desconectado!")


def ler_distancias(sensorHandle):
    distancias=[]
    for sensor in sensorHandle:
        returnCode, detectionState, detectedPoint,_,_ = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_streaming)
        if (returnCode == vrep.simx_return_ok):
            if(detectionState != 0):
                distancias.append(round(detectedPoint[2],5))
            else:
				#Muito distante
                distancias.append(noDetectionDist)
        else:
			#print ("Erro no sensor "+str(i+1))
            time.sleep(0.1)
    return distancias

def get_ang_atual():
	code, ang = vrep.simxGetObjectOrientation(clientID, handle_robo, -1, vrep.simx_opmode_streaming)
	while(code != vrep.simx_return_ok):
		code, ang = vrep.simxGetObjectOrientation(clientID, handle_robo, -1, vrep.simx_opmode_streaming)
		
	return ang[2]

def get_pos_atual():
	code, position = vrep.simxGetObjectPosition(clientID, handle_robo, -1, vrep.simx_opmode_streaming)
	
	while(code != vrep.simx_return_ok):
		code, position = vrep.simxGetObjectPosition(clientID, handle_robo, -1, vrep.simx_opmode_streaming)

	return position

def plotar_mapa():
	plt.scatter(pontos_x, pontos_y, s=0.5)
	line1, = plt.plot(odometria_x, odometria_y, 'r--', label='Odometria')
	line2, = plt.plot(trajetoria_x, trajetoria_y, 'g', label='Ground-truth')
	plt.legend(handles=[line1, line2])
	plt.show()

pontos_x = []
pontos_y = []
pontos = []
trajetoria_x = []
trajetoria_y = []
odometria_x = []
odometria_y = []

def salva_dados(dist, x_robo, y_robo, x_odom, y_odom, ang_robo):
	for i in range(len(dist)):
		if(dist[i] < noDetectionDist):
			x = x_robo + (dist[i] + RAIO_ROBO) * math.cos((ang_ultrassom[i]*math.pi/180) + ang_robo)
			y = y_robo + (dist[i] + RAIO_ROBO) * math.sin((ang_ultrassom[i]*math.pi/180) + ang_robo)
			if [x, y] not in pontos: 
				pontos.append([x, y])
	trajetoria_x.append(x_robo)
	trajetoria_y.append(y_robo)
	odometria_x.append(x_odom)
	odometria_y.append(y_odom)		


def left_side_obstacle():
    dist = ler_distancias(handle_sensores)
    if(dist):
        if(dist[0] < dist[7]):
            return True
        else:
            return False

def left_wall_follow():
    dist = ler_distancias(handle_sensores)
    if(dist):
        measured = min(dist[0],dist[1])
        error = reference - measured
        u = Kp * error
        vel = u
        vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, (min_speed - vel) , vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, (min_speed + vel), vrep.simx_opmode_streaming)
        
def right_wall_follow():
    dist = ler_distancias(handle_sensores)
    if(dist):
        measured = min(dist[6],dist[7])
        error = reference - measured
        u = Kp * error
        vel = u
        vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, (min_speed + vel) , vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, (min_speed - vel), vrep.simx_opmode_streaming)


localizacao = localization.localizacao()
localization.iniciar(clientID)
x_odom = []
y_odom = []

#------------------------------ Loop principal ----------------------------
while vrep.simxGetConnectionId(clientID) != -1:
    while(1):
        if(left_side_obstacle()):
        
            left_wall_follow()
        
        else:
        
            right_wall_follow()
        
           
            
#plt.plot(trajetoria_x, trajetoria_y, 'g', label='Ground-truth')
	
