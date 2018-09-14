import vrep, time, cv2, math, localization
import numpy as np
import matplotlib.pyplot as plt

server_IP = "127.0.0.1"
server_port = 25000
nome_sensor = []
handle_sensores = []
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6]
braitenbergR=[-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2]
detect = [0,0,0,0,0,0,0,0]
noDetectionDist=5.0
maxDetectionDist=0.2

RAIO_ROBO = 0.455/2

sala_atual = 1

sala_1 = [[-6.0, 0.0], [-6.0, 1.5], [-4.5, 2.0], [-4.5, 0.0], [-0.5, 0.0], [-0.5, 1.5], [2.0, 2.0], [3.0, 2.0], [3.5, 0.8], [4.0, 0.0], [4.0, -1.5], [3.0, -1.5], [3.0, -2.8], [1.0, -2.8], [1.0, -4.0], [-1.5, -4.0], [-1.5, -3.5], [-1.0, -3.5], [-2.0, -1.5]]

#sala_1 = [[-5.9, 1.3], [-5.8, 3.0], [-4.4, 3.9], [-3.3, 4.2], [-2.1, 6.0],  [-2.2, 4.4], [-3.3, 4.2], [-4.4, 3.9], [-5.8, 4.5], [-5.9, 3.5], [-6.0,0.0]]

#sala_2 = [[-2.0,0.0], [0.0, 1.5], [1.0,1.5], [1.0, 3.8], [3.1,4.3], [4.5, 4.3], [4.5,6.2], [3.5,6.2], [3.5,4.3], [-0.5, 4.3], [1.1, 4.3], [1.1, 3.0], [1.1, 2.0]]

#sala_3 = [[1.5, 2.0], [3.0, 2.0], [3.5, 2.0], [3.5, 1.0], [4.0, -1.5], [2.0, -1.5], [3.0, -3.0], [3.3, -4.7], [-2.1, -4.7], [-1.0, -2.4], [-3.3, -2.4]]

#sala_4 = [[-3.3, -4.5], [-6.0, -4.5], [-6.3, -1.5]]

#sala_2 = [[-3.0, 0.0], [0.0, 0.0], [1.0, 1.5], [2.0, 2.0], [3.0, 1.0], [4.0, 2.0], [4.0, -1.0], [3.0, -2.0], [2.0, -2.5], [1.0, -3.0], [0.5, -3.5], [0.0, -2.0], [-3.0, -2.3]]

#sala_3 = [[-3.8, -4.2], [-6.0, -4.5], [-3.8, -4.2], [-3.0, -2.3],  [-2.0, -2.2], [1.3,1.7]]

#sala_4 = [[1.4, 3.8], [4.5, 5.4], [1.9, 4.0], [0.0, 3.5], [1.2, 4.0]]


ang_ultrassom = [90, 50, 30, 10, -10, -30, -50, -90]



#---------------------Conecta no servidor---------------------------------
clientID = vrep.simxStart(server_IP, server_port, True, True, 2000, 5)

if (clientID!=-1):
	print ("Servidor Conectado!")
	

#------------------------------Inicializa Sensores ----------------------------
	for i in range(0,8):
		nome_sensor.append("Pioneer_p3dx_ultrasonicSensor" + str(i+1))

		res, handle = vrep.simxGetObjectHandle(clientID, nome_sensor[i], vrep.simx_opmode_oneshot_wait)

		if(res != vrep.simx_return_ok):
			print (nome_sensor[i] + " nao conectado")
		else:
			print (nome_sensor[i] + " conectado")
			handle_sensores.append(handle)
			
	#Vision sensor		
	res, visionHandle = vrep.simxGetObjectHandle(clientID, "Vision_sensor", vrep.simx_opmode_oneshot_wait)		
			
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

	"""
		Esse metodo ira ler a distancia de um conjunto de sensores ultrassonicos
		parametro: handle dos sensores
		retorna:   distancias em metros
	"""
	distancias = []
	
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

def braitenberg(dist, vel):
	vLeft = vRight = vel
	for i in range(len(dist)):
		if(dist[i] < noDetectionDist):
			detect[i] = 1 - ((dist[i]-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
		else:
			detect[i]=0
		
		for i in range(8):
			vLeft = vLeft + braitenbergL[i]*detect[i]
			vRight = vRight+ braitenbergR[i]*detect[i]
	
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vLeft, vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vRight, vrep.simx_opmode_streaming)

def get_angulo_alvo(x_robo, y_robo, ang_robo, x_alvo, y_alvo):
	tolerancia = 0.2
	
	ang_alvo = math.atan2((y_alvo - y_robo),(x_alvo - x_robo))
	if (abs(x_robo - x_alvo) < tolerancia) and (abs(y_robo - y_alvo) < tolerancia):
		ang_alvo = 0
	
	return ang_alvo

def get_pos_atual():
	code, position = vrep.simxGetObjectPosition(clientID, handle_robo, -1, vrep.simx_opmode_streaming)
	
	while(code != vrep.simx_return_ok):
		code, position = vrep.simxGetObjectPosition(clientID, handle_robo, -1, vrep.simx_opmode_streaming)

	return position

def get_ang_atual():
	code, ang = vrep.simxGetObjectOrientation(clientID, handle_robo, -1, vrep.simx_opmode_streaming)
	while(code != vrep.simx_return_ok):
		code, ang = vrep.simxGetObjectOrientation(clientID, handle_robo, -1, vrep.simx_opmode_streaming)
		
	return ang[2]

def virar(angulo):
	print("Virando para o angulo ", angulo*180.0/math.pi, " graus")
	#ang_inicial = get_ang_atual()
	
	if(ciclo_trig(get_ang_atual()) > ciclo_trig(angulo)):
		if(ciclo_trig(get_ang_atual()) - ciclo_trig(angulo) < math.pi):
			#Virar direita
			vel_esq = 0.5
			vel_dir = -0.5
		else:
			#Virar esquerda
			vel_esq = -0.5
			vel_dir = 0.5
		
	else:
		if(ciclo_trig(angulo) - ciclo_trig(get_ang_atual()) < math.pi):
			#Virar esquerda
			vel_esq = -0.5
			vel_dir = 0.5
		else:
			#Virar direita
			vel_esq = 0.5
			vel_dir = -0.5

		
	while(abs(ciclo_trig(get_ang_atual()) - ciclo_trig(angulo)) > 0.01):
		#print abs(get_ang_atual() - angulo)
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel_dir, vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel_esq, vrep.simx_opmode_streaming)
		#Atualiza localizacao	
		thetaDir = vrep.simxGetJointPosition(clientID, handle_motor_dir, vrep.simx_opmode_streaming)[1]
		thetaEsq = vrep.simxGetJointPosition(clientID, handle_motor_esq, vrep.simx_opmode_streaming)[1]
		localizacao.setAngulos(thetaDir, thetaEsq)

def ciclo_trig(ang):
	return (ang+2*math.pi)%(2*math.pi)

def mover_para(x,y):
	print("Movendo para ", x, ",", y)
	print("Angulo atual: ", get_ang_atual()*180.0/math.pi)
	time.sleep(0.1)
	vel = 2
	ang_alvo = get_angulo_alvo(get_pos_atual()[0], get_pos_atual()[1], get_ang_atual(), x, y)
	virar(ang_alvo)
	
	chegou = False
	
	while(not chegou):

		dist = ler_distancias(handle_sensores)
		if(dist):
			x_odom, y_odom = localizacao.getPosicao()
			salva_dados(dist, get_pos_atual()[0], get_pos_atual()[1], x_odom, y_odom, get_ang_atual())
		
			if(dist[3] < 0.1):
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, -vel, vrep.simx_opmode_streaming)
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel, vrep.simx_opmode_streaming)
			elif(dist[4] < 0.1):
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel, vrep.simx_opmode_streaming)
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, -vel, vrep.simx_opmode_streaming)
			elif(dist[2] < 0.2):
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, -vel, vrep.simx_opmode_streaming)
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel, vrep.simx_opmode_streaming)
			elif(dist[5] < 0.2):
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel, vrep.simx_opmode_streaming)
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, -vel, vrep.simx_opmode_streaming)
			else:	
				ang_alvo = get_angulo_alvo(get_pos_atual()[0], get_pos_atual()[1], get_ang_atual(), x, y)
				if(abs(ciclo_trig(get_ang_atual()) - ciclo_trig(ang_alvo)) > 0.1):
					#print(get_ang_atual())
					if((ang_alvo > 0 and dist[7] > 1.0) or (ang_alvo < 0 and dist[0] > 1.0)):
						virar(ang_alvo)
															
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel, vrep.simx_opmode_streaming)
				vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel, vrep.simx_opmode_streaming)		
				#Atualiza localizacao	
				thetaDir = vrep.simxGetJointPosition(clientID, handle_motor_dir, vrep.simx_opmode_streaming)[1]
				thetaEsq = vrep.simxGetJointPosition(clientID, handle_motor_esq, vrep.simx_opmode_streaming)[1]
				localizacao.setAngulos(thetaDir, thetaEsq)
				
			if(abs(get_pos_atual()[0]-x) < 0.1 and abs(get_pos_atual()[1]-y) < 0.1):
				chegou = True
				print "chegou"
				
			#else:
			#	print str(get_pos_atual()[0]),",",str(get_pos_atual()[1])


# Python code to remove duplicate elements 
def remove(duplicate): 
    final_list = [] 
    for point in duplicate: 
        if point not in final_list: 
            final_list.append(point) 
    return final_list 

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
		
def plotar_mapa():
	plt.scatter(pontos_x, pontos_y, s=0.5)
	line1, = plt.plot(odometria_x, odometria_y, 'r--', label='Odometria')
	line2, = plt.plot(trajetoria_x, trajetoria_y, 'g', label='Ground-truth')
	plt.legend(handles=[line1, line2])
	plt.show()
			
display = False
localizacao = localization.localizacao()
localization.iniciar(clientID)
#------------------------------ Loop principal ----------------------------
while vrep.simxGetConnectionId(clientID) != -1:

	
	if(sala_atual == 1):
		for pos in sala_1:
			mover_para(pos[0], pos[1])
			"""
			elif(sala_atual == 2):
				for pos in sala_2:
					mover_para(pos[0], pos[1])
			elif(sala_atual == 3):
				for pos in sala_3:
					mover_para(pos[0], pos[1])
			elif(sala_atual == 4):
					for pos in sala_4:
						mover_para(pos[0], pos[1])
			"""		
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, 0, vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, 0, vrep.simx_opmode_streaming)	
		print "Fim"
		for p in pontos:
			pontos_x.append(p[0])
			pontos_y.append(p[1])
		plotar_mapa()			

		while(1):
			vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, 0, vrep.simx_opmode_streaming)
			vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, 0, vrep.simx_opmode_streaming)	
	
			
	sala_atual+=1

