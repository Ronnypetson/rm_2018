# encoding=utf8  
import sys  
reload(sys)  
sys.setdefaultencoding('utf8')

import vrep, time, math, localization, avoid_obstacles, goToGoal_fuzzy, goToGoalPID, wall_follow_fuzzy
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


def get_pos_atual():
	code, position = vrep.simxGetObjectPosition(clientID, handle_robo, -1, vrep.simx_opmode_oneshot_wait)
	
	while(code != vrep.simx_return_ok):
		code, position = vrep.simxGetObjectPosition(clientID, handle_robo, -1, vrep.simx_opmode_oneshot_wait)

	return position

def get_ang_atual():
	code, ang = vrep.simxGetObjectOrientation(clientID, handle_robo, -1, vrep.simx_opmode_oneshot_wait)
	while(code != vrep.simx_return_ok):
		code, ang = vrep.simxGetObjectOrientation(clientID, handle_robo, -1, vrep.simx_opmode_oneshot_wait)
		
	return ang[2]

def ciclo_trig(ang):
	return (ang+2*math.pi)%(2*math.pi)
	
def get_dist(a,b):
	return np.linalg.norm(a-b) # np.array

def angle(v1,v2):
	
	v1_u = unit_vector(v1)
	v2_u = unit_vector(v2)
	
	theta = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
	rot_theta = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
	theta = theta*180.0/math.pi
	theta = (360.0+theta)%360.0
	
	if abs(np.dot(np.dot(rot_theta,v2_u),v1_u)-1.0) > 1e-2:
		theta = 360.0 - theta

	return theta
	
def unit_vector(vector):
	return vector / np.linalg.norm(vector)


localizacao = localization.localizacao()
localization.iniciar(clientID)

avoid_obst = avoid_obstacles.avoid_obstacles()
avoid_obst.init_fuzzy()

wall_follow = wall_follow_fuzzy.wall_follow_fuzzy()
wall_follow.init_fuzzy()

behaviour = {
	'go_to_goal': 2,		#Saida de nivel mais alto
	'follow_wall': 1,		#Saida intermediaria
	'avoid_obstacles': 0	#Saida de nivel mais baixo
}

action=0
OBST_TRESHOLD = 0.3
WALL_TRESHOLD = 0.4

print('Go to goal com (0) PID ou (1) Fuzzy ?')
fuzzy=1
fuzzy = input()
if not fuzzy:
	go_to_goal = goToGoalPID.GoToGoalPID()


#------------------------------ Loop principal ----------------------------
while vrep.simxGetConnectionId(clientID) != -1:
	goal = np.array([2.32, -3.8])
	dist = ler_distancias(handle_sensores)
	if(dist and len(dist)==8):

		'''
			Coordenacao comeca aqui!
		'''
		action = behaviour['go_to_goal']
		
		#Se houver alguma distancia muito pequena a saida mais baixa e ativada
		if min(dist) < OBST_TRESHOLD:
			action = behaviour['avoid_obstacles']
		elif min(dist[0], dist[1], dist[6], dist[7]) < WALL_TRESHOLD:
			action = behaviour['follow_wall']		
				
		#print action

		#Aplicando velocidades nas rodas de acordo com o sinal de controle
		if(action == behaviour['avoid_obstacles']):
			vel = avoid_obst.get_vel(dist)
			
		elif(action == behaviour['follow_wall']):
			vel_left, vel_right = wall_follow.get_vel(dist[0], dist[7])
			vel = [vel_left, vel_right]
				
		elif(action == behaviour['go_to_goal']):
			current_position = np.array(get_pos_atual()[:-1])
			#x, y = localizacao.getPosicao()
			#current_position=[x,y]
			
			if(fuzzy):
				current_angle = ciclo_trig(get_ang_atual())
				#current_angle = localizacao.getOrientacao()
				distance = get_dist(current_position, goal)
				ang_dist = angle(goal - current_position, np.array([math.cos(current_angle),math.sin(current_angle)])) # graus
				#print(distance, ang_dist)
				vel = goToGoal_fuzzy.get_fuzzy_control(distance, ang_dist)
			else:
				current_angle = get_ang_atual()
				vel = go_to_goal.goToGoal(current_position[0], current_position[1], current_angle, goal[0], goal[1])
				#print vel
		
			
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vel[0], vrep.simx_opmode_streaming)
		vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vel[1], vrep.simx_opmode_streaming)		
		
		thetaDir = vrep.simxGetJointPosition(clientID, handle_motor_dir, vrep.simx_opmode_streaming)[1]
		thetaEsq = vrep.simxGetJointPosition(clientID, handle_motor_esq, vrep.simx_opmode_streaming)[1]
		localizacao.setAngulos(thetaDir, thetaEsq)
