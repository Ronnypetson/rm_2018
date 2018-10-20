
import vrep, time, cv2
import numpy as np
server_IP = "127.0.0.1"
server_port = 25000
nome_sensor = []
handle_sensores = []
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6]
braitenbergR=[-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2]
detect = [0,0,0,0,0,0,0,0]
noDetectionDist=0.5
maxDetectionDist=0.2
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
        #Ponto muito distante, considerei 5 metros para nao pegar ruido
        distancias.append(0.5)
    else:
      #print ("Erro no sensor "+str(i+1))
      time.sleep(0.1)
  return distancias

display = False
pink_low = (140,100,0)
pink_high = (180,255,255)
green_low = (45,100,50)
green_high = (75,255,255)
min_radius = 10
#------------------------------ Loop principal ----------------------------
while vrep.simxGetConnectionId(clientID) != -1:
  vLeft = vRight = 5
  dist = ler_distancias(handle_sensores)

  #codigo pra ler da camera
  code, resolution, image = vrep.simxGetVisionSensorImage(clientID, visionHandle, 0, vrep.simx_opmode_streaming)
  center = None
  center_green = None
  radius = 0
  if code == vrep.simx_return_ok:
      """
      if not display:
        img = np.array(image,dtype=np.uint8)
        img.resize([resolution[1],resolution[0], 3])
        cv2.imshow('image',img)
				display=True
				if cv2.waitKey(1) & 0xFF == ord('q'):
          break
			"""
      img = np.array(image,dtype=np.uint8)
      img.resize([resolution[1],resolution[0], 3])
      # Filtro para remover ruido e inverter horizontalmente
      img = cv2.flip(img,0)
      img_filter = cv2.GaussianBlur(img.copy(), (3, 3), 0)
      # Converter de para HSV para detectar objetos cor-de-rosa
      img_hsv = cv2.cvtColor(img_filter, cv2.COLOR_RGB2HSV)
      #cv2.imshow('image',img)
      # Aplicar mascara da cor rosa
      img_binary = cv2.inRange(img_hsv, pink_low, pink_high)
      img_binary_green = cv2.inRange(img_hsv, green_low, green_high)
      # Obter os contornos
      contours = cv2.findContours(img_binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
      contours_green = cv2.findContours(img_binary_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
      # Encontrar o maior contorno
      if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
          center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
          if radius < min_radius:
            center = None
      if len(contours_green) > 0:
        c = max(contours_green, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
          center_green = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
          if radius < min_radius:
            center_green = None

      # Print out the location of the largest detected contour
      if center != None:
        print str(center) + ' ' +str(radius)

      # Draw a green circle around the largest enclosed contour
      if center != None:
        cv2.circle(img, center, int(round(radius)), (255,0,128))
      if center_green != None:
        cv2.circle(img, center_green, int(round(radius)), (0,255,0))

      cv2.imshow('image door',cv2.cvtColor(img,cv2.COLOR_RGB2BGR))

      display=True
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

  if dist:
    for i in range(len(dist)):
      if(dist[i] < noDetectionDist):
        detect[i] = 1 - ((dist[i]-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
      else:
        detect[i]=0

    for i in range(8):
      vLeft = vLeft + braitenbergL[i]*detect[i]
      vRight = vRight + braitenbergR[i]*detect[i]

  alpha = 0.8
  if center != None and center_green != None and radius < resolution[1]/1.85:
    door_center = alpha*np.array(center)+(1-alpha)*np.array(center_green)
    if door_center[0] < resolution[0]*1.0/3.0:
      vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, 2.8, vrep.simx_opmode_streaming)
      vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, 3.0, vrep.simx_opmode_streaming)
    else:
      vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, 3.0, vrep.simx_opmode_streaming)
      vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, 2.8, vrep.simx_opmode_streaming)
    #else:
    #  vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, vLeft, vrep.simx_opmode_streaming)
    #  vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, vRight, vrep.simx_opmode_streaming)
  else:
    vrep.simxSetJointTargetVelocity(clientID, handle_motor_esq, 0.60*vLeft, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, handle_motor_dir, 0.60*vRight, vrep.simx_opmode_streaming)

