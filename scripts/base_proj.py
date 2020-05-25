#! /usr/bin/env python
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Header
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan


import Videos
import cormodule
import visao_module
import cor_A4
import le_imu


color = "Azul"
objeto = "bird"


lineFrame = None
escapePoint = None
turnParameter = None
pointParameter = None
checker0 = None

Tcomplete = 0
Tcomplete2 = 0

mode = "Searching"

deploy = False
end = False

mean = []
center = []
biggestArea = None
creeperFrame = None

resetOdom = True
resetOdom2 = True 
timer = 0
timer2 = 0

target = 0
contadorContorno = 0
bridge = CvBridge()

saveTime = True
saveTime2 = True

foundPath = False
foundPath2 = False

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

dist = None

sleep = 0.1

detectedPoint = None

A = True
B = True

timeToGoHome = False
timeToGoHomeAgain = False

x = None
y = None
contador = 0
pula = 100
alfa = -1

vel = Twist(Vector3(0,0,0), Vector3(0,0,0))

saida_net = None

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()


def recebe(msg):
	global x # O global impede a recriacao de uma variavel local, para podermos usar o x global ja'  declarado
	global y
	global z
	global id
	for marker in msg.markers:
		id = marker.id
		marcador = "ar_marker_" + str(id)

		print(tf_buffer.can_transform(frame, marcador, rospy.Time(0)))
		header = Header(frame_id=marcador)
		# Procura a transformacao em sistema de coordenadas entre a base do robo e o marcador numero 100
		# Note que para seu projeto 1 voce nao vai precisar de nada que tem abaixo, a 
		# Nao ser que queira levar angulos em conta
		trans = tf_buffer.lookup_transform(frame, marcador, rospy.Time(0))
		
		# Separa as translacoes das rotacoes
		x = trans.transform.translation.x
		y = trans.transform.translation.y
		z = trans.transform.translation.z
		# ATENCAO: tudo o que vem a seguir e'  so para calcular um angulo
		# Para medirmos o angulo entre marcador e robo vamos projetar o eixo Z do marcador (perpendicular) 
		# no eixo X do robo (que e'  a direcao para a frente)
		t = transformations.translation_matrix([x, y, z])
		# Encontra as rotacoes e cria uma matriz de rotacao a partir dos quaternions
		r = transformations.quaternion_matrix([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
		m = numpy.dot(r,t) # Criamos a matriz composta por translacoes e rotacoes
		z_marker = [0,0,1,0] # Sao 4 coordenadas porque e'  um vetor em coordenadas homogeneas
		v2 = numpy.dot(m, z_marker)
		v2_n = v2[0:-1] # Descartamos a ultima posicao
		n2 = v2_n/linalg.norm(v2_n) # Normalizamos o vetor
		x_robo = [1,0,0]
		cosa = numpy.dot(n2, x_robo) # Projecao do vetor normal ao marcador no x do robo
		angulo_marcador_robo = math.degrees(math.acos(cosa))

		# Terminamos
		#print("id: {} x {} y {} z {} angulo {} ".format(id, x,y,z, angulo_marcador_robo))



# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):

    global cv_image
    global media
    global centro
    global resultados

    global lineFrame
    global escapePoint
    global turnParameter
    global pointParameter
    global checker0

    global mean
    global center
    global biggestArea
    global creeperFrame
    global contadorContorno
    global target

    global detected
    global detectedPoint
    global saida_net

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime # calcula o lag
    delay = lag.nsecs
    # print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        antes = time.clock()
        temp_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        centro, saida_net, resultados =  visao_module.processa(temp_image)
        mean, center, biggestArea, creeperFrame, contadorContorno, target =  cormodule.identifica_cor(temp_image, contadorContorno, target, color)        
        
        for r in resultados:
            detected = (r[0]) 
            detectedPoint = (r[3][0] - r[2][0], abs(r[2][1] - r[3][1]))            
            pass


        if detectedPoint != None:
            cv2.circle(saida_net, detectedPoint, 5,(0,0,255),3)

        depois = time.clock()
        # Desnecessário - Hough e MobileNet já abrem janelas
        cv_image = saida_net.copy()
    except CvBridgeError as e:
        print('ex', e)

    lineFrame, escapePoint, turnParameter, pointParameter, checker0 = Videos.main(imagem)

def scaneou(dado):
    global dist 
    dist = np.array(dado.ranges).round(decimals=2)[0]

def GoingToCreeper():
    global center
    global mean
    global dist 
    global mode
    global vel
    global timeToGoHome

    vel, mode = cor_A4.Go_to(mean, mode, center, dist)

    if mode == "In front of object":
        timeToGoHome = True

def GoingToObject():
    global center
    global detectedPoint
    global dist 
    global mode
    global vel
    global timeToGoHomeAgain

    vel, mode = cor_A4.Go_to(detectedPoint, mode, center, dist)

    if mode == "In front of object":
        timeToGoHomeAgain = True


def KeepInPath():

    global centro
    global escapePoint
    global turnParameter
    global pointParameter
    global checker0
    global vel

    vel = Twist(Vector3(0,0,0), Vector3(0,0,0)) #Marcos me explica pf - Documentacao

    
    if checker0 == "Max0":
        vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.15))

    elif checker0 == "Min0":
        vel = Twist(Vector3(0.15,0,0), Vector3(0,0, 0.15))
    
    else:
    
        if pointParameter == 'RA':
            vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.15))

        if pointParameter == 'LA':
            vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.15))

        if pointParameter == 'None':

            if turnParameter == "Different":

                if (escapePoint[0] > centro[0]):
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.1))      

                if (escapePoint[0] < centro[0]):
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.1))

                if (abs(escapePoint[0] - centro[0]) < 10):
                    vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))

            
            if turnParameter == "Negative":

                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.15))

            if turnParameter == "Positive":

                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.15))






    
if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/rgb/image_raw/compressed"

    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    #recebedor = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, recebe) # Para recebermos notificacoes de que marcadores foram vistos

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    tolerancia = 25

    # Exemplo de categoria de resultados
    # [('chair', 86.965459585189819, (90, 141), (177, 265))]

    try:
        
        while not rospy.is_shutdown():

            if timeToGoHome == True:
                mode = "Searching"
                if saveTime:
                    savedTime = timer
                    saveTime = False

                if Tcomplete:
                    if A:
                        timerA = time.time()
                        A = False

                    if time.time() - timerA > 5:
                        foundPath = True
                
                vel, timeToGoHome, Tcomplete, sleep = le_imu.go_home(savedTime, foundPath, escapePoint, turnParameter, centro)

                deploy = True
                target = 0
            
            elif deploy:
                if timeToGoHomeAgain == True:
                    if saveTime2:
                        savedTime2 = timer2
                        saveTime2 = False

                    if Tcomplete2:
                        if B:
                            timerB = time.time()
                            B = False

                        if time.time() - timerB > 5:
                            foundPath2 = True
                    
                    vel, timeToGoHomeAgain, Tcomplete2, sleep = le_imu.go_home(savedTime2, foundPath2, escapePoint, turnParameter, centro)
                    
                    end = True
                
                elif end:
                    KeepInPath()

                else:
                    if objeto == detected:
                        if resetOdom2:
                            timer2 = time.time()
                            resetOdom = False
                        GoingToObject()
                    
                    else:
                        KeepInPath()
            
            else: 
                if target:
                    if resetOdom:
                        timer = time.time()
                        resetOdom = False
                    GoingToCreeper()
                    
                else:
                    KeepInPath()

            #for r in resultados: #Rede neural
            #   print(r)

            if cv_image is not None:
                # Note que o imshow precisa ficar *ou* no codigo de tratamento de eventos *ou* no thread principal, não em ambos
                cv2.imshow("1", lineFrame)
                cv2.imshow("2", creeperFrame)
                cv2.imshow("3", saida_net)
                cv2.waitKey(1) #Botao que fecha a tela
   
            velocidade_saida.publish(vel) #envia a velocidade para o robo
            rospy.sleep(sleep) #intervalo entre processos

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")


