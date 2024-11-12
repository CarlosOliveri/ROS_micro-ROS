#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os 
from matplotlib import pyplot as plt
import math
import time
from std_msgs.msg import Int32  # Cambiamos de Float32 a Int32
from std_msgs.msg import Int32MultiArray
from .grafo import Grafo, Nodo
from std_srvs.srv import Trigger  # Servicio de ejemplo que no requiere parámetros

#CONSTANTES DE INTERES
proporcion_x = 0.7535 
proporcion_y = 0.4557
altura_camara = 3 #metros este parametro hay que verificar al momento de implementar
W_m = 2 * altura_camara * proporcion_x #metros
H_m = 2 * altura_camara * proporcion_y #metros
lado_robot = 0.26 #metros
W = 1280 #Corresponde a X cm
H = 720 #Corresponde a Y cm
num_div_x = int(W_m/0.26) # este numero debe ser igual a la proporcion entre la altura del frame y la altura del robot
num_div_y = int(H_m/0.26) # este numero debe ser igual a la proporcion entre la anchura del frame y la anchura del robot
nw = int(W/num_div_x)
nh = int(H/num_div_y)

#MASCARA PARA DETECCION DE CUERPOS EN MOVIMINETO
detection = cv2.createBackgroundSubtractorMOG2(history=10000,varThreshold=12)

#limites de la meta
redBajo1 = np.array([0, 200, 50], np.uint8)
redAlto1 = np.array([10, 255, 255], np.uint8)
redBajo2=np.array([177, 200, 50], np.uint8)
redAlto2=np.array([180, 255, 255], np.uint8)
#Limite del robot
blueBajo = np.array([100,250,50],np.uint8)
blueAlto = np.array([140,255,100],np.uint8)
#Limite de los obstaculos
obstBajo = np.array([0,0,0],np.uint8)
obstAlto = np.array([180,100,50],np.uint8)

#Enlace al servidor RTSP de la marca Dahua
PASS = "12345"#input("Ingrese la Contraseña Administador del dispositivo: ")
IP = "192.168.100.64"#input("Ingrese la direccion IP: ")
CH = "1"#input("Ingrese el numero del canal: ")
#URL = "rtsp://admin:"+PASS+"@"+IP+":554/cam/realmonitor?channel="+CH+"&subtype=0"
URL = "rtsp://admin:"+PASS+"@"+IP+":554/Streaming/Channels/"+CH+"01"
URL_Cam_IP = "rtsp://"+IP+":554/Streaming/Channels/"+CH+"01"
img_pueba = "./obstaculos.png"
class FrameAnalysisService(Node):
    def __init__(self):
        super().__init__('frame_analysis_service')
        self.srv = self.create_service(Trigger, 'analyze_frame', self.analize_frame_callback)
        #self.capture = cv2.VideoCapture(0)
        self.capture = cv2.imread("img_prueba",cv2.IMREAD_COLOR)

        self.get_logger().info('Frame Analysis Service is ready')

    def analize_frame_callback(self, request, response):
        ret, frame = self.capture.read()
        
        if not ret:
            self.capture.open(0)  # Reconecta si no se pudo obtener el frame
            time.sleep(1)  # Pausa breve para reconexión
            ret, frame = self.capture.read()

        if ret:
            # Procesamiento del frame (similar a tu código original)
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            maskRed1 = cv2.inRange(frame_hsv,redBajo1,redAlto1)
            maskRed2 = cv2.inRange(frame_hsv, redBajo2,redAlto2)
            maskBlue = cv2.inRange(frame_hsv,blueBajo,blueAlto)
            maskObst = cv2.inRange(frame_hsv,obstBajo,obstAlto)
            # Procesa el frame y añade el análisis adicional que necesites...
            #Obtener contorno del robot
            robot = cv2.Canny(maskBlue, 10, 150)
            robot = cv2.dilate(robot, None, iterations=1)
            robot = cv2.erode(robot, None, iterations=1)

            #Obtener contorno de la meta
            meta = cv2.Canny(maskRed2, 10, 150)
            meta = cv2.dilate(meta, None, iterations=1)
            meta = cv2.erode(meta, None, iterations=1)

            ##Obtener contorno de los obstaculos
            obstaculos = cv2.Canny(maskObst, 10, 150)
            obstaculos = cv2.dilate(obstaculos, None, iterations=1)
            obstaculos = cv2.erode(obstaculos, None, iterations=1)

            #Obtener coordenadas y dimencion del robot
            robot_coord = cv2.boundingRect(robot)
            #Obtener coordenadas y dimencion de la meta
            meta_coord = cv2.boundingRect(meta)
            meta_mean_point = (meta_coord[0] + int(meta_coord[2]/2) , meta_coord[1] + int(meta_coord[3]/2))

            #Obtener contornos de los ostaculos separados
            f,_ = cv2.findContours(obstaculos, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            elementos_obstaculos = [] #lista que contendra las coordenadas y dimenciones de los obstaculos
            for k in range(2,len(f)):
                elementos_obstaculos.append(cv2.boundingRect(f[k]))

            #Las siguientes lineas sirven para graficar el mapa
            #img_2 = 255*np.ones((570,1350,3),dtype=np.uint8)
            for j in range(len(elementos_obstaculos)):
                #Obstaculos
                cv2.line(frame,(elementos_obstaculos[j][0],elementos_obstaculos[j][1]),(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]),(30, 100, 0), 4)
                cv2.line(frame,(elementos_obstaculos[j][0],elementos_obstaculos[j][1]),(elementos_obstaculos[j][0],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(30, 100, 0), 4)
                cv2.line(frame,(elementos_obstaculos[j][0],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(30, 100, 0), 4)
                cv2.line(frame,(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]),(30, 100, 0), 4)
            #Meta
            cv2.line(frame,(meta_coord[0],meta_coord[1]),(meta_coord[0]+meta_coord[2],meta_coord[1]),(0, 0, 255), 4)
            cv2.line(frame,(meta_coord[0],meta_coord[1]),(meta_coord[0],meta_coord[1]+meta_coord[3]),(0, 0, 255), 4)
            cv2.line(frame,(meta_coord[0],meta_coord[1]+meta_coord[3]),(meta_coord[0]+meta_coord[2],meta_coord[1]+meta_coord[3]),(0, 0, 255), 4)
            cv2.line(frame,(meta_coord[0]+meta_coord[2],meta_coord[1]),(meta_coord[0]+meta_coord[2],meta_coord[1]+meta_coord[3]),(0, 0, 255), 4)
            #frame
            cv2.line(frame,(robot_coord[0],robot_coord[1]),(robot_coord[0]+robot_coord[2],robot_coord[1]),(255, 0, 0), 4)
            cv2.line(frame,(robot_coord[0],robot_coord[1]),(robot_coord[0],robot_coord[1]+robot_coord[3]),(255, 0, 0), 4)
            cv2.line(frame,(robot_coord[0],robot_coord[1]+robot_coord[3]),(robot_coord[0]+robot_coord[2],robot_coord[1]+robot_coord[3]),(255, 0, 0), 4)
            cv2.line(frame,(robot_coord[0]+robot_coord[2],robot_coord[1]),(robot_coord[0]+robot_coord[2],robot_coord[1]+robot_coord[3]),(255, 0, 0), 4)

            for k in range(num_div_x):
                cv2.line(frame,(nw*(k+1),0),(nw*(k+1),H),(0, 0, 0), 1)
            for k in range(num_div_y):
                cv2.line(frame,(0,nh*(k+1)),(W,nh*(k+1)),(0, 0, 0), 1)
            
            g1 = Grafo() # Se crea el grafo de nodos disponibles para transitar
            #Se crean los nodos y se agregan al grafo
            for k in range(num_div_x):
                for j in range(num_div_y):
                    g1.agregar_vertice(Nodo(nw*(k+1),nh*(j+1)))
            #verificamos los vecinos para agregar las aristas al grafo
            for k in g1.vertices:
                try_nodo = g1.buscar_vertice(k.clave[0]+nw,k.clave[1])
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0],k.clave[1]+nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0]-nw,k.clave[1])
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0],k.clave[1]-nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

            response.message = g1.menor_trayecto(g1.buscar_nodo_cercano(robot_coord))
            while True:
                cv2.imshow("Objetos Encontrados",frame)
                t = cv2.waitKey(1)
                if t & 0xFF == ord('t'):
                    break
            self.capture.release()
            cv2.destroyAllWindows()

            # Devuelve el frame procesado (por ejemplo, como un array de bytes o lo que necesites)
            # Aquí puedes establecer un mensaje en el response, como respuesta al cliente
            response.success = True
            #response.message = 'Frame analizado con exito'
        else:
            self.get_logger().error("No se pudo capturar el frame.")
            response.result = False
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FrameAnalysisService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
