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
import json
import os
print(f"Directorio de trabajo actual: {os.getcwd()}")

#CONSTANTES DE INTERES
proporcion_x = 1.16
proporcion_y = 0.5
altura_camara = 3 #metros este parametro hay que verificar al momento de implementar
W_m = 2 * altura_camara * proporcion_x #metros
H_m = 2 * altura_camara * proporcion_y #metros
lado_robot = 0.22 #metros
W = 1280 #Corresponde a X cm
H = 720 #Corresponde a Y cm
num_div_x = int(W_m/lado_robot) # este numero debe ser igual a la proporcion entre la altura del frame y la altura del robot
num_div_y = int(H_m/lado_robot) # este numero debe ser igual a la proporcion entre la anchura del frame y la anchura del robot
nw = int(W/num_div_x)
nh = int(H/num_div_y)

#MASCARA PARA DETECCION DE CUERPOS EN MOVIMINETO
detection = cv2.createBackgroundSubtractorMOG2(history=10000,varThreshold=12)

#limites de la meta
lower_green = np.array([45, 200, 200],np.uint8)# meta => verde
upper_green = np.array([70, 255, 255],np.uint8)
#Limites de la Caja
lower_yellow = np.array([27, 200, 200],np.uint8)#Caja => yellow
upper_yellow = np.array([33, 255, 255],np.uint8)
#Limite del robot
blueBajo = np.array([105,150,200],np.uint8)
blueAlto = np.array([130,255,255],np.uint8)
#Limite de los obstaculos
obstBajo = np.array([165,27,150],np.uint8) # obstaculos => rojo
obstAlto = np.array([180,255,255],np.uint8)

#Enlace al servidor RTSP de la marca Dahua
PASS = "pass1234"#input("Ingrese la Contraseña Administador del dispositivo: ")
IP = "192.168.1.64"#input("Ingrese la direccion IP: ")
CH = "1"#input("Ingrese el numero del canal: ")
#URL = "rtsp://admin:"+PASS+"@"+IP+":554/cam/realmonitor?channel="+CH+"&subtype=0"
URL = "rtsp://admin:"+PASS+"@"+IP+":554/Streaming/Channels/"+CH+"01"
URL_Cam_IP = "rtsp://"+IP+":554/Streaming/Channels/"+CH+"01"
img_pueba = "./obstaculos.png"

def ajustar_brillo(img, brillo_rojo=0, brillo_verde=0, brillo_azul = 0, constraste=0):
    """
    Ajusta el brillo de la imagen.
    
    Parámetros:
    - img: La imagen original (en formato BGR).
    - brillo: El nivel de ajuste de brillo, un valor entre -100 a 100.
    
    Retorna:
    - La imagen con el brillo ajustado.
    """
    b, g, r = cv2.split(img)
    #cv2.imshow("b",b)
    b_brillo= b + brillo_azul
    g_brillo= g + brillo_verde
    r_brillo= r + brillo_rojo
    r_brillo= np.clip(r_brillo, 0, 255)
    img = cv2.merge((b_brillo.astype(np.uint8), g_brillo.astype(np.uint8), r_brillo.astype(np.uint8)))
    img = img * constraste
    img = np.clip(img, 0, 255)
    return img.astype(np.uint8)
# Ajuste del brillo inicial (modifica este valor según sea necesario)
nivel_rojo = 0
nivel_verde = 5
nivel_azul = 0
nivel_constraste = 1.1

#try:
CAMERA_MATRIX = np.load("camera_matrix.npy")
DIST_COEFFS = np.load("dist_coeffs.npy")
print("Parámetros cargados exitosamente.")
#except FileNotFoundError as e:
    #print("Error: No se encontraron los archivos de calibración.")
    #print("Por favor, realiza la calibración para generar los archivos necesarios.")
    #exit(1)
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        CAMERA_MATRIX, DIST_COEFFS, (250, 250), 1, (250, 250)
    )

class FrameAnalysisService(Node):
    def __init__(self):
        super().__init__('frame_analysis_service')
        self.srv = self.create_service(Trigger, 'analyze_frame', self.analize_frame_callback)
        self.capture = cv2.VideoCapture(URL)#descomentar
        self.get_logger().info('Frame Analysis Service is ready')

    def analize_frame_callback(self, request, response):
        ret, frame = self.capture.read()#descomentar
        if not ret:
            self.capture.open(URL)  # Reconecta si no se pudo obtener el frame
            time.sleep(1)  # Pausa breve para reconexión
            ret, frame = self.capture.read()

        if ret:
            frame = cv2.undistort(frame,CAMERA_MATRIX,DIST_COEFFS)
            #frame = cv2.blur(frame, (5, 5)) #descomentar
            # Factor de escala
            """ recorte_izquierda = 250  # Número de píxeles a eliminar del lado izquierdo
            recorte_derecha = 250    # Número de píxeles a eliminar del lado derecho
    
            # Obtener el ancho original de la imagen
            alto, ancho, _ = frame.shape
    
            # Recortar la imagen en los laterales
            frame = frame[:, recorte_izquierda:(ancho - recorte_derecha)]
         """
            scale_factor = 0.9  # Redimensiona al 50% del tamaño original

            # Obtener el nuevo tamaño
            new_width = int(frame.shape[1]* scale_factor)
            new_height = int(frame.shape[0] * scale_factor)

            # Redimensionar la imagen
            frame = cv2.resize(frame, (new_width, new_height))

            #frame = ajustar_brillo(frame,nivel_rojo,nivel_verde,nivel_azul,nivel_constraste)
            # Procesamiento del frame (similar a tu código original)
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            maskGreen = cv2.inRange(frame_hsv,lower_green,upper_green)
            maskBlue = cv2.inRange(frame_hsv,blueBajo,blueAlto)
            maskObst = cv2.inRange(frame_hsv,obstBajo,obstAlto)
            maskCaja = cv2.inRange(frame_hsv,lower_yellow,upper_yellow)
            # Procesa el frame y añade el análisis adicional que necesites...
            #Obtener contorno del robot
            robot = cv2.Canny(maskBlue, 10, 150)
            robot = cv2.dilate(robot, None, iterations=1)
            robot = cv2.erode(robot, None, iterations=1)

            #Obtener contorno de la meta
            meta = cv2.Canny(maskGreen, 10, 150)
            meta = cv2.dilate(meta, None, iterations=1)
            meta = cv2.erode(meta, None, iterations=1)

            #Obtener contorno de la caja
            caja = cv2.Canny(maskCaja, 10, 150)
            caja = cv2.dilate(caja, None, iterations=1)
            caja = cv2.erode(caja, None, iterations=1)

            ##Obtener contorno de los obstaculos
            obstaculos = cv2.Canny(maskObst, 10, 150)
            obstaculos = cv2.dilate(obstaculos, None, iterations=1)
            obstaculos = cv2.erode(obstaculos, None, iterations=1)

            #Obtener coordenadas y dimencion del robot
            robot_coord = cv2.boundingRect(robot)
            #Obtener coordenadas y dimencion de la meta
            meta_coord = cv2.boundingRect(meta)
            #Obtener coordenadas y dimencion de la caja
            caja_coord = cv2.boundingRect(caja)
            meta_mean_point = (meta_coord[0] + int(meta_coord[2]/2) , meta_coord[1] + int(meta_coord[3]/2))

            #Obtener contornos de los ostaculos separados
            f,_ = cv2.findContours(obstaculos, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #elementos_obstaculos = [] #lista que contendra las coordenadas y dimenciones de los obstaculos
            elementos_obstaculos = np.empty((0, 4), int) #lista que contendra las coordenadas y dimenciones de los obstaculos

            for k in range(0,len(f)):
                x, y, w, h = cv2.boundingRect(f[k])
                area = w * h
                if area >= nw*nh/5:
                    elementos_obstaculos = np.vstack([elementos_obstaculos, np.array([(x, y, w, h)])])
        

            """ for k in range(2,len(f)):
                elementos_obstaculos.append(cv2.boundingRect(f[k]))
 """
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
            #Caja
            cv2.line(frame,(caja_coord[0],caja_coord[1]),(caja_coord[0]+caja_coord[2],caja_coord[1]),(0, 255, 255), 4)
            cv2.line(frame,(caja_coord[0],caja_coord[1]),(caja_coord[0],caja_coord[1]+caja_coord[3]),(0, 255, 255), 4)
            cv2.line(frame,(caja_coord[0],caja_coord[1]+caja_coord[3]),(caja_coord[0]+caja_coord[2],caja_coord[1]+caja_coord[3]),(0, 255, 255), 4)
            cv2.line(frame,(caja_coord[0]+caja_coord[2],caja_coord[1]),(caja_coord[0]+caja_coord[2],caja_coord[1]+caja_coord[3]),(0, 255, 255), 4)

            for k in range(num_div_x):
                cv2.line(frame,(nw*(k+1),0),(nw*(k+1),H),(0, 0, 0), 1)
            for k in range(num_div_y):
                cv2.line(frame,(0,nh*(k+1)),(W,nh*(k+1)),(0, 0, 0), 1)
            
            g1 = Grafo(meta_mean_point) # Se crea el grafo de nodos disponibles para transitar
            #Se crean los nodos y se agregan al grafo
            robot_cero_node = Nodo(robot_coord[0],robot_coord[1])
            meta_node = Nodo(meta_mean_point[0],meta_mean_point[1])
            for k in range(num_div_x):
                for j in range(num_div_y):
                    g1.agregar_vertice(Nodo(nw*(k+1),nh*(j+1)),elementos_obstaculos)
            #verificamos los vecinos para agregar las aristas al grafo
            for k in g1.vertices:
                try_nodo = g1.buscar_vertice(k.clave[0]+nw,k.clave[1])
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0],k.clave[1]+nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0]+nw,k.clave[1]+nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0]+nw,k.clave[1]-nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0]-nw,k.clave[1])
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0],k.clave[1]-nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)

                try_nodo = g1.buscar_vertice(k.clave[0]-nw,k.clave[1]+nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)
                
                try_nodo = g1.buscar_vertice(k.clave[0]-nw,k.clave[1]-nh)
                if  try_nodo != False:
                    g1.agregar_aristas(k,try_nodo)
 
            near_nodo_robot = g1.buscar_nodo_cercano(robot_coord)
            if near_nodo_robot == False:
                print("no se encontro nodo cercano")
                response.success = False
                response.message = json.dumps({"error": "No se encontró trayectoria válida"})
                while True:
                    cv2.imshow("Objetos Encontrados",frame)
                    t = cv2.waitKey(1)
                    if t & 0xFF == ord('t'):
                        break
                self.capture.release()
                cv2.destroyAllWindows()
                return response
            else:
                frame = g1.mostrarGrafos(frame)
                tray_nodos = g1.menor_trayecto(near_nodo_robot,robot_cero_node,meta_node,frame)
                if tray_nodos == False:
                    print("no se encontro trayectoria valida")
                    response.success = False
                    response.message = json.dumps({"error": "No se encontró trayectoria válida"})
                    self.get_logger().info("No se genero la trayectoria")
                    while True:
                        cv2.imshow("Objetos Encontrados",frame)
                        t = cv2.waitKey(1)
                        if t & 0xFF == ord('t'):
                            break
                    self.capture.release()
                    cv2.destroyAllWindows()
                    return response
                else:
                    tray = []
                    for k in tray_nodos:
                        aux = [0,0]
                        aux[0] = k.clave[0]*lado_robot/nw
                        aux[1] = k.clave[1]*lado_robot/nh    
                        tray.append(aux) 
                    #tray = [(640, 400), (720, 400), (800, 400), (800, 480)]  # Ejemplo real
                    self.get_logger().info("Se genero la trayectoria")
                    while True:
                        cv2.imshow("Objetos Encontrados",frame)
                        t = cv2.waitKey(1)
                        if t & 0xFF == ord('t'):
                            break
                    self.capture.release()
                    cv2.destroyAllWindows()
                    response.success = True
                    response.message = json.dumps({"tray": tray})
            """ while True:
                cv2.imshow("Objetos Encontrados",frame)
                t = cv2.waitKey(1)
                if t & 0xFF == ord('t'):
                    break
            self.capture.release()
            cv2.destroyAllWindows() """

            # Devuelve el frame procesado (por ejemplo, como un array de bytes o lo que necesites)
            # Aquí puedes establecer un mensaje en el response, como respuesta al cliente
            response.success = True
            #response.message = 'Frame analizado con exito'
        else:
            self.get_logger().error("No se pudo capturar el frame.")
            response.success = False
        #self.get_logger().info("Ya se va a enviar...")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = FrameAnalysisService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
