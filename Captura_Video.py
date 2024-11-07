import cv2
import numpy as np
import os 
from matplotlib import pyplot as plt
import math
import time

#CONSTANTES DE INTERES
proporcion_x = 0.7535 
proporcion_y = 0.4557
altura_camara = 1.5 #metros este parametro hay que verificar al momento de implementar
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
redBajo2=np.array([175, 200, 50], np.uint8)
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
Capture = cv2.VideoCapture(0)

def distancia_entre_coord(a,b):
  distancia = math.sqrt((a[0]-b.clave[0])**2 + (a[1]-b.clave[1])**2)
  return distancia

class Nodo:
  def __init__(self,x,y):
    self.clave = (x,y)
    self.padre = None
    self.vecinos = []
    self.dist = 0
    self.color = "blanco"

class Grafo:
  def __init__(self):
    self.vertices = []

  def agregar_vertice(self,vertice):
    es_apto = True
    for k in elementos_obstaculos:
      if distancia_entre_coord(k,vertice) >= math.sqrt(nw**2 + nh**2):
        if vertice.clave[0] < (k[0] + k[2]) and (vertice.clave[0] + nw) > k[0]:
          if vertice.clave[1] < (k[1] + k[3]) and (vertice.clave[1] + nh) > k[1]:
            es_apto = False
      else:
        es_apto = False
    if es_apto == True:
      vertice.dist = math.sqrt((vertice.clave[0]-meta_mean_point[0])**2 + (vertice.clave[1]-meta_mean_point[1])**2)
      self.vertices.append(vertice)

  def agregar_aristas(self,a,b):
    a.vecinos.append(b)

  def buscar_vertice(self,x,y):
    for j in self.vertices:
      #print(x,y,j.clave[0],j.clave[1])
      if j.clave[0] == x and j.clave[1] == y:
        return j
    return False

  def mostrarGrafos(self):
    #img = 255*np.ones((H,W,3),dtype=np.uint8)
    for k in self.vertices:
      cv2.line(frame,(k.clave[0],k.clave[1]),(k.clave[0],k.clave[1]),(255, 0, 0), 5)
    while True:
        cv2.imshow(frame)
        t = cv2.waitKey(1)
        if t & 0xFF == ord('t'):
            break
    Capture.release()
    cv2.destroyAllWindows()

  def menor_trayecto(self,pos):
    actual_dist = pos.dist
    cont = 0
    min_tray = []
    while actual_dist > math.sqrt(nw**2+nh**2):
        min_dist_nodo = pos.vecinos[0]
        for k in pos.vecinos:
            if k.dist < min_dist_nodo.dist:
                min_dist_nodo = k
        min_tray.append(min_dist_nodo)
        pos = min_dist_nodo
        actual_dist = min_dist_nodo.dist
        #cv2.line(frame,(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(0, 0, 255), 5)
    while True:
        cv2.imshow(frame)
        t = cv2.waitKey(1)
        if t & 0xFF == ord('t'):
            break
    Capture.release()
    cv2.destroyAllWindows()
    return min_tray

  def buscar_nodo_cercano(self,pos):
    min_dist_x = 500
    min_dist_y = 500
    x = 0
    for k in range(num_div_x):
      if min_dist_x > abs(pos[0] - (nw*(k+1))):
        min_dist_x = pos[0] - (nw*(k+1))
        x = nw*(k+1)
        print(x)
    for k in range(num_div_y):
      if min_dist_y > abs(pos[1] - (nh*(k+1))):
        min_dist_y = pos[1] - (nh*(k+1))
        y = nh*(k+1)
        print(y)
    return self.buscar_vertice(x,y)


def Frame_callback():
#while True:
    try:
        #Recuperamos los frames del video 1 a 1 en cada iteracion
        ret,frame = Capture.read()
        if ret == False:
            print("no se encontro frame")
            return False#break
        #print(frame.shape)
        return frame#
    except:
        #Mostramos el Frame
        print("No se pudo encontrar el Frames")
        return False
    
frame = Frame_callback()
#print(frame)
while True:
    cv2.imshow("Imagen para generar trayectoria",frame)
    t = cv2.waitKey(1)
    if t & 0xFF == ord('t'):
        break
Capture.release()
cv2.destroyAllWindows()

frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#Resalta los objetos de los colores deseados
maskRed1 = cv2.inRange(frame_hsv,redBajo1,redAlto1)
maskRed2 = cv2.inRange(frame_hsv,redBajo2,redAlto2)
maskBlue = cv2.inRange(frame_hsv,blueBajo,blueAlto)
maskObst = cv2.inRange(frame_hsv,obstBajo,obstAlto)
        
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

while True:
    cv2.imshow("Objetos Encontrados",frame)
    t = cv2.waitKey(1)
    if t & 0xFF == ord('t'):
        break
Capture.release()
cv2.destroyAllWindows()