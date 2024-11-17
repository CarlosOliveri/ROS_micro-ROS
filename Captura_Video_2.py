import cv2
import numpy as np
import os 
from matplotlib import pyplot as plt
import math
import time

#CONSTANTES DE INTERES
proporcion_x = 0.7535 
proporcion_y = 0.4557
altura_camara = 0.27 #metros este parametro hay que verificar al momento de implementar
W_m = 2 * altura_camara * proporcion_x #metros
H_m = 2 * altura_camara * proporcion_y #metros
lado_robot = 0.025 #metros
W = 1280 #Corresponde a X cm
H = 720 #Corresponde a Y cm
num_div_x = int(W_m/lado_robot) # este numero debe ser igual a la proporcion entre la altura del frame y la altura del robot
num_div_y = int(H_m/lado_robot) # este numero debe ser igual a la proporcion entre la anchura del frame y la anchura del robot
nw = int(W/num_div_x) #pixeles => lado_robot [m]
nh = int(H/num_div_y) #pixeles => lado_robot [m]

last_time_render = 0

#MASCARA PARA DETECCION DE CUERPOS EN MOVIMINETO
detection = cv2.createBackgroundSubtractorMOG2(history=10000,varThreshold=12)

#limites de la meta
lower_green = np.array([40, 100, 100],np.uint8)# meta => verde
upper_green = np.array([100, 255, 255],np.uint8)
#Limites de la Caja
lower_yellow = np.array([20, 100, 100],np.uint8)#Caja => yellow
upper_yellow = np.array([30, 255, 255],np.uint8)
#Limite del robot
blueBajo = np.array([100,100,50],np.uint8)#robot => azul[100,100,50][140,255,100]
blueAlto = np.array([140,255,100],np.uint8)
#Limite de los obstaculos
obstBajo = np.array([160,150,0],np.uint8) # obstaculos => rojo
obstAlto = np.array([180,255,255],np.uint8)

#Enlace al servidor RTSP de la marca Dahua
PASS = "12345"#input("Ingrese la Contraseña Administador del dispositivo: ")
IP = "192.168.100.64"#input("Ingrese la direccion IP: ")
CH = "1"#input("Ingrese el numero del canal: ")
#URL = "rtsp://admin:"+PASS+"@"+IP+":554/cam/realmonitor?channel="+CH+"&subtype=0"
URL = "rtsp://admin:"+PASS+"@"+IP+":554/Streaming/Channels/"+CH+"01"
URL_Cam_IP = "rtsp://"+IP+":554/Streaming/Channels/"+CH+"01"
Capture = cv2.VideoCapture(URL)

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

def distancia_entre_coord(a,b):
  dist_min = math.sqrt(nw**2 + nh**2)/1.8
  distancia = math.sqrt((a[0]-b.clave[0])**2 + (a[1]-b.clave[1])**2)
  if distancia >= dist_min:
     distancia = math.sqrt((a[0]+ a[2]-b.clave[0])**2 + (a[1] -b.clave[1])**2)
     if distancia >= dist_min:
        distancia = math.sqrt((a[0]-b.clave[0])**2 + (a[1]+a[3] -b.clave[1])**2)
        if distancia >= dist_min:
          distancia = math.sqrt((a[0]+a[1] -b.clave[0])**2 + (a[1]+a[3] -b.clave[1])**2)
          if distancia >= dist_min:
            return True
  return False

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
    pass
    for k in elementos_obstaculos:
      if distancia_entre_coord(k,vertice):
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
      cv2.line(frame,(k.clave[0],k.clave[1]),(k.clave[0],k.clave[1]),(255, 0, 0), 10)

  def menor_trayecto(self,pos):
    actual_dist = pos.dist
    cont = 0
    min_tray = []
    min_tray.append(robot_cero_node)
    cv2.line(frame,(robot_cero_node.clave[0],robot_cero_node.clave[1]),(robot_cero_node.clave[0],robot_cero_node.clave[1]),(0, 0, 255), 10)
    time_lapse = time.time()
    while actual_dist > math.sqrt(nw**2+nh**2):
      if time.time() - time_lapse >= 0.5:
        return False
      min_dist_nodo = pos.vecinos[0]
      for k in pos.vecinos:
          if k.dist < min_dist_nodo.dist:
              min_dist_nodo = k
      min_tray.append(min_dist_nodo)
      pos = min_dist_nodo
      actual_dist = min_dist_nodo.dist
      cv2.line(frame,(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(0, 0, 255), 10)
    min_tray.append(meta_node)
    cv2.line(frame,(meta_node.clave[0],meta_node.clave[1]),(meta_node.clave[0],meta_node.clave[1]),(0, 0, 255), 10)
    #return min_tray

  def buscar_nodo_cercano(self,pos):
    min_dist_x = 5000
    min_dist_y = 5000
    x = 0
    for k in range(num_div_x):
      if min_dist_x > abs(pos[0] - (nw*(k+1))):
        min_dist_x = abs(pos[0] - (nw*(k+1)))
        x = nw*(k)
        #print(x)
    for k in range(num_div_y):
      if min_dist_y > abs(pos[1] - (nh*(k+1))):
        min_dist_y = abs(pos[1] - (nh*(k+1)))
        y = nh*(k+2)
        #print(y)
    return self.buscar_vertice(x,y)

while True:
    #try:
        #Recuperamos los frames del video 1 a 1 en cada iteracion
        ret,frame = Capture.read()
        frame = cv2.blur(frame, (5, 5))
        if ret == False:
            print("no se encontro frame")
        #print(frame.shape)

        #if time.time() - last_time_render >= 1:
        last_time_render = time.time()
        # Factor de escala
        scale_factor = 0.9  # Redimensiona al 50% del tamaño original

        # Obtener el nuevo tamaño
        new_width = int(frame.shape[1] * scale_factor)
        new_height = int(frame.shape[0] * scale_factor)

        # Redimensionar la imagen
        frame = cv2.resize(frame, (new_width, new_height))

        frame = ajustar_brillo(frame,nivel_rojo,nivel_verde,nivel_azul,nivel_constraste)
      
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #Resalta los objetos de los colores deseados
        maskGreen = cv2.inRange(frame_hsv,lower_green,upper_green)
        #maskRed2 = cv2.inRange(frame_hsv,redBajo2,redAlto2)
        maskRobot = cv2.inRange(frame_hsv,blueBajo,blueAlto)
        maskObst = cv2.inRange(frame_hsv,obstBajo,obstAlto)
        maskCaja = cv2.inRange(frame_hsv,lower_yellow,upper_yellow)
                
        #Obtener contorno del robot
        robot = cv2.Canny(maskRobot, 10, 150)
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
        elementos_obstaculos = np.empty((0, 4), int) #lista que contendra las coordenadas y dimenciones de los obstaculos

        for k in range(0,len(f)):
          x, y, w, h = cv2.boundingRect(f[k])
          area = w * h

          # Agregar solo si el área es mayor o igual al umbral
          if area >= nw*nh/5:
            elementos_obstaculos = np.vstack([elementos_obstaculos, np.array([(x, y, w, h)])])
      
        #if len(elementos_obstaculos) != 0:
        #Las siguientes lineas sirven para graficar el mapa
        #img_2 = 255*np.ones((570,1350,3),dtype=np.uint8)
        for j in range(len(elementos_obstaculos)):
          #Obstaculost
          cv2.line(frame,(elementos_obstaculos[j][0],elementos_obstaculos[j][1]),(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]),(0, 0, 255), 4)
          cv2.line(frame,(elementos_obstaculos[j][0],elementos_obstaculos[j][1]),(elementos_obstaculos[j][0],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(0, 0, 255), 4)
          cv2.line(frame,(elementos_obstaculos[j][0],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(0, 0, 255), 4)
          cv2.line(frame,(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]+elementos_obstaculos[j][3]),(elementos_obstaculos[j][0]+elementos_obstaculos[j][2],elementos_obstaculos[j][1]),(0, 0, 255), 4)

          """ cv2.line(frame,(X_coord[j],Y_coord[j]),(X_coord[j] + W_obst[j],Y_coord[j]),(30, 100, 0), 4)
          cv2.line(frame,(X_coord[j],Y_coord[j]),(X_coord[j],Y_coord[j] + H_obst[j]),(30, 100, 0), 4)
          cv2.line(frame,(X_coord[j],Y_coord[j] + H_obst[j]),(X_coord[j] + W_obst[j],Y_coord[j] + H_obst[j]),(30, 100, 0), 4)
          cv2.line(frame,(X_coord[j] + W_obst[j],Y_coord[j] + H_obst[j]),(X_coord[j] + W_obst[j],Y_coord[j]),(30, 100, 0), 4)
          """
        #Meta
        cv2.line(frame,(meta_coord[0],meta_coord[1]),(meta_coord[0]+meta_coord[2],meta_coord[1]),(50, 255, 50), 4)
        cv2.line(frame,(meta_coord[0],meta_coord[1]),(meta_coord[0],meta_coord[1]+meta_coord[3]),(50, 255, 50), 4)
        cv2.line(frame,(meta_coord[0],meta_coord[1]+meta_coord[3]),(meta_coord[0]+meta_coord[2],meta_coord[1]+meta_coord[3]),(50, 255, 50), 4)
        cv2.line(frame,(meta_coord[0]+meta_coord[2],meta_coord[1]),(meta_coord[0]+meta_coord[2],meta_coord[1]+meta_coord[3]),(50, 255, 50), 4)
        #Robot
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

        #######################33
        g1 = Grafo() # Se crea el grafo de nodos disponibles para transitar
        #Se crean los nodos y se agregan al grafo
        robot_cero_node = Nodo(robot_coord[0],robot_coord[1])
        meta_node = Nodo(meta_mean_point[0],meta_mean_point[1])
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
        g1.mostrarGrafos()
        print("grilla")
        if g1.menor_trayecto(g1.buscar_nodo_cercano(robot_coord)) == False:
          pass
        #print("Grafo")
        ##########################33

        cv2.imshow("Imagen para generar trayectoria",frame)
        """ cv2.imshow("mascara obstauculos", maskObst)
        cv2.imshow("mascara rojo", maskRed2)"""
        #cv2.imshow("mascara azul", maskCaja) 
        t = cv2.waitKey(1)
        if t & 0xFF == ord('t'):
          break
        """ except:
      Capture.open(URL)
      time.sleep(1)  # Pausa para reconectar
      print("falle") 
    t = cv2.waitKey(1)
    if t & 0xFF == ord('t'):
        break"""
Capture.release()
cv2.destroyAllWindows()