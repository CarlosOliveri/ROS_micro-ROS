import math
import cv2
import time

#CONSTANTES DE INTERES
proporcion_x = 0.76732
proporcion_y = 0.5
altura_camara = 3 #metros este parametro hay que verificar al momento de implementar
W_m = 2.70 #2 * altura_camara * proporcion_x #metros
H_m = 2.70 #2 * altura_camara * proporcion_y #metros
lado_robot = 0.22 #metros
W = 499 #Corresponde a X cm
H = 448 #Corresponde a Y cm
num_div_x = int(W_m/lado_robot) # este numero debe ser igual a la proporcion entre la altura del frame y la altura del robot
nw = int(W/num_div_x)
nh = nw
num_div_y = int(H/nw)

class Nodo:
  def __init__(self,x,y):
    self.clave = (x,y)
    self.padre = None
    self.vecinos = []
    self.dist = 0
    self.color = "blanco"

def distancia_entre_coord(a,b):
  dist_min = math.sqrt(nw**2 + nh**2)/2.5
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


class Grafo:
  def __init__(self,meta):
    self.vertices = []
    self.meta_mean_point = meta

  def agregar_vertice(self,vertice,elementos_obstaculos):
    es_apto = True
    for k in elementos_obstaculos:
      if distancia_entre_coord(k,vertice):
        if vertice.clave[0] < (k[0] + k[2]) and (vertice.clave[0]) > k[0]:
          if vertice.clave[1] < (k[1] + k[3]) and (vertice.clave[1]) > k[1]:
            es_apto = False
      else:
        es_apto = False
    if es_apto == True:
      vertice.dist = math.sqrt((vertice.clave[0]-self.meta_mean_point[0])**2 + (vertice.clave[1]-self.meta_mean_point[1])**2)
      self.vertices.append(vertice)

  def agregar_aristas(self,a,b):
    a.vecinos.append(b)

  def buscar_vertice(self,x,y):
    for j in self.vertices:
      #print(x,y,j.clave[0],j.clave[1])
      if j.clave[0] == x and j.clave[1] == y:
        return j
    return False

  def mostrarGrafos(self,frame):
    #img = 255*np.ones((H,W,3),dtype=np.uint8)
    for k in self.vertices:
      cv2.line(frame,(k.clave[0],k.clave[1]),(k.clave[0],k.clave[1]),(255, 0, 0), 10)
    return frame

  def menor_trayecto(self,pos,robot_cero_node,meta_node,frame):
    actual_dist = pos.dist
    cont = 0
    min_tray = []
    min_tray.append(robot_cero_node)
    cv2.line(frame,(robot_cero_node.clave[0],robot_cero_node.clave[1]),(robot_cero_node.clave[0],robot_cero_node.clave[1]),(0, 0, 255), 10)
    time_lapse = time.time()
    while actual_dist > math.sqrt(nw**2+nh**2):
        if time.time() - time_lapse >= 1:
          return False
        min_dist_nodo = pos.vecinos[0]
        for k in pos.vecinos:
            if k.dist < min_dist_nodo.dist:
                min_dist_nodo = k
        min_tray.append(min_dist_nodo)
        pos = min_dist_nodo
        actual_dist = min_dist_nodo.dist
        cv2.line(frame,(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(0, 0, 255), 5)
    min_tray.append(meta_node)
    cv2.line(frame,(meta_node.clave[0],meta_node.clave[1]),(meta_node.clave[0],meta_node.clave[1]),(0, 0, 255), 10)
    return min_tray

  def buscar_nodo_cercano(self,pos):
    min_dist_x = 1000
    min_dist_y = 1000
    x = 0
    y = 0
    for k in range(num_div_x):
      if min_dist_x > abs(pos[0] - (nw*(k+1))):
        min_dist_x = abs(pos[0] - (nw*(k+1)))
        x = nw*(k+1)
        #print(x)
    for k in range(num_div_y):
      if min_dist_y > abs(pos[1] - (nh*(k+1))):
        min_dist_y = abs(pos[1] - (nh*(k+1)))
        y = nh*(k+1)
        #print(y)
    return self.buscar_vertice(x,y)
