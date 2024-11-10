import math
import cv2

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

class Nodo:
  def __init__(self,x,y):
    self.clave = (x,y)
    self.padre = None
    self.vecinos = []
    self.dist = 0
    self.color = "blanco"

def distancia_entre_coord(a,b):
  distancia = math.sqrt((a[0]-b.clave[0])**2 + (a[1]-b.clave[1])**2)
  return distancia


class Grafo:
  def __init__(self,meta):
    self.vertices = []
    self.meta_mean_point = meta

  def agregar_vertice(self,vertice,elementos_obstaculos):
    es_apto = True
    for k in elementos_obstaculos:
      if distancia_entre_coord(k,vertice) >= math.sqrt(nw**2 + nh**2):
        if vertice.clave[0] < (k[0] + k[2]) and (vertice.clave[0] + nw) > k[0]:
          if vertice.clave[1] < (k[1] + k[3]) and (vertice.clave[1] + nh) > k[1]:
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
      cv2.line(frame,(k.clave[0],k.clave[1]),(k.clave[0],k.clave[1]),(255, 0, 0), 5)
      return frame

  def menor_trayecto(self,pos,frame):
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
        cv2.line(frame,(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(min_dist_nodo.clave[0],min_dist_nodo.clave[1]),(0, 0, 255), 5)
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
