#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # Cambiamos de Float32 a Int32
from std_msgs.msg import Int32MultiArray

#Velocidades para el giro a la derecha 
vel_m_derecho_der = "20"
vel_m_izquierdo_der = "100"
#Velocidades para el giro a la derecha 
vel_m_derecho_izq = "100"
vel_m_izquierdo_izq = "20"
#Velocidades para el avance
vel_m_derecho_fre = "100"
vel_m_izquierdo_fre = "100"
#Velocidades para el retroceso
vel_m_derecho_ret = "100"
vel_m_izquierdo_ret = "100"

class Movimientos_Node(Node):
    def __init__(self):
        super.__init__('rutines_node')
    
    #Creamos funciones para los movimientos compuestos
    #Giro a la deracha

    #Giro a la izquierda

    #Avanzar 

    #Retroceder



def main(args=None):
    rclpy.init(args=args)
    node = Movimientos_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()