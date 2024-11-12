#include <micro_ros_arduino.h>
#include <WiFi.h>  // Se añade la librería WiFi
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>


// Pines del motor y encoder 1
#define MOTOR_PWM_PIN_1 0
#define MOTOR_DIR_PIN_1 4
#define ENCODER_PIN_A_1 18
#define ENCODER_PIN_B_1 19

// Pines del motor y encoder 2
#define MOTOR_PWM_PIN_2 33
#define MOTOR_DIR_PIN_2 26
#define ENCODER_PIN_A_2 36
#define ENCODER_PIN_B_2 39

//Pines de los Sensores Ultrasonicos 1
#define TRIG_PIN_1 25
#define ECHO_PIN_1 34

//Pines de los Sensores Ultrasonicos 2
#define TRIG_PIN_2 27
#define ECHO_PIN_2 35

//Pines de los Sensores Ultrasonicos 3
#define TRIG_PIN_3 2
#define ECHO_PIN_3 15

//Configuracion de salida PWM
#define LED_PIN 2
#define PWM_CHANNEL_0 0     // Canal de PWM 1
#define PWM_CHANNEL_1 1     // Canal de PWM 2
#define PWM_FREQUENCY 5000  // Frecuencia de PWM en Hz
#define PWM_RESOLUTION 8    // Resolución del PWM (8 bits = 0-255)
#define MAX_RPM 130         // Rango máximo de RPM de tu motor (ajusta según tu motor)
#define MAX_GPS 1080

//TIEMPOS DE RETARDO 
//Lecturas de Ultrasonicos
float ultima_medicion = 0;  // segundos
float tiempo_medicion = 3600000;  // milisegundos
//PID
int sample_time = 100;//ms
int last_time_pid = 0;
//Velocidades
unsigned long last_time = 3600000; //ms
//debugging
int wait_time = 500;
int last_wait_time = 0;
bool Debug = false;
///////////////////////////////////////////////

//Array de lecturas de los sensores
float distancias[3];
// Definiciones de microros
rcl_subscription_t speed_1_subscriber;          //escucha la velocidad deseada para el motor 1
rcl_subscription_t speed_2_subscriber;          //escucha la velocidad deseada para el motor 2
rcl_subscription_t sample_time_subscriber;      //escucha el tiempo de muestreo de los sensores ultrasonicos
rcl_subscription_t angulo_deseado_subscriber;
rcl_publisher_t speed_1_publisher;              //Publica la velocidad calculada del encoder 1
rcl_publisher_t speed_2_publisher;              //Publica la velocidad calculada del encoder 2
rcl_publisher_t ultrasonic_publisher;           //Publicas lecturas de los ultrasonicos en un array
rcl_publisher_t pid_debugger_publisher;                   //Publica lectura del PID
std_msgs__msg__Int32 speed_1_msg;               //variable de velocidad de motor 1
std_msgs__msg__Int32 speed_2_msg;               //variable de velocidad de motor 2
std_msgs__msg__Int32 sample_time_msg;           //variable de tiempo de muestreo de sensores
std_msgs__msg__Int32 angulo_deseado_msg;
std_msgs__msg__Int32MultiArray ultrasonic_msg;  //variable de lecturas de los sensores
std_msgs__msg__Int32MultiArray pid_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


volatile long encoder_1_ticks = 0;  // Variable para contar los ticks del encoder 1
volatile long encoder_2_ticks = 0;  // Variable para contar los ticks del encoder 2
float current_speed_1 = 0;  // Velocidad calculada del motor 1
float desired_speed_1 = 0;
float current_speed_2 = 0;  // Velocidad calculada del motor 2
float desired_speed_2 = 0;
float angulo_deseado_pid = 0;

//Variables de PID
float error_anterior = 0;//en angulos
float I = 0; //Termino Integral
int velocidad_base = 900; //grados/s
float radio_rueda = 0.0375;
float dist_base = 0.225;
int initial_time = 0;
float gps_speed_1 = 0;
float gps_speed_2 = 0;
float dist_1 = 0;
float dist_2 = 0;
float theta = 0;
int last_2_ticks = 0;
int last_1_ticks = 0;
int kp = 1;
int ki = 0;
int kd = 0;
float pid_Array [5];

#define PWM_CHANNEL_0 0     // Canal de PWM 1
#define PWM_CHANNEL_1 1     // Canal de PWM 2
#define PWM_FREQUENCY 5000  // Frecuencia de PWM en Hz
#define PWM_RESOLUTION 8    // Resolución del PWM (8 bits = 0-255)
#define MAX_RPM 130         // Rango máximo de RPM de tu motor (ajusta según tu motor)

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/*
// Credenciales Wi-Fi (añadir SSID y contraseña de tu red)
const char* ssid = "FIUNA";
const char* password = "fiuna#2024";

// Dirección IP y puerto del agente micro-ROS
IPAddress agent_ip(172, 16, 227, 52);  // IP del agente micro-ROS
size_t agent_port = 8888;              // Puerto del agente micro-ROS
*/

void IRAM_ATTR encoder_1_isr() {
  // Incrementar contador de ticks del encoder
  //Serial.println(encoder_1_ticks);
  encoder_1_ticks++;
}

void IRAM_ATTR encoder_2_isr() {
  // Incrementar contador de ticks del encoder
  //Serial.println(encoder_2_ticks);
  encoder_2_ticks++;
}

float calcular_angulo(float dist_1,float dist_2 ){
  return (dist_2 - dist_1) / dist_base;
  //Serial.println(theta * 180 / 3.1416);
  //Serial.println(encoder_1_ticks);
  //Serial.println(encoder_2_ticks);
}

void Control_PID(float actual_theta,float theta_deseado){
  float error = - theta_deseado + actual_theta;

  float P = kp * error;
  float I = I + ki * error;
  float D = kd*(error - error_anterior);

  float salida_control = P + I + D;

  float velocidad_1 = map(velocidad_base + salida_control, 0,MAX_GPS, 0, 255); //IZQUIRDA
  float velocidad_2 = map(velocidad_base - salida_control, 0,MAX_GPS, 0, 255); //DERECHA

  
  ledcWrite(PWM_CHANNEL_0, velocidad_1);  // Ajustar la señal PWM
  ledcWrite(PWM_CHANNEL_1, velocidad_2);  // Ajustar la señal PWM

  /* pid_Array[0] = gps_speed_1;
  pid_Array[1] = gps_speed_2;
  pid_Array[2] = error * 180/3.24;
  pid_Array[3] = encoder_1_ticks;
  pid_Array[4] = encoder_2_ticks; */

  pid_msg.data.data[0] = (int32_t)gps_speed_1;
  pid_msg.data.data[1] = (int32_t)gps_speed_2;
  pid_msg.data.data[2] = (int32_t)(error * 180/3.24);
  pid_msg.data.data[3] = (int32_t)encoder_1_ticks;
  pid_msg.data.data[4] = (int32_t)encoder_2_ticks; 

  Serial.print(gps_speed_1);
  Serial.print(", ");
  Serial.print(gps_speed_2);
  Serial.print(", ");
  Serial.print(error * 180/3.14);
  Serial.print(", ");
  Serial.print(encoder_1_ticks);
  Serial.print(", ");
  Serial.println(encoder_2_ticks);

  error_anterior = error;
}

// Callback de la suscripción: Ajusta la velocidad del motor
void subscription_speed_1_callback(const void *msgin) {
  //Serial.println("Se llamo al speed Callback 1");
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  int motor_speed_rad = msg->data;  // Velocidad recibida en rad/segundos
  int motor_speed_rpm = motor_speed_rad * 60/ 360;

  // Limitar la velocidad RPM dentro del rango permitido
  if (motor_speed_rpm < 0) motor_speed_rpm = 0;
  if (motor_speed_rpm > MAX_RPM) motor_speed_rpm = MAX_RPM;

  // Convertir la velocidad de RPM (0 a MAX_RPM) a PWM (0 a 255)
  int motor_speed_pwm = map(motor_speed_rpm, 0, MAX_RPM, 0, 255);
  
  desired_speed_1 = motor_speed_rad;

  //Serial.println(desired_speed_1);
  // Ajustar la velocidad del motor utilizando el canal PWM
  //ledcWrite(PWM_CHANNEL_0, motor_speed_pwm);  // Ajustar la señal PWM
  //Serial.println(motor_speed_pwm);
}

// Callback de la suscripción: Ajusta la velocidad del motor
void subscription_speed_2_callback(const void *msgin) {
  //Serial.println("Se llamo al speed Callback 2");
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  int motor_speed_rad = msg->data;  // Velocidad recibida en rad/segundos
  int motor_speed_rpm = motor_speed_rad * 60/ 360;

  // Limitar la velocidad RPM dentro del rango permitido
  if (motor_speed_rpm < 0) motor_speed_rpm = 0;
  if (motor_speed_rpm > MAX_RPM) motor_speed_rpm = MAX_RPM;

  // Convertir la velocidad de RPM (0 a MAX_RPM) a PWM (0 a 255)
  int motor_speed_pwm = map(motor_speed_rpm, 0, MAX_RPM, 0, 255);

  desired_speed_2 = motor_speed_rad;
  
  // Ajustar la velocidad del motor utilizando el canal PWM
  //ledcWrite(PWM_CHANNEL_1, motor_speed_pwm);  // Ajustar la señal PWM
  //Serial.println(motor_speed_pwm);
}

// Callback de la suscripción: Ajusta el tiempo de muestreo
void subscription_sample_time_callback(const void *msgin) {
  Serial.println("Se llamo al sample time");
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  int sample_time_ms = msg->data;  // Tiempo de muestreo recibido
  tiempo_medicion = msg->data;
  
}

void subscription_angulo_deseado_callback(const void *msgin){
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  angulo_deseado_pid = (msg->data)*3.14/180;
  Serial.print("Se Cambio el angulo deseado del PID: ");
  Serial.println(angulo_deseado_pid);
}

float leer_distancia(int TRIG_PIN, int ECHO_PIN) {
  //Lectura de sensores ultrasonicos
  long duration;
  float distancia_cm;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);

  distancia_cm = (duration * 0.0343 / 2);

  return distancia_cm;
}

void leer_sensores(float *distancias) {
  distancias[0] = leer_distancia(TRIG_PIN_1, ECHO_PIN_1);  // Sensor 1
  distancias[1] = leer_distancia(TRIG_PIN_2, ECHO_PIN_2);  // Sensor 2
  distancias[2] = leer_distancia(TRIG_PIN_3, ECHO_PIN_3);  // Sensor 3
}

void setup() {
  Serial.begin(115200);
  //set_microros_wifi_transports("Flia Martinez", "nomeacuerdo@", "192.168.100.175", 8888);
  set_microros_wifi_transports("FIUNA", "fiuna#2024", "172.16.245.144", 8888);

  //Configuracion de direccion de motores
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_1, HIGH);
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_2, HIGH);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);

  // Configuración del encoder 1
  pinMode(ENCODER_PIN_A_1, INPUT);
  pinMode(ENCODER_PIN_B_1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A_1), encoder_1_isr, RISING);
  // Configuración del encoder 1
  pinMode(ENCODER_PIN_A_2, INPUT);
  pinMode(ENCODER_PIN_B_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B_2), encoder_2_isr, RISING);


  // Configuracion de los Sensores Ultrasonicos
  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);
  pinMode(TRIG_PIN_3, OUTPUT);
  pinMode(ECHO_PIN_3, INPUT);

  // Configuración PWM para el motor 1
  ledcSetup(PWM_CHANNEL_0, PWM_FREQUENCY, PWM_RESOLUTION);  // Configura el canal de PWM
  ledcAttachPin(MOTOR_PWM_PIN_1, PWM_CHANNEL_0);            // Conecta el canal PWM al pin del motor
  // Configuración PWM para el motor 1
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);  // Configura el canal de PWM
  ledcAttachPin(MOTOR_PWM_PIN_2, PWM_CHANNEL_1);            // Conecta el canal PWM al pin del motor

  delay(2000);

  allocator = rcl_get_default_allocator();
  // Inicializa las opciones de soporte
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Crear nodo
  RCCHECK(rclc_node_init_default(&node, "motor_controller_node", "", &support));

  // Crear suscriptor para la velocidad deseada del motor 1
  RCCHECK(rclc_subscription_init_default(
    &speed_1_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_1_speed"));

  // Crear suscriptor para la velocidad deseada del motor 2
  RCCHECK(rclc_subscription_init_default(
    &speed_2_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_2_speed"));

  // Crear suscriptor para la velocidad deseada
  RCCHECK(rclc_subscription_init_default(
    &sample_time_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "sample_time"));

  // Crear suscriptor para la velocidad deseada
  RCCHECK(rclc_subscription_init_default(
    &angulo_deseado_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "angulo_deseado"));

  // Crear publicador para la velocidad actual del motor 1
  RCCHECK(rclc_publisher_init_default(
    &speed_1_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_current_speed_1"));

  // Crear publicador para la velocidad actual del motor 2
  RCCHECK(rclc_publisher_init_default(
    &speed_2_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor_current_speed_2"));

  RCCHECK(rclc_publisher_init_default(
    &ultrasonic_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "ultrasonic_sensor"));

  RCCHECK(rclc_publisher_init_default(
    &pid_debugger_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "PID_debugger"));

  // Crear ejecutor y agregar suscriptor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &speed_1_subscriber, &speed_1_msg, &subscription_speed_1_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &speed_2_subscriber, &speed_2_msg, &subscription_speed_2_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sample_time_subscriber, &sample_time_msg, &subscription_sample_time_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &angulo_deseado_subscriber,&angulo_deseado_msg,&subscription_angulo_deseado_callback,ON_NEW_DATA));
  
  speed_1_msg.data = 0;
  speed_2_msg.data = 0;
  sample_time_msg.data = 0;

  ultrasonic_msg.data.size = 3;
  ultrasonic_msg.data.capacity = 3;
  ultrasonic_msg.data.data = (int32_t *)malloc(3 * sizeof(int32_t));

  pid_msg.data.size = 5;  // Tamaño del array
  pid_msg.data.capacity = 5;
  pid_msg.data.data = (int32_t *)malloc(5 * sizeof(int32_t));
}

void loop() {
  if (Debug == true){
    if (millis() - last_wait_time >= wait_time){
      RCSOFTCHECK(rcl_publish(&pid_debugger_publisher, &pid_msg, NULL));
    }
  }

  if (millis() - ultima_medicion >= tiempo_medicion) {
    leer_sensores(distancias);
    distancias[0] = leer_distancia(TRIG_PIN_1, ECHO_PIN_1);  // Sensor 1
    distancias[1] = leer_distancia(TRIG_PIN_2, ECHO_PIN_2);  // Sensor 2
    distancias[2] = leer_distancia(TRIG_PIN_3, ECHO_PIN_3);  // Sensor 3
    ultima_medicion = millis();
    for (int i = 0; i < 3; i++) {
      ultrasonic_msg.data.data[i] = (int32_t)distancias[i];
    } 
    RCSOFTCHECK(rcl_publish(&ultrasonic_publisher, &ultrasonic_msg, NULL));
  }
  // Publicar la velocidad actual del motor (basada en el encoder)
  /*unsigned long current_time = millis();
  unsigned long elapsed_time = current_time - last_time;
  if (elapsed_time >= 1000) {
    // Calcular velocidad en rad/min (ejemplo simple)
    current_speed_1 = (encoder_1_ticks-last_1_ticks) * 360 / (506);  // 20 ticks por revolución, ajusta según tu encoder
    current_speed_2 = (encoder_2_ticks-last_2_ticks) * 360 / (506);
    Serial.println(current_speed_2);
    speed_1_msg.data = (int32_t)current_speed_1;
    speed_2_msg.data = (int32_t)current_speed_2;
    RCSOFTCHECK(rcl_publish(&speed_1_publisher, &speed_1_msg, NULL));
    RCSOFTCHECK(rcl_publish(&speed_2_publisher, &speed_2_msg, NULL));
     //Reiniciar contador de ticks
    last_1_ticks = encoder_1_ticks;
    last_2_ticks = encoder_2_ticks;
    last_time = current_time;
  }*/
  //////////////////////////////////////////////
  //Serial.print("Enviando paquetes ...");
  if (millis() - last_time_pid >= sample_time){
    dist_1 =  encoder_1_ticks * 2 * 3.1416 * radio_rueda / (506);
    dist_2 =  encoder_2_ticks * 2 * 3.1416 * radio_rueda / (506);
    gps_speed_1 = ((encoder_1_ticks - last_1_ticks) * 1000 * 360) / (50 * 506);
    gps_speed_2 = ((encoder_2_ticks - last_2_ticks) * 1000 * 360) / (50 * 506);
    theta = calcular_angulo(dist_1,dist_2);
    Control_PID(theta,angulo_deseado_pid);
    last_time_pid = millis();
    last_1_ticks = encoder_1_ticks;
    last_2_ticks = encoder_2_ticks;
  }
  //delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); //desconentar para ejecutar suscriptores
}
