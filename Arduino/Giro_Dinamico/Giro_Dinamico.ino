#define MOTOR_PWM_PIN_2 33
#define MOTOR_DIR_PIN_2 26
#define ENCODER_PIN_A_2 36
#define ENCODER_PIN_B_2 39

#define MOTOR_PWM_PIN_1 0
#define MOTOR_DIR_PIN_1 4
#define ENCODER_PIN_A_1 18
#define ENCODER_PIN_B_1 19

#define PWM_CHANNEL_0 1     // Canal de PWM 2
#define PWM_CHANNEL_1 2  
#define PWM_FREQUENCY 5000  // Frecuencia de PWM en Hz
#define PWM_RESOLUTION 8    // Resolución del PWM (8 bits = 0-255)
#define MAX_RPM 13.61        // Rango máximo de RPM de tu motor (ajusta según tu motor)
#define MAX_GPS 1079

#define sample_time 50 //ms

int last_time = 0;//ms
int velocidad_base = 500; //grados/s

float error_anterior = 0;//en angulos

float I = 0; //Termino Integral

float gps_speed_1 = 0;
float gps_speed_2 = 0;

float radio_rueda = 0.0375;
float dist_base = 0.225;
int initial_time = 0;

float dist_1 = 0;
float dist_2 = 0;
float theta = 0;
float theta_Des = 3.14/2;

float coord_x = 0.5;
float coord_y = 0.5;

#define kp 5000
#define ki 500
#define kd 2500

volatile int encoder_2_ticks = 0;
int last_2_ticks = 0;
volatile int encoder_1_ticks = 0;
int last_1_ticks = 0;

void IRAM_ATTR encoder_1_isr() {
  // Incrementar contador de ticks del encoder
  encoder_1_ticks++;
}

void IRAM_ATTR encoder_2_isr() {
  // Incrementar contador de ticks del encoder
  encoder_2_ticks++;
}

float calcular_angulo(float dist_1,float dist_2 ){
  return (dist_2 - dist_1) / dist_base;
  //Serial.println(theta * 180 / 3.1416);
  //Serial.println(encoder_1_ticks);
  //Serial.println(encoder_2_ticks);
}

void Control_PID(float actual_theta,float theta_deseada){
  float error = - theta_deseada + actual_theta;

  float P = kp * error;
  float I = I + ki * error;
  float D = kd*(error - error_anterior);

  float salida_control = P + I + D;

  float velocidad_1 = map(velocidad_base + salida_control, 0,MAX_GPS, 0, 255); //IZQUIRDA
  float velocidad_2 = map(velocidad_base - salida_control, 0,MAX_GPS, 0, 255); //DERECHA
  
  ledcWrite(PWM_CHANNEL_0, velocidad_1);  // Ajustar la señal PWM
  ledcWrite(PWM_CHANNEL_1, velocidad_2);  // Ajustar la señal PWM

  Serial.print(gps_speed_1);
  Serial.print(", ");
  Serial.print(gps_speed_2);
  Serial.print(", ");
  Serial.print(error * 180/3.14);
  Serial.print(", ");
  Serial.print(encoder_1_ticks);
  Serial.print(", ");
  Serial.print(encoder_2_ticks);
  Serial.print(", ");
  Serial.print(abs(theta*180/3.14));
  Serial.print(", ");
  Serial.println(theta_Des*180/3.14);
  Serial.print(", ");
  Serial.println(salida_control);

  error_anterior = error;
  salida_control = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_2, HIGH);
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_1, HIGH);
  
  pinMode(ENCODER_PIN_A_2, INPUT);
  pinMode(ENCODER_PIN_B_2, INPUT);
  pinMode(ENCODER_PIN_A_1, INPUT);
  pinMode(ENCODER_PIN_B_1, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B_2), encoder_2_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B_1), encoder_1_isr, RISING);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);  // Configura el canal de PWM
  ledcSetup(PWM_CHANNEL_0, PWM_FREQUENCY, PWM_RESOLUTION); 
  ledcAttachPin(MOTOR_PWM_PIN_2, PWM_CHANNEL_1); // derecha
  ledcAttachPin(MOTOR_PWM_PIN_1, PWM_CHANNEL_0); //izquierda
  delay(2000);
}
int bandera = 1;
void loop() {
  while (bandera == 0){
    float m = coord_y/coord_x;
    theta_Des = atan(m);
    dist_1 =  encoder_1_ticks * 2 * 3.1416 * radio_rueda / (506);
    dist_2 =  encoder_2_ticks * 2 * 3.1416 * radio_rueda / (506);
    theta = calcular_angulo(dist_1,dist_2);
    if (abs(theta) >= theta_Des){
      ledcWrite(PWM_CHANNEL_0, 0);  // Ajustar la señal PWM
      ledcWrite(PWM_CHANNEL_1, 0);  // Ajustar la señal PWM
      bandera = 1;
      //theta_Des = 0;
    }else{
      Control_PID(theta,theta_Des);
    }
  }
  while(bandera == 1){
    if ((millis() - last_time) >= sample_time){
      dist_1 =  encoder_1_ticks * 2 * 3.1416 * radio_rueda / (506);
      dist_2 =  encoder_2_ticks * 2 * 3.1416 * radio_rueda / (506);
      gps_speed_1 = ((encoder_1_ticks - last_1_ticks) * 1000 * 360) / (50 * 506);
      gps_speed_2 = ((encoder_2_ticks - last_2_ticks) * 1000 * 360) / (50 * 506);
      theta = calcular_angulo(dist_1,dist_2);
      Control_PID(theta,theta_Des);
      last_time = millis();
      last_1_ticks = encoder_1_ticks;
      last_2_ticks = encoder_2_ticks;
    }
  }
  ledcWrite(PWM_CHANNEL_0, 0);  // Ajustar la señal PWM
  ledcWrite(PWM_CHANNEL_1, 0);  // Ajustar la señal PWM
  
}
