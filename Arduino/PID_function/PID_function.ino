
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
#define sample_time 50

int last_time = 0;
int referencia = 500; //grados/s

float error_diferencial_1 = 0;
float error_diferencial_2 = 0;

float error_integral_1 = 0;
float error_integral_2 = 0;

float error_anterior_1 = 0;
float error_anterior_2 = 0;

float gps_speed_1 = 0;
float gps_speed_2 = 0;


#define kp 1
#define ki 0
#define kd 1

volatile int encoder_2_ticks = 0;
volatile int encoder_1_ticks = 0;

void IRAM_ATTR encoder_1_isr() {
  // Incrementar contador de ticks del encoder
  encoder_1_ticks++;
}

void IRAM_ATTR encoder_2_isr() {
  // Incrementar contador de ticks del encoder
  encoder_2_ticks++;
}

void function_PID (float error_actual_1,float error_actual_2){
  //error_Integral_1 = error_Integral_1 + error_actual_1;
  error_diferencial_1 = error_actual_1 - error_anterior_1;
  int esfuerzo_1 = int(kp*error_actual_1 + ki*error_integral_1 - kd*error_diferencial_1);
  error_anterior_1 = error_actual_1;

  error_diferencial_2 = error_actual_2 - error_anterior_2;
  int esfuerzo_2 = int(kp*error_actual_2 + ki*error_integral_2 - kd*error_diferencial_2);
  error_anterior_2 = error_actual_2;

  int motor_speed_pwm_1 = map(esfuerzo_1 + gps_speed_1,0,MAX_GPS,0,255);
  int motor_speed_pwm_2 = map(esfuerzo_2 + gps_speed_2,0,MAX_GPS,0,255);
  
  ledcWrite(PWM_CHANNEL_1, motor_speed_pwm_2);  // Ajustar la señal PWM
  ledcWrite(PWM_CHANNEL_0, motor_speed_pwm_1);  // Ajustar la señal PWM

  Serial.print(esfuerzo_1);
  Serial.print(",");
  Serial.print(error_actual_1);
  Serial.print(",");
  Serial.print(gps_speed_1);
  Serial.print(",");
  Serial.println(encoder_1_ticks);
}

void setup() {
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
  ledcAttachPin(MOTOR_PWM_PIN_2, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR_PWM_PIN_1, PWM_CHANNEL_0);
  delay(2000);

  //ledcWrite(PWM_CHANNEL_1, 12);  // Ajustar la señal PWM
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - last_time >= sample_time){
    gps_speed_1 = (encoder_1_ticks * 1000 * 360) / (50 * 497);
    gps_speed_2 = (encoder_2_ticks * 1000 * 360) / (50 * 497);
    function_PID(referencia - gps_speed_1,referencia - gps_speed_2);
    last_time = millis();
    encoder_1_ticks = 0;
    encoder_2_ticks = 0;
  }
}
