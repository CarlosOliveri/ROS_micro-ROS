#define MOTOR_PWM_PIN_2 33
#define MOTOR_DIR_PIN_2 26
#define ENCODER_PIN_A_2 36
#define ENCODER_PIN_B_2 39

#define PWM_CHANNEL_1 1     // Canal de PWM 2
#define PWM_FREQUENCY 5000  // Frecuencia de PWM en Hz
#define PWM_RESOLUTION 8    // Resolución del PWM (8 bits = 0-255)
#define MAX_RPM 180         // Rango máximo de RPM de tu motor (ajusta según tu motor)
#define MAX_GPS 1080 
volatile int encoder_2_ticks = 0;
int last_time = 0;
int sample_time = 50;

void IRAM_ATTR encoder_2_isr() {
  // Incrementar contador de ticks del encoder
  encoder_2_ticks++;
  //Serial.println(encoder_2_ticks);
}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN_2, HIGH);
  
  pinMode(ENCODER_PIN_A_2, INPUT);
  pinMode(ENCODER_PIN_B_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B_2), encoder_2_isr, RISING);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);  // Configura el canal de PWM
  ledcAttachPin(MOTOR_PWM_PIN_2, PWM_CHANNEL_1);
  delay(2000);

  ledcWrite(PWM_CHANNEL_1, 127);  // Ajustar la señal PWM
}

void loop() {
  // put your main code here, to run repeatedly:
  /* if (encoder_2_ticks >= 1800){
    ledcWrite(PWM_CHANNEL_1, 0);  // Ajustar la señal PWM
    //Serial.println(encoder_2_ticks);
  } */

  if ((millis()- last_time) >= sample_time){
    last_time = millis();
    Serial.println(encoder_2_ticks);
    encoder_2_ticks = 0;

  }
  
}
