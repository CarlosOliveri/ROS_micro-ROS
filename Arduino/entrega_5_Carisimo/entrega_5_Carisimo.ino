void Control_PID(float actual_theta) {
    float max_integral = 50.0;                // Límite del término integral
    static float integral = 0.0;             // Término acumulativo para el error integral
    static unsigned long previous_time = 0;  // Tiempo del ciclo anterior
    unsigned long current_time = millis();   // Tiempo actual en milisegundos
    float dt = (current_time - previous_time) / 1000.0; // Intervalo de tiempo en segundos

    // Evitar división por cero
    if (dt <= 0) dt = 0.01;

    // Cálculo del error
    float error = actual_theta;

    // Términos del PID
    float P = kp * error;                     // Término proporcional
    integral += ki * error * dt;              // Acumular término integral
    integral = constrain(integral, -max_integral, max_integral); // Limitar término integral
    float D = kd * (error - error_anterior) / dt; // Término derivativo (basado en el cambio del error)

    // Salida del PID
    float salida_control = P + integral + D;

    // Limitar el rango de salida_control para evitar saturación de PWM
    float max_adjustment = min(velocidad_base, 255 - velocidad_base); // Ajuste máximo permitido
    salida_control = constrain(salida_control, -max_adjustment, max_adjustment);

    // PWM ajustado para motores
    float velocidad_1 = constrain((velocidad_base + salida_control), 0, 255); // Motor izquierdo
    float velocidad_2 = constrain((velocidad_base + salida_control), 0, 255); // Motor derecho

    // Control de dirección del motor
    digitalWrite(MOTOR_DIR_PIN_1, (velocidad_1 < 0) ? LOW : HIGH); // Dirección motor izquierdo
    digitalWrite(MOTOR_DIR_PIN_2, (velocidad_2 < 0) ? LOW : HIGH); // Dirección motor derecho

    // Enviar PWM ajustado con factores de corrección
    ledcWrite(PWM_CHANNEL_0, abs(velocidad_1)); // Aplicar corrección al motor izquierdo
    ledcWrite(PWM_CHANNEL_1, abs(velocidad_2));       // Motor derecho sin corrección

    // Actualizar valores para el próximo ciclo
    error_anterior = error;           // Actualizar error previo
    previous_time = current_time;     // Actualizar tiempo previo

    // Enviar datos para depuración
    pid_msg.data.data[0] = (int32_t)gps_speed_1;  // Velocidad GPS del motor derecho
    pid_msg.data.data[1] = (int32_t)gps_speed_2;  // Velocidad GPS del motor izquierdo
    pid_msg.data.data[2] = (int32_t)(error * 180 / 3.14); // Error en grados
    pid_msg.data.data[3] = (int32_t)encoder_1_ticks;      // Ticks del encoder derecho
    pid_msg.data.data[4] = (int32_t)encoder_2_ticks;      // Ticks del encoder izquierdo
}