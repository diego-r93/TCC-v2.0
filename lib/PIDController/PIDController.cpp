#include <PIDController.h>

#include "Arduino.h"
#include "freertos/FreeRTOS.h"

#define PID_SAMPLE_TIME 100

Pid::Pid() {
   last_run = 0;
   last_error = 0;
   proportional = 0.0;
   integral = 0.0;
   derivative = 0.0;
   kp = 0.0;   // Valor ajustado
   ki = 0.0;   // Valor ajustado
   kd = 15.0;  // Valor ajustado
}

float Pid::pid_control(float input, float target) {
   TickType_t now = xTaskGetTickCount();
   TickType_t dt = (now - last_run) * portTICK_PERIOD_MS / 1000;

   if (last_run == 0 || dt == 0) {
      dt = PID_SAMPLE_TIME;  // Assume um intervalo padrão na primeira execução
   }

   double error = target - input;
   if (controller_direction == Action::reverse) error = -error;

   proportional = kp * error;

   integral += (ki * error * dt);

   // Implementação do Anti-Windup: limitando o termo integral
   integral = fmin(fmax(integral, 0), target);

   derivative = kd * ((error - last_error) / dt);

   last_error = error;
   last_run = now;

   return (proportional + integral + derivative);
}

void Pid::setKp(double value) { kp = value; };

void Pid::setKi(double value) { ki = value; };

void Pid::setKd(double value) { kd = value; };

void Pid::setControllerDirection(Action direction) {
   controller_direction = direction;
}
void Pid::setControllerDirection(uint8_t direction) {
   controller_direction = (Action)direction;
}

void Pid::setControllerDirection(const std::string& direction) {
   std::string dirLower = direction;
   std::transform(dirLower.begin(), dirLower.end(), dirLower.begin(), ::tolower);

   if (dirLower == "direct") {
      controller_direction = Action::direct;
   } else if (dirLower == "reverse") {
      controller_direction = Action::reverse;
   } else {
      // Registro de erro
      ESP_LOGE("PID", "Invalid controller direction: %s", direction.c_str());
      // Ou, se não estiver usando o ESP logging:
      // Serial.println("Error: Invalid controller direction provided.");

      // Defina um valor padrão ou mantenha o valor atual
      controller_direction = Action::direct;
   }
}

double Pid::getKp() { return kp; };

double Pid::getKi() { return ki; };

double Pid::getKd() { return kd; };

float Pid::get_lastError() { return last_error; };

float Pid::get_lastSpeed() { return last_speed; };

float Pid::get_pid() { return (proportional + integral + derivative); };
