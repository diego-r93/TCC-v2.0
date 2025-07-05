#include "Timer.h"

#include "CN0411.h"

// Definição do timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
   CN0411_pwm_gen();  // Substitui a ISR TIMER2_COMPA_vect
}

// Inicia o timer para gerar interrupção a 2kHz
void timer_start() {
   timer = timerBegin(0, 80, true);  // Timer 0, Prescaler 80 → 1 MHz (1 µs por tick)
   timerAttachInterrupt(timer, &onTimer, true);
   timerAlarmWrite(timer, 500, true);  // 500 µs = 2 kHz
   timerAlarmEnable(timer);
}

// Função de sleep padrão
void timer_sleep(uint32_t ticks) {
   delay(ticks);
}

// Função que gera um atraso de 5 µs repetidamente
void timer_sleep_5uS(uint32_t ticks) {
   if (ticks > (UINT32_MAX / 5))
      return;

   for (uint32_t i = 0; i < ticks; i++) {
      CN0411_pwm_gen();
      delayMicroseconds(5);
   }
}