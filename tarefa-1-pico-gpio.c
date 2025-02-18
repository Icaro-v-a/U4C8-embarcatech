#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Definição do pino GPIO utilizado para o controle do servo
#define SERVO 22

// Configuração do PWM para 50Hz (período de 20ms)
const uint16_t PERIOD = 20000;   // Período do PWM em microssegundos (20ms)
const float DIVIDER_PWM = 125.0; // Divisor de clock do PWM para ajustar a frequência

// Definição dos ciclos ativos para as posições do servo (em microssegundos)
const uint16_t STEP = 5;        // Incremento do duty cycle para movimento suave (5µs)
const uint16_t MIN_DUTY = 500;  // Posição 0° (500µs)
const uint16_t MAX_DUTY = 2400; // Posição 180° (2400µs)
const uint16_t MID_DUTY = 1470; // Posição 90° (1470µs)

// Função para configurar o PWM no pino do servo
void setup_pwm()
{
    uint slice;
    gpio_set_function(SERVO, GPIO_FUNC_PWM); // Configura o pino como saída PWM
    slice = pwm_gpio_to_slice_num(SERVO);    // Obtém o slice do PWM associado ao pino
    pwm_set_clkdiv(slice, DIVIDER_PWM);      // Define o divisor de clock para ajustar a frequência
    pwm_set_wrap(slice, PERIOD);             // Define o valor máximo do contador do PWM
    pwm_set_gpio_level(SERVO, MIN_DUTY);     // Define a posição inicial (0°)
    pwm_set_enabled(slice, true);            // Habilita o PWM
}

int main()
{
    stdio_init_all(); // Inicializa a comunicação serial
    setup_pwm();      // Configura o PWM para controlar o servo

    // Posiciona o servo em 180° (2400µs) e aguarda 5 segundos
    pwm_set_gpio_level(SERVO, MAX_DUTY);
    sleep_ms(5000);

    // Posiciona o servo em 90° (1470µs) e aguarda 5 segundos
    pwm_set_gpio_level(SERVO, MID_DUTY);
    sleep_ms(5000);

    // Posiciona o servo em 0° (500µs) e aguarda 5 segundos
    pwm_set_gpio_level(SERVO, MIN_DUTY);
    sleep_ms(5000);

    // Inicializa o movimento suave entre 0° e 180°
    uint16_t position = MIN_DUTY;  // Começa em 0°
    int step = STEP;               // Define o incremento inicial

    while (true)
    {
        pwm_set_gpio_level(SERVO, position); // Atualiza a posição do servo
        sleep_ms(10); // Aguarda 10ms para suavizar o movimento

        // Atualiza a posição do servo
        position += step;

        // Inverte o sentido do movimento ao atingir os limites (0° ou 180°)
        if (position >= MAX_DUTY || position <= MIN_DUTY)
        {
            step = -step;
        }
    }
}
