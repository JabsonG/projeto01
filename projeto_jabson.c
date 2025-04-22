#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "ws2812.pio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

//Incluir essas bibliotecas acima no CMAKELIST

#define IS_RGBW false
#define LED_R 13
#define LED_B 12
#define LED_G 11
#define BT_A 5
#define BT_B 6 
#define WS2812_PIN 7
#define NUM_PIXELS 25
#define VRX_PIN 26
#define VRY_PIN 27
#define SW_PIN 22

//Parametros para definição da conversão AD
//Parametros para definição do PWM
uint pwm_init_gpio(uint gpio, uint wrap){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, false);
    return slice_num;
}
//Função para enviar strings UART
void uart_print(const char* msg){
    uart_puts(uart0, msg);
}

//Parametros para definição da matriz 5x5
// Cores dos LEDs
uint8_t led_r = 0;
uint8_t led_g = 0;
uint8_t led_b = 200;

// Estado atual da matriz
volatile int estado_atual = 0;
const int estado_max = 3;

// Últimos tempos para debounce
absolute_time_t ultimo_botao_a = 0;
absolute_time_t ultimo_botao_b = 0;
const int tempo_debounce_ms = 200;

// 0: parado, 1: gerador, 2: motor, 3: alerta
const bool estados_maquina[4][5][5] = {
    // Parado
    {{0,0,0,0,0}, {0,0,0,0,0}, {0,0,1,0,0}, {0,0,0,0,0}, {0,0,0,0,0}},
    // Gerador (raio)
    {{0,0,1,0,0}, {0,1,0,0,0}, {1,1,1,1,1}, {0,1,0,0,0}, {0,0,1,0,0}},
    // Motor (hélice)
    {{0,0,1,0,0}, {0,0,0,1,0}, {1,1,1,1,1}, {0,0,0,1,0}, {0,0,1,0,0}},
    // Alerta / estado especial
    {{0,1,0,1,0}, {1,0,1,0,1}, {0,1,1,1,0}, {1,0,1,0,1}, {0,1,0,1,0}},
};

//Buffer da matriz 5x5
bool led_buffer[NUM_PIXELS] = {0};

//Funções auxiliares para controle dos LEDs da matrix WS2812
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

// Atualiza o buffer com o estado atual
void matriz_buffer(int num) {
    for (int y = 0; y < 5; y++) {
        for (int x = 0; x < 5; x++) {
            int idx = y * 5 + x;
            led_buffer[idx] = estados_maquina[num][y][x];
        }
    }
}

// Mostra o buffer na matriz WS2812
void mostrar_matriz() {
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (led_buffer[i]) {
            put_pixel(urgb_u32(led_r, led_g, led_b));
        } else {
            put_pixel(urgb_u32(0, 0, 0));
        }
    }
}

// Callback botão A (incrementa)
void botao_a_callback(uint gpio, uint32_t events) {
    absolute_time_t agora = get_absolute_time();
    if (absolute_time_diff_us(ultimo_botao_a, agora) < tempo_debounce_ms * 1000)
        return;
    ultimo_botao_a = agora;

    if (estado_atual < estado_max) estado_atual++;
    printf("Botão A pressionado - GERADOR\n");
    uart_puts(uart0, "Sentido horário: atuando como GERADOR\n");
    estado_atual = 1;
    matriz_buffer(estado_atual);
    mostrar_matriz();
}


// Callback botão B (decrementa)
void botao_b_callback(uint gpio, uint32_t events) {
    absolute_time_t agora = get_absolute_time();
    if (absolute_time_diff_us(ultimo_botao_b, agora) < tempo_debounce_ms * 1000)
        return;
    ultimo_botao_b = agora;

    if (estado_atual > 0) estado_atual--;
    printf("Botão B pressionado - MOTOR\n");
    uart_puts(uart0, "Sentido anti-horário: atuando como MOTOR\n");
    estado_atual = 2;
    matriz_buffer(estado_atual);
    mostrar_matriz();
}


void irq_handler(uint gpio, uint32_t events) {
    if (gpio == BT_A) botao_a_callback(gpio, events);
    else if (gpio == BT_B) botao_b_callback(gpio, events);
}


int main() //Para inicializar os LEDs, botões, matriz e 
    {
    stdio_init_all(); //Inicializa a comunicação serial

    //Inicialização da UART
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART); //Pino 0 como TX
    gpio_set_function(1, GPIO_FUNC_UART); //Pino 1 como RX

    //Habilitação do FIFO para evitar sobrecarga do buffer
    uart_set_fifo_enabled(uart0, true);

    //Inicialização do conversor AD
    adc_init();
    adc_gpio_init(VRX_PIN);
    adc_gpio_init(VRY_PIN);

    //Inicializa o PWM para os LEDs azul e verde
    uint pwm_wrap = 4096;
    uint b_pwm_slice = pwm_init_gpio(LED_B, pwm_wrap);
    uint g_pwm_slice = pwm_init_gpio(LED_G, pwm_wrap);

    // Inicializa a matriz WS2812
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    //LEDs
    gpio_init(LED_R);
    gpio_set_dir(LED_R, GPIO_OUT);

    /*gpio_init(LED_B);
    gpio_set_dir(LED_B, GPIO_OUT);
    gpio_put(LED_B, false);

    gpio_init(LED_G);
    gpio_set_dir(LED_G, GPIO_OUT);
    gpio_put(LED_G, false);
*/
    //Botoes
    gpio_init(BT_A);
    gpio_set_dir(BT_A, GPIO_IN);
    gpio_pull_up(BT_A);

    gpio_init(BT_B);
    gpio_set_dir(BT_B, GPIO_IN);
    gpio_pull_up(BT_B);
    
    //Botao do Joystick
    gpio_init(SW_PIN);
    gpio_set_dir(SW_PIN, GPIO_IN);
    gpio_pull_up(SW_PIN);

    //Interrupcoes
    //Configura/Habilita as interrupcoes para os botoes
    gpio_set_irq_enabled(BT_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BT_B, GPIO_IRQ_EDGE_FALL, true);

        // Configura interrupções
    // Registra a função global de callback
    gpio_set_irq_callback(irq_handler);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Configura as interrupções para os botões
    gpio_set_irq_enabled(BT_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BT_B, GPIO_IRQ_EDGE_FALL, true);

        
    // Mostra imagem inicial
    matriz_buffer(estado_atual);
    mostrar_matriz();

    uint16_t ultimo_vrx = 0;
    const int threshold = 50;

    while (true) {
        //Lendo os valores do ADC para VRX e VRY
        adc_select_input(0); //Canal 0 (GPIO 26)
        uint16_t vrx_valor = adc_read();

        adc_select_input(1); //Canal 1 (GPIO 27);
        uint16_t vry_valor = adc_read(); 

        static uint16_t vrx_anterior = 0;
        static uint16_t vry_anterior = 0;

        // Detecta movimento significativo (>10)
         // Detecção de movimentação do joystick (com limiar para evitar ruído)
         int diff = abs(vrx_valor - ultimo_vrx);
         if (diff > threshold) {
             ultimo_vrx = vrx_valor;
     
             // Mapeia o valor ADC (0–4095) para RPM (0–3000)
             int rpm = (vrx_valor * 3000) / 4095;
     
             char mensagem[50];
             sprintf(mensagem, "Joystick alterado - RPM ajustado para: %d RPM\n", rpm);
             printf("%s", mensagem);
             uart_puts(uart0, mensagem);
             
        }

        bool sw_valor = gpio_get(SW_PIN) == 0;

        pwm_set_gpio_level(LED_B, vrx_valor);
        pwm_set_enabled(b_pwm_slice, true);

        pwm_set_gpio_level(LED_G, vry_valor);
        pwm_set_enabled(g_pwm_slice, true);

        float b_duty_cycle = (vrx_valor / 4095.0)*100;
        float g_duty_cycle = (vry_valor / 4095.0)*100;
    }
}