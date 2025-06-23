#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "int_i2c.h"
#include "esp_log.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"

// === Pinos dos botões, buzzer, sensor e LEDs ===
#define BTN_INC GPIO_NUM_36  // Botão para aumentar o alarme
#define BTN_DEC GPIO_NUM_35  // Botão para diminuir o alarme
#define BUZZER GPIO_NUM_1    // Pino do buzzer
#define NTC_ADC_CHANNEL ADC1_CHANNEL_4  // Sensor NTC no GPIO 4
#define LED0 GPIO_NUM_17
#define LED1 GPIO_NUM_3
#define LED2 GPIO_NUM_11
#define LED3 GPIO_NUM_14

// === Pinos do LCD I2C ===
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_20
#define I2C_PORT I2C_NUM_0
#define LCD_ADDRESS 0x27
#define LCD_SIZE DISPLAY_16X02

// === Pinos do cartão SD ===
#define SD_MNT_POINT "/sdcard"
#define PIN_NUM_MISO GPIO_NUM_42
#define PIN_NUM_MOSI GPIO_NUM_41
#define PIN_NUM_CLK  GPIO_NUM_40
#define PIN_NUM_CS   GPIO_NUM_45

// === Parâmetros do buzzer com PWM ===
#define BUZZER_TIMER LEDC_TIMER_0
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_FREQ 2000
#define BUZZER_MODE LEDC_LOW_SPEED_MODE
#define BUZZER_DUTY_RES LEDC_TIMER_8_BIT

// Tempo para evitar clique duplo acidental
#define DEBOUNCE_US 150000

lcd_i2c_handle_t lcd;
int temp_alarme = 25;
int64_t last_press_inc = 0;
int64_t last_press_dec = 0;
volatile bool inc_pressed = false;
volatile bool dec_pressed = false;

// === Quando o botão de aumentar for apertado ===
static void IRAM_ATTR isr_inc(void* arg) {
    inc_pressed = true;
    gpio_intr_disable(BTN_INC);  // desativa até processar
}

// === Quando o botão de diminuir for apertado ===
static void IRAM_ATTR isr_dec(void* arg) {
    dec_pressed = true;
    gpio_intr_disable(BTN_DEC);
}

// === Simula montar cartão SD ===
void init_sd_card() {
    printf("Cartão SD PRONTO\n");
}

// === Função para ler a temperatura do sensor ===
float ler_temperatura() {
    int adc = adc1_get_raw(NTC_ADC_CHANNEL);  // Lê valor bruto
    float tensao = (adc / 4095.0) * 3.3;      // Converte para tensão
    float resistencia = (10000.0 * tensao) / (3.3 - tensao);  // Calcula resistência
    float temp_K = 1.0 / (1.0 / 298.15 + log(resistencia / 10000.0) / 3950.0);
    return temp_K - 273.15;  // Converte para °C
}

// === Atualiza o display com temperatura e limite ===
void atualizar_lcd(float temp, int alarme) {
    char l1[20], l2[20];
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);  // Limpa LCD
    snprintf(l1, sizeof(l1), "NTC: %.1f C", temp);
    snprintf(l2, sizeof(l2), "Alarme: %d C", alarme);
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, l1);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, l2);
}

// === Liga os LEDs conforme se aproxima do alarme ===
void atualizar_leds(float t, int a, bool alerta) {
    gpio_set_level(LED0, 0);
    gpio_set_level(LED1, 0);
    gpio_set_level(LED2, 0);
    gpio_set_level(LED3, 0);

    if (alerta) {
        // Acende todos os LEDs se passou do limite
        gpio_set_level(LED0, 1);
        gpio_set_level(LED1, 1);
        gpio_set_level(LED2, 1);
        gpio_set_level(LED3, 1);
    } else {
        // Acende conforme vai se aproximando
        if (a - t <= 20) gpio_set_level(LED0, 1);
        if (a - t <= 15) gpio_set_level(LED1, 1);
        if (a - t <= 10) gpio_set_level(LED2, 1);
        if (a - t <= 2)  gpio_set_level(LED3, 1);
    }
}

// === Liga ou desliga o buzzer ===
void buzzer_set(bool on) {
    ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, on ? 128 : 0);
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
}

// === Função principal que roda pra sempre ===
void app_main() {
    // Configura LEDs
    gpio_config_t leds = {
        .pin_bit_mask = (1ULL<<LED0)|(1ULL<<LED1)|(1ULL<<LED2)|(1ULL<<LED3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&leds);

    // Configura botões
    gpio_config_t botoes = {
        .pin_bit_mask = (1ULL<<BTN_INC)|(1ULL<<BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&botoes);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, isr_inc, NULL);
    gpio_isr_handler_add(BTN_DEC, isr_dec, NULL);

    // Configura o buzzer com PWM
    ledc_timer_config_t timer = {
        .speed_mode = BUZZER_MODE,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz = BUZZER_FREQ,
        .timer_num = BUZZER_TIMER
    };
    ledc_timer_config(&timer);
    ledc_channel_config_t ch = {
        .channel = BUZZER_CHANNEL,
        .duty = 0,
        .gpio_num = BUZZER,
        .speed_mode = BUZZER_MODE,
        .timer_sel = BUZZER_TIMER
    };
    ledc_channel_config(&ch);

    // Configura ADC do sensor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Inicia o LCD
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = 1,
        .scl_pullup_en = 1,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    lcd = (lcd_i2c_handle_t){.address=LCD_ADDRESS, .num=I2C_PORT, .backlight=1, .size=LCD_SIZE};
    lcd_i2c_init(&lcd);

    // Inicia o SDCard (simulado)
    init_sd_card();

    last_press_inc = esp_timer_get_time();
    last_press_dec = esp_timer_get_time();

    while (1) {
        int64_t agora = esp_timer_get_time();
        float temp = ler_temperatura();
        bool alerta = temp >= temp_alarme;

        // Verifica se botão de + foi apertado
        if (inc_pressed && (agora - last_press_inc > DEBOUNCE_US)) {
            temp_alarme += 5;
            last_press_inc = agora;
            inc_pressed = false;
            gpio_intr_enable(BTN_INC);
        }

        // Verifica se botão de - foi apertado
        if (dec_pressed && (agora - last_press_dec > DEBOUNCE_US)) {
            temp_alarme -= 5;
            last_press_dec = agora;
            dec_pressed = false;
            gpio_intr_enable(BTN_DEC);
        }

        atualizar_lcd(temp, temp_alarme);
        atualizar_leds(temp, temp_alarme, alerta);
        buzzer_set(alerta);

        // Mostra no console que os dados estão sendo registrados
        printf("| Temp: %.2f°C | Alarme: %d°C\n", temp, temp_alarme);

        vTaskDelay(pdMS_TO_TICKS(alerta ? 300 : 500));
    }
}
