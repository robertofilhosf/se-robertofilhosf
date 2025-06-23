#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "int_i2c.h"

// === PINOS ===
#define NTC_ADC_CHANNEL ADC1_CHANNEL_4   // GPIO4
#define BUZZER_GPIO GPIO_NUM_1
#define LED0 GPIO_NUM_17
#define LED1 GPIO_NUM_3
#define LED2 GPIO_NUM_11
#define LED3 GPIO_NUM_14
#define BTN_INC GPIO_NUM_36
#define BTN_DEC GPIO_NUM_35
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_20
#define I2C_PORT I2C_NUM_0

// === LCD ===
#define LCD_ADDRESS 0x27
#define LCD_SIZE DISPLAY_16X02

// === PWM (BUZZER) ===
#define BUZZER_TIMER LEDC_TIMER_0
#define BUZZER_CHANNEL LEDC_CHANNEL_0
#define BUZZER_FREQ 2000
#define BUZZER_MODE LEDC_LOW_SPEED_MODE
#define BUZZER_DUTY_RES LEDC_TIMER_8_BIT

// === DEBOUNCE ===
#define DEBOUNCE_US 150000

static const char *TAG = "TEMP_NTC";

lcd_i2c_handle_t lcd;
volatile int temp_alarme = 25;
volatile bool inc_pressed = false;
volatile bool dec_pressed = false;
int64_t last_press_inc = 0;
int64_t last_press_dec = 0;

// === ISR BOTÕES ===
static void IRAM_ATTR isr_inc(void* arg) {
    inc_pressed = true;
    gpio_intr_disable(BTN_INC);
}

static void IRAM_ATTR isr_dec(void* arg) {
    dec_pressed = true;
    gpio_intr_disable(BTN_DEC);
}

// === INICIALIZA ADC DO NTC ===
void ntc_adc_init() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

// === LEITURA DE TEMPERATURA SIMPLIFICADA ===
float ler_temperatura_ntc() {
    int adc_reading = adc1_get_raw(NTC_ADC_CHANNEL);
    float tensao = (adc_reading / 4095.0) * 3.3;
    float resistencia = (10000.0 * tensao) / (3.3 - tensao);
    float temp_K = 1.0 / (1.0 / 298.15 + log(resistencia / 10000.0) / 3950.0);
    return temp_K - 273.15;
}

// === CONFIGURAÇÃO DE HARDWARE ===
void init_hardware() {
    // === GPIO dos LEDs ===
    gpio_config_t leds = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&leds);

    // === GPIO dos botões ===
    gpio_config_t botoes = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1, // CORRIGIDO: ativar pull-down
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&botoes);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, isr_inc, NULL);
    gpio_isr_handler_add(BTN_DEC, isr_dec, NULL);

    // === PWM para buzzer ===
    ledc_timer_config_t timer_conf = {
        .speed_mode = BUZZER_MODE,
        .duty_resolution = BUZZER_DUTY_RES,
        .freq_hz = BUZZER_FREQ,
        .timer_num = BUZZER_TIMER
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .channel = BUZZER_CHANNEL,
        .duty = 0,
        .gpio_num = BUZZER_GPIO,
        .speed_mode = BUZZER_MODE,
        .timer_sel = BUZZER_TIMER
    };
    ledc_channel_config(&channel_conf);

    // === I2C + LCD ===
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

    lcd.address = LCD_ADDRESS;
    lcd.num = I2C_PORT;
    lcd.backlight = 1;
    lcd.size = LCD_SIZE;
    lcd_i2c_init(&lcd);
}

// === LCD ===
void atualizar_lcd(float temp_ntc, int temp_alarme) {
    char linha1[20], linha2[20];
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    sprintf(linha1, "NTC: %.1f C", temp_ntc);
    sprintf(linha2, "Alarme: %d C", temp_alarme);
    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, linha1);
    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, linha2);
}

// === LEDS ===
void atualizar_leds(float temp_ntc, int temp_alarme, bool piscar) {
    gpio_set_level(LED0, 0);
    gpio_set_level(LED1, 0);
    gpio_set_level(LED2, 0);
    gpio_set_level(LED3, 0);

    float diff = temp_alarme - temp_ntc;

    if (piscar) {
        gpio_set_level(LED0, 1);
        gpio_set_level(LED1, 1);
        gpio_set_level(LED2, 1);
        gpio_set_level(LED3, 1);
    } else {
        if (diff <= 20) gpio_set_level(LED0, 1);
        if (diff <= 15) gpio_set_level(LED1, 1);
        if (diff <= 10) gpio_set_level(LED2, 1);
        if (diff <= 2)  gpio_set_level(LED3, 1);
    }
}

// === BUZZER ===
void buzzer_set(bool on) {
    if (on) {
        ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 128);
    } else {
        ledc_set_duty(BUZZER_MODE, BUZZER_CHANNEL, 0);
    }
    ledc_update_duty(BUZZER_MODE, BUZZER_CHANNEL);
}

// === MAIN ===
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando...");
    init_hardware();
    ntc_adc_init();

    last_press_inc = esp_timer_get_time();
    last_press_dec = esp_timer_get_time();

    while (1) {
        int64_t agora = esp_timer_get_time();

        if (inc_pressed && (agora - last_press_inc > DEBOUNCE_US)) {
            temp_alarme += 5;
            last_press_inc = agora;
            inc_pressed = false;
            gpio_intr_enable(BTN_INC);
            ESP_LOGI(TAG, "Botão A (inc) pressionado. Alarme agora: %d", temp_alarme);
        }

        if (dec_pressed && (agora - last_press_dec > DEBOUNCE_US)) {
            temp_alarme -= 5;
            last_press_dec = agora;
            dec_pressed = false;
            gpio_intr_enable(BTN_DEC);
            ESP_LOGI(TAG, "Botão B (dec) pressionado. Alarme agora: %d", temp_alarme);
        }

        float temp_ntc = ler_temperatura_ntc();
        bool alarme = temp_ntc >= temp_alarme;

        atualizar_lcd(temp_ntc, temp_alarme);
        buzzer_set(alarme);
        atualizar_leds(temp_ntc, temp_alarme, alarme);

        if (alarme) {
            vTaskDelay(pdMS_TO_TICKS(300));
            atualizar_leds(temp_ntc, temp_alarme, false);
            vTaskDelay(pdMS_TO_TICKS(300));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}
