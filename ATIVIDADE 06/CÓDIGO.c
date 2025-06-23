#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "int_i2c.h"

// === DEFINIÇÕES DE PINOS ===
#define LED0 GPIO_NUM_17
#define LED1 GPIO_NUM_3
#define LED2 GPIO_NUM_11
#define LED3 GPIO_NUM_14

#define BTN_INC GPIO_NUM_36
#define BTN_DEC GPIO_NUM_35

#define LED_PWM GPIO_NUM_1

#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_20
#define I2C_PORT I2C_NUM_0

#define LCD_ADDRESS 0x27
#define LCD_SIZE DISPLAY_16X02

// === PWM CONFIG ===
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 5000

// === DEBOUNCE TIME ===
#define DEBOUNCE_TIME_US 200000

static const char *TAG = "CONTADOR_BIN";

lcd_i2c_handle_t lcd;

uint8_t counter = 0;
int64_t last_press_inc = 0;
int64_t last_press_dec = 0;

volatile bool inc_pressed = false;
volatile bool dec_pressed = false;

// === ISR BOTÕES ===
static void IRAM_ATTR isr_btn_inc(void* arg) {
    gpio_intr_disable(BTN_INC);
    inc_pressed = true;
}

static void IRAM_ATTR isr_btn_dec(void* arg) {
    gpio_intr_disable(BTN_DEC);
    dec_pressed = true;
}

// === INICIALIZA I2C MASTER ===
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

// === ATUALIZA OS LEDS E PWM ===
void update_leds(uint8_t value) {
    gpio_set_level(LED0, value & 0x01);
    gpio_set_level(LED1, (value >> 1) & 0x01);
    gpio_set_level(LED2, (value >> 2) & 0x01);
    gpio_set_level(LED3, (value >> 3) & 0x01);

    uint32_t duty = (value & 0x0F) * (255 / 15);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// === ATUALIZA O LCD ===
void update_lcd(uint8_t valor) {
    char linha1[20], linha2[20];
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    lcd_i2c_cursor_set(&lcd, 0, 0);
    snprintf(linha1, sizeof(linha1), "Hex: 0x%X", valor);
    lcd_i2c_print(&lcd, linha1);

    lcd_i2c_cursor_set(&lcd, 0, 1);
    snprintf(linha2, sizeof(linha2), "Dec: %d", valor);
    lcd_i2c_print(&lcd, linha2);
}

// === CONFIGURA GPIOs, LEDC E INTERRUPÇÕES ===
void init_gpio() {
    // LEDs binários
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED0) | (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Botões com interrupção
    io_conf.pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, isr_btn_inc, NULL);
    gpio_isr_handler_add(BTN_DEC, isr_btn_dec, NULL);

    // PWM do LED adicional
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .channel    = LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = LED_PWM,
        .speed_mode = LEDC_MODE,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&channel_conf);
}

// === FUNÇÃO PRINCIPAL ===
void app_main() {
    ESP_LOGI(TAG, "Iniciando sistema...");

    init_gpio();
    i2c_master_init();

    // LCD config
    lcd.address = LCD_ADDRESS;
    lcd.num = I2C_PORT;
    lcd.backlight = 1;
    lcd.size = LCD_SIZE;
    lcd_i2c_init(&lcd);

    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "Contador Binario");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);

    update_leds(counter);
    update_lcd(counter);

    last_press_inc = esp_timer_get_time();
    last_press_dec = esp_timer_get_time();

    while (1) {
        int64_t agora = esp_timer_get_time();

        // Verifica botão de incremento
        if (inc_pressed && (agora - last_press_inc > DEBOUNCE_TIME_US)) {
            last_press_inc = agora;
            counter = (counter + 1) & 0x0F;
            ESP_LOGI(TAG, "Incremento: %d (0x%X)", counter, counter);
            update_leds(counter);
            update_lcd(counter);
            inc_pressed = false;
            gpio_intr_enable(BTN_INC);
        }

        // Verifica botão de decremento
        if (dec_pressed && (agora - last_press_dec > DEBOUNCE_TIME_US)) {
            last_press_dec = agora;
            counter = (counter - 1) & 0x0F;
            ESP_LOGI(TAG, "Decremento: %d (0x%X)", counter, counter);
            update_leds(counter);
            update_lcd(counter);
            dec_pressed = false;
            gpio_intr_enable(BTN_DEC);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
