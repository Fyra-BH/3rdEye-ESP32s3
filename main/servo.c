#include "servo.h"

#include "driver/ledc.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <math.h>

static const char *TAG = "SERVO";
static QueueHandle_t servo_queue[3];

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT // Set duty resolution to 14 bits
#define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 50 Hz


#pragma pack(push, 1)
typedef struct {
    float start;
    float end;
    float T; // period
} servo_data_t;
#pragma pack(pop)


void put_servo_data(int servo_num, float start, float end, float t_seconds)
{
    static servo_data_t data;
    data.start = start;
    data.end = end;
    data.T = t_seconds;
    xQueueSend(servo_queue[servo_num], &data, portMAX_DELAY);
}

static void servo_loop_task(void *pvParameters)
{
    int servo_num = *((int*)pvParameters);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "servo_task %d start", servo_num);
    while (1) {
        servo_data_t data;
        xQueueReceive(servo_queue[servo_num], &data, portMAX_DELAY);
        ESP_LOGI(TAG, "servo %d: from %f to %f T %f", servo_num, data.start, data.end, data.T);
        fivetimesInterpolation(servo_num, data.start, data.end, data.T);
    }
}

/*########################### PWM #################################*/
// 对于ESP32C3，使用LEDC驱动PWM
static void led_pwm_init()
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel[3] = {
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_0,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = SERVO_PULSE_GPIO_0,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_1,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = SERVO_PULSE_GPIO_1,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        },
        {
            .speed_mode     = LEDC_MODE,
            .channel        = LEDC_CHANNEL_2,
            .timer_sel      = LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = SERVO_PULSE_GPIO_2,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        }
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
    }
}

static uint32_t duty_f2i(float duty, uint32_t duty_resolution)
{
    return (uint32_t)(duty * (float)(1 << duty_resolution));
}

void set_servo_pulse_width(int servo_num, int pulse_width)
{
    if (servo_num < 0 || servo_num >= 3) {
        ESP_LOGE(TAG, "Invalid servo number %d", servo_num);
        return;
    }
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t)servo_num, pulse_width));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t)servo_num));
}

void servo_tasks_init()
{
    ESP_LOGI(TAG, "Setup servo");
    led_pwm_init();
    for (int i = 0; i < 3; i++) {
        servo_queue[i] = xQueueCreate(1, sizeof(servo_data_t));
        xTaskCreate(servo_loop_task, "servo_tasks", 4096, &i, 10, NULL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 五次插值函数，输入为目标舵机、初始角度、目标角度
void fivetimesInterpolation(int myservo, float thetai, float thetaf, float t_seconds) {
    // 定义插值时间总长度 (ms)，可以根据需求调整
    // float T = 2000.0; // 插值时间为2秒 (2000 ms)
    float T = t_seconds * 1000.0; // 插值时间为2秒 (2000 ms)
    int steps = 20;   // 插值步数，越多越平滑
    float dt = T / steps;

    // 五次插值多项式系数
    float a0 = thetai;
    float a1 = 0;
    float a2 = 0;
    float a3 = 10 * (thetaf - thetai) / (T * T * T);
    float a4 = -15 * (thetaf - thetai) / (T * T * T * T);
    float a5 = 6 * (thetaf - thetai) / (T * T * T * T * T);

    // 插值过程
    for (int i = 0; i <= steps; i++) {
        float t = i * dt; // 当前时间
        float theta = a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
        
        // 将插值角度设置到舵机
        float pulse_width = (theta * 2.0 / 180.0 + 0.5) / 20.0; // 将角度转换为占空比;20.0代表20ms
        set_servo_pulse_width(myservo, duty_f2i(pulse_width, LEDC_DUTY_RES));
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}

