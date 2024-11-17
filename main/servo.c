#include "servo.h"

#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <math.h>

static const char *TAG = "SERVO";
static QueueHandle_t servo_queue[3];

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
mcpwm_cmpr_handle_t comparators[3];

static void setup_sync_strategy(mcpwm_timer_handle_t timers[])
{
    //       soft
    //        |
    //        v
    //    +-timer0--+
    //    |         |
    //    v         v
    // timer1    timer2
    ESP_LOGI(TAG, "Create software sync source");
    mcpwm_sync_handle_t soft_sync_source = NULL;
    mcpwm_soft_sync_config_t soft_sync_config = {};
    ESP_ERROR_CHECK(mcpwm_new_soft_sync_src(&soft_sync_config, &soft_sync_source));

    ESP_LOGI(TAG, "Create timer sync source to propagate the sync event");
    mcpwm_sync_handle_t timer_sync_source;
    mcpwm_timer_sync_src_config_t timer_sync_config = {
        .flags.propagate_input_sync = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timers[0], &timer_sync_config, &timer_sync_source));

    ESP_LOGI(TAG, "Set sync phase for timers");
    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .sync_src = soft_sync_source,
    };
    ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timers[0], &sync_phase_config));
    sync_phase_config.sync_src = timer_sync_source;
    for (int i = 1; i < 3; ++i) {
        ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timers[i], &sync_phase_config));
    }

    ESP_LOGI(TAG, "Trigger the software sync event");
    ESP_ERROR_CHECK(mcpwm_soft_sync_activate(soft_sync_source));
}

void mcpwm_init_grou()
{
    ESP_LOGI(TAG, "Create timers");
    mcpwm_timer_handle_t timers[3];
    mcpwm_timer_config_t timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .group_id = 0,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[i]));
    }

    ESP_LOGI(TAG, "Create operators");
    mcpwm_oper_handle_t operators[3];
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator should be in the same group of the above timers
    };
    for (int i = 0; i < 3; ++i) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
    }

    ESP_LOGI(TAG, "Connect timers and operators with each other");
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timers[i]));
    }

    ESP_LOGI(TAG, "Create comparators");
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tez = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        // init compare for each comparator
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 200));
    }

    ESP_LOGI(TAG, "Create generators");
    mcpwm_gen_handle_t generators[3];
    const int gen_gpios[3] = {SERVO_PULSE_GPIO_0, SERVO_PULSE_GPIO_1, SERVO_PULSE_GPIO_2};
    mcpwm_generator_config_t gen_config = {};
    for (int i = 0; i < 3; i++) {
        gen_config.gen_gpio_num = gen_gpios[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i]));
    }

    ESP_LOGI(TAG, "Set generator actions on timer and compare event");
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generators[i],
                                                                  // when the timer value is zero, and is counting up, set output to high
                                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generators[i],
                                                                    // when compare event happens, and timer is counting up, set output to low
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));
    }

    ESP_LOGI(TAG, "Start timers one by one, so they are not synced");
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timers[i]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[i], MCPWM_TIMER_START_NO_STOP));
    }

    ESP_LOGI(TAG, "Setup sync strategy");
    setup_sync_strategy(timers);

    ESP_LOGI(TAG, "Now the output PWMs should in sync");
    for (int i = 0; i < 3; ++i) {
        // remove the force level on the generator, so that we can see the PWM again
        ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generators[i], -1, true));
    }
}

void set_servo_pulse_width(int servo_num, int pulse_width)
{
    if (servo_num < 0 || servo_num >= 3) {
        ESP_LOGE(TAG, "Invalid servo number %d", servo_num);
        return;
    }
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[servo_num], pulse_width));
}

void servo_tasks_init()
{
    ESP_LOGI(TAG, "Setup servo");
    mcpwm_init_grou();
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
    int steps = 50;   // 插值步数，越多越平滑
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
        int pulse_width = (int)(theta * 2000.0 / 180.0 + 500); // 将角度转换为脉宽值
        set_servo_pulse_width(myservo, pulse_width);
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
}

