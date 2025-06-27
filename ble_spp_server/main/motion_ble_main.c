// Seeed XIAO ESP32C3 + MPU6500 + BLE 운동량 전송 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include <math.h>

#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU6500_SENSOR_ADDR         0x68
#define MPU6500_PWR_MGMT_1_REG      0x6B
#define MPU6500_ACCEL_XOUT_H        0x3B

static const char *TAG = "MOTION";
extern esp_gatt_if_t spp_gatts_if;
extern uint16_t spp_conn_id;
extern uint16_t spp_handle_table[];
#define SPP_IDX_SPP_DATA_NTY_VAL 4

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                      I2C_MASTER_RX_BUF_DISABLE,
                      I2C_MASTER_TX_BUF_DISABLE, 0);
}

void mpu6500_init() {
    uint8_t cmd[] = {MPU6500_PWR_MGMT_1_REG, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6500_SENSOR_ADDR,
                               cmd, sizeof(cmd), 1000 / portTICK_PERIOD_MS);
}

int16_t read_word(uint8_t reg) {
    uint8_t data[2];
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6500_SENSOR_ADDR,
                                 &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
    return (int16_t)((data[0] << 8) | data[1]);
}

void motion_task(void *arg) {
    float motion_score = 0;
    int count = 0;
    while (1) {
        int16_t ax = read_word(MPU6500_ACCEL_XOUT_H);
        int16_t ay = read_word(MPU6500_ACCEL_XOUT_H + 2);
        int16_t az = read_word(MPU6500_ACCEL_XOUT_H + 4);

        float accel_x = ax / 16384.0f;
        float accel_y = ay / 16384.0f;
        float accel_z = az / 16384.0f;

        float acc_mag = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        float delta = fabsf(acc_mag - 1.0f); // 중력 제거

        if (delta > 0.2f) {
            motion_score += delta;
        }

        count++;
        if (count >= 300) { // 100ms * 300 = 30초
            char level[16];
            if (motion_score < 50) strcpy(level, "low");
            else if (motion_score < 150) strcpy(level, "medium");
            else strcpy(level, "high");

            char msg[64];
            snprintf(msg, sizeof(msg), "Activity:%s Score:%.1f", level, motion_score);

            if (spp_gatts_if != ESP_GATT_IF_NONE && spp_conn_id != 0xFFFF) {
                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id,
                    spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], strlen(msg), (uint8_t *)msg, false);
            }

            ESP_LOGI(TAG, "%s", msg);
            motion_score = 0;
            count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    i2c_master_init();
    mpu6500_init();
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
} // BLE 서버는 기존 ble_spp_server로 유지
