#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "esp_timer.h"

#define I2C_MASTER_SCL_IO          5
#define I2C_MASTER_SDA_IO          4    
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define I2C_MASTER_TX_BUF_DISABLE  0
#define I2C_MASTER_RX_BUF_DISABLE  0

#define MPU6050_SENSOR_ADDR        0x68
#define MPU6050_ACCEL_XOUT_H       0x3B
#define MPU6050_GYRO_XOUT_H        0x43
#define MPU6050_PWR_MGMT_1         0x6B
#define MPU6050_SMPLRT_DIV         0x19

static const char *TAG = "mpu6050-example";

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config error: %d", err);
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install error: %d", err);
        return err;
    }

    return ESP_OK;
}

esp_err_t i2c_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* data, size_t data_len)
{
    if (data_len == 0)
    {
        return ESP_OK;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);

    if (data_len > 1)
    {
        i2c_master_read(cmd, data, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + data_len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); // 确保在每次分配cmd后正确地删除了cmd
    return ret;
}


static esp_err_t i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static void mpu6050_init() {
    i2c_write_byte(MPU6050_SENSOR_ADDR, MPU6050_PWR_MGMT_1, 0x00); // Wake up
    i2c_write_byte(MPU6050_SENSOR_ADDR, MPU6050_SMPLRT_DIV, 0x07); // Set sample rate divider
}

static void read_mpu6050_data(int16_t *accel_data, int16_t *gyro_data) {
    uint8_t buffer[14];
    i2c_read(MPU6050_SENSOR_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14);

    for (int i = 0; i < 3; ++i) {
        accel_data[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        gyro_data[i] = (buffer[i * 2 + 8] << 8) | buffer[i * 2 + 9];
    }
}

#define ALPHA 0.95  // 互补滤波器的权重，范围为 0 到 1，可以根据需要调整
static int64_t last_time = 0;

static void compute_euler_angles(int16_t *accel_data, int16_t *gyro_data, float *euler_angles) {
    // Compute pitch and roll from accelerometer data
    float ax = accel_data[0] / 16384.0;
    float ay = accel_data[1] / 16384.0;
    float az = accel_data[2] / 16384.0;

    float pitch_accel = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    float roll_accel = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;

    // Get the current time
    int64_t current_time = esp_timer_get_time();
    float dt = (current_time - last_time) / 1000000.0;  // Calculate delta time in seconds
    last_time = current_time;

    // Integrate gyro data to get angles
    float pitch_gyro_rate = gyro_data[0] * 250.0 / 32768.0;  // Convert to degrees/s
    float roll_gyro_rate = gyro_data[1] * 250.0 / 32768.0;   // Convert to degrees/s
    float yaw_gyro_rate = gyro_data[2] * 250.0 / 32768.0;    // Convert to degrees/s

    float pitch_gyro = euler_angles[0] + pitch_gyro_rate * dt;
    float roll_gyro = euler_angles[1] + roll_gyro_rate * dt;
    float yaw_gyro = euler_angles[2] + yaw_gyro_rate * dt;

    // Apply complementary filter
    euler_angles[0] = ALPHA * pitch_gyro + (1 - ALPHA) * pitch_accel;
    euler_angles[1] = ALPHA * roll_gyro + (1 - ALPHA) * roll_accel;
    euler_angles[2] = yaw_gyro;
}



static void timer_task(void *arg) {
    int16_t accel_data[3];
    int16_t gyro_data[3];
    float euler_angles[3];

    while (1) {
        read_mpu6050_data(accel_data, gyro_data);
        compute_euler_angles(accel_data, gyro_data, euler_angles);

        printf("Pitch: %.2f, Roll: %.2f, Yaw: %.2f \n", euler_angles[0], euler_angles[1], euler_angles[2]);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_init();

    xTaskCreate(timer_task, "timer_task", 2048, NULL, 5, NULL);
}

