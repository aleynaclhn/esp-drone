/*
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ESP-Drone MPU9250 Entegrasyonu
 * ESP32 ile MPU9250 9-eksen IMU sensörü için drone konfigürasyonu
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "stm32_legacy.h"
#include "platform.h"
#include "system.h"
#define DEBUG_MODULE "APP_MAIN"
#include "debug_cf.h"

// MPU9250 I2C Ayarları
#define I2C_MASTER_SCL_IO           22      // GPIO SCL
#define I2C_MASTER_SDA_IO           21      // GPIO SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz
#define MPU9250_ADDR                0x68    // MPU9250 I2C Adresi (AD0=LOW)

// MPU9250 Register Adresleri
#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_CONFIG              0x1A
#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG2       0x1D
#define MPU9250_ACCEL_XOUT_H        0x3B
#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_WHO_AM_I            0x75
#define MPU9250_MAG_ADDR            0x0C    // AK8963 Magnetometre
#define MPU9250_USER_CTRL           0x6A
#define MPU9250_INT_PIN_CFG         0x37

// Magnetometre (AK8963) Registerleri
#define AK8963_WHO_AM_I             0x00
#define AK8963_CNTL                 0x0A
#define AK8963_XOUT_L               0x03

static const char *TAG = "MPU9250";

// Sensör Veri Yapısı
typedef struct {
    float accel_x, accel_y, accel_z;  // Accelerometre (m/s²)
    float gyro_x, gyro_y, gyro_z;     // Gyroscope (°/s)
    float mag_x, mag_y, mag_z;        // Magnetometre (μT)
    float temp;                        // Sıcaklık (°C)
    float roll, pitch, yaw;           // Euler Açıları
} mpu9250_data_t;

mpu9250_data_t sensor_data = {0};

// I2C Okuma Fonksiyonu
esp_err_t mpu9250_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C Yazma Fonksiyonu
esp_err_t mpu9250_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C Bus Başlatma
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "I2C başlatıldı");
}

// MPU9250 Başlatma
esp_err_t mpu9250_init(void) {
    uint8_t who_am_i;
    
    // WHO_AM_I kontrolü
    mpu9250_read_reg(MPU9250_ADDR, MPU9250_WHO_AM_I, &who_am_i, 1);
    if (who_am_i != 0x71 && who_am_i != 0x73) {
        ESP_LOGE(TAG, "MPU9250 bulunamadı! WHO_AM_I: 0x%02X", who_am_i);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU9250 tespit edildi! WHO_AM_I: 0x%02X", who_am_i);
    
    // Sensör Sıfırlama ve Başlatma
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x80);  // Reset
    vTaskDelay(100 / portTICK_PERIOD_MS);
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);  // Clock source
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Gyroscope: ±2000 °/s
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x18);
    
    // Accelerometer: ±16g
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG, 0x18);
    
    // DLPF ayarı: 184Hz bandwidth
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_CONFIG, 0x01);
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_ACCEL_CONFIG2, 0x01);
    
    // Magnetometre için I2C bypass aktif
    mpu9250_write_reg(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x02);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // AK8963 Magnetometre başlatma
    uint8_t mag_who_am_i;
    mpu9250_read_reg(MPU9250_MAG_ADDR, AK8963_WHO_AM_I, &mag_who_am_i, 1);
    if (mag_who_am_i == 0x48) {
        ESP_LOGI(TAG, "AK8963 Magnetometre tespit edildi!");
        // Continuous measurement mode (16-bit)
        mpu9250_write_reg(MPU9250_MAG_ADDR, AK8963_CNTL, 0x16);
    } else {
        ESP_LOGW(TAG, "AK8963 bulunamadı: 0x%02X", mag_who_am_i);
    }
    
    ESP_LOGI(TAG, "MPU9250 başarıyla başlatıldı!");
    return ESP_OK;
}

// Accel/Gyro Verilerini Okuma
void mpu9250_read_accel_gyro(void) {
    uint8_t data[14];
    mpu9250_read_reg(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, data, 14);
    
    // Accelerometre (±16g, 16-bit)
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    
    // Sıcaklık
    int16_t temp = (data[6] << 8) | data[7];
    
    // Gyroscope (±2000°/s, 16-bit)
    int16_t gx = (data[8] << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];
    
    // Ölçeklendirme
    sensor_data.accel_x = ax / 2048.0f * 9.81f;  // m/s²
    sensor_data.accel_y = ay / 2048.0f * 9.81f;
    sensor_data.accel_z = az / 2048.0f * 9.81f;
    
    sensor_data.gyro_x = gx / 16.4f;  // °/s
    sensor_data.gyro_y = gy / 16.4f;
    sensor_data.gyro_z = gz / 16.4f;
    
    sensor_data.temp = temp / 333.87f + 21.0f;  // °C
}

// Magnetometre Verilerini Okuma
void mpu9250_read_mag(void) {
    uint8_t data[7];
    mpu9250_read_reg(MPU9250_MAG_ADDR, AK8963_XOUT_L, data, 7);
    
    // Magnetometre (16-bit)
    int16_t mx = (data[1] << 8) | data[0];
    int16_t my = (data[3] << 8) | data[2];
    int16_t mz = (data[5] << 8) | data[4];
    
    // μT'ye çevirme (±4800 μT)
    sensor_data.mag_x = mx * 0.15f;
    sensor_data.mag_y = my * 0.15f;
    sensor_data.mag_z = mz * 0.15f;
}

// Euler Açılarını Hesaplama (Basit Complementary Filter)
void calculate_euler_angles(void) {
    // Roll ve Pitch (Accelerometer'den)
    sensor_data.roll = atan2f(sensor_data.accel_y, sensor_data.accel_z) * 180.0f / M_PI;
    sensor_data.pitch = atan2f(-sensor_data.accel_x, 
                               sqrtf(sensor_data.accel_y * sensor_data.accel_y + 
                                    sensor_data.accel_z * sensor_data.accel_z)) * 180.0f / M_PI;
    
    // Yaw (Magnetometer'den - tilt kompanzasyonlu)
    float cos_roll = cosf(sensor_data.roll * M_PI / 180.0f);
    float sin_roll = sinf(sensor_data.roll * M_PI / 180.0f);
    float cos_pitch = cosf(sensor_data.pitch * M_PI / 180.0f);
    float sin_pitch = sinf(sensor_data.pitch * M_PI / 180.0f);
    
    float mag_x = sensor_data.mag_x * cos_pitch + 
                  sensor_data.mag_z * sin_pitch;
    float mag_y = sensor_data.mag_x * sin_roll * sin_pitch + 
                  sensor_data.mag_y * cos_roll - 
                  sensor_data.mag_z * sin_roll * cos_pitch;
    
    sensor_data.yaw = atan2f(-mag_y, mag_x) * 180.0f / M_PI;
}

// Ana Sensör Okuma Task'ı
void mpu9250_task(void *pvParameters) {
    while (1) {
        // Accel/Gyro okuma
        mpu9250_read_accel_gyro();
        
        // Magnetometre okuma (daha düşük frekansta)
        static int mag_counter = 0;
        if (++mag_counter >= 10) {  // Her 10 döngüde 1
            mpu9250_read_mag();
            mag_counter = 0;
        }
        
        // Euler açılarını hesapla
        calculate_euler_angles();
        
        // Verileri yazdır
        ESP_LOGI(TAG, "Accel: X=%.2f Y=%.2f Z=%.2f m/s²", 
                 sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z);
        ESP_LOGI(TAG, "Gyro: X=%.2f Y=%.2f Z=%.2f °/s", 
                 sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
        ESP_LOGI(TAG, "Mag: X=%.2f Y=%.2f Z=%.2f μT", 
                 sensor_data.mag_x, sensor_data.mag_y, sensor_data.mag_z);
        ESP_LOGI(TAG, "Euler: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°", 
                 sensor_data.roll, sensor_data.pitch, sensor_data.yaw);
        ESP_LOGI(TAG, "Sıcaklık: %.1f°C\n", sensor_data.temp);
        
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 10Hz güncelleme
    }
}

void app_main()
{
    /*
     * Initialize the platform and Launch the system task
     * app_main will initialize and start everything
     */

    /* initialize nvs flash prepare for Wi-Fi */
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize the platform. */
    if (platformInit() == false) {
        while (1); // if firmware is running on the wrong hardware, Halt
    }

    /* MPU9250 başlat */
    ESP_LOGI(TAG, "ESP-Drone MPU9250 başlatılıyor...");
    i2c_master_init();
    
    if (mpu9250_init() == ESP_OK) {
        // Sensör okuma task'ını oluştur
        xTaskCreate(mpu9250_task, "mpu9250_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "MPU9250 başlatılamadı!");
    }

    /* launch the system task */
    systemLaunch();
}
