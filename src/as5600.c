#include "as5600.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "as5600";

static esp_err_t i2c_read_reg(const as5600_t *as5600, uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (as5600->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (as5600->address << 1) | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(as5600->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t read_8_bit(const as5600_t *as5600, uint8_t reg)
{
    uint8_t b;
    if (i2c_read_reg(as5600, reg, &b, 1) != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read_8_bit failed");
        return 0xFF;
    }
    return b;
}

static uint16_t read_12_bit(const as5600_t *as5600, uint8_t reg)
{
    uint8_t buf[2];
    if (i2c_read_reg(as5600, reg, buf, 2) != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read_12_bit failed");
        return 0;
    }
    return ((uint16_t)buf[0] << 8 | buf[1]) & 0x0FFF;
}

static float get_raw_angle(as5600_t *as5600)
{
    uint16_t raw = read_12_bit(as5600, AS5600_REG_RAW_ANGLE_MSB);
    return (raw * 2.0f * M_PI) / 4096.0f;
}

bool as5600_init(as5600_t *as5600, i2c_port_t i2c_port, uint8_t address, float alpha, float deadband, float scale_factor)
{
    as5600->i2c_port = i2c_port;
    as5600->address = address;
    as5600->alpha = alpha;
    as5600->deadband = deadband;
    as5600->scale_factor = scale_factor;
    as5600->position = 0;
    as5600->velocity = 0;

    ESP_LOGI(TAG, "Initializing AS5600: addr=0x%02X, alpha=%.3f, deadband=%.5f, scale=%.3f", address, alpha, deadband, scale_factor);

    uint8_t status = read_8_bit(as5600, AS5600_REG_STATUS);
    if (status == 0xFF)
    {
        ESP_LOGE(TAG, "AS5600 I2C read failed (status 0xFF)");
        return false;
    }
    if ((status & (1 << 5)) == 0)
    {
        ESP_LOGE(TAG, "AS5600 magnet not detected (status=0x%02X)", status);
        return false;
    }

    ESP_LOGI(TAG, "AS5600 magnet detected (status=0x%02X)", status);

    as5600->raw_angle = get_raw_angle(as5600);
    as5600->last_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "AS5600 init done. Initial angle: %.6f rad", as5600->raw_angle);

    return true;
}

void as5600_update(as5600_t *as5600)
{
    float current = get_raw_angle(as5600);
    int64_t now_us = esp_timer_get_time();
    float delta = current - as5600->raw_angle;

    if (delta > M_PI)
        delta -= 2.0f * M_PI;
    else if (delta < -M_PI)
        delta += 2.0f * M_PI;

    as5600->position += delta;

    float dt = (now_us - as5600->last_time_us) / 1e6f;
    if (dt > 0)
    {
        float raw_velocity = delta / dt;

        if (fabsf(raw_velocity) < as5600->deadband)
            raw_velocity = 0.0f;

        as5600->velocity = as5600->alpha * raw_velocity + (1.0f - as5600->alpha) * as5600->velocity;
    }

    as5600->raw_angle = current;
    as5600->last_time_us = now_us;
}

void as5600_set_position(as5600_t *as5600, float angle)
{
    as5600->position = angle * as5600->scale_factor;
    as5600->raw_angle = get_raw_angle(as5600);
    ESP_LOGI(TAG, "Set position to %.6f rad (scaled: %.6f)", angle, as5600->position);
}

float as5600_get_position(const as5600_t *as5600)
{
    return as5600->position / as5600->scale_factor;
}

float as5600_get_velocity(const as5600_t *as5600)
{
    return as5600->velocity / as5600->scale_factor;
}

bool as5600_magnet_detected(const as5600_t *as5600)
{
    uint8_t status = read_8_bit(as5600, AS5600_REG_STATUS);
    return (status & (1 << 5)) != 0;
}