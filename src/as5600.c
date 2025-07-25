#include "as5600.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static const char *TAG = "as5600";
static const char *NVS_NAMESPACE = "as5600";
static const char *NVS_KEY_OFFSET = "offset";

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
    float angle = (raw * 2.0f * M_PI) / 4096.0f; // Round to actual encoder resolution
    return roundf(angle * 1000.0f) / 1000.0f;
}

static float offset = 0.0f;

static esp_err_t save_offset_to_nvs(float value)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return err;
    }
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    err = nvs_set_u32(nvs_handle, NVS_KEY_OFFSET, raw);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS set offset failed: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
    return err;
}

static esp_err_t load_offset_from_nvs(float *value)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "NVS namespace not found, no offset saved yet");
        *value = 0.0f;
        return ESP_OK; // Treat as success with default offset
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS open failed: %s", esp_err_to_name(err));
        *value = 0.0f;
        return err; // Other errors propagate
    }

    uint32_t raw = 0;
    err = nvs_get_u32(nvs_handle, NVS_KEY_OFFSET, &raw);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No saved offset found");
        *value = 0.0f;
        err = ESP_OK; // no saved offset, but still success
    }
    else if (err == ESP_OK)
    {
        memcpy(value, &raw, sizeof(float));
    }
    else
    {
        ESP_LOGE(TAG, "NVS get offset failed: %s", esp_err_to_name(err));
    }
    nvs_close(nvs_handle);
    return err;
}

bool as5600_init(as5600_t *as5600, i2c_port_t i2c_port, uint8_t address, float alpha, float deadband, float scale_factor, int8_t direction)
{
    as5600->i2c_port = i2c_port;
    as5600->address = address;
    as5600->alpha = alpha;
    as5600->deadband = deadband;
    as5600->scale_factor = scale_factor;
    as5600->direction = (direction >= 0) ? 1 : -1;
    as5600->velocity = 0;

    ESP_LOGI(TAG, "Initializing AS5600: addr=0x%02X, alpha=%.3f, deadband=%.5f, scale=%.3f, direction=%d",
             address, alpha, deadband, scale_factor, as5600->direction);

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

    float raw_angle = get_raw_angle(as5600);
    float signed_angle = (raw_angle > M_PI) ? (raw_angle - 2.0f * M_PI) : raw_angle;

    // Load saved offset from NVS
    if (load_offset_from_nvs(&offset) != ESP_OK)
    {
        offset = 0.0f;
    }
    ESP_LOGI(TAG, "Loaded offset from NVS: %.6f rad", offset);

    // Apply offset to initial position
    float corrected_angle = signed_angle - offset;
    if (corrected_angle > M_PI)
        corrected_angle -= 2.0f * M_PI;
    else if (corrected_angle < -M_PI)
        corrected_angle += 2.0f * M_PI;

    as5600->raw_angle = raw_angle;
    as5600->position = corrected_angle * as5600->scale_factor * as5600->direction;
    as5600->last_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "AS5600 init done. Raw angle: %.6f rad, signed: %.6f rad, corrected pos: %.6f (scaled)",
             raw_angle, signed_angle, as5600->position);

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

    delta *= as5600->direction;

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
    // Save offset = current raw_angle - desired zero position (in radians)
    float raw_angle = get_raw_angle(as5600);
    offset = raw_angle - angle;

    // Wrap offset to [-pi, pi]
    if (offset > M_PI)
        offset -= 2.0f * M_PI;
    else if (offset < -M_PI)
        offset += 2.0f * M_PI;

    // Save to NVS
    esp_err_t err = save_offset_to_nvs(offset);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Saved offset %.6f rad to NVS", offset);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to save offset to NVS");
    }

    // Update position with new offset applied
    as5600->position = angle * as5600->scale_factor;
    as5600->raw_angle = raw_angle;

    ESP_LOGI(TAG, "Set position to %.6f rad (scaled: %.6f), raw_angle: %.6f", angle, as5600->position, raw_angle);
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
