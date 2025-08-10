#ifndef AS5600_H
#define AS5600_H

#include "driver/i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define AS5600_DEFAULT_ADDR 0x36
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_RAW_ANGLE_MSB 0x0E

typedef struct
{
    i2c_port_t i2c_port;
    uint8_t address;
    float alpha;
    float deadband;
    float scale_factor;
    int8_t direction;
    float raw_angle;
    float position;
    int64_t last_time_us;
    float velocity;
    float zero_offset;
    bool enable_nvs;
    char tag[16];
    float offset;
} as5600_t;

bool as5600_init(as5600_t *as5600, i2c_port_t i2c_port, uint8_t address, float alpha, float deadband, float scale_factor, bool invert_direction, bool enable_nvs);
void as5600_update(as5600_t *as5600);
void as5600_set_position(as5600_t *as5600, float angle);
float as5600_get_position(const as5600_t *as5600);
float as5600_get_velocity(const as5600_t *as5600);
bool as5600_magnet_detected(const as5600_t *as5600);

bool as5600_load_zero_offset(as5600_t *as5600);
bool as5600_save_zero_offset(const as5600_t *as5600);
void as5600_set_zero(as5600_t *as5600);

#endif // AS5600_H
