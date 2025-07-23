# ESP-IDF AS5600 Component

This is an ESP-IDF driver for the AS5600 magnetic rotary encoder using I²C.

## Features

- Read 12-bit angle measurements
- Compute filtered velocity
- Position tracking with wraparound handling
- Detect magnet presence

## Usage

### Clone as a component

```bash
cd your-esp-idf-project
git submodule add https://github.com/andrinwinzap/as5600_espidf_component.git components/as5600
```
