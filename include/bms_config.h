#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

// Pin Mapping
#define RELAY_PIN       18
#define LCD_ADDR        0x27

// Battery Specs (3S)
#define NUM_CELLS           3
#define BAT_CAPACITY_MAH    2500.0f
#define SIM_LOAD_CURRENT_MA 500.0f
#define DT_SEC              0.1f

// Thresholds
#define OVP_THRESH      4.25f
#define UVP_THRESH      2.90f
#define OTP_THRESH      45.0f
#define HYSTERESIS      0.1f
#define BALANCE_THRESHOLD 0.02f
#define BALANCING_CURRENT 0.05f

#endif
