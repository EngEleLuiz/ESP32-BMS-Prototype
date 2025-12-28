#ifndef BMS_STRUCTS_H
#define BMS_STRUCTS_H

typedef struct {
    float voltage[3];
    float current_a;
    float temp_c;
    bool balancing_active[3];
} BatteryPack_t;

typedef struct {
    float soc_percent;
    float charge_accumulated_mah;
    bool fault_ovp;
    bool fault_uvp;
    bool fault_otp;
    bool relay_state;
} BMS_State_t;

#endif
