/*
 * Real-Time BMS Prototype - ESP32
 * Environment: Wokwi Simulator
 * Style: Low-level C++ / C-Style
 */

#include <Arduino.h>
#include <LiquidCrystal_I2C.h> // Requires LiquidCrystal I2C library in Wokwi

// --- Hardware Definitions ---
#define RELAY_PIN       18   // Virtual Relay Control
#define LCD_ADDR        0x27 // Default I2C address
#define LCD_COLS        16
#define LCD_ROWS        2

// --- Battery Constants ---
#define BAT_CAPACITY_MAH    2500.0f
#define SIM_LOAD_CURRENT_MA 500.0f  // Simulated load
#define DT_SEC              0.1f    // Time step for simulation & integration (100ms)

// --- Safety Thresholds ---
#define OVP_THRESH      4.25f // Over Voltage
#define UVP_THRESH      2.90f // Under Voltage
#define OTP_THRESH      45.0f // Over Temperature
#define HYSTERESIS      0.1f  // Recovery buffer

// --- Data Structures ---

// Structure for the Simulated Battery Physics
typedef struct {
  float true_voltage;
  float true_current;
  float true_temp;
  float internal_resistance;
} BatteryPhys_t;

// Structure for the BMS Logic and State
typedef struct {
  float voltage_v;
  float current_a;
  float temp_c;
  float soc_percent;     // 0.0 to 100.0
  float charge_accumulated_mah; 
  bool fault_ovp;
  bool fault_uvp;
  bool fault_otp;
  bool relay_state;      // true = CLOSED (connected), false = OPEN (safe)
} BMS_t;
// ---  3S ---
#define NUM_CELLS 3
#define BALANCE_THRESHOLD 0.02f // 20mV 
#define BALANCING_CURRENT 0.05f  // 50mA 

typedef struct {
    float voltage[NUM_CELLS];
    bool balancing_active[NUM_CELLS]; // bleeder
} BatteryPack_t;

BatteryPack_t pack = {{4.10f, 4.15f, 4.08f}, {false, false, false}}; 

void run_balancing_logic(BatteryPack_t *p) {
    // 1. Encontrar a menor voltagem
    float min_v = p->voltage[0];
    for(int i = 1; i < NUM_CELLS; i++) {
        if(p->voltage[i] < min_v) min_v = p->voltage[i];
    }

    // 2. above threshold
    for(int i = 0; i < NUM_CELLS; i++) {
        if(p->voltage[i] > (min_v + BALANCE_THRESHOLD)) {
            p->balancing_active[i] = true;
            p->voltage[i] -= (BALANCING_CURRENT * 0.001f); 
        } else {
            p->balancing_active[i] = false;
        }
    }
}

// --- Global Instances ---
BatteryPhys_t physModel = {4.20f, 0.0f, 25.0f, 0.05f}; // Init at full charge
BMS_t bms = {0};
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// --- Task Timers ---
unsigned long last_sim_time = 0;
unsigned long last_lcd_time = 0;

// ==========================================
// 1. Battery Simulation Logic (The Physics)
// ==========================================
void simulate_battery_physics(BatteryPhys_t *phys, bool load_connected) {
  // 1. Simulate Discharge Curve (Linear approx for simplicity + internal resistance drop)
  // V_term = V_ocv - (I * R_int)
  // We slowly decrease V_ocv based on current drawn.
  
  float discharge_rate = 0.0005f; // Voltage drop per tick per Amp
  
  if (load_connected && phys->true_voltage > 2.5f) {
      phys->true_current = SIM_LOAD_CURRENT_MA / 1000.0f; // Convert to Amps
      phys->true_voltage -= (discharge_rate * phys->true_current);
      
      // Heat generation: P = I^2 * R
      float power_heat = (phys->true_current * phys->true_current) * phys->internal_resistance;
      phys->true_temp += (power_heat * 0.5f); // Arbitrary temp rise factor
  } else {
      phys->true_current = 0.0f;
      // Cool down logic
      if(phys->true_temp > 25.0f) phys->true_temp -= 0.05f; 
  }

  // 2. Add Gaussian-like Noise to mimic real sensors
  // (using simple rand for demo)
  float v_noise = ((rand() % 20) - 10) / 1000.0f; // +/- 10mV noise
  float i_noise = ((rand() % 10) - 5) / 1000.0f;  // +/- 5mA noise
  float t_noise = ((rand() % 10) - 5) / 10.0f;    // +/- 0.5C noise

  // Update "Sensor" values in BMS struct (ADC Abstraction)
  bms.voltage_v = phys->true_voltage + v_noise;
  bms.current_a = phys->true_current + i_noise;
  bms.temp_c    = phys->true_temp + t_noise;
}

// ==========================================
// 2. SoC Algorithm: Coulomb Counting
// ==========================================
void update_soc_coulomb_counting(BMS_t *sys, float dt_seconds) {
  /* * Formula: SoC(t) = SoC(t0) - (Integral(I * dt) / Q_total)
   * * Initial Calibration Note:
   * Normally, we use OCV (Open Circuit Voltage) lookup table at startup 
   * to estimate initial SoC. Here, we assume we start at known state defined in physModel.
   */

  // Convert current (Amps) * time (Seconds) to mAh
  // 1 Amp-Second = 1/3600 Amp-Hours = 1000/3600 mAh = 0.2777 mAh
  float mah_change = (sys->current_a * dt_seconds) * (1000.0f / 3600.0f);

  // Subtract because we are discharging
  sys->charge_accumulated_mah -= mah_change; 

  // Calculate Percentage
  sys->soc_percent = (sys->charge_accumulated_mah / BAT_CAPACITY_MAH) * 100.0f;

  // Clamp limits
  if (sys->soc_percent > 100.0f) sys->soc_percent = 100.0f;
  if (sys->soc_percent < 0.0f) sys->soc_percent = 0.0f;
}

// ==========================================
// 3. Safety & Protection Layer
// ==========================================
void check_protections(BMS_t *sys) {
  bool fault_detected = false;

  // 1. Over Voltage Protection
  if (sys->voltage_v > OVP_THRESH) sys->fault_ovp = true;
  else if (sys->voltage_v < (OVP_THRESH - HYSTERESIS)) sys->fault_ovp = false;

  // 2. Under Voltage Protection
  if (sys->voltage_v < UVP_THRESH) sys->fault_uvp = true;
  else if (sys->voltage_v > (UVP_THRESH + HYSTERESIS)) sys->fault_uvp = false;

  // 3. Over Temperature Protection
  if (sys->temp_c > OTP_THRESH) sys->fault_otp = true;
  else if (sys->temp_c < (OTP_THRESH - 5.0f)) sys->fault_otp = false;

  // Aggregate Faults
  if (sys->fault_ovp || sys->fault_uvp || sys->fault_otp) {
      fault_detected = true;
  }

  // Actuate Hardware (Virtual Relay)
  // Active LOW relay logic is common, but let's assume HIGH = ON (Closed)
  if (fault_detected) {
      digitalWrite(RELAY_PIN, LOW); // Open relay (Safe)
      sys->relay_state = false;
  } else {
      digitalWrite(RELAY_PIN, HIGH); // Close relay (Connect Load)
      sys->relay_state = true;
  }
}

// ==========================================
// 4. Hardware Abstraction & Display
// ==========================================
void update_display(BMS_t *sys) {
  // Serial Monitor for Debugging
  Serial.printf("V:%.2fV | I:%.3fA | T:%.1fC | SoC:%.1f%% | Relay:%d\n", 
      sys->voltage_v, sys->current_a, sys->temp_c, sys->soc_percent, sys->relay_state);

  // LCD Update
  lcd.setCursor(0, 0);
  lcd.printf("V:%.2f S:%d%%", sys->voltage_v, (int)sys->soc_percent);
  
  lcd.setCursor(0, 1);
  if (!sys->relay_state) {
    if (sys->fault_uvp) lcd.print("FAULT: UVP!     ");
    else if (sys->fault_ovp) lcd.print("FAULT: OVP!     ");
    else if (sys->fault_otp) lcd.print("FAULT: TEMP!    ");
  } else {
    lcd.printf("I:%.2fA T:%.1fC", sys->current_a, sys->temp_c);
  }
}

// ==========================================
// 5. Setup & Main Loop (Scheduler)
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Hardware Init
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Default ON
  
  lcd.init();
  lcd.backlight();
  lcd.print("BMS Init...");
  delay(1000);
  lcd.clear();

  // SoC Initial Calibration
  // Since we know the sim starts at 4.2V, we set SoC to 100%
  // In a real system, you would read OCV here and map it to a Lookup Table.
  bms.charge_accumulated_mah = BAT_CAPACITY_MAH; 
  bms.soc_percent = 100.0f;
  bms.relay_state = true;
}

void loop() {
  unsigned long current_time = millis();

  // Task 1: 10Hz (100ms) - Physics, SoC, Protection
  if (current_time - last_sim_time >= (DT_SEC * 1000)) {
    last_sim_time = current_time;

    // A. Run Physics Simulation (Pass relay state as load state)
    simulate_battery_physics(&physModel, bms.relay_state);

    // B. Run Algorithms
    update_soc_coulomb_counting(&bms, DT_SEC);

    // C. Run Safety Layer
    check_protections(&bms);
  }

  // Task 2: 2Hz (500ms) - Display & Reporting
  if (current_time - last_lcd_time >= 500) {
    last_lcd_time = current_time;
    update_display(&bms);
  }
}
