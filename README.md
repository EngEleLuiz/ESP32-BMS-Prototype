# ESP32 Real-Time Battery Management System (BMS) Prototype

A high-performance, low-level C++ implementation of a Battery Management System for 3S Li-Ion packs, simulated using the Wokwi environment.

## 🚀 Features
- **Real-Time Task Scheduling**: Non-blocking architecture using `millis()` for deterministic execution.
- **3S Multi-Cell Monitoring**: Individual cell voltage tracking.
- **Passive Cell Balancing**: Bleeder resistor simulation to equalize cell voltages (20mV threshold).
- **State of Charge (SoC)**: Integrated Coulomb Counting algorithm for precise capacity estimation.
- **Safety Protection Layer**: 
  - Over-Voltage Protection (OVP)
  - Under-Voltage Protection (UVP)
  - Over-Temperature Protection (OTP)
- **Hardware Abstraction**: Virtual Relay control and I2C LCD telemetry.

## 🛠️ Technical Stack
- **Language**: C++ (Low-level / Pure C style)
- **Platform**: ESP32
- **Simulation**: [Wokwi](https://wokwi.com)
- **Protocol**: I2C for LCD display

## 📊 Logic Breakdown
### State of Charge (SoC)
The system uses Coulomb Counting to integrate current over time:
$$SoC(t) = SoC(t_0) - \frac{\int I dt}{Capacity}$$

### Passive Balancing
When $V_{max} - V_{min} > 20mV$, the system activates a virtual bleeder resistor on the highest cell until equilibrium is reached.

## 🔌 Simulation Setup
1. Copy the code from `src/` to a Wokwi project.
2. Use the `simulation/diagram.json` to auto-configure the wiring.
3. Install the `LiquidCrystal I2C` library in the Wokwi Library Manager.

## 📜 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
