# Bimanual-Tele-Rehabilitation
STM32-based bimanual rehabilitation robot for post-stroke patients featuring sensorless torque estimation, dual exercise modes, and real-time monitoring via ESP32 web interface. Award-winning project with custom DAC libraries. Using a bilateral controller with Disturbance Observer (DOB) and Reaction Torque Observer (RTOB) for simultaneus arm movement.

## Project Description

This repository contains the complete codebase and documentation for a sensorless bimanual tele-rehabilitation robot designed for hemiparetic post-stroke patients. The system uses an STM32F746ZG microcontroller with custom control algorithms to enable effective rehabilitation exercises.

### Key Features
- **Sensorless torque estimation** using Disturbance Observer (DOB) and Reaction Torque Observer (RTOB)
- **Dual exercise modes** in a single device: Flexion/Extension and Internal/External Rotation
- **Ergonomic design** that maintains a relaxed shoulder position during therapy
- **Real-time monitoring** via WiFi-connected web interface using ESP32
- **Adaptive resistance** based on patient-specific torque profiles
- **Custom DAC module library** developed for precise motor control

### Technical Components
- STM32F746ZG microcontroller with RTOS implementation
- Maxon 20W BLDC motors with ESCON motor drivers
- Custom PCB design with optocoupler protection circuits
- I2C-based communication between microcontroller and peripherals
- Web-based user interface for data visualization and parameter adjustment

This award-winning project (Best Poster at ERU Symposium 2023, Best Paper at MERCON 2024) demonstrates effective robotic-assisted therapy techniques for stroke rehabilitation.
