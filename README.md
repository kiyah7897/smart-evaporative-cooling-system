# smart-evaporative-cooling-system
Arduino-based smart evaporative cooling system with sensor integration and state machine control

# Smart Evaporative Cooling System

## Overview
This project is an Arduino-based smart evaporative cooling system designed to provide
cooling in hot, dry environments. The system monitors temperature, humidity, and water
levels to safely and efficiently control airflow and cooling behavior in real time.

## Hardware Components
- Arduino Mega 2560
- DHT11 Temperature & Humidity Sensor
- Water Level Sensor
- 16x2 LCD Display
- Fan Motor (external power supply)
- Stepper Motor
- Potentiometer and Push Buttons (Start, Stop, Reset)
- Real-Time Clock (RTC) Module
- Status LEDs (Disabled, Idle, Running, Error)

## System Design
The system operates using a finite state machine with four states:
- **Disabled**: System inactive, awaiting start command
- **Idle**: Monitoring environmental conditions
- **Running**: Actively cooling when temperature exceeds threshold
- **Error**: Triggered when water level is too low

State changes are visually indicated using LEDs and logged with timestamps.

## Key Features
- Interrupt-driven user input using ISR
- Real-time environmental monitoring
- Safety checks to prevent unsafe operation
- Stepper motor control for airflow direction
- Timestamped event logging via RTC
- USB serial communication to host computer

## Technologies Used
- C / C++
- Arduino
- Embedded Systems Design
- Finite State Machines
- Interrupt Service Routines
- Serial Communication

## Course Context
Developed as a final project for **CPE 301 – Embedded Systems**.
