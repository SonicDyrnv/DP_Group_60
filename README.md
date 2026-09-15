# Streetlight Fault Detection & Ambulance Alerting System

An off-grid IoT system designed to monitor streetlight operations, detect physical and electrical failures in real time, and establish emergency acoustic corridors for ambulances using hybrid LoRa and ESP-NOW protocols.

---

## Overview

Traditional streetlight monitoring relies on internet connectivity or expensive cellular recharges. This project provides a cost-effective alternative that works without internet dependency. It identifies power, bulb, or structural failures at the pole level and integrates an ambulance path-clearing alert system managed from a central headquarters web portal.

---

## Key Features

* **Instant Structural Detection**: Detects pole tilting or falling immediately using 3-axis motion sensors.
* **Electrical & Light Fault Analysis**: Identifies tripped circuit breakers (MCBs), light failures when current is active, and LED flickering.
* **Ambulance Priority Corridor**: Allows HQ operators to send dynamic, targeted acoustic alerts to specific nodes to clear traffic along emergency routes.
* **Hybrid Connectivity**: Uses ESP-NOW for zero-latency node-to-node communication and long-range LoRa for transmission to the central server.


* **Continuous Power Backup**: Features an integrated battery management system recharged directly via streetlight power lines.

---

## Hardware Specifications

| Component | Function | Module Role |
| --- | --- | --- |
| **ESP32 Microcontroller** | Master controller handling LoRa & ESP-NOW stacks | Head Node |
| **ESP8266 Microcontroller** | Branch controller executing local sensing and peer routing | Branch Node |
| **LoRa SX1278 (433 MHz)** | Long-range low-power radio for HQ data link | Head Node |
| **ACS712 Current Sensor** | Measures real-time current draw to detect line flow | Both Nodes |
| **MPU6050 Gyro/Accel** | Monitors pole tilt, orientation changes, and falls | Both Nodes |
| **BH1750 Lux Sensor** | Measures digital light output intensity | Both Nodes |
| **HC-SR04 Ultrasonic** | Traffic and object distance monitoring | Both Nodes |
| **TP4056 Module** | Li-ion battery charging and protection circuit | Power System |

---

## Network Architecture

```text
               +-----------------------------+
               |  Central HQ Web Controller  |
               +--------------+--------------+
                              |
                       (LoRa 433 MHz)[cite: 1]
                              |
                              v
                   +------------------+
                   |    Head Node     | (ESP32)
                   +--------+---------+
                            |
                     (ESP-NOW Protocol)
                            |
           +----------------+----------------+
           v                                 v
+--------------------+             +--------------------+
|   Branch Node 1    | <--ESP-NOW->|   Branch Node 2    | (ESP8266)
+--------------------+             +--------------------+

```

---

## Working Principle

1. **Self-Diagnostic Loop**: Every 30 seconds, nodes read sensor inputs to evaluate power, illumination, and circuit status.
2. **Priority Interrupts**: Pole fall events bypass normal timer cycles and immediately trigger critical alarm frames.
3. **Emergency Routing**: HQ dispatches target messages formatted with `HeadNode_ID` and `BranchNode_ID`. If IDs match, local buzzers activate for a software-defined time interval.
4. **Data Relay**: Branch nodes hop messages via ESP-NOW until reaching the Head Node, which transmits packet logs to HQ over LoRa.

---

## Hardware Assembly & Manufacturing

* **Printed Circuit Board (PCB)**: Layout designed with isolated sensor tracks and centralized MCU placement to prevent signal attenuation.
* **Enclosure**: Built from 4mm laser-cut acrylic with a transparent top plate allowing unrestricted light intake for the BH1750 sensor.

---

## Authors

* **Team Members**: Rathod Dhruv Shailesh, Mayank Garg, Amey Shashikant Ballal, Avanish Kumar Patel, Divya Jain, Dilip Prajapat, Parth Vadhel.
