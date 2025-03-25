# Inverted Pendulum Arduino

This repository contains two Arduino-based control programs used in the inverted pendulum system for COMP0216 :

- `Stabilisation_Code/`: Code responsible for stabilizing the pendulum using different control strategies.
- `Travel_Code/`: Code designed for evaluating system performance during linear motion (forward travel).

---

##  1. Stabilisation Code

The `Stabilising_Code` folder contains the balancing code. This code stabilises the system from disturbances and takes the pendulum to start in the upright poisitono. 
- The files are split into tow and `stabilising_code.ino` handles sensor readings, actuation, and control flow.
- The actual control logic (PID, LQR, Pole Placement) is done via `controller.ino`.


### Dependencies

- `RotaryEncoder` library.
- **Important:** This project uses a *modified version* of the `RotaryEncoder` library for improved angle measurements.

To use this code:
1. Unzip the contents of the `RotaryEncoder/` folder into your local Arduino `libraries` directory.

---

##  2. Travel Code

In the `Travel_code` folder, this sketch was used in the second evaluation phase to control the cart's forward motion while keeping the pendulum balanced.

The travel code builds on the stabilisation system, with additional logic for driving the cart forward a fixed distance while maintaining upright stability. The main change is implememnting a PID control on the position of the cart. The controller function additionally takes another variable of the `target_distance`. 

---

##  Hardware

This code is intended to be used with a 4 wheeled cart controlled by and Arduino Uno R3 and 2 dfmotors L298P motor shields. 

The configuration of the system can be seen in the image below:

![Arduino and motor shield stack](Images/stack_systems.jpg){: style="width:300px" padding-left="100px"}
![Pinout](Images/pins_systems.jpg){: style="width:300px"}

The cart has 4 motors, each has their own encoder, however only 1 motor encoder needs to be connected to the arduino on pins 8 and 9. To power the encoder on the motor we connected the motor encoder to the 5v pin and ground on the boards. Then the optical encoder is attached to the pivot point needs to be connected to the pins 2 and 3 as these are the interupt pins. Which is powered by the 3.3V pin and is grounded by the arduino board. 

---

## 📌 Notes

- Ensure your hardware setup (motor driver, sensors, encoder) matches the image in the ahrdware section.
- The controller selection in `controller.ino` can be switched by toggling flags at the beginning of the file.
- This codebase is modular to support rapid tuning and debugging.



