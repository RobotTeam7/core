# Firmware for Midnight Rambler and Fiddler

This repository contains the firmware for two autonomous robots, named Midnight Rambler and Fiddler, built in a team of four by Rylan Stutters, Cohen Weston, Felix Toft and Joshua Riefman at UBC for the 2024 Engineering Physics robot competition (ENPH253).

If you have time, please read about the overview section first, or if you're in a hurry feel free to skip ahead to just a discussion of the firmware we wrote and this repository. 

## Table of Contents

1. [Competition Overview](#competition-overview)
2. [Firmware](#firmware)

## Competition Overview

The difficulty of the challenge was enormously augmented this year with the requireent to build two robots instead of one in just a six week period, almost entirely from scratch. The competition this year was to design two robots to autonomously and collaboratively to play a real-life version of the game Overcooked: robots were required to find ingredients, grab them, cook them, and assemble them into burgers and other menu items — all autonomously with only two minutes — which required fantastic precision in localization, control, and perception (with the twist of inter-robot communication!). Read the [rules here](https://docs.google.com/document/d/1Vxu5wNgwj39HadtlyTthznHr0H7n1BYksZ4to-ojEKY/edit?tab=t.0).

![Fiddler and Midnight Rambler](/media/together.jpeg)
![Midnight Rambler](/media/midnight_rambler.jpeg)

The body of our robots were laser cut wood with a water-jetted aluminum structure, and many other functional elements 3D printed for rapid iteration. We designed and brought up our own PCBs after prototyping on breadboards, and each robot featured two ESP32 Pico32 microcontrollers each unique firmware to maximize performance. 

Omniwheels coupled to DC motors provided omnidirectional drive control, and various servo motors provided millimeter-accurate fine manipulation capabilities, with limit switches and IR sensors for localization and ingredient detection.

The result was two robots with incredibly tight control able to localize and home into desired locations with sub-centimeter accuracy while remaining fast, only needing a few seconds to cross the competition surface. We achieved precise handling of ingredients, able to pick up and place ingredients to assemble menu items with a 100% success rate, as well as transport completed menu items across the competition surface to the serving station without spontaneously disassembly or anything being dropped.

## Firmware

Our design required more pins for digital and analog I/O than a single ESP32 Pico32 provided, and so we opted to use two for each robot. Early on, we identified that we should separate the responsibilities of each microcontroller such that one (which we just called `main`) would handle the robot's state, objective, object manipulation, and inter-robot communication whilst the second (called `motion`) would solely be responsible for motor control and localization, acting on high-level orders from `main` such as "goto tomato ingredient station". This required communication (we used UART) for between the two microcontrollers on each robot, and inter-robot communication for coordination between robots.

Despite the `.cpp` file extensions, we primarily used C, with only a handful of C++ usages to make calls into the Arduino framework. Additionally, we used FreeRTOS at the core of our firmware to allow for careful orchestration of simultaneous tasks.
Each microcontroller required unique (but closely shared) firmware, and so we used PlatformIO with build flags to easily build and flash firmware for separate microcontrollers in the same repository. 

Generally, the documentation for functions will be contained in the definition `.h` and not the implementation `.cpp` files.

We separated our firmware into four modules.
1. [`common`](#common) for shared functionality for all microcontrollers and hardware abstractions
2. [`communication`](#communication) for inter-robot and intra-robot communication protocols
3. [`motion`](#motion) for drive control related functionality and state management
4. [`robot`](#robot) with high-level abstractions and competition strategy

### `common`

The `common` module contained most of the functionality that would be used by all four microcontrollers.
1. `hal.cpp`: our custom hardware abstraction layer which abstracts away all of the physical components such as drive motors, servo motors, stepper motors, IR sensors, and limit switches allowing for high-level setup and control.
2. `pwm.cpp`: an abstraction over the ESP32's Pulse Width Modulation digital IO with careful care to hardware timers (which the Arduino framework didn't do carefully enough for our use case)
3. `utils.cpp`: a handful of utility functions, mostly for logging over USB serial

### `communication`

The `communication` module contains intra-robot communication routines using UART and inter-robot communication routines using ESP-NOW using a common communication protocol designed for the high EMI environment with the necessary error checking and retry procedures to ensure 100% reliable communication.

1. `communication.cpp`: routines for encoding and decoding commands into our custom communication protocol according to definitions in `communication.h`.
2. `uart.cpp`: routines for listening and writing asynchronously to the UART bus with FreeRTOS, as well as initializing the hardware with the ESP32 framework.
3. `wifi.cpp`: routines for listening and writing asynchronously using ESP-NOW with FreeRTOS, as well as initializing the hardware with the ESP32 framework.

### `motion`

The `motion` module contains all functionality for controlling drive control and localization.

1. `main.cpp`: the entrypoint for both `motion` boards with FreeRTOS tasks to manage state
2. `motion.cpp`: abstractions for controlling all robot wheels at once ("drive forwards at speed 10000", "rotate", "drive left at speed 4000")
3. `state.cpp`: defaults (initial conditions)
4. `tasks.cpp`: high-level abstractions as FreeRTOS tasks to perform actions such as "go to lettuce ingredient station" and "swap to other counter" involving multiple steps and coordination with sensory data.
5. `utils.cpp`: some utilities for interpreting localization state and checking device flags 

### `robot`

The `robot` module contains the entrypoints for the `main` controllers for each robot, as well as their competition strategies and procedures.

1. `robot.cpp`: implementations of communication procedures to adhere to the retry and error checking requirements
2. `midnight_rambler.cpp`: entrypoint and competition strategy for Midnight Rambler
3. `fiddler.cpp`: entrypoint and competition strategy for Fiddler
