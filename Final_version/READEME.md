# Embedded-system-Project (Final Version)

This repository contains the final version of the STM32 bootloader and application project.  
It includes source code, documentation, build artifacts, logs, and test tools used during development and validation.

## Directory Structure

- **AppProject/**  
  Contains the source code and build outputs of the application project.  
  The application is built in **STM32CubeIDE** and produces the main firmware image (`AppProject.bin`).

- **Bootloader/**  
  Contains the custom bootloader implementation for STM32.  
  Responsible for firmware update, validation (CRC/flag check), and transition to the application.

- **Docs/**  
  Documentation related to the bootloader and project design.  
  Includes memory layout, protocol descriptions, test cases, and boot process explanation.

- **artifacts/**  
  Stores generated firmware binaries (e.g., `AppProject.bin`, `AppProject_bad.bin`) and other test images.  
  - `AppProject.bin`: valid image built in STM32CubeIDE.  
  - `AppProject_bad.bin`: corrupted image generated in the terminal for error-handling tests.

- **logs/**  
  Contains logs from bootloader tests, such as timeout tests, corrupted image tests, and protocol communication results.

- **tools/**  
  Python scripts and utilities for bootloader testing (e.g., flashing, corruption tests, timeout handling).

- **README.md**  
  General introduction and navigation file for this final version.

## Purpose

The goal of this project is to:
1. Implement and validate a reliable STM32 bootloader.  
2. Test firmware update and error-handling mechanisms.  
3. Provide clear documentation and reproducible results for academic and project submission purposes.

