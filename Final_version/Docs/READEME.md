# Bootloader Documentation

This repository contains documentation and test materials for the STM32 bootloader project.  
The files describe the memory layout, boot process, protocol specification, and test cases used during development.

## Contents

- **Boot process (Boot→App).md**  
  Explains the transition process from the bootloader to the application, including initialization and jump procedures.

- **Memory layout and flag description.md**  
  Provides detailed information about the STM32F401RE Flash memory map and the reserved valid-flag mechanism.  
  Used to define how the bootloader verifies and locates the application image.

- **Protocol Description and Test Cases.md**  
  Describes the communication protocol between host and bootloader.  
  Includes supported commands and the corresponding test cases.

- **README.md**  
  General introduction and navigation file for this directory.

## Purpose

The documentation here is intended to support:
1. Understanding the design of the custom bootloader.  
2. Testing and validating the bootloader behavior.  
3. Providing references for further development and debugging.

---

For firmware images (`AppProject.bin` and `AppProject_bad.bin`), please check the corresponding folder for details.

