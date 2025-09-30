## Firmware Images

This directory contains two application firmware images used for bootloader testing:

- **AppProject.bin**  
  The valid firmware image generated from the application build process in **STM32CubeIDE**.  
  - Purpose: Used to verify correct bootloader behavior with a proper application.  
  - Generated via: STM32CubeIDE build output.

- **AppProject_bad.bin**  
  A corrupted firmware image intentionally generated in the **terminal** to simulate transmission errors or flash corruption.  
  - Purpose: Used to test bootloader fault handling, such as CRC mismatch or invalid application detection.  
  - Generated via: custom terminal command for corruption testing.

> Together, these files are essential for validating the bootloader’s ability to distinguish between valid and invalid application images.

