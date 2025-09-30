### bl_test.py

- **Purpose**:  
  Main host-side integration test script for the UART bootloader.

- **Functions**:  
  - Establishes UART connection (`PING`).  
  - Sends commands to erase the App region, set write offset, and write firmware in chunks.  
  - Queries device CRC32 and compares it against host CRC32.  
  - If they match, instructs the bootloader to write the valid-flag (`0xA5A5A5A5`) and reboot into the App.  
  - Supports both normal upgrade tests and forced CRC mismatch (`FORCE_BAD_CRC=1`) for the corrupted image scenario.

- **Usage**:  
  ```powershell
  # Normal upgrade with a valid image
  python bl_test.py --port COM3 --bin AppProject.bin > logs/log_normal.txt 2>&1

  # Corrupted image test (force CRC mismatch)
  set FORCE_BAD_CRC=1
  python bl_test.py --port COM3 --bin AppProject_bad.bin > logs/log_bad.txt 2>&1

