### Normal Image Test (log_normal.txt)

- **Purpose**: Verify that the bootloader correctly accepts and runs a valid application image.  
- **Procedure**:  
  1. Start programming the valid `AppProject.bin`.  
  2. Run the upgrade script with normal settings:  
     ```powershell
     python bl_test.py --port COM3 --bin AppProject.bin > logs/log_normal.txt 2>&1
     ```
- **Expected Result**:  
  - Host and device CRC32 values match.  
  - Log output shows:  
    ```
    DEV_CRC=0x70F5CA83 / HOST_CRC=0x70F5CA83
    CRC_CMP: MATCH (flag written)
    UPGRADE: OK
    ```
  - Bootloader writes the valid-flag (`0xA5A5A5A5`).  
  - After reset, the board jumps to the App and runs normally.  
- **Actual Result**:  
  - `log_normal.txt` matches the expectation.  
  - Device successfully booted into the application after reset.

