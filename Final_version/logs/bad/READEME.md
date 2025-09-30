### Corrupted Image Test (log_bad.txt)

- **Purpose**: Verify that the bootloader’s CRC validation correctly rejects a corrupted firmware image.  
- **Procedure**:  
  1. Generate `AppProject_bad.bin` by flipping one byte in the valid image or by forcing a CRC mismatch.  
  2. Run the upgrade script with CRC corruption enabled:  
     ```powershell
     set FORCE_BAD_CRC=1
     python bl_test.py --port COM3 --bin AppProject_bad.bin > logs/log_bad.txt 2>&1
     ```
- **Expected Result**:  
  - CRC mismatch between host and device → NACK response.  
  - Log output similar to:  
    ```
    [TEST] Host CRC has been corrupted for mismatch test
    DEV_CRC=0x70F5CA83 / HOST_CRC=0x70F5CA82
    CRC_CMP: MISMATCH
    UPGRADE: FAIL
    ```
  - Bootloader does **not** write the valid-flag (`0xA5A5A5A5`).  
  - After reset, the board stays in the bootloader and does not jump to the App.  

- **Actual Result**:  
  - `log_bad.txt` matches the expectation, confirming that the bootloader safely prevents execution of corrupted firmware.

