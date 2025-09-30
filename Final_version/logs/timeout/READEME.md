### Timeout Test (log_timeout.txt)

- **Purpose**: Verify that the bootloader safely handles host or device communication timeouts without writing the valid-flag.  
- **Procedure**:  
  1. Modify the PC script to use a very short read/write timeout (e.g., 5 ms).  
  2. Run the upgrade process with a valid `AppProject.bin`.  
  3. Save the script output to `log_timeout.txt`.  
- **Expected Result**:  
  - Host reports a timeout (e.g., `read 0/1 bytes`).  
  - No `CRC_CMP` or “flag written” message appears.  
  - The valid-flag is not written.  
  - After reset, the board remains in the bootloader.  
- **Actual Result**:  
  - `log_timeout.txt` contains `ERROR: read 0/1 bytes`.  
  - The device did not jump to the App and stayed in bootloader mode.  
  - Reprogramming with normal timeout settings succeeded, proving that recovery is possible.

