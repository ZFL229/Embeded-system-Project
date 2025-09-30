### Interrupted Transfer Test (log_abort.txt)

- **Purpose**: Verify that the bootloader does not accept incomplete firmware when the transfer is interrupted.  
- **Procedure**:  
  1. Start programming `AppProject.bin`.  
  2. Disconnect the USB cable during the transfer (~50–70% progress).  
  3. Reconnect the board; save the script output to `log_abort.txt`.  
- **Expected Result**:  
  - Host reports a transfer error (e.g., write timeout).  
  - No `CRC_CMP` or “flag written” message is printed.  
  - The valid-flag (`0xA5A5A5A5`) is not written.  
  - After reset, the board stays in the bootloader and does not jump to the App.  
- **Actual Result**:  
  - `log_abort.txt` shows `ERROR: Write timeout`.  
  - Device remained in the bootloader after reconnecting.  
  - A subsequent full reprogram with a valid image succeeded, confirming recoverability.

