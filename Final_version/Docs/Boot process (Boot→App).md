# STM32F401RE Memory Layout, Valid Flag, and Boot Process

## Chip Information
- **Model**: STM32F401RE  
- **Flash**: 512 KB (0x08000000 – 0x0807FFFF)  

## Partition Table
| Region    | Start Address | End Address | Description                           |
|-----------|---------------|-------------|---------------------------------------|
| Boot      | 0x08000000    | 0x08003FFF  | Bootloader (16 KB)                    |
| App       | 0x08004000    | 0x0807FFFB  | Application area (excluding last 4 B) |
| ValidFlag | 0x0807FFFC    | 0x0807FFFF  | Valid flag (0xA5A5A5A5)               |

## Valid Flag Description
- **Meaning**: Writing `0xA5A5A5A5` indicates that the App is valid.  
- **CRC Calculation**: The last 4 bytes are excluded to avoid being affected by the flag.  
- **Bootloader Behavior**:  
  - The flag is written only if the CRC check passes.  
  - In abnormal situations (NACK / timeout / interruption), the flag is **not** written.  

---

# Boot Process (Boot → App)

## Flow Steps
1. **Power-on / Reset** → Bootloader runs.  
2. Check Boot entry conditions:  
   - Button pressed.  
   - Valid flag not present.  
   - CRC check failed.  
   - Host command request.  
3. If any condition is met → Stay in Boot and wait for commands.  
4. Otherwise → Disable peripherals and interrupts → Set VTOR → Jump to App.  

## Rollback Mechanism
- **CRC failure** → Boot stops, no jump to App.  
- **Transfer interruption / timeout** → Valid flag is not written, so Boot allows upgrade at next power-on.  
- **Abnormal cases** → Ensure device never runs a corrupted image.  

## State Diagram
```mermaid
flowchart TD
    A[Power-on / Reset] --> B[Boot]
    B -- Check conditions --> C[Stay in Boot (waiting for commands)]
    B --> D[Conditions passed + CRC valid]
    D --> E[Jump to App]
