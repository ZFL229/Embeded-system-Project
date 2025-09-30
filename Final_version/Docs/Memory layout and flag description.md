# STM32F401RE Memory Layout and Valid Flag

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

