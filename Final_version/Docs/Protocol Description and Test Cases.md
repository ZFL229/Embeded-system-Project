# Protocol Description and Test Cases

## Protocol Frame Format

## Command Table
| CMD  | Name        | Function                                |
|------|-------------|-----------------------------------------|
| 0x01 | PING        | Connection check                        |
| 0x10 | ERASE       | Erase App area                          |
| 0x11 | SET_OFFSET  | Set write offset                        |
| 0x12 | WFLASH      | Write Flash in chunks                   |
| 0x13 | CRC_QUERY   | Device returns CRC32                    |
| 0x14 | CRC_CMP     | Compare CRC, return ACK/NACK + write flag |

## Test Matrix
| Scenario     | Operation Steps                   | Expected Result                              | Actual Result | Log File          |
|--------------|-----------------------------------|----------------------------------------------|---------------|-------------------|
| Normal Image | Flash `good.bin`                  | CRC matches, flag written, App runs          | Match         | log_normal.txt    |
| Corrupted    | Flash `bad.bin` + FORCE_BAD_CRC   | CRC mismatch, NACK, stay in Boot             | Match         | log_bad.txt       |
| Interrupted  | Disconnect USB during flashing    | Host error, stay in Boot                     | Match         | log_abort.txt     |
| Timeout      | PC script timeout set to 5 ms     | Timeout error, stay in Boot                  | Match         | log_timeout.txt   |

## Attachments
- `log_normal.txt`  
- `log_bad.txt`  
- `log_abort.txt`  
- `log_timeout.txt`  

