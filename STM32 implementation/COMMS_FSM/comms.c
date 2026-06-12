#include "comms.h"

// ================================================================
// Module state
// ================================================================
static int buffer_size           = 10;
static int packet_id_received    = -1;
static int all_messages_received = 0;

// ================================================================
// UART ring buffer
// ================================================================
RING_BUFFER    buffer_frame  = {0};
static uint8_t current_byte;

void uartRingBufferInitialise(void) {
    HAL_UART_Receive_IT(&huart5, &current_byte, 1);
    buffer_frame.write_ptr = 0;
    buffer_frame.read_ptr  = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        uint16_t next = (buffer_frame.write_ptr + 1) & RING_BUFFER_MASK;
        if (next != buffer_frame.read_ptr) {
            buffer_frame.buffer[buffer_frame.write_ptr] = current_byte;
            buffer_frame.write_ptr = next;
        }
        HAL_UART_Receive_IT(&huart5, &current_byte, 1);
    }
}

int uartAvailable(RING_BUFFER *buffer) {
    return buffer->write_ptr != buffer->read_ptr;
}

uint8_t uartReadByte(RING_BUFFER *buffer) {
    uint8_t b       = buffer->buffer[buffer->read_ptr];
    buffer->read_ptr = (buffer->read_ptr + 1) & RING_BUFFER_MASK;
    return b;
}

// Redirect printf to USART2 debug port
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// ================================================================
// Command table
// ================================================================
const COMMAND_AND_ID COMMAND_ENTRY[] = {
    { "TELEMETRY", TELEMETRY },
    { "PING",      PING      },
    { "PAYLOAD",   PAYLOAD   },
    { "CLEAR",     CLEAR     },
    { "POINT",     COMMAND   },
    { "MEMORY",    MEMORY    },
    { "DUMMYDATA", DUMMYDATA },
    { "MODEGET",   MODEGET   },
    { "MODESET",   MODESET   }
};
const int COMMAND_ENTRY_SIZE = sizeof(COMMAND_ENTRY) / sizeof(COMMAND_ENTRY[0]);

static const char *VALID_MODES[] = {
    "NOMINAL","DETUMBLE","SAFE","POINTING","SCIENCE","DOWNLINK","UPLINK"
};
static const int VALID_MODES_COUNT = 7;

int validCommand(const char *command) {
    char *sp = strchr(command, ' ');
    char  first[60] = {0};

    if (sp) {
        int len = sp - command;
        strncpy(first, command, len);
        first[len] = '\0';
    } else {
        strncpy(first, command, sizeof(first) - 1);
    }

    for (int i = 0; i < COMMAND_ENTRY_SIZE; i++) {
        if (strcmp(COMMAND_ENTRY[i].name, first) != 0) continue;

        if (i == 8) {   // MODESET — requires mode argument
            if (!sp) { printf("MODESET requires mode\r\n"); return -1; }
            const char *arg = sp + 1;
            for (int d = 0; d < VALID_MODES_COUNT; d++)
                if (strcmp(arg, VALID_MODES[d]) == 0)
                    return COMMAND_ENTRY[i].packet_id;
            printf("Invalid MODESET argument\r\n");
            return -1;
        }

        if (i == 4) {   // POINT — requires direction argument
            if (!sp) { printf("POINT requires direction\r\n"); return -1; }
            return COMMAND_ENTRY[i].packet_id;
        }

        // All other commands: no argument allowed
        if (!sp) return COMMAND_ENTRY[i].packet_id;
        printf("Not Valid Command\r\n");
        return -1;
    }

    printf("Not Valid Command\r\n");
    return -1;
}

int validCommandDebug(const char *command) {
    char *sp = strchr(command, ' ');
    char  first[60] = {0};

    if (sp) {
        int len = sp - command;
        strncpy(first, command, len);
    } else {
        strncpy(first, command, sizeof(first) - 1);
    }

    for (int i = 0; i < COMMAND_ENTRY_SIZE; i++) {
        if (strcmp(COMMAND_ENTRY[i].name, first) == 0) {
            printf("Valid Command: %s | Packet ID: %d\r\n",
                   COMMAND_ENTRY[i].name, COMMAND_ENTRY[i].packet_id);
            return COMMAND_ENTRY[i].packet_id;
        }
    }
    printf("Not Valid Command\r\n");
    return -1;
}

// ================================================================
// CRC-16
// ================================================================
uint16_t calculateCRC(uint8_t *data, int length) {
    uint16_t crc = 0xFFFF;
    static const uint16_t table[] = {
        0x0000,0x1081,0x2102,0x3183,0x4204,0x5285,
        0x6306,0x7387,0x8408,0x9489,0xa50a,0xb58b,
        0xc60c,0xd68d,0xe70e,0xf78f
    };
    for (int i = 0; i < length; i++) {
        crc = (crc >> 4) ^ table[(crc & 0xf) ^ (data[i] & 0xf)];
        crc = (crc >> 4) ^ table[(crc & 0xf) ^ (data[i] >> 4)];
    }
    return ~crc;
}

// ================================================================
// AX.25 frame builder
// ================================================================
int buildAX25Frame(uint8_t *payload, int payload_len, uint8_t *frame,
                   uint8_t packet_id, uint16_t frame_number,
                   uint16_t total_frame_count) {
    int i = 0;
    frame[i++] = FLAG_BYTE;

    for (int j = 0; j < 7; j++) frame[i+j] = DESTINATION_ADDRESS[j];
    i += 7;
    for (int j = 0; j < 7; j++) frame[i+j] = SOURCE_ADDRESS[j];
    i += 7;

    frame[i++] = AX25_CONTROL;
    frame[i++] = PID;
    frame[i++] = packet_id;
    frame[i++] = (frame_number     >> 8) & 0xFF;
    frame[i++] =  frame_number           & 0xFF;
    frame[i++] = (total_frame_count >> 8) & 0xFF;
    frame[i++] =  total_frame_count       & 0xFF;

    for (int j = 0; j < payload_len; j++) {
        if (payload[j] == FLAG_BYTE || payload[j] == ESCAPE_BYTE) {
            frame[i++] = ESCAPE_BYTE;
            frame[i++] = payload[j] ^ ESCAPE_MASK;
        } else {
            frame[i++] = payload[j];
        }
    }

    uint16_t fcs     = calculateCRC(&frame[1], i - 1);
    uint8_t  fcs_lo  = fcs & 0xFF;
    uint8_t  fcs_hi  = (fcs >> 8) & 0xFF;

    if (fcs_lo == FLAG_BYTE || fcs_lo == ESCAPE_BYTE) {
        frame[i++] = ESCAPE_BYTE;
        frame[i++] = fcs_lo ^ ESCAPE_MASK;
    } else {
        frame[i++] = fcs_lo;
    }
    if (fcs_hi == FLAG_BYTE || fcs_hi == ESCAPE_BYTE) {
        frame[i++] = ESCAPE_BYTE;
        frame[i++] = fcs_hi ^ ESCAPE_MASK;
    } else {
        frame[i++] = fcs_hi;
    }

    frame[i++] = FLAG_BYTE;
    return i;
}

// ================================================================
// waitForACK
// ================================================================
bool waitForACK(uint16_t frame_number) {
    uint8_t  ack_buf[MAX_FRAME_SIZE];
    int      idx = 0, in_frame = 0;
    uint32_t deadline = HAL_GetTick() + ACK_TIMEOUT_MS;

    while (HAL_GetTick() < deadline) {
        if (!uartAvailable(&buffer_frame)) continue;
        uint8_t b = uartReadByte(&buffer_frame);

        if (b == FLAG_BYTE) {
            if (in_frame && idx >= 19) {
                uint8_t  pid = ack_buf[16];
                uint16_t fn  = ((uint16_t)ack_buf[17] << 8) | ack_buf[18];
                if (fn == frame_number) {
                    if (pid == ACK)  { printf("ACK received for frame %d\r\n",  frame_number); return true;  }
                    if (pid == NACK) { printf("NACK received for frame %d\r\n", frame_number); return false; }
                }
                idx = 0;
            }
            in_frame = 1;
            idx = 0;
        } else if (in_frame && idx < (int)sizeof(ack_buf)) {
            ack_buf[idx++] = b;
        }
    }

    printf("Timeout waiting for ACK on frame %d\r\n", frame_number);
    return false;
}

// ================================================================
// sendWithRetry
// ================================================================
static int sendWithRetry(uint8_t *frame, int frame_len, int frame_number) {
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        HAL_UART_Transmit(&huart5, frame, frame_len, HAL_MAX_DELAY);
        HAL_Delay(frame_len * 2);
        if (waitForACK(frame_number)) return 1;
        if (attempt < MAX_RETRIES)
            printf("Retry %d/%d for frame %d\r\n", attempt, MAX_RETRIES, frame_number);
    }
    printf("ERROR: frame %d failed after %d retries\r\n", frame_number, MAX_RETRIES);
    return 0;
}

// ================================================================
// AX25Packaging
// ================================================================
int AX25Packaging(uint8_t *payload, int payload_len,
                  int packet_id, int buf_size) {
    int buffer_count = payload_len / buf_size;
    int remainder    = payload_len % buf_size;
    int total_frames = buffer_count + (remainder > 0 ? 1 : 0);
    int last_acked   = 0;

    for (int i = 0; i < buffer_count; i++) {
        int     fn = i + 1;
        uint8_t dummy[buf_size], frame[MAX_AX25_FRAME_LEN];
        memset(dummy, 0, buf_size);
        memcpy(dummy, payload + i * buf_size, buf_size);
        int len = buildAX25Frame(dummy, buf_size, frame, packet_id, fn, total_frames);
        if (!sendWithRetry(frame, len, fn)) return last_acked;
        last_acked = fn;
    }

    if (remainder > 0) {
        int     fn = buffer_count + 1;
        uint8_t dummy[buf_size], frame[MAX_AX25_FRAME_LEN];
        memset(dummy, 0, buf_size);
        memcpy(dummy, payload + buffer_count * buf_size, remainder);
        int len = buildAX25Frame(dummy, buf_size, frame, packet_id, fn, total_frames);
        if (!sendWithRetry(frame, len, fn)) return last_acked;
        last_acked = fn;
    }

    return last_acked;
}

int AX25PackagingDebug(uint8_t *payload, int payload_len,
                       int packet_id, int buf_size) {
    int buffer_count = payload_len / buf_size;
    int remainder    = payload_len % buf_size;
    int total_frames = buffer_count + (remainder > 0 ? 1 : 0);
    int last_acked   = 0;

    for (int i = 0; i < buffer_count; i++) {
        int     fn = i + 1;
        uint8_t dummy[buf_size], frame[MAX_AX25_FRAME_LEN];
        memset(dummy, 0, buf_size);
        memcpy(dummy, payload + i * buf_size, buf_size);
        int len = buildAX25Frame(dummy, buf_size, frame, packet_id, fn, total_frames);
        printf("Raw frame %d/%d: ", fn, total_frames);
        for (int j = 0; j < len; j++) printf("%02X ", frame[j]);
        printf("\r\n");
        if (!sendWithRetry(frame, len, fn)) return last_acked;
        last_acked = fn;
    }

    if (remainder > 0) {
        int     fn = buffer_count + 1;
        uint8_t dummy[buf_size], frame[MAX_AX25_FRAME_LEN];
        memset(dummy, 0, buf_size);
        memcpy(dummy, payload + buffer_count * buf_size, remainder);
        int len = buildAX25Frame(dummy, buf_size, frame, packet_id, fn, total_frames);
        printf("Remainder frame %d/%d: ", fn, total_frames);
        for (int j = 0; j < len; j++) printf("%02X ", frame[j]);
        printf("\r\n");
        if (!sendWithRetry(frame, len, fn)) return last_acked;
        last_acked = fn;
    }

    return last_acked;
}

// ================================================================
// collectAX25Frame
// ================================================================
int collectAX25Frame(uint8_t *frame, int max_len) {
    int frame_index = 0;

    // Wait for opening FLAG_BYTE
    while (1) {
        if (!uartAvailable(&buffer_frame)) continue;
        uint8_t b = uartReadByte(&buffer_frame);
        if (b == FLAG_BYTE) {
            frame[frame_index++] = b;
            break;
        }
    }

    uint32_t deadline = HAL_GetTick() + MAX_TIMEOUT_WAIT;

    while (HAL_GetTick() < deadline) {
        if (!uartAvailable(&buffer_frame)) continue;
        uint8_t b = uartReadByte(&buffer_frame);

        bool was_escaped = false;
        if (b == ESCAPE_BYTE) {
            uint32_t et = HAL_GetTick() + 100;
            while (!uartAvailable(&buffer_frame)) {
                if (HAL_GetTick() > et) { printf("Error: Escape timeout\r\n"); return 0; }
            }
            b           = uartReadByte(&buffer_frame) ^ ESCAPE_MASK;
            was_escaped = true;
        }

        frame[frame_index++] = b;

        if (!was_escaped && b == FLAG_BYTE && frame_index > (buffer_size + 12))
            return frame_index;

        if (frame_index >= max_len) { printf("Error: Frame overflow\r\n"); return -1; }

        deadline = HAL_GetTick() + 1000; // reset inter-byte timeout
    }

    printf("Error: Timeout\r\n");
    return 0;
}

// ================================================================
// Frame detail / validity check
// ================================================================
void printFrameDetail(uint8_t *frame, int length) {
    int bi = 1;
    printf("Frame length: %d\r\n", length);
    printf("===================================\r\n");
    printf("Destination: ");
    for (int j = 0; j < 7; j++) printf("%02X ", frame[bi+j]);
    bi += 7; printf("\r\n");
    printf("Source: ");
    for (int j = 0; j < 7; j++) printf("%02X ", frame[bi+j]);
    bi += 7; printf("\r\n");
    printf("Control: %d\r\n",   frame[bi++]);
    printf("PID: %d\r\n",       frame[bi++]);
    printf("Packet Id: %d\r\n", frame[bi++]);
    uint16_t fn = ((uint16_t)frame[bi] << 8) | frame[bi+1]; bi += 2;
    printf("Frame number: %d\r\n", fn);
    uint16_t tf = ((uint16_t)frame[bi] << 8) | frame[bi+1]; bi += 2;
    printf("Total frames: %d\r\n", tf);
    printf("Payload Data:\r\n");
    for (int j = 0; j < (length - bi - 3); j++) printf("%02X ", frame[bi+j]);
    printf("\r\nCRC: %02X %02X\r\n", frame[length-3], frame[length-2]);
    printf("===================================\r\n");
}

int AX25ValidityCheck(uint8_t *payload, int data_field_length, int raw_len) {
    if (payload[0] != FLAG_BYTE || payload[raw_len-1] != FLAG_BYTE) {
        printf("FAIL: Flag bytes missing\r\n"); return 0;
    }
    if (raw_len < 25) {
        printf("FAIL: Frame too short, got %d\r\n", raw_len); return 0;
    }
    uint16_t recv_crc = payload[raw_len-3] | ((uint16_t)payload[raw_len-2] << 8);
    uint16_t comp_crc = calculateCRC(&payload[1], raw_len - 4);
    if (recv_crc != comp_crc) {
        printf("FAIL: CRC mismatch\r\n"); return 0;
    }
    printf("Frame is valid\r\n");
    printFrameDetail(payload, raw_len);
    return 1;
}

// ================================================================
// ACK / NACK
// ================================================================
void sendACK(uint16_t frame_number, uint16_t total_frames) {
    uint8_t buf[64], empty[1] = {0};
    int len = buildAX25Frame(empty, 1, buf, ACK, frame_number, total_frames);
    HAL_UART_Transmit(&huart5, buf, len, HAL_MAX_DELAY);
    printf("ACK sent for frame %d\r\n", frame_number);
}

void sendNACK(uint16_t frame_number, uint16_t total_frames) {
    uint8_t buf[64], empty[1] = {0};
    int len = buildAX25Frame(empty, 1, buf, NACK, frame_number, total_frames);
    HAL_UART_Transmit(&huart5, buf, len, HAL_MAX_DELAY);
    printf("NACK sent for frame %d\r\n", frame_number);
}

// ================================================================
// Frame store / reassemble
// ================================================================
static uint8_t frame_store[MAX_FRAMES][INFO_FIELD_SIZE];
static bool    frames_received[MAX_FRAMES]   = {false};
static int     frame_payload_len[MAX_FRAMES] = {0};
static int     total_frames_expected         = 0;

int storeFrame(uint8_t *payload, int raw_len) {
    uint16_t frame_number = ((uint16_t)payload[18] << 8) | payload[19];
    uint16_t total        = ((uint16_t)payload[20] << 8) | payload[21];

    if (total_frames_expected == 0) total_frames_expected = total;

    int payload_len = raw_len - 25;
    printf("Storing frame %d of %d (payload bytes: %d)\r\n",
           frame_number, total, payload_len);

    memset(frame_store[frame_number - 1], 0, INFO_FIELD_SIZE);
    for (int i = 0; i < payload_len && i < INFO_FIELD_SIZE; i++)
        frame_store[frame_number - 1][i] = payload[22 + i];

    frame_payload_len[frame_number - 1] = payload_len;
    frames_received[frame_number - 1]   = true;
    return payload[17]; // packet_id
}

bool allFramesReceived(void) {
    for (int i = 0; i < total_frames_expected; i++)
        if (!frames_received[i]) return false;
    return true;
}

int reassemble(void) {
    printf("===================================\r\n");
    printf("Reassembled message:\r\n");
    for (int i = 0; i < total_frames_expected; i++)
        for (int j = 0; j < frame_payload_len[i]; j++)
            printf("%c", (char)frame_store[i][j]);
    printf("\r\n===================================\r\n");
    return 1;
}

static void reset_frame_store(void) {
    total_frames_expected = 0;
    memset(frames_received,   false, sizeof(frames_received));
    memset(frame_store,       0,     sizeof(frame_store));
    memset(frame_payload_len, 0,     sizeof(frame_payload_len));
}

// ================================================================
// QSPI primitives
// ================================================================
HAL_StatusTypeDef QSPI_WriteEnable(void) {
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_WRITE_ENABLE;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    return HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

HAL_StatusTypeDef QSPI_AutoPollingMemReady(void) {
    QSPI_CommandTypeDef    cmd = {0};
    QSPI_AutoPollingTypeDef ap = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_READ_STATUS_REG1;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_1_LINE;

    ap.Match            = 0x00;
    ap.Mask             = W25Q_STATUS_BUSY;
    ap.MatchMode        = QSPI_MATCH_MODE_AND;
    ap.StatusBytesSize  = 1;
    ap.Interval         = 0x10;
    ap.AutomaticStop    = QSPI_AUTOMATIC_STOP_ENABLE;

    return HAL_QSPI_AutoPolling(&hqspi, &cmd, &ap,
                                HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

HAL_StatusTypeDef QSPI_EnterQuadMode(void) {
    QSPI_CommandTypeDef cmd = {0};
    uint8_t sr2 = 0;

    // Read SR2
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_READ_STATUS_REG2;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_1_LINE;
    cmd.NbData          = 1;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Receive(&hqspi, &sr2,  HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

    if (sr2 & W25Q_STATUS_QE) return HAL_OK; // QE already set

    cmd.Instruction   = W25Q_WRITE_STATUS_REG;
    cmd.AddressMode   = QSPI_ADDRESS_NONE;   // ← add this line
    cmd.DataMode      = QSPI_DATA_1_LINE;    // ← add this line
    cmd.NbData        = 2;

    if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;

    uint8_t regs[2] = {0x00, sr2 | W25Q_STATUS_QE};
    cmd.Instruction = W25Q_WRITE_STATUS_REG;
    cmd.NbData      = 2;
    if (HAL_QSPI_Command( &hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Transmit(&hqspi, regs, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

    return QSPI_AutoPollingMemReady();
}

HAL_StatusTypeDef QSPI_Enter4ByteAddr(void) {
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_ENTER_4BYTE_ADDR;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    return HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

HAL_StatusTypeDef QSPI_EraseSector(uint32_t addr) {
    if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_SECTOR_ERASE_4K;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_32_BITS;
    cmd.Address         = addr;
    cmd.DataMode        = QSPI_DATA_NONE;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

    return QSPI_AutoPollingMemReady(); // typ 45 ms, max 400 ms
}

HAL_StatusTypeDef QSPI_WritePage(uint32_t addr, uint8_t *data, uint16_t size) {
    if (size == 0 || size > (uint16_t)W25Q_PAGE_SIZE) return HAL_ERROR;
    if (QSPI_WriteEnable() != HAL_OK) return HAL_ERROR;

    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_PAGE_PROGRAM;
    cmd.AddressMode     = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = QSPI_ADDRESS_32_BITS;
    cmd.Address         = addr;
    cmd.DataMode        = QSPI_DATA_4_LINES;
    cmd.NbData          = size;
    if (HAL_QSPI_Command( &hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    if (HAL_QSPI_Transmit(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;

    return QSPI_AutoPollingMemReady();
}

HAL_StatusTypeDef QSPI_Read(uint32_t addr, uint8_t *data, uint32_t size) {
    QSPI_CommandTypeDef cmd = {0};
    cmd.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction        = W25Q_READ_DATA;
    cmd.AddressMode        = QSPI_ADDRESS_4_LINES;
    cmd.AddressSize        = QSPI_ADDRESS_32_BITS;
    cmd.Address            = addr;
    cmd.AlternateByteMode  = QSPI_ALTERNATE_BYTES_4_LINES;
    cmd.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    cmd.AlternateBytes     = 0xFF;  // exit continuous read mode
    cmd.DummyCycles        = 4;     // W25Q256 EB command requirement
    cmd.DataMode           = QSPI_DATA_4_LINES;
    cmd.NbData             = size;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    return HAL_QSPI_Receive(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

// Helper: write arbitrary byte count across page boundaries
static HAL_StatusTypeDef qspi_write_bytes(uint32_t addr,
                                           uint8_t *data, uint32_t size) {
    while (size > 0) {
        uint32_t page_offset = addr & (W25Q_PAGE_SIZE - 1);
        uint32_t chunk       = W25Q_PAGE_SIZE - page_offset;
        if (chunk > size) chunk = size;
        if (QSPI_WritePage(addr, data, (uint16_t)chunk) != HAL_OK) return HAL_ERROR;
        addr += chunk;
        data += chunk;
        size -= chunk;
    }
    return HAL_OK;
}

// ================================================================
// QSPI flash initialisation — call once after MX_QUADSPI_Init()
// ================================================================
HAL_StatusTypeDef QSPI_FlashInit(void) {
    QSPI_CommandTypeDef cmd = {0};

    // Software reset sequence
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.Instruction     = W25Q_ENABLE_RESET;
    cmd.AddressMode     = QSPI_ADDRESS_NONE;
    cmd.DataMode        = QSPI_DATA_NONE;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    cmd.Instruction = W25Q_RESET_DEVICE;
    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1); // tRST = 30 µs minimum

    if (QSPI_EnterQuadMode()  != HAL_OK) return HAL_ERROR;
    if (QSPI_Enter4ByteAddr() != HAL_OK) return HAL_ERROR;

//    printf("QSPI flash init OK\r\n");
    return HAL_OK;
}

// ================================================================
// Meta data  (one 4 KB sector at META_SECTOR_ADDR)
// ================================================================
void restoreContext(fsw_ctx_t *context) {
    context->payloadWritePtr   = PAYLOAD_START;
    context->payloadReadPtr    = PAYLOAD_START;
    context->telemetryWritePtr = TELEMETRY_START;
    context->telemetryReadPtr  = TELEMETRY_START;
    context->totalWords        = 0;
}

void onStartup(fsw_ctx_t *context) {
    FLASH_META_DATA meta;

    if (QSPI_Read(META_SECTOR_ADDR, (uint8_t *)&meta, sizeof(meta)) != HAL_OK
        || meta.magic != MAGIC_NUMBER) {
        printf("Meta invalid — using defaults\r\n");
        restoreContext(context);
        return;
    }

    // Verify CRC using H7 hardware CRC peripheral
    // HAL_CRC_Calculate on H7 counts 32-bit words
    uint32_t words[6] = {
        meta.magic,             meta.payloadWritePtr,
        meta.payloadReadPtr,    meta.telemetryWritePtr,
        meta.telemetryReadPtr,  meta.totalWords
    };
    uint32_t checksum = HAL_CRC_Calculate(&hcrc, words, 6);

    if (meta.checksum == checksum) {
        context->payloadWritePtr   = meta.payloadWritePtr;
        context->payloadReadPtr    = meta.payloadReadPtr;
        context->telemetryWritePtr = meta.telemetryWritePtr;
        context->telemetryReadPtr  = meta.telemetryReadPtr;
        context->totalWords        = meta.totalWords;
        printf("Meta restored OK — %lu words stored\r\n", context->totalWords);
    } else {
        printf("Meta CRC mismatch — using defaults\r\n");
        restoreContext(context);
    }
}

void updateMetaData(fsw_ctx_t *context) {
    FLASH_META_DATA meta;
    meta.magic             = MAGIC_NUMBER;
    meta.payloadWritePtr   = context->payloadWritePtr;
    meta.payloadReadPtr    = context->payloadReadPtr;
    meta.telemetryWritePtr = context->telemetryWritePtr;
    meta.telemetryReadPtr  = context->telemetryReadPtr;
    meta.totalWords        = context->totalWords;

    uint32_t words[6] = {
        meta.magic,             meta.payloadWritePtr,
        meta.payloadReadPtr,    meta.telemetryWritePtr,
        meta.telemetryReadPtr,  meta.totalWords
    };
    meta.checksum = HAL_CRC_Calculate(&hcrc, words, 6);

    if (QSPI_EraseSector(META_SECTOR_ADDR) != HAL_OK) {
        printf("Meta sector erase failed\r\n"); return;
    }
    if (qspi_write_bytes(META_SECTOR_ADDR,
                         (uint8_t *)&meta, sizeof(meta)) != HAL_OK) {
        printf("Meta write failed\r\n");
    }
}

// ================================================================
// storeMeasurement
// One float (4 bytes) written per 4 KB sector — same circular
// buffer model as the F3 page-per-measurement scheme.
// ================================================================
void storeMeasurement(fsw_ctx_t *context, float value, int data_type) {
    uint32_t *write_ptr, *read_ptr;
    uint32_t  part_start, part_end;

    if (data_type == PAYLOAD) {
        write_ptr  = &context->payloadWritePtr;
        read_ptr   = &context->payloadReadPtr;
        part_start = PAYLOAD_START;
        part_end   = PAYLOAD_END;
    } else {
        write_ptr  = &context->telemetryWritePtr;
        read_ptr   = &context->telemetryReadPtr;
        part_start = TELEMETRY_START;
        part_end   = TELEMETRY_END;
    }

    // Wrap write pointer at partition end
    if (*write_ptr + W25Q_SECTOR_SIZE > part_end)
        *write_ptr = part_start;

    if (QSPI_EraseSector(*write_ptr) != HAL_OK) {
        printf("Sector erase failed at 0x%08lX\r\n", *write_ptr); return;
    }

    uint8_t buf[4];
    memcpy(buf, &value, 4);
    if (qspi_write_bytes(*write_ptr, buf, 4) != HAL_OK) {
        printf("Write failed at 0x%08lX\r\n", *write_ptr); return;
    }

    // Advance read pointer if write has lapped it
    if (*write_ptr == *read_ptr && *write_ptr != part_start) {
        *read_ptr += W25Q_SECTOR_SIZE;
        if (*read_ptr >= part_end) *read_ptr = part_start;
    }

    *write_ptr += W25Q_SECTOR_SIZE;
    context->totalWords += 1;
    updateMetaData(context);
}

// ================================================================
// downlinkDataAllMemory
// ================================================================
void downlinkDataAllMemory(fsw_ctx_t *context, int packet_id) {
    uint32_t start, end;

    if (packet_id == PAYLOAD) {
        start = context->payloadReadPtr;
        end   = context->payloadWritePtr;
    } else {
        start = context->telemetryReadPtr;
        end   = context->telemetryWritePtr;
    }

    if (end <= start) { printf("No new data to downlink\r\n"); return; }

    uint32_t total_sectors     = (end - start) / W25Q_SECTOR_SIZE;
    uint32_t sectors_per_chunk = MAX_BUFFER_SIZE / 4;
    uint32_t sectors_sent      = 0;

    while (sectors_sent < total_sectors) {
        uint32_t chunk = total_sectors - sectors_sent;
        if (chunk > sectors_per_chunk) chunk = sectors_per_chunk;

        uint8_t  payload_buf[MAX_BUFFER_SIZE];
        uint32_t byte_idx = 0;
        memset(payload_buf, 0, MAX_BUFFER_SIZE);

        for (uint32_t i = 0; i < chunk; i++) {
            uint32_t addr = start + (sectors_sent + i) * W25Q_SECTOR_SIZE;
            uint8_t  raw[4];
            if (QSPI_Read(addr, raw, 4) != HAL_OK) {
                printf("Read error at 0x%08lX\r\n", addr); return;
            }
            payload_buf[byte_idx++] = raw[0];
            payload_buf[byte_idx++] = raw[1];
            payload_buf[byte_idx++] = raw[2];
            payload_buf[byte_idx++] = raw[3];
        }

        int      frames_acked = AX25Packaging(payload_buf, chunk * 4,
                                              packet_id, BUFFER_SIZE);
        uint32_t confirmed    = ((uint32_t)frames_acked * BUFFER_SIZE) / 4;

        if (packet_id == PAYLOAD) {
            context->payloadReadPtr += confirmed * W25Q_SECTOR_SIZE;
            if (context->payloadReadPtr > context->payloadWritePtr)
                context->payloadReadPtr = context->payloadWritePtr;
        } else {
            context->telemetryReadPtr += confirmed * W25Q_SECTOR_SIZE;
            if (context->telemetryReadPtr > context->telemetryWritePtr)
                context->telemetryReadPtr = context->telemetryWritePtr;
        }

        updateMetaData(context);
        sectors_sent += confirmed;

        if (confirmed < chunk) { printf("Transmission cut off\r\n"); break; }
    }

    printf("Downlink complete\r\n");
}

// ================================================================
// eraseAllStorage
// ================================================================
void eraseAllStorage(fsw_ctx_t *context) {
    printf("Erasing payload partition...\r\n");
    for (uint32_t addr = PAYLOAD_START; addr < PAYLOAD_END;
         addr += W25Q_SECTOR_SIZE) {
        if (QSPI_EraseSector(addr) != HAL_OK) {
            printf("Erase failed at 0x%08lX\r\n", addr); return;
        }
    }

    printf("Erasing telemetry partition...\r\n");
    for (uint32_t addr = TELEMETRY_START; addr < TELEMETRY_END;
         addr += W25Q_SECTOR_SIZE) {
        if (QSPI_EraseSector(addr) != HAL_OK) {
            printf("Erase failed at 0x%08lX\r\n", addr); return;
        }
    }

    restoreContext(context);
    updateMetaData(context);
    printf("All storage erased\r\n");
}

// ================================================================
// printFlash  — stream entire data area over RF
// ================================================================
void printFlash(void) {
    uint32_t total_sectors     = PAYLOAD_SECTOR_COUNT + TELEMETRY_SECTOR_COUNT;
    uint32_t sectors_per_chunk = MAX_BUFFER_SIZE / 4;
    uint32_t sectors_sent      = 0;

    while (sectors_sent < total_sectors) {
        uint32_t chunk = total_sectors - sectors_sent;
        if (chunk > sectors_per_chunk) chunk = sectors_per_chunk;

        uint8_t  payload_buf[MAX_BUFFER_SIZE];
        uint32_t byte_idx = 0;
        memset(payload_buf, 0, MAX_BUFFER_SIZE);

        for (uint32_t i = 0; i < chunk; i++) {
            uint32_t global = sectors_sent + i;
            uint32_t addr;
            // Map flat index across both partitions
            if (global < PAYLOAD_SECTOR_COUNT)
                addr = PAYLOAD_START + global * W25Q_SECTOR_SIZE;
            else
                addr = TELEMETRY_START + (global - PAYLOAD_SECTOR_COUNT) * W25Q_SECTOR_SIZE;

            uint8_t raw[4];
            if (QSPI_Read(addr, raw, 4) != HAL_OK) {
                printf("Read error\r\n"); return;
            }
            payload_buf[byte_idx++] = raw[0];
            payload_buf[byte_idx++] = raw[1];
            payload_buf[byte_idx++] = raw[2];
            payload_buf[byte_idx++] = raw[3];
        }

        int      frames_acked = AX25Packaging(payload_buf, chunk * 4, MEMORY, BUFFER_SIZE);
        uint32_t confirmed    = ((uint32_t)frames_acked * BUFFER_SIZE) / 4;
        sectors_sent         += confirmed;
        if (confirmed < chunk) { printf("Transmission cut off\r\n"); break; }
    }
}

// ================================================================
// writeDummyData
// ================================================================
void writeDummyData(void) {
    uint32_t counter = 0;

    for (uint32_t addr = PAYLOAD_START; addr < PAYLOAD_END;
         addr += W25Q_SECTOR_SIZE) {
        if (QSPI_EraseSector(addr) != HAL_OK) {
            printf("Dummy erase fail at 0x%08lX\r\n", addr); return;
        }
        uint8_t buf[4];
        memcpy(buf, &counter, 4);
        if (qspi_write_bytes(addr, buf, 4) != HAL_OK) {
            printf("Dummy write fail\r\n"); return;
        }
        counter = (counter + 1) % 10;
    }

    for (uint32_t addr = TELEMETRY_START; addr < TELEMETRY_END;
         addr += W25Q_SECTOR_SIZE) {
        if (QSPI_EraseSector(addr) != HAL_OK) {
            printf("Dummy erase fail at 0x%08lX\r\n", addr); return;
        }
        uint8_t buf[4];
        memcpy(buf, &counter, 4);
        if (qspi_write_bytes(addr, buf, 4) != HAL_OK) {
            printf("Dummy write fail\r\n"); return;
        }
        counter = (counter + 1) % 10;
    }

    printf("Dummy data written\r\n");
}

// ================================================================
// Mode table / modeSet / modeGet
// ================================================================
static const MODE_ENTRY_T MODE_TABLE[] = {
    { "NOMINAL",  MODE_NOMINAL  },
    { "DETUMBLE", MODE_DETUMBLE },
    { "SAFE",     MODE_SAFE     },
    { "POINTING", MODE_POINTING },
    { "SCIENCE",  MODE_SCIENCE  },
    { "DOWNLINK", MODE_DOWNLINK },
    { "UPLINK",   MODE_UPLINK   }
};
static const int MODE_TABLE_SIZE = sizeof(MODE_TABLE) / sizeof(MODE_TABLE[0]);

void modeSet(sat_mode_t *mode) {
    char mode_str[32] = {0};
    int  k = 0;
    for (int i = 0; i < total_frames_expected && k < (int)sizeof(mode_str)-1; i++)
        for (int j = 0; j < frame_payload_len[i] && k < (int)sizeof(mode_str)-1; j++)
            mode_str[k++] = (char)frame_store[i][j];

    char *sp = strchr(mode_str, ' ');
    if (!sp) { printf("MODESET: no mode argument\r\n"); return; }

    const char *arg = sp + 1;
    for (int i = 0; i < MODE_TABLE_SIZE; i++) {
        if (strcmp(arg, MODE_TABLE[i].name) == 0) {
            *mode = MODE_TABLE[i].value;
            printf("Mode set to: %s\r\n", MODE_TABLE[i].name);
            return;
        }
    }
    printf("MODESET: unknown mode '%s'\r\n", arg);
}

void modeGet(sat_mode_t *mode) {
    const char *name = "UNKNOWN";
    for (int i = 0; i < MODE_TABLE_SIZE; i++)
        if (MODE_TABLE[i].value == *mode) { name = MODE_TABLE[i].name; break; }
    printf("Current mode: %s\r\n", name);
    AX25Packaging((uint8_t *)name, strlen(name), MODEGET, BUFFER_SIZE);
}

// ================================================================
// comms_task
// ================================================================
void comms_task(sat_mode_t *mode, fsw_ctx_t *context) {


	switch (mode) {
	case MODE_UPLINK:
		if (uartAvailable(&buffer_frame)) {
		        uint8_t frame_buffer[360];
		        int     frame_len = collectAX25Frame(frame_buffer, 360);
		        if (frame_len <= 0) return;

		        uint8_t  pid = frame_buffer[17];
		        uint16_t fn  = ((uint16_t)frame_buffer[18] << 8) | frame_buffer[19];
		        uint16_t tf  = ((uint16_t)frame_buffer[20] << 8) | frame_buffer[21];

		        if (pid == ACK || pid == NACK) return;

		        if (AX25ValidityCheck(frame_buffer, buffer_size, frame_len)) {
		            sendACK(fn, tf);
		            packet_id_received = storeFrame(frame_buffer, frame_len);
		            if (allFramesReceived())
		                all_messages_received = reassemble();
		        } else {
		            sendNACK(fn, tf);
		        }
		    }

		    if (all_messages_received == 1) {

		        if (packet_id_received == CLEAR) {
		            eraseAllStorage(context);
		            printf("Cleared Memory\r\n");

		        } else if (packet_id_received == TELEMETRY ||
		                   packet_id_received == PAYLOAD) {
		            printf("Sending downlink\r\n");
		            downlinkDataAllMemory(context, packet_id_received);

		        } else if (packet_id_received == MEMORY) {
		            printFlash();
		            printf("Sent all of memory\r\n");

		        } else if (packet_id_received == DUMMYDATA) {
		            writeDummyData();

		        } else if (packet_id_received == MODEGET) {
		            modeGet(mode);

		        } else if (packet_id_received == MODESET) {
		            modeSet(mode);
		        }

		        packet_id_received    = -1;
		        all_messages_received =  0;
		        reset_frame_store();
		    }
	default:
		break;
	}
}


//if (uartAvailable(&buffer_frame)) {
//        uint8_t frame_buffer[360];
//        int     frame_len = collectAX25Frame(frame_buffer, 360);
//        if (frame_len <= 0) return;
//
//        uint8_t  pid = frame_buffer[17];
//        uint16_t fn  = ((uint16_t)frame_buffer[18] << 8) | frame_buffer[19];
//        uint16_t tf  = ((uint16_t)frame_buffer[20] << 8) | frame_buffer[21];
//
//        if (pid == ACK || pid == NACK) return;
//
//        if (AX25ValidityCheck(frame_buffer, buffer_size, frame_len)) {
//            sendACK(fn, tf);
//            packet_id_received = storeFrame(frame_buffer, frame_len);
//            if (allFramesReceived())
//                all_messages_received = reassemble();
//        } else {
//            sendNACK(fn, tf);
//        }
//    }
//
//    if (all_messages_received == 1) {
//
//        if (packet_id_received == CLEAR) {
//            eraseAllStorage(context);
//            printf("Cleared Memory\r\n");
//
//        } else if (packet_id_received == TELEMETRY ||
//                   packet_id_received == PAYLOAD) {
//            printf("Sending downlink\r\n");
//            downlinkDataAllMemory(context, packet_id_received);
//
//        } else if (packet_id_received == MEMORY) {
//            printFlash();
//            printf("Sent all of memory\r\n");
//
//        } else if (packet_id_received == DUMMYDATA) {
//            writeDummyData();
//
//        } else if (packet_id_received == MODEGET) {
//            modeGet(mode);
//
//        } else if (packet_id_received == MODESET) {
//            modeSet(mode);
//        }
//
//        packet_id_received    = -1;
//        all_messages_received =  0;
//        reset_frame_store();
//    }
