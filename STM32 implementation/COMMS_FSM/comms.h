#pragma once

#ifndef COMMS_H
#define COMMS_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32h7xx_hal.h"

// ================================================================
// STM32 peripheral handles (defined by CubeMX in main.c)
// ================================================================
extern CRC_HandleTypeDef   hcrc;
extern UART_HandleTypeDef  huart5;   // RF link to ESP32  (PA9/PA10)
extern UART_HandleTypeDef  huart2;   // Debug printf      (PA2/PA3)
extern QSPI_HandleTypeDef  hqspi;    // W25Q256 NOR flash (Bank 1)

// ================================================================
// UART ring buffer
// ================================================================
#define RING_BUFFER_SIZE  512
#define RING_BUFFER_MASK  (RING_BUFFER_SIZE - 1)

typedef struct {
    volatile uint8_t  buffer[RING_BUFFER_SIZE];
    volatile uint16_t write_ptr;
    volatile uint16_t read_ptr;
} RING_BUFFER;

extern RING_BUFFER buffer_frame;

// ================================================================
// AX.25 constants
// ================================================================
#define FLAG_BYTE     0x7E
#define ESCAPE_BYTE   0x7D
#define ESCAPE_MASK   0x20
#define AX25_CONTROL  0x03
#define PID           0xF0

static const uint8_t DESTINATION_ADDRESS[7] = {
    'G'<<1,'O'<<1,'O'<<1,'S'<<1,'E'<<1,' '<<1, 0xE0
};
static const uint8_t SOURCE_ADDRESS[7] = {
    'G'<<1,'O'<<1,'O'<<1,'S'<<1,'E'<<1,' '<<1, 0x61
};

#define MAX_AX25_FRAME_LEN  100
#define INFO_FIELD_SIZE     236
#define MAX_FRAME_SIZE      256

// ================================================================
// Packet IDs
// ================================================================
#define PING       0
#define TELEMETRY  1
#define PAYLOAD    2
#define ACK        3
#define NACK       4
#define COMMAND    5
#define CLEAR      6
#define MEMORY     7
#define DUMMYDATA  8
#define MODEGET    9
#define MODESET    10

// ================================================================
// ACK / retry timing
// ================================================================
#define ACK_TIMEOUT_MS  1000
#define MAX_RETRIES     3

// ================================================================
// Receive constants
// ================================================================
#define MAX_FRAMES        100
#define MAX_TIMEOUT_WAIT  1000

// ================================================================
// AX.25 framing buffer sizes
// ================================================================
#define BUFFER_SIZE      10
#define MAX_BUFFER_SIZE  256

// ================================================================
// W25Q256 QSPI NOR flash — 32 MB
// Geometry: 8192 sectors x 4KB  /  512 blocks x 64KB
// ================================================================

// Commands
#define W25Q_WRITE_ENABLE      0x06
#define W25Q_WRITE_DISABLE     0x04
#define W25Q_READ_STATUS_REG1  0x05
#define W25Q_READ_STATUS_REG2  0x35
#define W25Q_WRITE_STATUS_REG  0x01
#define W25Q_READ_DATA         0xEB   // Fast Read Quad I/O  (4-byte addr)
#define W25Q_PAGE_PROGRAM      0x34   // Quad Input Page Program (4-byte addr)
#define W25Q_SECTOR_ERASE_4K   0x21   // 4 KB sector erase (4-byte addr)
#define W25Q_BLOCK_ERASE_64K   0xDC   // 64 KB block erase (4-byte addr)
#define W25Q_CHIP_ERASE        0xC7
#define W25Q_ENTER_4BYTE_ADDR  0xB7
#define W25Q_ENABLE_RESET      0x66
#define W25Q_RESET_DEVICE      0x99

// Status register bits
#define W25Q_STATUS_BUSY  0x01
#define W25Q_STATUS_WEL   0x02
#define W25Q_STATUS_QE    0x02   // Quad Enable — bit 1 of SR2

// Geometry
#define W25Q_FLASH_SIZE    (32UL * 1024UL * 1024UL)
#define W25Q_SECTOR_SIZE   (4UL  * 1024UL)
#define W25Q_BLOCK_SIZE    (64UL * 1024UL)
#define W25Q_PAGE_SIZE     256UL
#define W25Q_TOTAL_SECTORS 8192UL

// ================================================================
// Flash partition layout  (all offsets from W25Q256 base 0x00000000)
//
//  [0x000000 – 0x00FFFF]  64 KB   Block 0 — RESERVED
//  [0x010000 – 0x18FFFF] ~1.5 MB  PAYLOAD   (sectors  64 – 399)
//  [0x190000 – 0x199FFF]  40 KB   Gap 1     (sectors 400 – 409)
//  [0x19A000 – 0x31FFFF] ~1.5 MB  TELEMETRY (sectors 410 – 799)
//  [0x320000 – 0x329FFF]  40 KB   Gap 2     (sectors 800 – 809)
//  [0x32A000 …          ]          Reserved / future
//  [0xCCC000 – 0xCCCFFF]   4 KB   META      (sector 3276)
//
// All boundaries are 4 KB sector-aligned.
// The 40 KB gaps prevent cross-partition wear propagation.
// ================================================================
#define PAYLOAD_START          0x00010000UL
#define PAYLOAD_END            0x00190000UL   // exclusive
#define PAYLOAD_SECTOR_COUNT   336UL

#define TELEMETRY_START        0x0019A000UL
#define TELEMETRY_END          0x00320000UL   // exclusive
#define TELEMETRY_SECTOR_COUNT 390UL

#define META_SECTOR_ADDR       0x00CCC000UL

#define MAGIC_NUMBER           0xDEADBEEF

// ================================================================
// Structs / enums
// ================================================================
typedef struct {
    const char *name;
    int         packet_id;
} COMMAND_AND_ID;

extern const COMMAND_AND_ID COMMAND_ENTRY[];
extern const int            COMMAND_ENTRY_SIZE;

typedef struct {
    uint32_t magic;
    uint32_t payloadWritePtr;
    uint32_t payloadReadPtr;
    uint32_t telemetryWritePtr;
    uint32_t telemetryReadPtr;
    uint32_t totalWords;
    uint32_t checksum;
} FLASH_META_DATA;

typedef struct { uint8_t stub; } adcs_state_t;

typedef struct {
    adcs_state_t est;
    float        B_lvlh[3];
    float        B_body[3];

    uint8_t science_due;
    uint8_t soc_low;
    uint8_t science_window;
    uint8_t ground_contact;
    uint8_t deploy_elapsed;
    uint8_t downlink_due;
    uint8_t uplink_pending;
    uint8_t uplink_done;

    uint32_t payloadWritePtr;
    uint32_t payloadReadPtr;
    uint32_t telemetryWritePtr;
    uint32_t telemetryReadPtr;
    uint32_t totalWords;
} fsw_ctx_t;

typedef enum {
    MODE_NOMINAL = 0,
    MODE_DETUMBLE,
    MODE_SAFE,
    MODE_POINTING,
    MODE_SCIENCE,
    MODE_DOWNLINK,
    MODE_UPLINK,
    MODE_COUNT
} sat_mode_t;

typedef struct {
    const char *name;
    sat_mode_t  value;
} MODE_ENTRY_T;

// ================================================================
// Function prototypes
// ================================================================

// --- UART / ring buffer ---
void    uartRingBufferInitialise(void);
int     uartAvailable(RING_BUFFER *buffer);
uint8_t uartReadByte(RING_BUFFER *buffer);

// --- AX.25 TX ---
int      validCommand(const char *command);
int      validCommandDebug(const char *command);
uint16_t calculateCRC(uint8_t *data, int length);
int      buildAX25Frame(uint8_t *payload, int payload_len, uint8_t *frame,
                        uint8_t packet_id, uint16_t frame_number,
                        uint16_t total_frame_count);
bool     waitForACK(uint16_t expected_frame);
int      AX25Packaging(uint8_t *payload, int payload_len,
                       int packet_id, int buf_size);
int      AX25PackagingDebug(uint8_t *payload, int payload_len,
                            int packet_id, int buf_size);

// --- AX.25 RX ---
int  collectAX25Frame(uint8_t *frame, int max_len);
void printFrameDetail(uint8_t *frame, int length);
int  AX25ValidityCheck(uint8_t *payload, int data_field_length, int raw_len);
void sendACK(uint16_t frame_number, uint16_t total_frames);
void sendNACK(uint16_t frame_number, uint16_t total_frames);
int  storeFrame(uint8_t *payload, int raw_len);
bool allFramesReceived(void);
int  reassemble(void);

// --- QSPI primitives ---
HAL_StatusTypeDef QSPI_FlashInit(void);
HAL_StatusTypeDef QSPI_WriteEnable(void);
HAL_StatusTypeDef QSPI_AutoPollingMemReady(void);
HAL_StatusTypeDef QSPI_EnterQuadMode(void);
HAL_StatusTypeDef QSPI_Enter4ByteAddr(void);
HAL_StatusTypeDef QSPI_EraseSector(uint32_t addr);
HAL_StatusTypeDef QSPI_WritePage(uint32_t addr, uint8_t *data, uint16_t size);
HAL_StatusTypeDef QSPI_Read(uint32_t addr, uint8_t *data, uint32_t size);

// --- Flash storage ---
void onStartup(fsw_ctx_t *context);
void updateMetaData(fsw_ctx_t *context);
void restoreContext(fsw_ctx_t *context);
void storeMeasurement(fsw_ctx_t *context, float value, int data_type);
void downlinkDataAllMemory(fsw_ctx_t *context, int packet_id);
void eraseAllStorage(fsw_ctx_t *context);
void printFlash(void);
void writeDummyData(void);

// --- Mode control ---
void modeSet(sat_mode_t *mode);
void modeGet(sat_mode_t *mode);

// --- Top-level task ---
void comms_task(sat_mode_t *mode, fsw_ctx_t *context);

#endif // COMMS_H
