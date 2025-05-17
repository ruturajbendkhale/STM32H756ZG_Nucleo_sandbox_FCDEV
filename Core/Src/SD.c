#include "SD.h"
#include <string.h> // For memset if used

// UART handle for debugging (ensure it's declared extern if used here, or pass messages back)
extern UART_HandleTypeDef huart3; // Assuming huart3 is used for debug prints

static SD_CardInfo_t sd_card_info;

// --- Private Helper Functions ---
static void SD_CS_Assert(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void SD_CS_Deassert(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

// Transmit a byte and receive a byte
static uint8_t SD_SPI_TransmitReceive(uint8_t data_out) {
    uint8_t data_in = 0;
    if (sd_card_info.hspi != NULL) {
        HAL_SPI_TransmitReceive(sd_card_info.hspi, &data_out, &data_in, 1, HAL_MAX_DELAY);
    }
    return data_in;
}

// Send a command to the SD card
// Returns the R1 response byte (or the first byte of a multi-byte response)
static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t* response_buffer, uint16_t response_len) {
    uint8_t r1_response = 0xFF;
    uint8_t cmd_packet[6];
    int i;

    SD_CS_Assert();

    // Send 0xFF for a few clocks before sending command
    SD_SPI_TransmitReceive(0xFF);

    cmd_packet[0] = 0x40 | cmd;
    cmd_packet[1] = (uint8_t)(arg >> 24);
    cmd_packet[2] = (uint8_t)(arg >> 16);
    cmd_packet[3] = (uint8_t)(arg >> 8);
    cmd_packet[4] = (uint8_t)(arg);
    cmd_packet[5] = crc;

    for (i = 0; i < 6; i++) {
        SD_SPI_TransmitReceive(cmd_packet[i]);
    }

    // Wait for a valid response (not 0xFF). Max 10 tries (Ncr timeout).
    uint8_t n = 10;
    do {
        r1_response = SD_SPI_TransmitReceive(0xFF);
        n--;
    } while ((r1_response == 0xFF) && (n > 0));

    if (r1_response != 0xFF && response_buffer && response_len > 0) { // Check if R1 is valid before reading more
        // response_buffer[0] = r1_response; // DO NOT store R1 in the payload buffer
        for (i = 0; i < response_len; i++) { // Read response_len bytes into the buffer starting at index 0
            response_buffer[i] = SD_SPI_TransmitReceive(0xFF);
        }
    }
    
    // For commands like CMD0, no CS de-assert here yet, or for CMD58 etc.
    // For simple R1 commands, we might de-assert, but init sequence needs CS low for some part.
    // SD_CS_Deassert(); // De-assert typically after command completion unless in a sequence

    return r1_response;
}

// --- Logging Globals ---
#define SD_BLOCK_SIZE 512
// static uint8_t sd_log_data_buffer[SD_BLOCK_SIZE]; // Old binary buffer
static char sd_log_csv_buffer[SD_BLOCK_SIZE];     // New buffer for CSV strings
// static uint16_t sd_log_buffer_offset = 0; // Old binary buffer offset
static uint16_t sd_log_csv_buffer_char_count = 0; // Current character count in sd_log_csv_buffer
static uint32_t sd_current_log_block = 0; // Current SD card block to write to
static bool logging_active = false;

// Forward declaration for internal use
static void SD_Log_Write_CSV_Header(void);

// --- Public API Functions ---
void SD_SPI_Init(SPI_HandleTypeDef *hspi_instance) {
    sd_card_info.hspi = hspi_instance;
    sd_card_info.initialized = false;
    sd_card_info.card_type = 0;
}

uint8_t SD_Init(void) {
    uint8_t r1, response_buf[4];
    uint16_t retry_count = 0;
    char dbg_buf[64];

    if (!sd_card_info.hspi) {
        sprintf(dbg_buf, "SD_Init: SPI handle not set!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 1; // Error
    }

    sd_card_info.initialized = false;
    SD_CS_Deassert();

    // Send 80 dummy clocks (10 bytes of 0xFF)
    for (int i = 0; i < 10; i++) {
        SD_SPI_TransmitReceive(0xFF);
    }

    // CMD0: GO_IDLE_STATE
    sprintf(dbg_buf, "SD_Init: Sending CMD0\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    r1 = SD_SendCommand(CMD0, 0, 0x95, NULL, 0);
    SD_CS_Deassert(); // CMD0 needs CS to go high after
    if (r1 != R1_IDLE_STATE) {
        sprintf(dbg_buf, "SD_Init: CMD0 failed, response: 0x%02X\r\n", r1);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return r1 == 0xFF ? 2 : 3; // Timeout or other error
    }
    sprintf(dbg_buf, "SD_Init: CMD0 OK\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);

    // CMD8: SEND_IF_COND (for SDv2 cards)
    sprintf(dbg_buf, "SD_Init: Sending CMD8\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    r1 = SD_SendCommand(CMD8, 0x000001AA, 0x87, response_buf, 4); // Argument: VHS=2.7-3.6V, Check Pattern=0xAA
    // SD_CS_Deassert(); // CMD8 response is on data lines, keep CS low for response
    if (r1 == R1_IDLE_STATE) { // If CMD8 is accepted
        sprintf(dbg_buf, "SD_Init: CMD8 Response: 0x%02X 0x%02X 0x%02X 0x%02X\r\n", response_buf[0], response_buf[1], response_buf[2], response_buf[3]);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        if (response_buf[2] == 0x01 && response_buf[3] == 0xAA) { // Check pattern and VHS match
            sd_card_info.card_type = SD_CARD_TYPE_SD2; // SDv2 card
            sprintf(dbg_buf, "SD_Init: SDv2 card detected\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
            // Try to initialize SDHC/SDXC (CMD55 + ACMD41 with HCS bit)
            retry_count = 0xFFFF; // Generous timeout for initialization
            do {
                r1 = SD_SendCommand(CMD55, 0, 0xFF, NULL, 0); // APP_CMD
                SD_CS_Deassert(); // CMD55 needs CS toggle
                if (r1 > 1) break; // Error on CMD55
                
                r1 = SD_SendCommand(ACMD41, 0x40000000, 0xFF, NULL, 0); // HCS=1 (Host Capacity Support)
                SD_CS_Deassert(); // ACMD41 needs CS toggle
                retry_count--;
                HAL_Delay(1); // Small delay
            } while ((r1 != 0x00) && (retry_count > 0));

            if (r1 == 0x00) { // Card is initialized
                // Check OCR for CCS (Card Capacity Status) bit for SDHC/SDXC
                r1 = SD_SendCommand(CMD58, 0, 0xFF, response_buf, 4);
                SD_CS_Deassert();
                if (r1 == 0x00) {
                    if (response_buf[0] & 0x40) { // CCS bit is set
                         sd_card_info.card_type = SD_CARD_TYPE_SDHC;
                         sprintf(dbg_buf, "SD_Init: SDHC/SDXC card detected (CCS=1)\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
                    } else {
                         sprintf(dbg_buf, "SD_Init: SDv2 (standard capacity) card (CCS=0)\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
                         // Standard capacity SDv2 cards still need CMD16 to set block size to 512 if not default
                    }
                }
            } else {
                sprintf(dbg_buf, "SD_Init: ACMD41(HCS) failed, response: 0x%02X\r\n", r1);
                HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
                SD_CS_Deassert(); return 4; // Initialization error
            }
        } else {
             sprintf(dbg_buf, "SD_Init: CMD8 pattern mismatch or VHS error\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
            SD_CS_Deassert(); return 5; // CMD8 pattern mismatch
        }
    } else if (r1 & R1_ILLEGAL_COMMAND) { // CMD8 not supported (SDv1 card)
        sd_card_info.card_type = SD_CARD_TYPE_SD1;
        sprintf(dbg_buf, "SD_Init: SDv1 card detected (CMD8 illegal)\r\n"); HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        SD_CS_Deassert(); // CMD8 failed, de-assert CS
        // Initialize SDv1 (CMD55 + ACMD41 without HCS bit)
        retry_count = 0xFFFF;
        do {
            r1 = SD_SendCommand(CMD55, 0, 0xFF, NULL, 0);
            SD_CS_Deassert();
            if (r1 > 1) break;
            r1 = SD_SendCommand(ACMD41, 0x00000000, 0xFF, NULL, 0); // HCS=0
            SD_CS_Deassert();
            retry_count--;
            HAL_Delay(1);
        } while ((r1 != 0x00) && (retry_count > 0));
        if (r1 != 0x00) {
            sprintf(dbg_buf, "SD_Init: ACMD41(SDv1) failed, response: 0x%02X\r\n", r1);
            HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
            return 6; // Initialization error
        }
    } else { // CMD8 failed for other reasons
        sprintf(dbg_buf, "SD_Init: CMD8 failed, response: 0x%02X\r\n", r1);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        SD_CS_Deassert();
        return 7; // CMD8 failed
    }

    SD_CS_Deassert();
    if (r1 != 0x00) {
        sprintf(dbg_buf, "SD_Init: Card initialization failed. Last R1: 0x%02X\r\n", r1);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 8; // Final initialization check fail
    }

    // For non-SDHC cards (SDv1, SDv2 standard capacity), explicitly set block size to 512 bytes if needed.
    // SDHC cards always use 512-byte blocks and CMD16 is ignored.
    if (sd_card_info.card_type != SD_CARD_TYPE_SDHC) {
        r1 = SD_SendCommand(CMD16, 512, 0xFF, NULL, 0);
        SD_CS_Deassert();
        if (r1 != 0x00) {
            sprintf(dbg_buf, "SD_Init: CMD16 (Set Blocklen) failed, response: 0x%02X\r\n", r1);
            HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
            return 9;
        }
    }

    sd_card_info.initialized = true;
    sprintf(dbg_buf, "SD_Init: Card Initialized Successfully. Type: %d\r\n", sd_card_info.card_type);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    return 0; // Success
}

// Write a 512-byte block to the SD card
uint8_t SD_Write_Block(uint32_t block_addr, const uint8_t *data_buffer) {
    uint8_t r1, token;
    char dbg_buf[64];

    if (!sd_card_info.initialized) {
        sprintf(dbg_buf, "SD_Write_Block: Card not initialized!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 1;
    }

    // For SDSC cards, address is byte address. For SDHC/XC, address is block address.
    // This simplified version assumes block_addr is already correct for the card type.
    // For SDHC, block_addr is used directly. For SDSC, it would be block_addr * 512.
    uint32_t send_arg = (sd_card_info.card_type == SD_CARD_TYPE_SDHC) ? block_addr : (block_addr * 512);

    r1 = SD_SendCommand(CMD24, send_arg, 0xFF, NULL, 0);
    if (r1 != 0x00) {
        SD_CS_Deassert();
        sprintf(dbg_buf, "SD_Write_Block: CMD24 failed, response: 0x%02X\r\n", r1);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 2;
    }

    // Send start token (0xFE for single block write)
    SD_SPI_TransmitReceive(0xFF); // Dummy byte
    SD_SPI_TransmitReceive(0xFE);

    // Send data block (512 bytes)
    for (int i = 0; i < 512; i++) {
        SD_SPI_TransmitReceive(data_buffer[i]);
    }

    // Send dummy CRC (2 bytes)
    SD_SPI_TransmitReceive(0xFF);
    SD_SPI_TransmitReceive(0xFF);

    // Read data response token
    token = SD_SPI_TransmitReceive(0xFF);
    if ((token & 0x1F) != 0x05) { // Check if data accepted (0b00101)
        SD_CS_Deassert();
        sprintf(dbg_buf, "SD_Write_Block: Data not accepted, token: 0x%02X\r\n", token);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 3;
    }

    // Wait for card to finish busy (sends 0x00 when not busy, 0xFF while busy)
    uint32_t timeout = 0xFFFFFF; // Generous timeout
    while (SD_SPI_TransmitReceive(0xFF) == 0x00 && timeout > 0) {
        timeout--;
    }
    SD_CS_Deassert();
    if(timeout == 0){
        sprintf(dbg_buf, "SD_Write_Block: Timeout waiting for card to become not busy\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 4; // Busy timeout
    }

    sprintf(dbg_buf, "SD_Write_Block: Block %lu written successfully.\r\n", block_addr);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    return 0; // Success
}

// Read a 512-byte block from the SD card (Basic implementation for testing)
uint8_t SD_Read_Block(uint32_t block_addr, uint8_t *data_buffer) {
    uint8_t r1, token;
    char dbg_buf[64];

    if (!sd_card_info.initialized) {
         sprintf(dbg_buf, "SD_Read_Block: Card not initialized!\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 1;
    }
    
    uint32_t send_arg = (sd_card_info.card_type == SD_CARD_TYPE_SDHC) ? block_addr : (block_addr * 512);

    r1 = SD_SendCommand(CMD17, send_arg, 0xFF, NULL, 0);
    if (r1 != 0x00) {
        SD_CS_Deassert();
        sprintf(dbg_buf, "SD_Read_Block: CMD17 failed, response: 0x%02X\r\n", r1);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 2;
    }

    // Wait for data start token (0xFE)
    uint32_t timeout = 0xFFFFFF; 
    do {
        token = SD_SPI_TransmitReceive(0xFF);
        timeout--;
    } while (token == 0xFF && timeout > 0);

    if (token != 0xFE) {
        SD_CS_Deassert();
        sprintf(dbg_buf, "SD_Read_Block: Invalid start token 0x%02X\r\n", token);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return 3;
    }

    // Read data block
    for (int i = 0; i < 512; i++) {
        data_buffer[i] = SD_SPI_TransmitReceive(0xFF);
    }

    // Read CRC (2 bytes, ignored)
    SD_SPI_TransmitReceive(0xFF);
    SD_SPI_TransmitReceive(0xFF);
    SD_CS_Deassert();
    
    sprintf(dbg_buf, "SD_Read_Block: Block %lu read successfully.\r\n", block_addr);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    return 0; // Success
}

// --- Data Logging Functions ---

void SD_Log_Start(uint32_t start_block_address) {
    char dbg_buf[128]; // Increased buffer for longer messages
    if (!sd_card_info.initialized) {
        sprintf(dbg_buf, "SD_Log_Start: Card not initialized! Cannot start CSV logging.\r\n");
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        logging_active = false;
        return;
    }
    sd_current_log_block = start_block_address;
    sd_log_csv_buffer_char_count = 0;
    memset(sd_log_csv_buffer, 0, SD_BLOCK_SIZE); // Clear CSV buffer initially
    logging_active = true;
    sprintf(dbg_buf, "SD_Log_Start: CSV Logging will start at block %lu\r\n", sd_current_log_block);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);

    // Write the CSV header as the first thing
    SD_Log_Write_CSV_Header();
}

static void SD_Log_Write_CSV_Header(void) {
    char header_str[300]; // Ensure this is large enough for your header
    int header_len;

    if (!logging_active || !sd_card_info.initialized) return;

    // Define your CSV header string here
    header_len = snprintf(header_str, sizeof(header_str),
                          "Timestamp_ms,LSM_Acc_X_mg,LSM_Acc_Y_mg,LSM_Acc_Z_mg,LSM_Gyro_X_mdps,LSM_Gyro_Y_mdps,LSM_Gyro_Z_mdps,BMP_Pres_Pa,BMP_Alt_m,ADXL_Acc_X_g,ADXL_Acc_Y_g,ADXL_Acc_Z_g,Q0,Q1,Q2,Q3,KF_Alt_m,KF_Vel_mps,Loop_Exec_ms,Flight_Phase\r\n");

    if (header_len > 0 && (size_t)header_len < sizeof(header_str)) {
        if (sd_log_csv_buffer_char_count + header_len > SD_BLOCK_SIZE) {
            SD_Log_FlushBuffer(); // Flush if header won't fit current buffer (should be empty here ideally)
        }
        // Check again if buffer is clear for new data
        if (sd_log_csv_buffer_char_count + header_len <= SD_BLOCK_SIZE) {
            memcpy(&sd_log_csv_buffer[sd_log_csv_buffer_char_count], header_str, header_len);
            sd_log_csv_buffer_char_count += header_len;
            SD_Log_FlushBuffer(); // Flush header immediately to start clean file on SD
        } else {
            char dbg_buf[128];
            sprintf(dbg_buf, "SD_Log_Write_CSV_Header: CSV Header (len %d) too large for block size (%d). Not written.\r\n", header_len, SD_BLOCK_SIZE);
            HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        }
    } else {
        char dbg_buf[128];
        sprintf(dbg_buf, "SD_Log_Write_CSV_Header: snprintf failed or header too long (reported len: %d). Header not written.\r\n", header_len);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    }
}

void SD_Log_Data(const FlightData_LogEntry_t *log_entry) {
    char csv_line_str[350]; // Temporary buffer for a single CSV line, ensure it's large enough!
    int line_len;
    char dbg_buf[128];

    if (!logging_active || !sd_card_info.initialized) {
        return;
    }

    // Format the log_entry into a CSV string
    line_len = snprintf(csv_line_str, sizeof(csv_line_str),
                        "%lu,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%.1f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%lu,%u\r\n",
                        log_entry->timestamp_ms,
                        log_entry->lsm_acc_x_mg, log_entry->lsm_acc_y_mg, log_entry->lsm_acc_z_mg,
                        log_entry->lsm_gyro_x_mdps, log_entry->lsm_gyro_y_mdps, log_entry->lsm_gyro_z_mdps,
                        log_entry->bmp_pres_pa, log_entry->bmp_alt_m,
                        log_entry->adxl_hi_g_x, log_entry->adxl_hi_g_y, log_entry->adxl_hi_g_z,
                        log_entry->q0, log_entry->q1, log_entry->q2, log_entry->q3,
                        log_entry->kf_altitude_m, log_entry->kf_velocity_mps,
                        log_entry->loop_exec_time_ms, log_entry->flight_phase);

    if (line_len <= 0 || (size_t)line_len >= sizeof(csv_line_str)) {
        // snprintf failed or truncated, handle error
        sprintf(dbg_buf, "SD_Log_Data: snprintf failed or CSV line too long (reported len: %d). Data not logged.\r\n", line_len);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return;
    }

    // Check if the new line fits into the current CSV buffer
    if (sd_log_csv_buffer_char_count + line_len > SD_BLOCK_SIZE) {
        SD_Log_FlushBuffer(); // Buffer is full, flush it
    }

    // Check again: if after flush, the line still doesn't fit, it's too big for a block.
    if (sd_log_csv_buffer_char_count + line_len <= SD_BLOCK_SIZE) {
        memcpy(&sd_log_csv_buffer[sd_log_csv_buffer_char_count], csv_line_str, line_len);
        sd_log_csv_buffer_char_count += line_len;
    } else {
        // This implies a single CSV line is longer than SD_BLOCK_SIZE
        sprintf(dbg_buf, "SD_Log_Data: Single CSV line (len %d) too large for block size (%d). Data not logged.\r\n", line_len, SD_BLOCK_SIZE);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    }
}

void SD_Log_FlushBuffer(void) {
    char dbg_buf[128];
    if (!logging_active || !sd_card_info.initialized) {
        return;
    }

    if (sd_log_csv_buffer_char_count > 0) { // Only write if there's data in the buffer
        // Optional: sprintf(dbg_buf, "SD_Log_FlushBuffer: Writing %u chars to SD block %lu\r\n", (unsigned int)sd_log_csv_buffer_char_count, sd_current_log_block);
        // HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);

        // Ensure remaining part of the buffer is null if not full, SD_Write_Block writes 512 bytes.
        // memset after current data ensures padding with zeros.
        // This is already handled as sd_log_csv_buffer is memset to 0 after each successful flush.

        if (SD_Write_Block(sd_current_log_block, (uint8_t*)sd_log_csv_buffer) == 0) {
            // Optional: sprintf(dbg_buf, "SD_Log_FlushBuffer: Successfully wrote to block %lu\r\n", sd_current_log_block);
            // HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
            sd_current_log_block++;
        } else {
            sprintf(dbg_buf, "SD_Log_FlushBuffer: FAILED to write to block %lu\r\n", sd_current_log_block);
            HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
            logging_active = false; // Stop logging on write failure
        }
        // Clear the buffer for the next set of CSV lines
        memset(sd_log_csv_buffer, 0, SD_BLOCK_SIZE);
        sd_log_csv_buffer_char_count = 0;
    }
}

void SD_Log_Stop(void) {
    char dbg_buf[64];
    sprintf(dbg_buf, "SD_Log_Stop: Stopping logging.\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);

    if (logging_active && sd_card_info.initialized) {
        SD_Log_FlushBuffer(); // Ensure any remaining data is written
    }
    logging_active = false;
    // No need to de-init SD card here unless power saving is critical
    // and re-init is acceptable on next start.
}

void SD_Run_Test_Write(void) {
    uint8_t test_buffer[512];
    uint8_t read_buffer[512];
    char dbg_buf[128];
    uint8_t status;

    sprintf(dbg_buf, "--- SD Card Test Write ---\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);

    // Initialize SD card
    status = SD_Init();
    if (status != 0) {
        sprintf(dbg_buf, "SD Card Initialization Failed! Error code: %d\r\n", status);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        return;
    }
    sprintf(dbg_buf, "SD Card Initialized Successfully.\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);

    // Prepare test data
    for (int i = 0; i < 512; i++) {
        test_buffer[i] = (uint8_t)(i % 256);
    }
    test_buffer[0] = 'H'; test_buffer[1] = 'E'; test_buffer[2] = 'L'; test_buffer[3] = 'L'; test_buffer[4] = 'O';

    // Write data to block 0 (use a high block number like 1000 to avoid MBR if card is formatted)
    uint32_t test_block_addr = 1000;
    sprintf(dbg_buf, "Attempting to write to block %lu...\r\n", test_block_addr);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    status = SD_Write_Block(test_block_addr, test_buffer);
    if (status != 0) {
        sprintf(dbg_buf, "SD_Write_Block Failed! Error code: %d\r\n", status);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        // return; // Optionally return early
    } else {
        sprintf(dbg_buf, "SD_Write_Block Succeeded for block %lu.\r\n", test_block_addr);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    }
    
    // Optional: Read back and verify
    HAL_Delay(100); // Short delay before reading back
    memset(read_buffer, 0, 512);
    sprintf(dbg_buf, "Attempting to read back block %lu...\r\n", test_block_addr);
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    status = SD_Read_Block(test_block_addr, read_buffer);
    if (status != 0) {
        sprintf(dbg_buf, "SD_Read_Block Failed! Error code: %d\r\n", status);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
    } else {
        sprintf(dbg_buf, "SD_Read_Block Succeeded for block %lu.\r\n", test_block_addr);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        // Verify data
        bool match = true;
        for(int i=0; i<512; i++){
            if(test_buffer[i] != read_buffer[i]){
                match = false;
                sprintf(dbg_buf, "Data mismatch at index %d. Written: 0x%02X, Read: 0x%02X\r\n", i, test_buffer[i], read_buffer[i]);
                HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
                break;
            }
        }
        if(match){
            sprintf(dbg_buf, "Data verification successful for block %lu!\r\n", test_block_addr);
            HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
        }
    }

    sprintf(dbg_buf, "--- SD Card Test Write Complete ---\r\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)dbg_buf, strlen(dbg_buf), 100);
}


