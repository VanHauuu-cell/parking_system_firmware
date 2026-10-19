#ifndef MFRC522_H
#define MFRC522_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// --- Lệnh của MFRC522 ---
#define PCD_IDLE              0x00
#define PCD_TRANSCEIVE        0x0C
#define PCD_RESETPHASE        0x0F

// --- Lệnh giao tiếp thẻ (PICC) ---
#define PICC_CMD_REQA         0x26 // Tìm thẻ chưa ở trạng thái Sleep
#define PICC_CMD_ANTICOLL     0x93 // Đọc UID

// --- Mã lỗi nội bộ ---
#define MFRC522_STATUS_OK     0
#define MFRC522_STATUS_ERROR  1
#define MFRC522_STATUS_NO_TAG 2

// --- Thanh ghi (Registers) ---
#define MFRC522_REG_COMMAND     0x01
#define MFRC522_REG_COMM_IE     0x02
#define MFRC522_REG_COMM_IRQ    0x04
#define MFRC522_REG_DIV_IRQ     0x05
#define MFRC522_REG_ERROR       0x06
#define MFRC522_REG_FIFO_DATA   0x09
#define MFRC522_REG_FIFO_LEVEL  0x0A
#define MFRC522_REG_CONTROL     0x0C
#define MFRC522_REG_BIT_FRAMING 0x0D
#define MFRC522_REG_MODE        0x11
#define MFRC522_REG_TX_CONTROL  0x14
#define MFRC522_REG_TX_ASK      0x15
#define MFRC522_REG_T_MODE      0x2A
#define MFRC522_REG_T_PRESCALER 0x2B
#define MFRC522_REG_T_RELOAD_H  0x2C
#define MFRC522_REG_T_RELOAD_L  0x2D
#define MFRC522_REG_VERSION     0x37

esp_err_t mfrc522_init(void);
esp_err_t mfrc522_write_reg(uint8_t reg, uint8_t val);
uint8_t mfrc522_read_reg(uint8_t reg);
uint8_t mfrc522_get_version(void);

void mfrc522_set_bit_mask(uint8_t reg, uint8_t mask);
void mfrc522_clear_bit_mask(uint8_t reg, uint8_t mask);
void mfrc522_antenna_on(void);

// Gửi dữ liệu đến thẻ và nhận dữ liệu từ thẻ
uint8_t mfrc522_to_card(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen);

//kiểm tra xem có thẻ không, reqMode = PICC_CMD_REQA hoặc PICC_CMD_WUPA
uint8_t mfrc522_request(uint8_t reqMode, uint8_t *tagType);

//doc the
uint8_t mfrc522_anticoll(uint8_t *serNum);

#endif // MFRC522_H