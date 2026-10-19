#include "mfrc522.h"
#include "config/gpio_config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MFRC522";
static spi_device_handle_t spi_handle;

esp_err_t mfrc522_write_reg(uint8_t reg, uint8_t val) {
    // Format địa chỉ cho mode WRITE
    uint8_t addr = (reg << 1) & 0x7E; 
    uint8_t data[2] = {addr, val};

    spi_transaction_t t = {
        .length = 16,        // 2 bytes = 16 bits
        .tx_buffer = data,
    };
    return spi_device_polling_transmit(spi_handle, &t);
}

uint8_t mfrc522_read_reg(uint8_t reg) {
    // Format địa chỉ cho mode READ
    uint8_t addr = ((reg << 1) & 0x7E) | 0x80;
    uint8_t tx_data[2] = {addr, 0x00}; // Byte thứ 2 là dummy byte để đẩy clock
    uint8_t rx_data[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI Read Failed");
        return 0;
    }
    // Dữ liệu nhận được nằm ở byte thứ 2
    return rx_data[1]; 
}

esp_err_t mfrc522_init(void) {
    esp_err_t ret;

    // 1. Cấu hình chân RST
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << GPIO_MFRC522_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&rst_conf);
    
    // Kéo RST lên mức cao để module hoạt động
    gpio_set_level(GPIO_MFRC522_RST, 1); 
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Khởi tạo SPI Bus
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_MFRC522_MISO,
        .mosi_io_num = GPIO_MFRC522_MOSI,
        .sclk_io_num = GPIO_MFRC522_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    // Sử dụng SPI3_HOST (hoặc VSPI_HOST)
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Loi khoi tao SPI bus");
        return ret;
    }

    // 3. Khởi tạo SPI Device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 5 * 1000 * 1000, // 5 MHz
        .mode = 0,                         // MFRC522 dùng SPI mode 0
        .spics_io_num = GPIO_MFRC522_CS,
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Loi add SPI device");
        return ret;
    }

    // 4. Soft Reset module MFRC522
    mfrc522_write_reg(MFRC522_REG_COMMAND, 0x0F); 
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "MFRC522 SPI Khoi tao thanh cong");

    mfrc522_write_reg(MFRC522_REG_T_MODE, 0x8D);
    mfrc522_write_reg(MFRC522_REG_T_PRESCALER, 0x3E);
    mfrc522_write_reg(MFRC522_REG_T_RELOAD_L, 30);
    mfrc522_write_reg(MFRC522_REG_T_RELOAD_H, 0);

    mfrc522_write_reg(MFRC522_REG_TX_ASK, 0x40);
    mfrc522_write_reg(MFRC522_REG_MODE, 0x3D);

    mfrc522_antenna_on();

    return ESP_OK;
    
}

uint8_t mfrc522_get_version(void) {
    return mfrc522_read_reg(MFRC522_REG_VERSION);
}

void mfrc522_set_bit_mask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = mfrc522_read_reg(reg);
    mfrc522_write_reg(reg, tmp | mask);
}

void mfrc522_clear_bit_mask(uint8_t reg, uint8_t mask) {
    uint8_t tmp = mfrc522_read_reg(reg);
    mfrc522_write_reg(reg, tmp & (~mask));
}

void mfrc522_antenna_on(void) {
    uint8_t temp = mfrc522_read_reg(MFRC522_REG_TX_CONTROL);
    if (!(temp & 0x03)) {
        mfrc522_set_bit_mask(MFRC522_REG_TX_CONTROL, 0x03);
    }
}

uint8_t mfrc522_to_card(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint32_t *backLen) {
    uint8_t status = MFRC522_STATUS_ERROR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits, n;
    uint32_t i;

    if (command == PCD_TRANSCEIVE) {
        irqEn = 0x77;
        waitIRq = 0x30;
    }

    mfrc522_write_reg(MFRC522_REG_COMM_IE, irqEn | 0x80);
    mfrc522_clear_bit_mask(MFRC522_REG_COMM_IRQ, 0x80);
    mfrc522_set_bit_mask(MFRC522_REG_FIFO_LEVEL, 0x80); // Xóa bộ đệm FIFO
    
    mfrc522_write_reg(MFRC522_REG_COMMAND, PCD_IDLE);

    // Ghi dữ liệu cần gửi vào FIFO
    for (i = 0; i < sendLen; i++) {
        mfrc522_write_reg(MFRC522_REG_FIFO_DATA, sendData[i]);
    }

    // Ra lệnh truyền
    mfrc522_write_reg(MFRC522_REG_COMMAND, command);
    if (command == PCD_TRANSCEIVE) {
        mfrc522_set_bit_mask(MFRC522_REG_BIT_FRAMING, 0x80); // Bắt đầu truyền
    }

    // Chờ nhận dữ liệu hoặc timeout
    i = 2000;
    do {
        n = mfrc522_read_reg(MFRC522_REG_COMM_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    mfrc522_clear_bit_mask(MFRC522_REG_BIT_FRAMING, 0x80);

    if (i != 0) {
        if (!(mfrc522_read_reg(MFRC522_REG_ERROR) & 0x1B)) {
            status = MFRC522_STATUS_OK;
            if (n & irqEn & 0x01) { status = MFRC522_STATUS_NO_TAG; }
            if (command == PCD_TRANSCEIVE) {
                n = mfrc522_read_reg(MFRC522_REG_FIFO_LEVEL);
                lastBits = mfrc522_read_reg(MFRC522_REG_CONTROL) & 0x07;
                if (lastBits) { *backLen = (n - 1) * 8 + lastBits; } 
                else { *backLen = n * 8; }
                
                if (n == 0) { n = 1; }
                if (n > 16) { n = 16; } // Max buffer
                
                for (i = 0; i < n; i++) {
                    backData[i] = mfrc522_read_reg(MFRC522_REG_FIFO_DATA);
                }
            }
        } else { status = MFRC522_STATUS_ERROR; }
    }
    return status;
}

uint8_t mfrc522_request(uint8_t reqMode, uint8_t *tagType) {
    uint8_t status;  
    uint32_t backBits;
    
    mfrc522_write_reg(MFRC522_REG_BIT_FRAMING, 0x07); // Gửi 7 bits trong lệnh Request
    
    tagType[0] = reqMode;
    status = mfrc522_to_card(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
    
    if ((status != MFRC522_STATUS_OK) || (backBits != 0x10)) {
        status = MFRC522_STATUS_ERROR;
    }
    return status;
}

uint8_t mfrc522_anticoll(uint8_t *serNum) {
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint32_t unLen;
    
    mfrc522_write_reg(MFRC522_REG_BIT_FRAMING, 0x00);
    
    serNum[0] = PICC_CMD_ANTICOLL;
    serNum[1] = 0x20; // Yêu cầu gửi UID
    
    status = mfrc522_to_card(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
    
    if (status == MFRC522_STATUS_OK) {
        // Checksum UID (Byte thứ 5 là XOR của 4 byte đầu)
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[4]) {
            status = MFRC522_STATUS_ERROR;
        }
    }
    return status;
}