#ifndef GPIO_CONFIG_H
#define GPIO_CONFIG_H

#include "driver/gpio.h"


#define GPIO_MFRC522_SCK       GPIO_NUM_18
#define GPIO_MFRC522_MISO      GPIO_NUM_19
#define GPIO_MFRC522_MOSI      GPIO_NUM_23
#define GPIO_MFRC522_CS        GPIO_NUM_13
#define GPIO_MFRC522_RST       GPIO_NUM_4

#define GPIO_LCD_SDA           GPIO_NUM_21
#define GPIO_LCD_SCL           GPIO_NUM_22

#define GPIO_IR_SLOT_1         GPIO_NUM_34
#define GPIO_IR_SLOT_2         GPIO_NUM_35
#define GPIO_IR_SLOT_3         GPIO_NUM_36
#define GPIO_IR_SLOT_4         GPIO_NUM_39

#define GPIO_SERVO_ENTRY       GPIO_NUM_27
#define GPIO_SERVO_EXIT        GPIO_NUM_14

#define GPIO_CAM_ENTRY_RX      GPIO_NUM_16
#define GPIO_CAM_ENTRY_TX      GPIO_NUM_17

#define GPIO_CAM_EXIT_RX       GPIO_NUM_25
#define GPIO_CAM_EXIT_TX       GPIO_NUM_26

#endif
