#ifndef PARKING_FSM_CONFIG_H
#define PARKING_FSM_CONFIG_H

// One shared RC522, two physical cameras and separate barriers.
#define FSM_ENTRY_CAMERA_ID           1
#define FSM_EXIT_CAMERA_ID            2
// 0: select direction with Serial 'in'/'out'. 1: select expected direction from
// the ledger, then still require the correct plate at the selected camera.
#define FSM_AUTO_DIRECTION            1
#define FSM_ENABLE_SERIAL_COMMANDS    1

#define FSM_OCR_TIMEOUT_MS            60000
#define FSM_MIN_OCR_CONFIDENCE        60
#define FSM_NOTICE_MS                 2500
#define FSM_PASSAGE_WARNING_MS        10000
#define FSM_GATE_IR_POLL_MS           20
#define FSM_GATE_IR_DEBOUNCE_SAMPLES  3
#define FSM_GATE_IR_CLEAR_STABLE_MS   500
#define FSM_GATE_IR_MAX_GAP_MS        250
#define FSM_CLOSE_DELAY_MS            3000
#define FSM_RFID_SCAN_MS              100
#define FSM_CARD_RELEASE_MS           500
#define FSM_IR_POLL_MS                50
#define FSM_IR_DEBOUNCE_SAMPLES       3

// Set these to the angles you already verified on your model before running.
#define FSM_ENTRY_CLOSED_ANGLE        0
#define FSM_ENTRY_OPEN_ANGLE          90
#define FSM_EXIT_CLOSED_ANGLE         0
#define FSM_EXIT_OPEN_ANGLE           90
#define FSM_SERVO_TRAVEL_MS           1000
#define FSM_GRANT_BEEP_MS             150

// 0 = probe SSD1306 addresses 0x3C and 0x3D.
#define FSM_OLED_ADDRESS             0

#endif
