#ifndef PARKING_FSM_CONFIG_H
#define PARKING_FSM_CONFIG_H

// One shared RC522, two physical cameras and separate barriers.
#define FSM_ENTRY_CAMERA_ID           1
#define FSM_EXIT_CAMERA_ID            2
// 0: select direction with Serial 'in'/'out'. 1: select expected direction from
// the ledger, then still require the correct plate at the selected camera.
#define FSM_AUTO_DIRECTION            1
#define FSM_ENABLE_SERIAL_COMMANDS    1
#define FSM_CAMERA_CONFIRMATION       1

#define FSM_OCR_TIMEOUT_MS            60000
#define FSM_MIN_OCR_CONFIDENCE        60
#define FSM_NOTICE_MS                 2500
#define FSM_PASSAGE_WARNING_MS        15000
#define FSM_CAMERA_CLEAR_STABLE_MS    1500
#define FSM_CAMERA_MIN_CLEAR_SAMPLES  3
#define FSM_CLOSE_DELAY_MS            3000
#define FSM_CAMERA_STATUS_STALE_MS    2000
// Tolerate slower observations while waiting. A new CLEAR is required after
// the close delay, and the shorter watchdog still applies during movement.
#define FSM_CAMERA_SAMPLE_GAP_MS       5000
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

// 0 = scan the PCF8574 addresses using your existing LCD driver.
#define FSM_LCD_ADDRESS              0

#endif
