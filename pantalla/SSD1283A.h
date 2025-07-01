#ifndef __SSD1283A_H__
#define __SSD1283A_H__

#include <stdint.h>
#include "pico/stdlib.h"

typedef uint8_t SSD1283A_pin;

typedef enum {
  SSD1283A_STATUS_OK = 0,         ///< Success
  SSD1283A_STATUS_ERR_MALLOC,     ///< malloc() call failed
  SSD1283A_STATUS_ERR_PERIPHERAL, ///< Peripheral (e.g. timer) not found
} SSD1283A_status;

typedef struct {
    SSD1283A_pin cs;  // Chip Select pin
    SSD1283A_pin dc;  // Data/Command pin
    SSD1283A_pin rst; // Reset pin
    SSD1283A_pin led; // Backlight pin 
} SSD1283A_pins;

typedef struct {
  uint16_t reg;   ///< Register address
  uint16_t value; ///< Value to store
} SSD1283A_command;

typedef struct {
  SSD1283A_pins *pins; ///< Physical connection to camera
  void *platform;    ///< Platform-specific data
} SSD1283A_host;

// SSD1283A Register Addresses (Command Set)
#define SSD1283A_CMD_OSCILLATION_START     0x00
#define SSD1283A_CMD_DRIVER_OUTPUT_CONTROL 0x01
#define SSD1283A_CMD_LCD_DRIVE_AC_CONTROL  0x02
#define SSD1283A_CMD_ENTRY_MODE            0x03
#define SSD1283A_CMD_COMPARE_REGISTER1     0x04
#define SSD1283A_CMD_COMPARE_REGISTER2     0x05
#define SSD1283A_CMD_DISPLAY_CONTROL       0x07
#define SSD1283A_CMD_FRAME_CYCLE_CONTROL   0x0B
#define SSD1283A_CMD_POWER_CONTROL1        0x10
#define SSD1283A_CMD_POWER_CONTROL2        0x11
#define SSD1283A_CMD_POWER_CONTROL3        0x12
#define SSD1283A_CMD_POWER_CONTROL4        0x13
#define SSD1283A_CMD_POWER_CONTROL5        0x1E
#define SSD1283A_CMD_POWER_CONTROL6        0x1F

#define SSD1283A_CMD_HORIZONTAL_PORCH      0x16
#define SSD1283A_CMD_VERTICAL_PORCH        0x17

#define SSD1283A_CMD_RAM_WRITE_MASK1       0x23
#define SSD1283A_CMD_RAM_WRITE_MASK2       0x24

#define SSD1283A_CMD_GAMMA_CONTROL1        0x30
#define SSD1283A_CMD_GAMMA_CONTROL2        0x31
#define SSD1283A_CMD_GAMMA_CONTROL3        0x32
#define SSD1283A_CMD_GAMMA_CONTROL4        0x33
#define SSD1283A_CMD_GAMMA_CONTROL5        0x34
#define SSD1283A_CMD_GAMMA_CONTROL6        0x35
#define SSD1283A_CMD_GAMMA_CONTROL7        0x36
#define SSD1283A_CMD_GAMMA_CONTROL8        0x37
#define SSD1283A_CMD_GAMMA_CONTROL9        0x38
#define SSD1283A_CMD_GAMMA_CONTROL10       0x39

#define SSD1283A_CMD_GATE_SCAN_POS         0x40
#define SSD1283A_CMD_VERT_SCROLL_CONTROL   0x41
#define SSD1283A_CMD_FIRST_OUTPUT_POS      0x42
#define SSD1283A_CMD_SECOND_OUTPUT_POS     0x43
#define SSD1283A_CMD_HORIZONTAL_RAM_ADDR   0x44
#define SSD1283A_CMD_VERTICAL_RAM_ADDR     0x45

#define SSD1283A_CMD_RAM_WRITE             0x22  // Iniciar escritura de datos
#define SSD1283A_CMD_SET_GDDRAM_XY         0x21  // RAM write address (posición actual)


SSD1283A_status SSD1283A_begin(SSD1283A_host *host);
void SSD1283A_write_list(SSD1283A_host *host, void *platform,const SSD1283A_command *cmd);

#endif // __SSD1283A_H__