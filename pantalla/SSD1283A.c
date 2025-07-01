#include "SSD1283A.h"

//#define digital_write(pin, hi) gpio_put(pin, hi ? 1 : 0)
#define TFTLCD_DELAY16 0xFF

extern void SSD1283A_write_register(SSD1283A_host *host, void *platform, uint8_t reg, uint16_t value);

static const SSD1283A_command SSD1283A_init[] = {
    {0x10, 0x2F8E},
    {0x11, 0x000C},
    {0x07, 0x0021},
    {0x28, 0x0006},
    {0x28, 0x0005},
    {0x27, 0x057F},
    {0x29, 0x89A1},
    {0x00, 0x0001},
    {TFTLCD_DELAY16, 100},
    {0x29, 0x80B0},
    {TFTLCD_DELAY16, 30},
    {0x29, 0xFFFE},
    {0x07, 0x0223},
    {TFTLCD_DELAY16, 30},
    {0x07, 0x0233},
    {0x01, 0x2183},
    {0x03, 0x6830},
    {0x2F, 0xFFFF},
    {0x2C, 0x8000},
    {0x27, 0x0570},
    {0x02, 0x0300},
    {0x0B, 0x580C},
    {0x12, 0x0609},
    {0x13, 0x3100},
};

SSD1283A_status SSD1283A_begin(SSD1283A_host *host) {
    gpio_put(host->pins->rst, 0);
    sleep_ms(50);
    gpio_put(host->pins->rst, 1);

    // Initialize the display
    SSD1283A_write_list(host, host->platform, SSD1283A_init);

    return SSD1283A_STATUS_OK;
}

void SSD1283A_write_list(SSD1283A_host *host,void *platform, const SSD1283A_command *cmd) {
    for (int i = 0; i < sizeof(SSD1283A_init) / sizeof(SSD1283A_command); i ++) {
            uint16_t reg = cmd[i].reg;
            uint16_t val = cmd[i].value;
            if (reg == TFTLCD_DELAY16) {
                sleep_ms(val);
            } else {
                SSD1283A_write_register(host, platform, reg, val);
                sleep_ms(1); // Small delay after each command
            }
        }
}