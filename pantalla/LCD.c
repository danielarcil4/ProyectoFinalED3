#include "LCD.h"
#include <stdio.h>

#define PIN_DC   16
#define PIN_CS   17 
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_RST  20
#define PIN_VCC  15
#define PIN_LED  22

static inline void cs_select(SSD1283A_host *host) {
    asm volatile("nop \n nop \n nop");
    gpio_put(host->pins->cs, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(SSD1283A_host *host) {
    asm volatile("nop \n nop \n nop");
    gpio_put(host->pins->cs, 1);
    asm volatile("nop \n nop \n nop");
}

static inline void dc_command(SSD1283A_host *host) {
    asm volatile("nop \n nop \n nop");
    gpio_put(host->pins->dc, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void dc_data(SSD1283A_host *host) {
    asm volatile("nop \n nop \n nop");
    gpio_put(host->pins->dc, 1);
    asm volatile("nop \n nop \n nop");
}

SSD1283A_status lcd_init(struct LCD *lcd, struct lcd_platform_config *platform) {
    *lcd = (struct LCD){ 0 };

    gpio_init(PIN_VCC);
    gpio_set_dir(PIN_VCC, GPIO_OUT);
    gpio_put(PIN_VCC, 1); // Encender VCC

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1); // Mantener RST alto

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1); // Encender LED de retroiluminación
    
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_DC);
    gpio_set_dir(PIN_DC, GPIO_OUT);  
    gpio_set_function(PIN_DC, GPIO_FUNC_SIO);

    static SSD1283A_pins pins = {
        .cs = PIN_CS,
        .dc = PIN_DC,
        .rst = PIN_RST,
        .led = PIN_LED,
    };

    lcd->driver_host = (SSD1283A_host){
        .pins = &pins,
        .platform = platform,
    };

    // Inicializa el controlador SSD1283A
    SSD1283A_status status = SSD1283A_begin(&lcd->driver_host);
    if (status != SSD1283A_STATUS_OK) {
        return status; // Error al inicializar el controlador
    }
    
    return SSD1283A_STATUS_OK; // Inicialización exitosa
}

void SSD1283A_write_command(SSD1283A_host *host, struct lcd_platform_config *pcfg, uint8_t command){
    // Asume que tienes control de DC y CS en tu plataforma
    // 1. Selecciona el chip
    cs_select(host);

    // 2. DC para comando
    dc_command(host);
    pcfg->spi_write_blocking(pcfg->spi_handle, &command, 1);

    // 3. Deselecciona el chip
    cs_deselect(host);
}

void SSD1283A_write_data(SSD1283A_host *host, struct lcd_platform_config *pcfg, uint8_t data)
{
    // Asume que tienes control de DC y CS en tu plataforma
    // 1. Selecciona el chip
    cs_select(host);

    // 2. DC para datos
    dc_data(host);
    pcfg->spi_write_blocking(pcfg->spi_handle, &data, 1);

    // 3. Deselecciona el chip
    cs_deselect(host);
}

void SSD1283A_write_register(SSD1283A_host *host, struct lcd_platform_config *pcfg, uint8_t reg, uint16_t value)
{
    // Asume que tienes control de DC y CS en tu plataforma
    // 1. Selecciona el chip
    cs_select(host);

    // 2. DC bajo para comando
    dc_command(host);
    pcfg->spi_write_blocking(pcfg->spi_handle, &reg, 1);

    // 3. DC alto para datos
    dc_data(host);
    uint8_t val_buf[2] = { value >> 8, value & 0xFF };
    pcfg->spi_write_blocking(pcfg->spi_handle, val_buf, 2);

    // 4. Deselecciona el chip
    cs_deselect(host);
}

void SSD1283A_write_color_16bit(SSD1283A_host *host, struct lcd_platform_config *pcfg, uint16_t color) {
    uint8_t buf[2] = { color >> 8, color & 0xFF };

    cs_select(host);
    dc_data(host);
    pcfg->spi_write_blocking(pcfg->spi_handle, buf, 2);
    cs_deselect(host);
}

void lcd_fill_screen(struct LCD *lcd, uint16_t *color) {
    uint8_t x_start = 0x00;
    uint8_t x_end   = 80;
    uint8_t y_start = 0x00;
    uint8_t y_end   = 60;

    struct lcd_platform_config *platform = (struct lcd_platform_config *)lcd->driver_host.platform;

    // 1. Set horizontal window: reg 0x44 (HEA << 8) | HSA
    SSD1283A_write_register(&lcd->driver_host, platform, SSD1283A_CMD_HORIZONTAL_RAM_ADDR, (x_end << 8) | x_start);

    // 2. Set vertical window: reg 0x45 (VEA << 8) | VSA
    SSD1283A_write_register(&lcd->driver_host, platform, SSD1283A_CMD_VERTICAL_RAM_ADDR, (y_end << 8) | y_start);

    // 3. Set GDDRAM address pointer: reg 0x21 (Y << 8) | X
    SSD1283A_write_command(&lcd->driver_host, platform, SSD1283A_CMD_SET_GDDRAM_XY);
    SSD1283A_write_data(&lcd->driver_host, platform, x_start); // X address
    SSD1283A_write_data(&lcd->driver_host, platform, y_start); // Y address

    // 4. RAM write command
    SSD1283A_write_command(&lcd->driver_host, platform, SSD1283A_CMD_RAM_WRITE);
    // 5. Write pixels (RGB565 format)
    for (uint32_t i = 0; i < (x_end - x_start) * (y_end - y_start); i++) {
        SSD1283A_write_color_16bit(&lcd->driver_host, platform, color[i]);
    }
}

