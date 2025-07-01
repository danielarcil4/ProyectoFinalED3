#include "LCD.h"
#include "SSD1283A.h"
#include <stdio.h>

#define PIN_LED  15
#define PIN_DC   16
#define PIN_CS   17 
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_RST  20
#define PIN_VCC  22

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

    lcd->driver_host = (SSD1283A_host){
        .pins = &(SSD1283A_pins){
            .cs = PIN_CS,  // Chip Select pin
            .dc = PIN_DC,  // Data/Command pin
            .rst = PIN_RST, // Reset pin
            .led = PIN_LED, // Backlight pin
        },
		.platform = platform,
	};

    // Inicializa el controlador SSD1283A
    SSD1283A_status status = SSD1283A_begin(&lcd->driver_host);

    if (status != SSD1283A_STATUS_OK) {
        return status; // Error al inicializar el controlador
    }
    
    return SSD1283A_STATUS_OK; // Inicialización exitosa
}

void SSD1283A_write_command(SSD1283A_host *host, void *platform, uint8_t command){
    struct lcd_platform_config *pcfg = (struct lcd_platform_config *)platform;

    // Asume que tienes control de DC y CS en tu plataforma
    // 1. Selecciona el chip
    cs_select(host);

    // 2. DC para comando
    dc_command(host);
    pcfg->spi_write_blocking(pcfg->spi_handle, &command, 1);

    // 3. Deselecciona el chip
    cs_deselect(host);
}

void SSD1283A_write_data(SSD1283A_host *host, void *platform, uint8_t data)
{
    struct lcd_platform_config *pcfg = (struct lcd_platform_config *)platform;

    // Asume que tienes control de DC y CS en tu plataforma
    // 1. Selecciona el chip
    cs_select(host);

    // 2. DC para datos
    dc_data(host);
    pcfg->spi_write_blocking(pcfg->spi_handle, &data, 1);

    // 3. Deselecciona el chip
    cs_deselect(host);
}

void SSD1283A_write_register(SSD1283A_host *host, void *platform, uint8_t reg, uint16_t value)
{
    struct lcd_platform_config *pcfg = (struct lcd_platform_config *)platform;

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

void SSD1283A_write_data_prueba(SSD1283A_host *host, void *platform, uint16_t data)
{
    struct lcd_platform_config *pcfg = (struct lcd_platform_config *)platform;

    // Asume que tienes control de DC y CS en tu plataforma
    // 1. Selecciona el chip
    cs_select(host);

    // 2. DC para datos
    dc_data(host);
    // Enviar el dato de 16 bits como dos bytes
    uint8_t data_buf[2] = { data >> 8, data & 0xFF };
    pcfg->spi_write_blocking(pcfg->spi_handle, data_buf, 2);

    // 3. Deselecciona el chip
    cs_deselect(host);
}

void lcd_fill_screen(struct LCD *lcd, uint16_t color) {
    // Establece la ventana de dirección
    uint16_t Ncolumn = 131<<8; // Posición final
    uint16_t Nrow = 131<<8; // Posición final

    SSD1283A_write_register(&lcd->driver_host, lcd->driver_host.platform, SSD1283A_CMD_HORIZONTAL_RAM_ADDR, Ncolumn); // GRAM Address Set. // 0X00 start x column
    SSD1283A_write_register(&lcd->driver_host, lcd->driver_host.platform, SSD1283A_CMD_VERTICAL_RAM_ADDR, Nrow); // GRAM Address Set. // 0X00 start y row

    SSD1283A_write_command(&lcd->driver_host, lcd->driver_host.platform, SSD1283A_CMD_SET_GDDRAM_XY); // Set GDDRAM X and Y
    SSD1283A_write_data(&lcd->driver_host, lcd->driver_host.platform, 0x00); // Columna
    SSD1283A_write_data(&lcd->driver_host, lcd->driver_host.platform, 0x00); // Fila

    SSD1283A_write_command(&lcd->driver_host, lcd->driver_host.platform, SSD1283A_CMD_RAM_WRITE); // Iniciar escritura de datos
    for (uint32_t i = 0; i < (Ncolumn * Nrow); i++) {
        // Enviar el color
        SSD1283A_write_data(&lcd->driver_host, lcd->driver_host.platform, color >> 8);     // Byte alto
        SSD1283A_write_data(&lcd->driver_host, lcd->driver_host.platform, color & 0xFF);   // Byte bajo
    }
}
