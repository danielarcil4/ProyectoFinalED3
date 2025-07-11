#ifndef __LCD__H
#define __LCD_H__

#include <stdint.h>

#include "hardware/spi.h"
#include "hardware/dma.h"
#include "SSD1283A.h"

#define CAMERA_MAX_N_PLANES 3

struct lcd_platform_config{
    // Funciones de acceso SPI (pueden ser wrappers para usar DMA o no)
    int8_t (*spi_write_blocking)(void *spi, const uint8_t *src, size_t len);
    spi_inst_t *spi_handle;

    // DMA
    int8_t base_dma_channel; // Deben ser varios canales. uno por plano.
};

struct lcd_config {
	uint32_t format;
	uint16_t width;
	uint16_t height;
	uint dma_transfers[CAMERA_MAX_N_PLANES];
	uint dma_offset[CAMERA_MAX_N_PLANES];
	dma_channel_config dma_cfgs[CAMERA_MAX_N_PLANES];
};

// Estructura para el controlador lcd
struct LCD{
    SSD1283A_host driver_host;
    int dma_channels[CAMERA_MAX_N_PLANES];
    struct lcd_config config;
};

// Prototipos de funciones
SSD1283A_status lcd_init(struct LCD *lcd, struct lcd_platform_config *platform);
void lcd_fill_screen(struct LCD *lcd, uint16_t color);
void lcd_show_image(struct LCD *lcd,uint16_t width, uint16_t height, uint16_t *color);


#endif // __LCD__H