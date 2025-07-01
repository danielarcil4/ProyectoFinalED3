#include <stdio.h>

#include "pantalla/LCD.h"

#define SPI_PORT spi0

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

static inline int8_t __spi_write_blocking(void *spi_handle, const uint8_t *src, size_t len)
{
	return spi_write_blocking((spi_inst_t *)spi_handle, src, len);
}

int main()
{
    stdio_init_all();
    sleep_ms(1000); // Espera 1 segundo para que la consola se inicialice

    spi_init(SPI_PORT, 500 * 1000);

    struct LCD lcd;
    struct lcd_platform_config platform = {
        .spi_handle = SPI_PORT,
        .spi_write_blocking = __spi_write_blocking,
        .base_dma_channel = -1, // No se usa DMA en este ejemplo
    };

    SSD1283A_status status = lcd_init(&lcd, &platform);
    if (status != SSD1283A_STATUS_OK) {
        printf("Error initializing LCD: %d\n", status);
        return -1; // Error al inicializar el LCD
    }
    
    lcd_fill_screen(&lcd, 0x001F); // Llena la pantalla con color azul
    while (1);
}
