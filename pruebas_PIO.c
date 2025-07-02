/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "camera/camera.h"
#include "camera/format.h"
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

#define CAMERA_PIO      pio0
#define CAMERA_BASE_PIN 2
#define CAMERA_XCLK_PIN 21
#define CAM_RET_PIN     14
#define CAMERA_SDA      0
#define CAMERA_SCL      1

static inline int __i2c_write_blocking(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len)
{
	return i2c_write_blocking((i2c_inst_t *)i2c_handle, addr, src, len, false);
}

static inline int __i2c_read_blocking(void *i2c_handle, uint8_t addr, uint8_t *dst, size_t len)
{
	return i2c_read_blocking((i2c_inst_t *)i2c_handle, addr, dst, len, false);
}

static inline int8_t __spi_write_blocking(void *spi_handle, const uint8_t *src, size_t len)
{
	return spi_write_blocking((spi_inst_t *)spi_handle, src, len);
}

// From http://www.paulbourke.net/dataformats/asciiart/
const char charmap[] = " .:-=+*#%@";

int main() {
	stdio_init_all();

	// Wait some time for USB serial connection
	sleep_ms(1000);

	gpio_init(CAM_RET_PIN);
    gpio_set_dir(CAM_RET_PIN, GPIO_OUT);
    gpio_put(CAM_RET_PIN, 1);

	const uint LED_PIN = PICO_DEFAULT_LED_PIN;
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	i2c_init(i2c0, 100000);
	gpio_set_function(CAMERA_SDA, GPIO_FUNC_I2C);
	gpio_set_function(CAMERA_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(CAMERA_SDA);
	gpio_pull_up(CAMERA_SCL);

	spi_init(SPI_PORT, 500 * 1000);

	struct camera camera;
	struct camera_platform_config platform_camera = {
		.i2c_write_blocking = __i2c_write_blocking,
		.i2c_read_blocking = __i2c_read_blocking,
		.i2c_handle = i2c0,

		.pio = CAMERA_PIO,
		.xclk_pin = CAMERA_XCLK_PIN,
		.xclk_divider = 9,
		.base_pin = CAMERA_BASE_PIN,
		.base_dma_channel = -1,
	};

	int ret = camera_init(&camera, &platform_camera);
	if (ret) {
		printf("camera_init failed: %d\n", ret);
		return 1;
	}

	const uint16_t width = CAMERA_WIDTH_DIV8;
	const uint16_t height = CAMERA_HEIGHT_DIV8;

	struct camera_buffer *buf = camera_buffer_alloc(FORMAT_RGB565, width, height);
	assert(buf);

	struct LCD lcd;
    struct lcd_platform_config platform_lcd = {
        .spi_handle = SPI_PORT,
        .spi_write_blocking = __spi_write_blocking,
        .base_dma_channel = -1, // No se usa DMA en este ejemplo
    };

    SSD1283A_status status = lcd_init(&lcd, &platform_lcd);
    if (status != SSD1283A_STATUS_OK) {
        printf("Error initializing LCD: %d\n", status);
        return -1; // Error al inicializar el LCD
    }

	while (1) {
		printf("Capturing...\n");
		gpio_put(LED_PIN, 1);
		ret = camera_capture_blocking(&camera, buf, true);
		gpio_put(LED_PIN, 0);
		if (ret != 0) {
			printf("Capture error: %d\n", ret);
		} else {
			printf("Capture success\n");
			int y, x;
			uint16_t image[height * width]; // Matriz completa en memoria
			for (y = 0; y < height; y++) {
				uint16_t row[width];
				for (x = 0; x < width; x++) {
					uint16_t pixel565 = buf->data[0][buf->strides[0] * y + x];
					uint8_t r = (pixel565 >> 11) & 0x1F;
					uint8_t g = (pixel565 >> 5) & 0x3F;
					uint8_t b = pixel565 & 0x1F;
					printf("r: %d, g: %d, b: %d\n", r, g, b);
					// Convertir a RGB565
					image[y * width + x] = (r  << 11) | (g  << 5) | b; ;
				}
			}
			//camera_term(&camera);
			lcd_fill_screen(&lcd, image); // Llenar pantalla con el primer pixel
			sleep_ms(1000); // Esperar un segundo antes de la siguiente captura
		}
	}
}