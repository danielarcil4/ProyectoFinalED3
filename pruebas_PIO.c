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
#define BUTTON_PIN      13

bool take_picture = false;

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

static void take_picture_callback(uint gpio, uint32_t events) {
	printf("gpio %d event: %d\n", gpio, events);
	if (gpio != BUTTON_PIN) {
		return; // Ignore other GPIO events
	}
	gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_RISE, false, take_picture_callback);
	take_picture = true;
}

int main() {
	stdio_init_all();

	// Wait some time for USB serial connection
	sleep_ms(1000);

	gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

	gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_RISE, true, take_picture_callback);

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

	lcd_fill_screen(&lcd, BLACK);

	while (1) {
		if(take_picture){
			take_picture = false;
			camera_term(&camera);
		}

		printf("Capturing...\n");
		gpio_put(LED_PIN, 1);
		ret = camera_capture_blocking(&camera, buf, true);
		gpio_put(LED_PIN, 0);
		if (ret != 0) {
			printf("Capture error: %d\n", ret);
		} else {
			printf("Capture success\n");
			uint8_t y, x;
			uint16_t image[height * width]; // Matriz completa en memoria
			printf("Capture success\n");
			for (y = 0; y < height; y++) {
				for (x = 0; x < buf->strides[0]; x+=2) {
					uint32_t idx = buf->strides[0] * y + x;
					uint16_t pixel = (buf->data[0][idx + 1]) | buf->data[0][idx]<<8;   
					image[width * y + x/2] = pixel;
					printf("Pixel at (%d, %d): 0x%04X\n", x, y, pixel);
				}
			}
			lcd_show_image(&lcd, width, height, image);
			sleep_ms(1000);
		}
	}
}