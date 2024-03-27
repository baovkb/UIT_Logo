#include <ssd1306.h>
#include <esp_err.h>
#include <esp_log.h>
#include <string.h>
#include "driver/i2c.h"
#include "font8x8_basic.h"

static char *TAG = "ssd1306";
uint8_t buffer[128][8];

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(TAG, "OLED configured successfully");
	} else {
		ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_text(const void *arg_text) {
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	uint8_t cur_page = 0;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, 0x00, true); // reset column - choose column --> 0
	i2c_master_write_byte(cmd, 0x10, true); // reset line - choose line --> 0
	i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0; i < text_len; i++) {
		if (text[i] == '\n') {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, 0x00, true); // reset column
			i2c_master_write_byte(cmd, 0x10, true);
			i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		} else {
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
		}
	}

	vTaskDelete(NULL);
}

void task_ssd1306_display_clear() {
	i2c_cmd_handle_t cmd;

	uint8_t clear[128];
	for (uint8_t i = 0; i < 128; i++) {
		clear[i] = 0;
	}
	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, clear, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}
	vTaskDelete(NULL);
}

void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }

    if (color == 1) {
        buffer[x][y/8] |= (1 << (y%8)); // Set bit lên 1
    } else {
        buffer[x][y/8] &= ~(1 << (y%8)); // Clear bit xuống 0
    }
}

void task_ssd1306_draw_bitmap(const uint8_t *bitmap, uint8_t xPos, uint8_t yPos, uint8_t width, uint8_t height) {

    uint8_t k = (width + 7) / 8;
    uint8_t i, j, pixel;
	uint16_t index;

    for (j = 0; j < height; j++) {
        for (i = 0; i < width; i++) {
            index = i / 8 + j * k;
            uint8_t q = 7 - (i % 8);
            pixel = (bitmap[index] >> q) & 0x01;
			ssd1306_draw_pixel(xPos + i, yPos + j, pixel);
        }
    }

	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
    i2c_master_write_byte(cmd, 0xAE, true);  //tat man hinh
    i2c_master_write_byte(cmd, 0xA1, true);  // Ánh xạ hàng đầu tiên lên trên
	

	i2c_master_write_byte(cmd, 0x20, true);  // Đặt chế độ ánh xạ
	i2c_master_write_byte(cmd, 0x01, true);
	
	uint8_t tmp2[] = {0x21, 0, 127};
	i2c_master_write(cmd, tmp2, sizeof(tmp2), true);
	tmp2[0] = 0x22;
	tmp2[2] = 7;
	i2c_master_write(cmd, tmp2, sizeof(tmp2), true);

	i2c_master_write_byte(cmd, 0xAF, true); 

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 100/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
	i2c_master_write(cmd, buffer, sizeof(buffer), true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_1, cmd, 100/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}