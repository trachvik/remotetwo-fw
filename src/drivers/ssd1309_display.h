#ifndef SSD1309_DISPLAY_H
#define SSD1309_DISPLAY_H

#include <stddef.h>
#include <stdint.h>

int ssd1309_display_init(void);
int ssd1309_display_write_cmd(uint8_t cmd);
int ssd1309_display_write_data(const uint8_t *data, size_t len);
int ssd1309_display_set_page_col(uint8_t page, uint8_t col);
int ssd1309_display_clear(void);

#endif /* SSD1309_DISPLAY_H */
