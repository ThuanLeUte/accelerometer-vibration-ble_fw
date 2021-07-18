/**
 * @file       bsp_lcd.h
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Board Support LCD for Spo2 and Heartrate board
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_LCD_H
#define __BSP_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "gc9a01.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef enum
{
  LCD_BACKGROUND,
  LCD_BATT_FULL,
  LCD_BATT_75,
  LCD_BATT_50,
  LCD_BATT_25,
  LCD_BATT_0,
  LCD_DOT,
  LCD_DOT_N,
  LCD_SP02_NUM,
  LCD_HEART_RATE_NUM,
  LCD_ITEM_CNT
}
bsp_lcd_item_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         LCD init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_lcd_init(void);

/* Public function for project ---------------------------------------- */
/**
 * @brief         LDC display image
 *
 * @param[in]     item    Image item
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_lcd_display_image(bsp_lcd_item_t item);

/**
 * @brief         LDC display number
 *
 * @param[in]     item    Image item
 * @param[in]     num     Number
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_lcd_display_number(bsp_lcd_item_t item, uint8_t num);
#define bsp_lcd_display_spo2_number(num)      bsp_lcd_display_number(LCD_SP02_NUM, num)
#define bsp_lcd_display_heartrate_number(num) bsp_lcd_display_number(LCD_HEART_RATE_NUM , num)

/* Public function basic --------------------------------------------- */
/**
 * @brief         LCD fill by color
 *
 * @param[in]     color   Color
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_lcd_fill(uint16_t color);

/**
 * @brief         LCD write string
 *
 * @param[in]     x         X position
 * @param[in]     y         Y position
 * @param[in]     c         Char array
 * @param[in]     color     Color
 * @param[in]     bg_color  Background color
 * @param[in]     size      Size
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_lcd_write_string(uint16_t x, uint16_t y, const char c[],
                          uint16_t color, uint16_t bgcolor, uint8_t size);

/* -------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __BSP_LCD_H

/* End of file -------------------------------------------------------- */
