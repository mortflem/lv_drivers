/**
 * @file SSD1963.h
 *
 */

#ifndef SSD1963_H
#define SSD1963_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifndef LV_DRV_NO_CONF
#ifdef LV_CONF_INCLUDE_SIMPLE
#include "lv_drv_conf.h"
#else
#include "../../lv_drv_conf.h"
#endif
#endif

#if USE_SSD1963

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/
// SSD1963 command table
#define CMD_NOP                ((uint8_t)0x00) //No operation
#define CMD_SOFT_RESET         ((uint8_t)0x01) //Software reset
#define CMD_GET_PWR_MODE       ((uint8_t)0x0A) //Get the current power mode
#define CMD_GET_ADDR_MODE      ((uint8_t)0x0B) //Get the frame memory to the display panel read order
#define CMD_GET_PIXEL_FORMAT   ((uint8_t)0x0C) //Get the current pixel format
#define CMD_GET_DISPLAY_MODE   ((uint8_t)0x0D) //Returns the display mode
#define CMD_GET_SIGNAL_MODE    ((uint8_t)0x0E) //
#define CMD_GET_DIAGNOSTIC     ((uint8_t)0x0F)
#define CMD_ENT_SLEEP          ((uint8_t)0x10)
#define CMD_EXIT_SLEEP         ((uint8_t)0x11)
#define CMD_ENT_PARTIAL_MODE   ((uint8_t)0x12)
#define CMD_ENT_NORMAL_MODE    ((uint8_t)0x13)
#define CMD_EXIT_INVERT_MODE   ((uint8_t)0x20)
#define CMD_ENT_INVERT_MODE    ((uint8_t)0x21)
#define CMD_SET_GAMMA          ((uint8_t)0x26)
#define CMD_BLANK_DISPLAY      ((uint8_t)0x28)
#define CMD_ON_DISPLAY         ((uint8_t)0x29)
#define CMD_SET_COLUMN         ((uint8_t)0x2A)
#define CMD_SET_PAGE           ((uint8_t)0x2B)
#define CMD_WR_MEMSTART        ((uint8_t)0x2C)
#define CMD_RD_MEMSTART        ((uint8_t)0x2E)
#define CMD_SET_PARTIAL_AREA   ((uint8_t)0x30)
#define CMD_SET_SCROLL_AREA    ((uint8_t)0x33)
#define CMD_SET_TEAR_OFF       ((uint8_t)0x34) //synchronization information is not sent from the display
#define CMD_SET_TEAR_ON        ((uint8_t)0x35) //sync. information is sent from the display
#define CMD_SET_ADDR_MODE      ((uint8_t)0x36) //set fram buffer read order to the display panel
#define CMD_SET_SCROLL_START   ((uint8_t)0x37)
#define CMD_EXIT_IDLE_MODE     ((uint8_t)0x38)
#define CMD_ENT_IDLE_MODE      ((uint8_t)0x39)
#define CMD_SET_PIXEL_FORMAT   ((uint8_t)0x3A) //defines how many bits per pixel is used
#define CMD_WR_MEM_AUTO        ((uint8_t)0x3C)
#define CMD_RD_MEM_AUTO        ((uint8_t)0x3E)
#define CMD_SET_TEAR_SCANLINE  ((uint8_t)0x44)
#define CMD_GET_SCANLINE       ((uint8_t)0x45)
#define CMD_RD_DDB_START       ((uint8_t)0xA1)
#define CMD_RD_DDB_AUTO        ((uint8_t)0xA8)
#define CMD_SET_PANEL_MODE     ((uint8_t)0xB0)
#define CMD_GET_PANEL_MODE     ((uint8_t)0xB1)
#define CMD_SET_HOR_PERIOD     ((uint8_t)0xB4)
#define CMD_GET_HOR_PERIOD     ((uint8_t)0xB5)
#define CMD_SET_VER_PERIOD     ((uint8_t)0xB6)
#define CMD_GET_VER_PERIOD     ((uint8_t)0xB7)
#define CMD_SET_GPIO_CONF      ((uint8_t)0xB8)
#define CMD_GET_GPIO_CONF      ((uint8_t)0xB9)
#define CMD_SET_GPIO_VAL       ((uint8_t)0xBA)
#define CMD_GET_GPIO_STATUS    ((uint8_t)0xBB)
#define CMD_SET_POST_PROC      ((uint8_t)0xBC)
#define CMD_GET_POST_PROC      ((uint8_t)0xBD)
#define CMD_SET_PWM_CONF       ((uint8_t)0xBE)
#define CMD_GET_PWM_CONF       ((uint8_t)0xBF)
#define CMD_SET_LCD_GEN0       ((uint8_t)0xC0)
#define CMD_GET_LCD_GEN0       ((uint8_t)0xC1)
#define CMD_SET_LCD_GEN1       ((uint8_t)0xC2)
#define CMD_GET_LCD_GEN1       ((uint8_t)0xC3)
#define CMD_SET_LCD_GEN2       ((uint8_t)0xC4)
#define CMD_GET_LCD_GEN2       ((uint8_t)0xC5)
#define CMD_SET_LCD_GEN3       ((uint8_t)0xC6)
#define CMD_GET_LCD_GEN3       ((uint8_t)0xC7)
#define CMD_SET_GPIO0_ROP      ((uint8_t)0xC8)
#define CMD_GET_GPIO0_ROP      ((uint8_t)0xC9)
#define CMD_SET_GPIO1_ROP      ((uint8_t)0xCA)
#define CMD_GET_GPIO1_ROP      ((uint8_t)0xCB)
#define CMD_SET_GPIO2_ROP      ((uint8_t)0xCC)
#define CMD_GET_GPIO2_ROP      ((uint8_t)0xCD)
#define CMD_SET_GPIO3_ROP      ((uint8_t)0xCE)
#define CMD_GET_GPIO3_ROP      ((uint8_t)0xCF)
#define CMD_SET_ABC_DBC_CONF   ((uint8_t)0xD0)
#define CMD_GET_ABC_DBC_CONF   ((uint8_t)0xD1)
#define CMD_SET_DBC_HISTO_PTR  ((uint8_t)0xD2)
#define CMD_GET_DBC_HISTO_PTR  ((uint8_t)0xD3)
#define CMD_SET_DBC_THRES      ((uint8_t)0xD4)
#define CMD_GET_DBC_THRES      ((uint8_t)0xD5)
#define CMD_SET_ABM_TMR        ((uint8_t)0xD6)
#define CMD_GET_ABM_TMR        ((uint8_t)0xD7)
#define CMD_SET_AMB_LVL0       ((uint8_t)0xD8)
#define CMD_GET_AMB_LVL0       ((uint8_t)0xD9)
#define CMD_SET_AMB_LVL1       ((uint8_t)0xDA)
#define CMD_GET_AMB_LVL1       ((uint8_t)0xDB)
#define CMD_SET_AMB_LVL2       ((uint8_t)0xDC)
#define CMD_GET_AMB_LVL2       ((uint8_t)0xDD)
#define CMD_SET_AMB_LVL3       ((uint8_t)0xDE)
#define CMD_GET_AMB_LVL3       ((uint8_t)0xDF)
#define CMD_PLL_START          ((uint8_t)0xE0) //start the PLL
#define CMD_PLL_STOP           ((uint8_t)0xE1) //disable the PLL
#define CMD_SET_PLL_MN         ((uint8_t)0xE2)
#define CMD_GET_PLL_MN         ((uint8_t)0xE3)
#define CMD_GET_PLL_STATUS     ((uint8_t)0xE4) //get the current PLL status
#define CMD_ENT_DEEP_SLEEP     ((uint8_t)0xE5)
#define CMD_SET_PCLK           ((uint8_t)0xE6) //set pixel clock (LSHIFT signal) frequency
#define CMD_GET_PCLK           ((uint8_t)0xE7) //get pixel clock (LSHIFT signal) freq. settings
#define CMD_SET_DATA_INTERFACE ((uint8_t)0xF0)
#define CMD_GET_DATA_INTERFACE ((uint8_t)0xF1)


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void ssd1963_init(void);
void ssd1963_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void ssd1963_sleep(void);
void ssd1963_wakeup(void);
void ssd1963_set_brightness(uint8_t);
void ssd1963_set_backlight(uint8_t);
void ssd1963_set_contrast(uint8_t);
  
/**********************
 *      MACROS
 **********************/

#endif /* USE_SSD1963 */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SSD1963_H */
