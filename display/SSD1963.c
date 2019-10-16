/**
 * @file SSD1963.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "SSD1963.h"
#if USE_SSD1963

#include <stdbool.h>
#include LV_DRV_DISP_INCLUDE
#include LV_DRV_DELAY_INCLUDE
#include <stdio.h>

/*********************
 *      DEFINES
 *********************/
#define SSD1963_CMD_MODE     0
#define SSD1963_DATA_MODE    1

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static inline void ssd1963_cmd_mode(void);
static inline void ssd1963_data_mode(void);
static inline void ssd1963_cmd(uint8_t cmd);
static inline void ssd1963_data(uint8_t data);
static inline void ssd1963_wdata(uint16_t data);
static inline void ssd1963_data_read(uint8_t* data);
static void ssd1963_io_init(void);
static void ssd1963_reset(void);
static void ssd1963_set_clk(void);
static void ssd1963_set_tft_spec(void);
static void ssd1963_init_bl(void);

/**********************
 *  STATIC VARIABLES
 **********************/
static bool cmd_mode = true;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void ssd1963_init(void)
{

    LV_DRV_DISP_CMD_DATA(SSD1963_CMD_MODE);
    cmd_mode = true;
    bool debug = true;

    uint8_t rd = 0;
    ssd1963_cmd(CMD_RD_DDB_START);
    ssd1963_data_read(&rd);
    if(debug) printf("Got 0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("Got 0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("Got 0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("Got 0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("Got 0x%x\n", rd);
  
    ssd1963_cmd(CMD_GET_PLL_MN);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PLL_MN(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PLL_MN(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PLL_MN(p3)=0x%x\n", rd);

    ssd1963_cmd(CMD_GET_PCLK);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PCLK(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PCLK(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PCLK(p3)=0x%x\n", rd);
  
    ssd1963_cmd(CMD_GET_PANEL_MODE);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PANEL_MODE(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
     if (debug) printf("CMD_GET_PANEL_MODE(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PANEL_MODE(p3)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PANEL_MODE(p4)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PANEL_MODE(p5)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PANEL_MODE(p6)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PANEL_MODE(p7)=0x%x\n", rd);

    ssd1963_cmd(CMD_GET_HOR_PERIOD);              //HSYNC
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p3)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p4)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p5)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p6)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p7)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_HOR_PERIOD(p8)=0x%x\n", rd);

    ssd1963_cmd(CMD_GET_VER_PERIOD);                //VSYNC
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p3)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p4)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p5)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p6)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_VER_PERIOD(p7)=0x%x\n", rd);
  
    ssd1963_cmd(CMD_GET_GPIO_CONF);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_GPIO_CONF(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_GPIO_CONF(p2)=0x%x\n", rd);

    ssd1963_cmd(CMD_GET_PWM_CONF);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p3)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p4)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p5)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p6)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_PWM_CONF(p7)=0x%x\n", rd);
  
    ssd1963_cmd(CMD_GET_ABC_DBC_CONF);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_ABC_DBC_CONF(p1)=0x%x\n", rd);
  
    //CMD_GET_ABC_DBC_CONF(p1)=0x42 (disable; 

    ssd1963_cmd(CMD_GET_POST_PROC); 
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_POST_PROC(p1)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_POST_PROC(p2)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_POST_PROC(p3)=0x%x\n", rd);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_POST_PROC(p4)=0x%x\n", rd);
  
  
    ssd1963_cmd(CMD_GET_DATA_INTERFACE);
    ssd1963_data_read(&rd);
    if (debug) printf("CMD_GET_DATA_INTERFACE(p1)=0x%x\n", rd);
#if 1
    ssd1963_cmd(CMD_SET_DATA_INTERFACE);      //Pixel Data Interface Format
    ssd1963_data((uint8_t)0x03);     //16-bit(565 format) data 
    //ssd1963_data((uint8_t)0x00);      //8-bit data 
    //ssd1963_cmd(CMD_SET_PIXEL_FORMAT);      //Set the current pixel format for RGB image data
    //ssd1963_data((uint8_t)0x50);     //16-bit/pixel
    //ssd1963_cmd(CMD_SET_ADDR_MODE);       //rotation
    //ssd1963_data((uint8_t)0x00);      //RGB=BGR

    //@todo ssd1963_set_backlight(0);

    ssd1963_cmd(CMD_ON_DISPLAY);   //display on

    return;
#endif

    //Log::Put(Log::Level::INFO, "Resetting display\n");

    LV_DRV_DELAY_MS(250);


    ssd1963_cmd(CMD_SET_PLL_MN);    //PLL multiplier, set PLL clock to 120M
    ssd1963_data((uint8_t)0x2A);   //N=0x36 for 6.5M, 0x23 for 10M crystal
    ssd1963_data((uint8_t)0x02);
    ssd1963_data((uint8_t)0x04);
    ssd1963_data_read(&rd);
    ssd1963_cmd(CMD_PLL_START);    // PLL enable
    ssd1963_data((uint8_t)0x01);
    LV_DRV_DELAY_MS(1);
    ssd1963_cmd(CMD_PLL_START);
    ssd1963_data((uint8_t)0x03);   // now, use PLL output as system clock
    LV_DRV_DELAY_MS(1);
    ssd1963_cmd(CMD_SOFT_RESET);    // software reset
    LV_DRV_DELAY_MS(5);
    ssd1963_cmd(CMD_SET_PCLK);    //PLL setting for PCLK, depends on resolution

    ssd1963_data((uint8_t)0x03);   //HX8257C
    ssd1963_data((uint8_t)0xF9);   //HX8257C
    ssd1963_data((uint8_t)0x01);   //HX8257C


    ssd1963_cmd(CMD_SET_PANEL_MODE);    //LCD SPECIFICATION
    ssd1963_data((uint8_t)0x19);
    ssd1963_data((uint8_t)0x01);
    ssd1963_data((uint8_t)(((SSD1963_HOR_RES - 1) >> 8) & 0xFF));  //Set HDP
    ssd1963_data((uint8_t)((SSD1963_HOR_RES - 1) & 0xFF));
    ssd1963_data((uint8_t)(((SSD1963_VER_RES - 1) >> 8) & 0xFF));  //Set VDP
    ssd1963_data((uint8_t)((SSD1963_VER_RES - 1) & 0xFF));
    ssd1963_data((uint8_t)0x01);
    LV_DRV_DELAY_MS(1);//Delay10us(5);
    ssd1963_cmd(CMD_SET_HOR_PERIOD);             //HSYNC
    ssd1963_data((SSD1963_HT >> 8) & 0X00FF); //Set HT
    ssd1963_data(SSD1963_HT & 0X00FF);
    ssd1963_data((SSD1963_HPS >> 8) & 0X00FF); //Set HPS
    ssd1963_data(SSD1963_HPS & 0X00FF);
    ssd1963_data(SSD1963_HPW);              //Set HPW
    ssd1963_data((SSD1963_LPS >> 8) & 0X00FF); //SetLPS
    ssd1963_data(SSD1963_LPS & 0X00FF);
    ssd1963_data((uint8_t)0x01);

    ssd1963_cmd(CMD_SET_VER_PERIOD);             //VSYNC
    ssd1963_data((SSD1963_VT >> 8) & 0X00FF); //Set VT
    ssd1963_data(SSD1963_VT & 0X00FF);
    ssd1963_data((SSD1963_VPS >> 8) & 0X00FF); //Set VPS
    ssd1963_data(SSD1963_VPS & 0X00FF);
    ssd1963_data(SSD1963_VPW);              //Set VPW
    ssd1963_data((SSD1963_FPS >> 8) & 0X00FF); //Set FPS
    ssd1963_data(SSD1963_FPS & 0X00FF);

    ssd1963_cmd(CMD_SET_GPIO_CONF);
    ssd1963_data((uint8_t)0x0f);     //GPIO is controlled by host GPIO[3:0]=output   GPIO[0]=1  LCD ON  GPIO[0]=1  LCD OFF 
	ssd1963_data((uint8_t)0x01);     //GPIO0 normal

    //ssd1963_cmd(CMD_SET_GPIO_VAL);
    //ssd1963_data((uint8_t)0x01);     //GPIO[0] out 1 --- LCD display on/off control PIN
	//ssd1963_cmd(CMD_SET_ADDR_MODE);     //rotation
	//ssd1963_data((uint8_t)0x08);    //RGB=BGR

    ssd1963_cmd(CMD_SET_PIXEL_FORMAT);    //Set the current pixel format for RGB image data
    ssd1963_data((uint8_t)0x50);    //16-bit/pixel

    ssd1963_cmd(CMD_SET_DATA_INTERFACE);    //Pixel Data Interface Format
	ssd1963_data((uint8_t)0x03);    //16-bit(565 format) data 

    ssd1963_cmd(CMD_SET_POST_PROC);
    ssd1963_data((uint8_t)0x41);    //contrast value
    ssd1963_data((uint8_t)0x11);    //brightness value
    ssd1963_data((uint8_t)0x11);    //saturation value
    ssd1963_data((uint8_t)0x01);    //Post Processor Enable

    LV_DRV_DELAY_MS(1);

    ssd1963_cmd(CMD_ON_DISPLAY);  //display on

    ssd1963_cmd(CMD_SET_PWM_CONF);  //set PWM for B/L
    ssd1963_data((uint8_t)0x0e);
    ssd1963_data((uint8_t)0xf0);
    ssd1963_data((uint8_t)0x01);
    ssd1963_data((uint8_t)0x01);
    ssd1963_data((uint8_t)0x00);
    ssd1963_data((uint8_t)0x0f);

    ssd1963_cmd(CMD_SET_ABC_DBC_CONF);
    ssd1963_data((uint8_t)0x0d);

    //DisplayBacklightOn();

    LV_DRV_DELAY_MS(30);
}

void ssd1963_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{

    /*Return if the area is out the screen*/
    if(area->x2 < 0) return;
    if(area->y2 < 0) return;
    if(area->x1 > SSD1963_HOR_RES - 1) return;
    if(area->y1 > SSD1963_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > SSD1963_HOR_RES - 1 ? SSD1963_HOR_RES - 1 : area->x2;
    int32_t act_y2 = area->y2 > SSD1963_VER_RES - 1 ? SSD1963_VER_RES - 1 : area->y2;

    //Set the rectangular area
    ssd1963_cmd(CMD_SET_COLUMN);
    ssd1963_data((uint8_t)(act_x1 >> 8));
    ssd1963_data((uint8_t)act_x1);
    ssd1963_data((uint8_t)(act_x2 >> 8));
    ssd1963_data((uint8_t)act_x2);

    ssd1963_cmd(CMD_SET_PAGE);
    ssd1963_data((uint8_t)(act_y1 >> 8));
    ssd1963_data((uint8_t)act_y1);
    ssd1963_data((uint8_t)(act_y2 >> 8));
    ssd1963_data((uint8_t)act_y2);

    ssd1963_cmd(CMD_WR_MEMSTART);
    int16_t i;
    uint16_t full_w = area->x2 - area->x1 + 1;

    ssd1963_data_mode();
    LV_DRV_DISP_PAR_CS(0);
#if LV_COLOR_DEPTH == 16
    uint16_t act_w = act_x2 - act_x1 + 1;
    for(i = act_y1; i <= act_y2; i++) {
        LV_DRV_DISP_PAR_WR_ARRAY(color_p, act_w);
        color_p += full_w;
    }
    LV_DRV_DISP_PAR_CS(1);
#else
    int16_t j;
    for(i = act_y1; i <= act_y2; i++) {
        for(j = 0; j <= act_x2 - act_x1 + 1; j++) {
            LV_DRV_DISP_PAR_WR_WORD(color_p[j]);
            color_p += full_w;
        }
    }
#endif

    lv_disp_flush_ready(disp_drv);
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

static void ssd1963_io_init(void)
{
    LV_DRV_DISP_CMD_DATA(SSD1963_CMD_MODE);
    cmd_mode = true;
}

static void ssd1963_reset(void)
{
    /*Hardware reset*/
    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(0);
    LV_DRV_DELAY_MS(50);
    LV_DRV_DISP_RST(1);
    LV_DRV_DELAY_MS(50);

    /*Chip enable*/
    LV_DRV_DISP_PAR_CS(0);
    LV_DRV_DELAY_MS(10);
    LV_DRV_DISP_PAR_CS(1);
    LV_DRV_DELAY_MS(5);

    /*Software reset*/
    ssd1963_cmd(CMD_SOFT_RESET);
    LV_DRV_DELAY_MS(20);

    ssd1963_cmd(CMD_SOFT_RESET);
    LV_DRV_DELAY_MS(20);

    ssd1963_cmd(CMD_SOFT_RESET);
    LV_DRV_DELAY_MS(20);

}

/**
 * Command mode
 */
static inline void ssd1963_cmd_mode(void)
{
    if(cmd_mode == false) {
        LV_DRV_DISP_CMD_DATA(SSD1963_CMD_MODE);
        cmd_mode = true;
    }
}

/**
 * Data mode
 */
static inline void ssd1963_data_mode(void)
{
    if(cmd_mode != false) {
        LV_DRV_DISP_CMD_DATA(SSD1963_DATA_MODE);
        cmd_mode = false;
    }
}

/**
 * Write command
 * @param cmd the command
 */
static inline void ssd1963_cmd(uint8_t cmd)
{

    LV_DRV_DISP_PAR_CS(0);
    ssd1963_cmd_mode();
    LV_DRV_DISP_PAR_WR_WORD(cmd);
    LV_DRV_DISP_PAR_CS(1);

}

/**
 * Write data
 * @param data the data
 */
static inline void ssd1963_data(uint8_t data)
{

    LV_DRV_DISP_PAR_CS(0);
    ssd1963_data_mode();
    LV_DRV_DISP_PAR_WR_WORD(data);
    LV_DRV_DISP_PAR_CS(1);

}

static inline void ssd1963_data_read(uint8_t* data)
{

	LV_DRV_DISP_PAR_CS(0);
	ssd1963_data_mode();
	LV_DRV_DISP_PAR_RD_WORD(data);
	LV_DRV_DISP_PAR_CS(1);

}

void ssd1963_sleep(void)
{
    ssd1963_cmd(CMD_ENT_SLEEP);
    ssd1963_cmd(CMD_ENT_DEEP_SLEEP);
}

void ssd1963_wakeup(void)
{
    /* , do two dummy reads to SSD1961/2/3 */
    /* Wait for 100us to let the PLL stable and read the PLL lock status bit. */
    /*READ COMMAND “0xE4”(Bit 2 = 1 if PLL locked) */
    /* Turn on the display */
    ssd1963_cmd(CMD_EXIT_SLEEP);
    /* Wait for 5ms */
}

void ssd1963_set_backlight(uint8_t intensity)
{
#if 1
    ssd1963_cmd(CMD_SET_PWM_CONF);
    ssd1963_data(0x0e); //p1
    ssd1963_data(0xff); //p2
    ssd1963_data(0x09); //p3
    ssd1963_data((uint8_t)intensity);   //p4
    ssd1963_data(0x00); //p5
    ssd1963_data(0x00);  //p6
#endif
    ssd1963_cmd(0xD4);
    ssd1963_data((uint8_t)0x08);

    ssd1963_cmd(CMD_SET_ABC_DBC_CONF);
    ssd1963_data((uint8_t)0x2D);
}

void ssd1963_set_brightness(uint8_t brightness)
{
    uint8_t contrast = 0;
    uint8_t saturation = 0;
    uint8_t ppe = 0;
    uint8_t rd = 0;

    ssd1963_cmd(CMD_GET_POST_PROC); 
    ssd1963_data_read(&contrast);
    if (1) printf("CMD_GET_POST_PROC(p1)=0x%x\n", contrast);
    ssd1963_data_read(&rd);
    if (1) printf("CMD_GET_POST_PROC(p2)=0x%x\n", rd);
    ssd1963_data_read(&saturation);
    if (1) printf("CMD_GET_POST_PROC(p3)=0x%x\n", saturation);
    ssd1963_data_read(&ppe);
    if (1) printf("CMD_GET_POST_PROC(p4)=0x%x\n", ppe);
  
    ssd1963_cmd(CMD_SET_POST_PROC); 
    ssd1963_data((uint8_t)contrast);    //contrast value
    ssd1963_data((uint8_t)brightness);  //brightness value
    ssd1963_data((uint8_t)saturation);  //saturation value
    ssd1963_data((uint8_t)ppe|1);         //Post Processor Enable
}

void ssd1963_set_contrast(uint8_t contrast)
{
    uint8_t brightness = 0;
    uint8_t saturation = 0;
    uint8_t ppe = 0;
    uint8_t rd = 0;
  
    ssd1963_cmd(CMD_GET_POST_PROC); 
    ssd1963_data_read(&rd);
    if (1) printf("CMD_GET_POST_PROC(p1)=0x%x\n", rd);
    ssd1963_data_read(&brightness);
    if (1) printf("CMD_GET_POST_PROC(p2)=0x%x\n", brightness);
    ssd1963_data_read(&saturation);
    if (1) printf("CMD_GET_POST_PROC(p3)=0x%x\n", saturation);
    ssd1963_data_read(&ppe);
    if (1) printf("CMD_GET_POST_PROC(p4)=0x%x\n", ppe);
  
    ssd1963_cmd(CMD_SET_POST_PROC); 
    ssd1963_data((uint8_t)contrast);         //contrast value
    ssd1963_data((uint8_t)brightness);          //brightness value
    ssd1963_data((uint8_t)saturation);        //saturation value
    ssd1963_data((uint8_t)ppe | 1);        //Post Processor Enable
}

#endif
