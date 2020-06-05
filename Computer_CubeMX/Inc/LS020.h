#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include <math.h>
#define pinset HAL_GPIO_WritePin
#define lcd_reset_1 pinset(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_SET);
#define lcd_reset_0 pinset(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_RESET);
#define lcd_rs_1 pinset(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_SET);
#define lcd_rs_0 pinset(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_RESET);
#define lcd_cs_1 pinset(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET);
#define lcd_cs_0 pinset(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET);
#define lcd_power_on  GPIOA->CRH|=2; pinset(LCD_POWER_GPIO_Port,LCD_POWER_Pin,GPIO_PIN_SET);
#define lcd_power_off GPIOA->CRH&=~3;
//pinset(LCD_POWER_GPIO_Port,LCD_POWER_Pin,GPIO_PIN_RESET);
#define lcd_backlight_1_on pinset(backlight_1_GPIO_Port,backlight_1_Pin,GPIO_PIN_SET); 
#define lcd_backlight_2_on pinset(backlight_2_GPIO_Port,backlight_2_Pin,GPIO_PIN_SET); 
#define lcd_backlight_1_off pinset(backlight_1_GPIO_Port,backlight_1_Pin,GPIO_PIN_RESET); 
#define lcd_backlight_2_off pinset(backlight_2_GPIO_Port,backlight_2_Pin,GPIO_PIN_RESET); 
#define lcd_backlight_off lcd_backlight_1_off lcd_backlight_2_off
#define delay_ms HAL_Delay
extern void LCD_SPI(uint8_t data);
extern void soft_delay_ms(uint16_t ms);
#define LCD_8bit


#ifndef LCD_8bit

#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x07FF

#else

#define Black          0x00
#define Red            0xE0
#define White          0xFF
#define Grey           0x6D //09
#define Blue           0x02
#define Blue2          0x0F
#define Blue3          0x07
#define Magenta        0xE3
#define Lime           0x1C
#define Green          0x0C
#define Green2         0x5D
#define Green3         0x9C
#define Cyan           0x1F
#define Yellow         0xFC
#define Yellow2        0xF4
#define Orange         0xE8
#define color1         0xC3
#define color2         0x83
#define fon1           0xDB
#define fon2           0x7F
#endif

typedef uint8_t u8 ;
typedef uint16_t u16; 
typedef uint32_t u32; 
typedef int16_t s16;

void lcd_init(void);
void lcd_off(void);
void lcd_led_mode(u8 mode);
void red_led(bool mode);
void lcd_put_char(u8 x,u8 y,u8 c,u16 chColor,u16 bkColor);
void lcd_put_char_xy(u8 x,u8 y,u8 c,u16 chColor,u16 bkColor);
void lcd_set_pos (u8 x,u8 y);
void lcd_set_dot(u16 color);
void lcd_SetPoint(u8 x,u8 y,u8 color);
void lcd_SetWindows(u8 ,u8 ,u8 ,u8,u8 );
void lcd_put_char_s(u8 x,u8 y,u8 c,u16 chColor);
void lcd_put_s(u8 x,u8 y,char *str,u8 len,u16 chColor,u16 bkColor);
void lcd_put_s_xy(u8 x,u8 y,char *str,u8 len,u16 chColor,u16 bkColor);
void lcd_put_s_s(u8 x,u8 y,u8 *str,u8 len,u16 chColor);
void lcd_draw_picture(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture);
void lcd_draw_rle_picture_CT(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture,u16 size,const u8 *ColorBar);
void lcd_draw_rle_picture(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture,u16);
void lcd_draw_rle_picture_fon(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture,u16 size,u8 rColor,u8 bcColor);
void lcd_draw_rle_digit_24(u16 xs,u16 ys,const u8 *picture,u16 size,u8 dColor,u8 bcColor);
void lcd_draw_rle_digit_16(u16 xs,u16 ys,const u8 *picture,const u8 *ColorBar,u16 size,u8 dColor,u8 bcColor);
void lcd_draw_rle_digit_14(u16 xs,u16 ys,const u8 *picture,u16 size,u8 dColor,u8 bcColor);
void lcd_clear(u8 color);
void GUI_Line(u16 x0, u16 y0, u16 x1, u16 y1,u16 color);
void GUI_Rectangle(u16 x0, u16 y0, u16 x1, u16 y1,u16 color,u8 fill);
void GUI_Circle(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color);
void GUI_Arc(int16_t cx,int16_t cy,int16_t r,int8_t d,float f1,float f2,u8 color);
void Circle_I(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color);
void Circle_II(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color);
void Circle_III(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color);
void Circle_IV(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color);
