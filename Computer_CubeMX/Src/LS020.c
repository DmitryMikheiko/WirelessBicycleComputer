#include "LS020.h"
#include "LS020_font.h"
//#include "delay.h"
uint16_t ic,char_code;
uint32_t i;
const unsigned char init1 [20] = 
{0xEF, 0x00,0xEE, 0x04,0x1B, 0x04,0xFE, 0xFE,
0xFE, 0xFE,0xEF, 0x90,0x4A, 0x04,0x7F, 0x1F,
0xEE, 0x04,0x43, 0x06};   /* 3F-16bit, 1F-8bit */
const unsigned char init2 [40] = 
{0xEF, 0x90,0x09, 0x83,0x08, 0x00,0x0B, 0xAF,0x0A, 0x00,
0x05, 0x00,0x06, 0x00,0x07, 0x00,0xEF, 0x00,0xEE, 0x0C,
0xEF, 0x90,0x00, 0x80,0xEF, 0xB0,0x49, 0x02,0xEF, 0x00,
0x7F, 0x01,0xE1, 0x81,0xE2, 0x02,0xE2, 0x76,0xE1, 0x83};
const unsigned char init3 [2] = 
{0x80, 0x01};
const unsigned char display_off [52] = 
{0xEF,0x00,0x7E,0x04,0xEF,0xB0,0x5A,0x48,0xEF,0x00,0x7F,0x01,0xEF,0xB0,0x64,0xFF,0x65,0x00,0xEF,0x00,0x7F,0x01,0xE2,0x62,0xE2,0x02,0xEF,0xB0,0xBC,0x02,0xEF,0x00,0x7F,0x01,0xE2,0x00,0x80,0x00,0xE2,0x04,0xE2,0x00,0xE1,0x00,0xEF,0xB0,0xBC,0x00,0xEF,0x00,0x7F,0x01};

void lcd_init(void) 
{ 
lcd_power_on
delay_ms(5);
lcd_cs_1
lcd_rs_1
lcd_reset_0
delay_ms(5);
lcd_reset_1
delay_ms(45);
LCD_SPI(0xFD) ;	
LCD_SPI(0xFD) ;
LCD_SPI(0xFD) ;
LCD_SPI(0xFD) ;
delay_ms(6);
	for (i=0;i<20;i++) {LCD_SPI(init1[i]);}
delay_ms(6);
  for (i=0;i<40;i++) {LCD_SPI(init2[i]);}
delay_ms(49);
  for (i=0;i<2;i++)  {LCD_SPI(init3[i]);}
delay_ms(1);
LCD_SPI(0xEF) ;
LCD_SPI(0x90) ;
LCD_SPI(0x00) ;
LCD_SPI(0x00) ;
for (i=0;i<100;i++) {delay_ms(10);}
LCD_SPI(0xEF) ;
LCD_SPI(0x90) ;
LCD_SPI(0x05) ;
LCD_SPI(0x04) ;//04
LCD_SPI(0x06) ;
LCD_SPI(0x00) ;
LCD_SPI(0x07) ;
LCD_SPI(0x00) ;
lcd_rs_1
/*
lcd_rs_0
for (i=0;i<23232;i++) 
{
	//LCD_SPI(Green >> 8);
  LCD_SPI(Green & 0xFF);
}
lcd_rs_1
*/
/*
for (i=0;i<46464;i++) {SPI_I2S_SendData(LCD_SPI,Ford_GT40[i]);}

for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0xFF);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0xFD);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0x1F);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0x18);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0xEA);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0xE9);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0x03);}
for (i=0;i<2904;i++) {SPI_I2S_SendData(LCD_SPI,0x00);} */

//GPIO_SetBits(GPIOA, RS);	
}
void lcd_off(void)
{
	
	lcd_clear(Black);
	soft_delay_ms(50);
	for (i=0;i<52;i++)  {LCD_SPI(display_off[i]);} 		
}
#ifndef LCD_8bit
void lcd_set_dot(u16 color)
{ lcd_rs_0
	LCD_SPI(color >> 8);
	LCD_SPI(color & 0xFF);
	lcd_rs_1
}
#else 
void lcd_set_dot(u16 color)
{ lcd_rs_0
	LCD_SPI(color);
	lcd_rs_1
}
#endif
void lcd_SetPoint(u8 x,u8 y,u8 color)
{
 LCD_SPI(0xEF);
 LCD_SPI(0x90);
 LCD_SPI(0x05);
 LCD_SPI(0x04);
 LCD_SPI(0x06);
 LCD_SPI(y);
 LCD_SPI(0x07);
 LCD_SPI(x);	
	lcd_rs_0
	LCD_SPI(color);
	lcd_rs_1
}
void lcd_set_pos (u8 x,u8 y)
{	
 LCD_SPI(0xEF);
 LCD_SPI(0x90);
 LCD_SPI(0x05);
 LCD_SPI(0x04);
 LCD_SPI(0x06);
 LCD_SPI(y);
 LCD_SPI(0x07);
 LCD_SPI(x);	
}
void lcd_SetWindows(u8 x1,u8 y1,u8 x2,u8 y2,u8 dir)
{
 LCD_SPI(0xEF);
 LCD_SPI(0x90);
 LCD_SPI(0x05);
	if(dir)LCD_SPI(0x00);
 else LCD_SPI(0x04);//04
 LCD_SPI(0x08);
 LCD_SPI(y1  ); //y1
 LCD_SPI(0x09);
 LCD_SPI(y2  );//y2	
 LCD_SPI(0x0A);
 LCD_SPI(x1  );//x1
 LCD_SPI(0x0B);
 LCD_SPI(x2  );//x2		
}
	void lcd_draw_picture(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture)
	{u16 yp,xpp;
	lcd_SetWindows(xs,ys,xe,ye,0);
	xpp=xe-xs;
	xpp++;
	yp=ye-ys;
	lcd_rs_0
	for(ys=yp-1;ys<yp;ys--) 
	{ 
		 for(xs=0;xs<xpp;xs++)
		 {
	// data=picture[ys*xpp+xs];
			 LCD_SPI(picture[ys*xpp+xs]);
			 
	//SPI_I2S_SendData(LCD_SPI, (data >> 8)) ;
	//SPI_I2S_SendData(LCD_SPI, (data & 0xFF) ) ;
	
		 }
	}
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);
} 
void lcd_draw_rle_picture(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture,u16 size)
{
		u32 i;
		u8 color,count;
	  lcd_SetWindows(xs,ys,xe,ye,1);
	  
	  lcd_rs_0
	
for(i=0;i<size;i+=2)
		{
       color=picture[i];
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			 LCD_SPI(color);
			}
			 	
		}
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);
} 
void lcd_draw_rle_picture_fon(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture,u16 size,u8 rColor,u8 bcColor)
{
		u32 i;
		u8 color,count;
	  lcd_SetWindows(xs,131-ye,xe,131-ys,1);
	  
	  lcd_rs_0
	
for(i=0;i<size;i+=2)
		{
       color=picture[i];
			if(color==rColor) color=bcColor;
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			 LCD_SPI(color);
			}
			 	
		}
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);
} 
	void lcd_draw_rle_picture_CT(u16 xs,u16 ys,u16 xe,u16 ye,const u8 *picture,u16 size,const u8 *ColorBar)
{   u32 i;
		u8 color,count;

	  lcd_SetWindows(xs,ys,xe,ye,1);
	  
	  lcd_rs_0
	
for(i=0;i<size;i+=2)
		{
       color=ColorBar[picture[i]];
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			 LCD_SPI(color);
			}
			 	
		}
	
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);		
} 
void lcd_draw_rle_digit_24(u16 xs,u16 ys,const u8 *picture,u16 size,u8 dColor,u8 bcColor)
	{
		u32 i;
		u8 color,count;
	  lcd_SetWindows(xs,131-ys-39,xs+23,131-ys,1);
	  lcd_rs_0
	
for(i=0;i<size;i+=2)
		{
       if(picture[i]==White)color=bcColor;
			else color=dColor;
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			 LCD_SPI(color);
			}
			 	
		}
	
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);
} 
void lcd_draw_rle_digit_16(u16 xs,u16 ys,const u8 *picture,const u8 *ColorBar,u16 size,u8 dColor,u8 bcColor)
{  u32 i;
	 u8 color,count;
		
	 lcd_SetWindows(xs,131-ys-15,xs+15,131-ys,1); 	
	 lcd_rs_0
	
for(i=0;i<size;i+=2)
		{
       color=ColorBar[picture[i]];
			if(color==dColor)color=bcColor;
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			 LCD_SPI(color);
			}
			 	
		}
	
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);
	
} 
void lcd_draw_rle_digit_14(u16 xs,u16 ys,const u8 *picture,u16 size,u8 dColor,u8 bcColor)
{ u32 i;
		u8 color,count;
		
	  lcd_SetWindows(xs,131-ys-24,xs+13,131-ys,0);
	  
	 lcd_rs_0
	
for(i=0;i<size;i+=2)
		{
       if(picture[i]==White)color=bcColor;
			else color=dColor;
			 count=picture[i+1];
      while(count>0)
			{
				count--;
			 LCD_SPI(color);
			}
			 	
		}
	
	lcd_rs_1
	lcd_SetWindows(0,0,175,131,0);
	
} 
void lcd_put_char(u8 x,u8 y,u8 c,u16 chColor,u16 bkColor)
{ lcd_SetWindows(x*8,131-y*16-15,x*8+7,131-y*16,0);
	for (i=15;i<16;i--) 
	{ char_code=ascii_8x16[((c-0x20)*16)+i];
		for (ic=0;ic<8;ic++)
		 { if ( ((char_code >> (7-ic)) & 0x01) == 0x01)
          { lcd_set_dot(chColor);
					}
          else
          {
            lcd_set_dot(bkColor);
          }
			 
			 
		 }
		
	}
}
void lcd_put_char_xy(u8 x,u8 y,u8 c,u16 chColor,u16 bkColor)
{ lcd_SetWindows(x,131-y-15,x+7,131-y,0);
	for (i=15;i<16;i--) 
	{ char_code=ascii_8x16[((c-0x20)*16)+i];
		for (ic=0;ic<8;ic++)
		 { if ( ((char_code >> (7-ic)) & 0x01) == 0x01)
          { lcd_set_dot(chColor);
					}
          else
          {
            lcd_set_dot(bkColor);
          }
			 
			 
		 }
		
	}
}
void lcd_put_char_s(u8 x,u8 y,u8 c,u16 chColor)
{ lcd_SetWindows(x*8,131-y*16-15, x*8+7,131-y*16,0);
	for (i=15;i<16;i--) 
	{ char_code=ascii_8x16[((c-0x20)*16)+i];
		for (ic=0;ic<8;ic++)
		 { if ( ((char_code >> (7-ic)) & 0x01) == 0x01)
          { lcd_set_dot(chColor);
					}
          else
          { if (ic==7) {lcd_set_pos (x*8,131-y*16-i+1);}
					  else {lcd_set_pos (x*8+ic+1,131-y*16-i);}
          }
			 
			 
		 }
		
	}
}
void lcd_put_s(u8 x,u8 y,char *str,u8 len,u16 chColor,u16 bkColor)
{ u8 i;
  
  for (i=0;i<len;i++)
  {
    lcd_put_char((x+i),y,*str++,chColor,bkColor);
  }
	
}
void lcd_put_s_xy(u8 x,u8 y,char *str,u8 len,u16 chColor,u16 bkColor)
{ u8 i;
  
  for (i=0;i<len;i++)
  {
    lcd_put_char_xy((x+i*8),y,*str++,chColor,bkColor);
  }
	
}
void lcd_put_s_s(u8 x,u8 y,u8 *str,u8 len,u16 chColor)
{ u8 i;
  
  for (i=0;i<len;i++)
  {
    lcd_put_char_s((x+i),y,*str++,chColor);
  }
	
}
void GUI_Rectangle(u16 x0, u16 y0, u16 x1, u16 y1,u16 color,u8 fill)
{
	y0=131-y0;
	y1=131-y1;
	if(fill)
	{
		u16 i;
		if(x0>x1)
		{
			i=x1;
			x1=x0;
		}
		else
		{
			i=x0;
		}
		for(;i<=x1;i++)
		{
			GUI_Line(i,y0,i,y1,color);
		}
		return;
	}
	GUI_Line(x0,y0,x0,y1,color);
	GUI_Line(x0,y1,x1,y1,color);
	GUI_Line(x1,y1,x1,y0,color);
	GUI_Line(x1,y0,x0,y0,color);
}
void GUI_Line(u16 x0, u16 y0, u16 x1, u16 y1,u16 color)
{
 	u16 x,y;
 	u16 dx;// = abs(x1 - x0);
 	u16 dy;// = abs(y1 - y0);

	if(y0==y1)
	{
		if(x0<=x1)
		{
			x=x0;
		}
		else
		{
			x=x1;
			x1=x0;
		}
  		while(x <= x1)
  		{
   			lcd_SetPoint(x,y0,color);
   			x++;
  		}
  		return;
	}
	else if(y0>y1)
	{
		dy=y0-y1;
	}
	else
	{
		dy=y1-y0;
	}
 
 	if(x0==x1)
	{
		if(y0<=y1)
		{
			y=y0;
		}
		else
		{
			y=y1;
			y1=y0;
		}
  		while(y <= y1)
  		{
   			lcd_SetPoint(x0,y,color);
   			y++;
  		}
  		return;
	}
	else if(x0 > x1)
 	{
		dx=x0-x1;
  		x = x1;
  		x1 = x0;
  		y = y1;
  		y1 = y0;
 	}
 	else
 	{
		dx=x1-x0;
  		x = x0;
  		y = y0;
 	}

 	if(dx == dy)
 	{
  		while(x <= x1)
  		{

   			x++;
			if(y>y1)
			{
				y--;
			}
			else
			{
   				y++;
			}
   			lcd_SetPoint(x,y,color);
  		}
 	}
 	else
 	{
 		lcd_SetPoint(x, y, color);
  		if(y < y1)
  		{
   			if(dx > dy)
   			{
    			s16 p = dy * 2 - dx;
    			s16 twoDy = 2 * dy;
    			s16 twoDyMinusDx = 2 * (dy - dx);
    			while(x < x1)
    			{
     				x++;
     				if(p < 0)
     				{
      					p += twoDy;
     				}
     				else
     				{
      					y++;
      					p += twoDyMinusDx;
     				}
     				lcd_SetPoint(x, y,color);
    			}
   			}
   			else
   			{
    			s16 p = dx * 2 - dy;
    			s16 twoDx = 2 * dx;
    			s16 twoDxMinusDy = 2 * (dx - dy);
    			while(y < y1)
    			{
     				y++;
     				if(p < 0)
     				{
      					p += twoDx;
     				}
     				else
     				{
      					x++;
      					p+= twoDxMinusDy;
     				}
     				lcd_SetPoint(x, y, color);
    			}
   			}
  		}
  		else
  		{
   			if(dx > dy)
   			{
    			s16 p = dy * 2 - dx;
    			s16 twoDy = 2 * dy;
	    		s16 twoDyMinusDx = 2 * (dy - dx);
    			while(x < x1)
    			{
     				x++;
     				if(p < 0)
	     			{
    	  				p += twoDy;
     				}
     				else
     				{
      					y--;
	      				p += twoDyMinusDx;
    	 			}
     				lcd_SetPoint(x, y,color);
    			}
   			}
	   		else
   			{
    			s16 p = dx * 2 - dy;
    			s16 twoDx = 2 * dx;
	    		s16 twoDxMinusDy = 2 * (dx - dy);
    			while(y1 < y)
    			{
     				y--;
     				if(p < 0)
	     			{
    	  				p += twoDx;
     				}
     				else
     				{
      					x++;
	      				p+= twoDxMinusDy;
    	 			}
     				lcd_SetPoint(x, y,color);
    			}
   			}
  		}
 	}
}
void lcd_clear(u8 color)
{ int i;
	lcd_SetWindows(0,0,175,131,0);
	lcd_rs_0
	lcd_cs_0
	for(i=0;i<176*132;i++) 
{	
	while((SPI2->SR &SPI_SR_BSY)>0);
	SPI2->DR=color;
}
  lcd_cs_1
	lcd_rs_1
}
void Circle_I(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color)
{
	int x,y,xs,xe;
	int delta=r;
	int quad,rquad;
	double fs,fe;
	rquad=r*r;
	
	if(f1<90.0)
	{	
	if(f1<0.0)  { Circle_IV (cx,cy,r,360.0+f1,360.0,color);}
  if(f1<=0.0) {xs=r;}
  else      {fs=(3.14*f1)/180.0;xs=r*cos(fs);}
  if(f2>90.0) { xe=0;}
  else      { fe=(3.14*f2)/180.0; xe=r*cos(fe);}
	
	for(x=xe;x<=xs;x++) for(y=0;y<=r;y++)
	{
		quad=rquad-(x*x)-(y*y);
		if(quad>=delta*(-1) && quad<=delta)lcd_SetPoint(x+cx, cy+y,color);
	}
	
	}
}
void Circle_II(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color)
{
	int x,y,xs,xe;
	int delta=r;
	int quad,rquad;
	double fs,fe;
	rquad=r*r;
	
	if(f1<180.0)
	{
	if(f1<=90.0){xs=0;}
	else{fs=(3.14*f1)/180.0;xs=r*cos(fs);}
	if(f2>180.0){ xe=-r;}
	else {fe=(3.14*f2)/180.0;xe=r*cos(fe);}

	for(x=xs;x>=xe;x--) for(y=0;y<=r;y++)
	{
		quad=rquad-(x*x)-(y*y);
		if(quad>=delta*(-1) && quad<=delta)lcd_SetPoint(x+cx, cy+y,color);
	}
	}
}
void Circle_III(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color)
{
	int x,y,xs,xe;
	int delta=r;
	int quad,rquad;
	double fs,fe;
	rquad=r*r;
	
	if(f1<270.0)
	{
	if(f1<=180.0){xs=-r;}
	else{fs=(3.14*f1)/180.0;xs=r*cos(fs);}
	if(f2>270.0){xe=0;}
	else {fe=(3.14*f2)/180.0;xe=r*cos(fe);}
		
	for(x=xs;x<=xe;x++) for(y=0;y<=r;y++)
	{
		quad=rquad-(x*x)-(y*y);
		if(quad>=delta*(-1) && quad<=delta)lcd_SetPoint(x+cx, cy-y,color);
	}
	}
}
void Circle_IV(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color)
{
	int x,y,xs,xe;
	int delta=r;
	int quad,rquad;
	double fs,fe;
	rquad=r*r;
	
	if(f1<360.0)
	{
	if(f1<=270.0){xs=0;}
	else{fs=(3.14*f1)/180.0;xs=r*cos(fs);}
	if(f2>=360.0){xe=r;}
	else {fe=(3.14*f2)/180.0;xe=r*cos(fe);}
		
	for(x=xs;x<=xe;x++) for(y=0;y<=r;y++)
	{
		quad=rquad-(x*x)-(y*y);
		if(quad>=delta*(-1) && quad<=delta)lcd_SetPoint(x+cx, cy-y,color);
	}
	}
}
void GUI_Circle(int16_t cx,int16_t cy,int16_t r,float f1,float f2,u8 color)
{	
	Circle_I  (cx,cy,r,f1,f2,color);
	Circle_II (cx,cy,r,f1,f2,color);
	Circle_III(cx,cy,r,f1,f2,color);
	Circle_IV (cx,cy,r,f1,f2,color);
}
void GUI_Arc(int16_t cx,int16_t cy,int16_t r,int8_t d,float f1,float f2,u8 color)
{ 
	u8 dx;	
	for(dx=0;dx<d;dx++) GUI_Circle(cx,131-cy,r-dx,f1,f2,color);	
}
