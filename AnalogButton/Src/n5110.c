/*
 * ��������     :  ��� ������� ��� ������������ LCD �� Nokia 5110, � ����� ��� ��������� ������.
 * �����        :  Xander Gresolio <xugres@gmail.com>
 * ���-�������� :  https://github.com/gresolio/N3310Lib
 * ��������     :  GPL v3.0
 *
 * �����������  : kobzar aka kobraz ��� http://cxem.net/ maodzedun@gmail.com
 */

#include <string.h>
#include <stdio.h>
#include "delay.h"
#include "n5110.h"



//������� ��� ������ � ������
#define     S_CLK       HAL_GPIO_WritePin(GPIOA, LCD_SCLK_Pin, GPIO_PIN_SET)
#define     R_CLK       HAL_GPIO_WritePin(GPIOA, LCD_SCLK_Pin, GPIO_PIN_RESET)

#define     S_MOSI      HAL_GPIO_WritePin(GPIOA, LCD_MOSI_Pin, GPIO_PIN_SET)
#define     R_MOSI      HAL_GPIO_WritePin(GPIOA, LCD_MOSI_Pin, GPIO_PIN_RESET)

#define     S_DC        HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin, GPIO_PIN_SET)
#define     R_DC        HAL_GPIO_WritePin(GPIOA, LCD_DC_Pin, GPIO_PIN_RESET)

#define     S_CE        HAL_GPIO_WritePin(GPIOA, LCD_CE_Pin, GPIO_PIN_SET)
#define     R_CE        HAL_GPIO_WritePin(GPIOA, LCD_CE_Pin, GPIO_PIN_RESET)

#define     S_RST       HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin, GPIO_PIN_SET)
#define     R_RST       HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin, GPIO_PIN_RESET)

/*
#define   RB(reg, bit)       reg &= (~(1<<(bit)))
#define   SB(reg, bit)       reg |= (1<<(bit))
#define   IB(reg, bit)       reg ^= (1<<bit)
#define   iBS(reg, bit)     ((reg & (1<<bit)) != 0)
#define   iBR(reg, bit)     ((reg & (1<<bit)) == 0)
*/



/********************************************************************************************/
/********************************************************************************************/

//������ � float
int gpow(int n, int power) {
	int res = 1;
	while(power--) res *= n;
	return res;
}


uint8_t *gftoa(float f, int dec) {

    static uint8_t buf[7] = {'0','0','0','0','0','0',0};
	uint8_t *p = buf + 6;
	unsigned int i = (unsigned int)(f * gpow(10, dec));
    uint8_t c = 4;

	while (c--) {
		*--p = '0' + (i % 10);
		i /= 10;
		if (--dec == 0) *--p = '.';
	}

	return p;


}
/********************************************************************************************/
/********************************************************************************************/



// ��������� ��������� ������� ��������

static void LcdSend    ( uint8_t data, LcdCmdData cd );

void SPI_Send(unsigned char x);

// ���������� ����������

// ��� � ��� 84*48 ��� ��� 504 �����
static uint8_t  LcdCache [ LCD_CACHE_SIZE ];

// ����� �� ��������� ���� �������, � ���� �� ����� ��� ����������,
// ����� �������� ��� ������� ���� ��� ��������� ���������. �����
// ����� ���������� ��� ����� ���� ����� ��������� � ��� �������.
static int   LoWaterMark;   // ������ �������
static int   HiWaterMark;   // ������� �������

// ��������� ��� ������ � LcdCache[]
static int   LcdCacheIdx;

// ���� ��������� ����
static uint8_t  UpdateLcd;




/*
 * ���                   :  Lcd_init
 * ��������              :  ���������� ������������� ����� � SPI ��, ����������� LCD
 * ��������(�)           :  ���
 * ������������ �������� :  ���
 */
void init_lcd ( void )
{
		GPIO_InitTypeDef GPIO_InitStruct;
	
		__HAL_RCC_GPIOA_CLK_ENABLE();
	
		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(LCD_PORT, LCD_CE_Pin|LCD_RST_Pin|LCD_DC_Pin|LCD_MOSI_Pin 
														|LCD_SCLK_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pins : PAPin PAPin PAPin PAPin 
														 PAPin */
		GPIO_InitStruct.Pin = LCD_CE_Pin|LCD_RST_Pin|LCD_DC_Pin|LCD_MOSI_Pin 
														|LCD_SCLK_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
    // ��������������� ��������
    HAL_Delay(10);

    // ������� reset
    R_RST;
    HAL_Delay(10);
    S_RST;

    // ���������� SPI:
    //SPI_Init();

    // ��������� LCD ���������� - ������� ������� �� SCE
    //SB(LCD_PORT, LCD_CE_PIN);

    // ���������� ������� �������
    LcdSend( 0x21, LCD_CMD ); // �������� ����������� ����� ������ (LCD Extended Commands)
    LcdSend( 0xC8, LCD_CMD ); // ��������� ������������� (LCD Vop)
    LcdSend( 0x06, LCD_CMD ); // ��������� �������������� ������������ (Temp coefficent)
    LcdSend( 0x13, LCD_CMD ); // ��������� ������� (LCD bias mode 1:48)
    LcdSend( 0x20, LCD_CMD ); // �������� ����������� ����� ������ � �������������� ��������� (LCD Standard Commands,Horizontal addressing mode)
    LcdSend( 0x0C, LCD_CMD ); // ���������� ����� (LCD in normal mode)

    // ��������� ������� �������
    Lcd_clear();
    Lcd_update();
}


/*
 * ���                   :  Lcd_clear
 * ��������              :  ������� �������. ����� ���������� ��������� LcdUpdate
 * ��������(�)           :  ���
 * ������������ �������� :  ���
 */
void Lcd_clear ( void )
{
    memset( LcdCache, 0x00, LCD_CACHE_SIZE );

    // ����� ���������� ������ � ������������ ��������
    LoWaterMark = 0;
    HiWaterMark = LCD_CACHE_SIZE - 1;

    // ��������� ����� ��������� ����
    UpdateLcd = TRUE;
}



/*
 * ���                   :  Lcd_update
 * ��������              :  �������� ��� � ��� �������
 * ��������(�)           :  ���
 * ������������ �������� :  ���
 */
void Lcd_update (void)
{
    int i;

    if ( LoWaterMark < 0 )
        LoWaterMark = 0;
    else if ( LoWaterMark >= LCD_CACHE_SIZE )
        LoWaterMark = LCD_CACHE_SIZE - 1;

    if ( HiWaterMark < 0 )
        HiWaterMark = 0;
    else if ( HiWaterMark >= LCD_CACHE_SIZE )
        HiWaterMark = LCD_CACHE_SIZE - 1;

    #ifdef CHINA_LCD  // �������� ��� ���������� �� �� ������������� ������������

    uint8_t x,y;

    // 102 x 64 - ������ �������������� ���������� ������ ���������� ��, ��� ���
    // ������ ������ ������������ �� ������� �� ������� ����� �� 3 �������.
    // ������� ������� �������� ���� - ������� � ������ ������ y+1, � �����
    // ������� ����� (����� ���� ���� �������, �������� � ������ ������)

    x = LoWaterMark % LCD_X_RES;      // ������������� ��������� ����� x
    LcdSend( 0x80 | x, LCD_CMD );     // ������������ ������ ������� LoWaterMark

    y = LoWaterMark / LCD_X_RES + 1;  // ������������� ��������� ����� y+1
    LcdSend( 0x40 | y, LCD_CMD );     // ������������ ������ ������� LoWaterMark

        for ( i = LoWaterMark; i <= HiWaterMark; i++ )
        {
            // �������� ������ � ����� �������
            LcdSend( LcdCache[i], LCD_DATA );

            x++;                 // ������ ������������ ���������� x, ����� ������� ������� �� ����� ������
            if (x >= LCD_X_RES)  // ���� ����� �� ������, �� ��������� �� ��������� ������ (x=0; y++)
            {
                // ����� ������, ����� ����� ��������� ������ ����� �������������� ������,
                // �������� ���� ��������� ��������� �����, ����� ��� �������� :)
                x=0;
                LcdSend( 0x80, LCD_CMD );
                y++;
                LcdSend( 0x40 | y, LCD_CMD );
            }
        }

        LcdSend( 0x21, LCD_CMD );    // �������� ����������� ����� ������
        LcdSend( 0x45, LCD_CMD );    // �������� �������� �� 5 �������� ����� (������������� ������� �������, �������� � ����������)
        LcdSend( 0x20, LCD_CMD );    // �������� ����������� ����� ������ � �������������� ���������

    #else  // �������� ��� ������������� �������

        // ������������� ��������� ����� � ������������ � LoWaterMark
        LcdSend( 0x80 | ( LoWaterMark % LCD_X_RES ), LCD_CMD );
        LcdSend( 0x40 | ( LoWaterMark / LCD_X_RES ), LCD_CMD );

        // ��������� ����������� ����� ������ �������
        for ( i = LoWaterMark; i <= HiWaterMark; i++ )
        {
            // ��� ������������� ������� �� ����� ������� �� ������� � ������,
            // ����� ������ ��������������� �������� ������
            LcdSend( LcdCache[i], LCD_DATA );
        }

    #endif

    // ����� ���������� ������ � �������
    LoWaterMark = LCD_CACHE_SIZE - 1;
    HiWaterMark = 0;

    // ����� ����� ��������� ����
    UpdateLcd = FALSE;
}


/*
 * ���                   :  LcdSend
 * ��������              :  ���������� ������ � ���������� �������
 * ��������(�)           :  data -> ������ ��� ��������
 *                          cd   -> ������� ��� ������ (������ enum � n5110.h)
 * ������������ �������� :  ���
 */
static void LcdSend ( uint8_t data, LcdCmdData cd )
{
    // �������� ���������� ������� (������ ������� ��������)

    R_CE;
    if ( cd == LCD_DATA )
    {
        S_DC;
    }
    else
    {
        R_DC;
    }

	/*
    SPDR = data;
    while ( (SPSR & 0x80) != 0x80 );
	*/
	SPI_Send(data);

    // ��������� ���������� �������
    //LCD_PORT |= _BV( LCD_CE_PIN );
    //SB(LCD_PORT, LCD_CE_PIN);
}


/*
 * ���                   :  LcdContrast
 * ��������              :  ������������� ������������� �������
 * ��������(�)           :  �������� -> �������� �� 0x00 � 0x7F
 * ������������ �������� :  ���
 */
void LcdContrast ( uint8_t contrast )
{
    LcdSend( 0x21, LCD_CMD );              // ����������� ����� ������
    LcdSend( 0x80 | contrast, LCD_CMD );   // ��������� ������ �������������
    LcdSend( 0x20, LCD_CMD );              // ����������� ����� ������, �������������� ���������
}



/*
 * ���                   :  LcdGotoXY
 * ��������              :  ������������� ������ � ������� x,y ������������ ������������ ������� ������
 * ��������(�)           :  x,y -> ���������� ����� ������� �������. ��������: 0,0 .. 13,5
 * ������������ �������� :  ������ ������������ �������� � n5110.h
 */
uint8_t LcdGotoXY ( uint8_t x, uint8_t y )
{
    // �������� ������
    if( x > 13 || y > 5 ) return OUT_OF_BORDER;

    //  ���������� ���������. ��������� ��� ����� � �������� 504 ����
    LcdCacheIdx = x * 6 + y * 84;
    return OK;
}


/*
 * ���                   :  LcdChr
 * ��������              :  ������� ������ � ������� ������� �������, ����� �������������� ��������� �������
 * ��������(�)           :  size -> ������ ������. ������ enum � n5110.h
 *                          ch   -> ������ ��� ������
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t LcdChr ( uint8_t ch )
{
    uint8_t i;

    if ( LcdCacheIdx < LoWaterMark )
    {
        // ��������� ������ �������
        LoWaterMark = LcdCacheIdx;
    }

    if ( (ch >= 0x20) && (ch <= 0x7F) )
    {
        // �������� � ������� ��� �������� ASCII[0x20-0x7F]
        ch -= 32;
    }
    else if ( ch >= 0xC0 )
    {
        // �������� � ������� ��� �������� CP1251[0xC0-0xFF]
        ch -= 96;
    }
    else
    {
        // ��������� ���������� (�� ������ ��� � ������� ��� �������� ������)
        ch = 95;
    }

    for ( i = 0; i < 5; i++ )
    {
        // �������� ��� ������� �� ������� � ���
        LcdCache[LcdCacheIdx++] = (FontLookup[ch][i]) << 1;
    }



    if ( LcdCacheIdx > HiWaterMark )
    {
        // ��������� ������� �������
        HiWaterMark = LcdCacheIdx;
    }

    // �������������� ������ ����� ���������
    LcdCache[LcdCacheIdx] = 0x00;
    // ���� �������� ������� ��������� LCD_CACHE_SIZE - 1, ��������� � ������
    if(LcdCacheIdx == (LCD_CACHE_SIZE - 1) )
    {
        LcdCacheIdx = 0;
        return OK_WITH_WRAP;
    }
    // ����� ������ �������������� ���������
    LcdCacheIdx++;
    return OK;
}



/*
 * ���                   :  LcdChr
 * ��������              :  ������� ������ � ������� ������� �������, ����� �������������� ��������� �������
 * ��������(�)           :  size -> ������ ������. ������ enum � n5110.h
 *                          ch   -> ������ ��� ������
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t LcdChrB ( uint8_t ch )
{
    uint8_t i;

    ch -= 46;

    if ( LcdCacheIdx < LoWaterMark )
    {
        // ��������� ������ �������
        LoWaterMark = LcdCacheIdx;
    }

    for ( i = 0; i < 24; i++ )
    {
        // �������� ��� ������� �� ������� � ���
        if (i == 13) LcdCacheIdx += 72;
        LcdCache[LcdCacheIdx++] = (deg12x16[ch][i]);
    }


    if ( LcdCacheIdx > HiWaterMark )
    {
        // ��������� ������� �������
        HiWaterMark = LcdCacheIdx;
    }

    // �������������� ������ ����� ���������
    LcdCache[LcdCacheIdx] = 0x00;
    // ���� �������� ������� ��������� LCD_CACHE_SIZE - 1, ��������� � ������
    if(LcdCacheIdx == (LCD_CACHE_SIZE - 1) )
    {
        LcdCacheIdx = 0;
        return OK_WITH_WRAP;
    }
    // ����� ������ �������������� ���������
    LcdCacheIdx++;

        LcdCacheIdx -= 84;

    return OK;
}




void Lcd_printf( uint8_t x, uint8_t y, float data, int accuracy )
{
	LcdGotoXY(x,y);
    uint8_t tmpIdx=0;

    uint8_t * digits = gftoa(data, 2);

    while( digits[ tmpIdx ] != '\0' )
    {
        // ������� ������
        LcdChrB( digits[ tmpIdx ] );
        // ����������� ���������
        tmpIdx++;
    }

}
















/*
 * ���                   :  Lcd_print
 * ��������              :  ��� ������� ������������� ��� ������ ������ �� ����������
 * ��������(�)           :  size      -> ������ ������. ������ enum � n5110.h
 *                       :  dataArray -> ������ ���������� ������ ������� ����� ����������
 *						 :  x,y -> ����������
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 * ������                :  LcdFStr(0, 0, FONT_1X,(unsigned char*)some_char);
 *                          LcdFStr(0, 0, FONT_1X, &name_of_string_as_array);
 */
uint8_t Lcd_print ( uint8_t x, uint8_t y, uint8_t dataArray[] )
{
	LcdGotoXY(x,y);
    uint8_t tmpIdx=0;
    uint8_t response;
    while( dataArray[ tmpIdx ] != '\0' )
    {
        // ������� ������
        response = LcdChr( dataArray[ tmpIdx ] );
        // �� ����� ����������� ���� ���������� OUT_OF_BORDER,
        // ������ ����� ���������� ������ �� ������ �������
        if( response == OUT_OF_BORDER)
            return OUT_OF_BORDER;
        // ����������� ���������
        tmpIdx++;
    }
    return OK;
}



/*
 * ���                   :  Lcd_prints
 * ��������              :  ��� ������� ������������� ��� ������ ��������� ������
 * ��������(�)           :  size    -> ������ ������. ������ enum � n5110.h
 *                          dataPtr -> ��������� �� ������ ������� ����� ����������
 *						 :  x,y -> ����������
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 * ������                :  LcdFStr(0, 0, FONT_1X, PSTR("Hello World"));
 *                          LcdFStr(0, 0, FONT_1X, &name_of_string_as_array);
 */
uint8_t Lcd_prints ( uint8_t x, uint8_t y, uint8_t *dataPtr )
{
    LcdGotoXY(x,y);
	uint8_t c;
    uint8_t response;
    for ( c = *dataPtr; c; ++dataPtr, c = *dataPtr )
    {
        // ������� ������
        response = LcdChr( c );
        if(response == OUT_OF_BORDER)
            return OUT_OF_BORDER;
    }

    return OK;
}




/*
 * ���                   :  Lcd_pixel
 * ��������              :  ���������� ������� �� ���������� ����������� (x,y)
 * ��������(�)           :  x,y  -> ���������� ���������� �������
 *                          mode -> Off, On ��� Xor. ������ enum � n5110.h
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t Lcd_pixel ( uint8_t x, uint8_t y, LcdPixelMode mode )
{
    int  index;
    uint8_t  offset;
    uint8_t  data;

    // ������ �� ������ �� �������
    if ( x >= LCD_X_RES || y >= LCD_Y_RES) return OUT_OF_BORDER;

    // �������� ������� � ��������
    index = ( ( y / 8 ) * 84 ) + x;
    offset  = y - ( ( y / 8 ) * 8 );

    data = LcdCache[ index ];

    // ��������� �����

    // ����� PIXEL_OFF
    if ( mode == PIXEL_OFF )
    {
        data &= ( ~( 0x01 << offset ) );
    }
    // ����� PIXEL_ON
    else if ( mode == PIXEL_ON )
    {
        data |= ( 0x01 << offset );
    }
    // ����� PIXEL_XOR
    else if ( mode  == PIXEL_XOR )
    {
        data ^= ( 0x01 << offset );
    }

    // ������������� ��������� �������� � ���
    LcdCache[ index ] = data;

    if ( index < LoWaterMark )
    {
        // ��������� ������ �������
        LoWaterMark = index;
    }

    if ( index > HiWaterMark )
    {
        // ��������� ������� �������
        HiWaterMark = index;
    }
    return OK;
}



/*
 * ���                   :  Lcd_line
 * ��������              :  ������ ����� ����� ����� ������� �� ������� (�������� ����������)
 * ��������(�)           :  x1, y1  -> ���������� ���������� ������ �����
 *                          x2, y2  -> ���������� ���������� ����� �����
 *                          mode    -> Off, On ��� Xor. ������ enum � n5110.h
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t Lcd_line ( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LcdPixelMode mode )
{
    int dx, dy, stepx, stepy, fraction;
    uint8_t response;

    // dy   y2 - y1
    // -- = -------
    // dx   x2 - x1

    dy = y2 - y1;
    dx = x2 - x1;

    // dy �������������
    if ( dy < 0 )
    {
        dy    = -dy;
        stepy = -1;
    }
    else
    {
        stepy = 1;
    }

    // dx �������������
    if ( dx < 0 )
    {
        dx    = -dx;
        stepx = -1;
    }
    else
    {
        stepx = 1;
    }

    dx <<= 1;
    dy <<= 1;

    // ������ ��������� �����
    response = Lcd_pixel( x1, y1, mode );
    if(response)
        return response;

    // ������ ��������� ����� �� �����
    if ( dx > dy )
    {
        fraction = dy - ( dx >> 1);
        while ( x1 != x2 )
        {
            if ( fraction >= 0 )
            {
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;

            response = Lcd_pixel( x1, y1, mode );
            if(response)
                return response;

        }
    }
    else
    {
        fraction = dx - ( dy >> 1);
        while ( y1 != y2 )
        {
            if ( fraction >= 0 )
            {
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;

            response = Lcd_pixel( x1, y1, mode );
            if(response)
                return response;
        }
    }

    // ��������� ����� ��������� ����
    UpdateLcd = TRUE;
    return OK;
}



/*
 * ���                   :  Lcd_circle
 * ��������              :  ������ ���������� (�������� ����������)
 * ��������(�)           :  x, y   -> ���������� ���������� ������
 *                          radius -> ������ ����������
 *                          mode   -> Off, On ��� Xor. ������ enum � n5110.h
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t Lcd_circle(uint8_t x, uint8_t y, uint8_t radius, LcdPixelMode mode)
{
    signed char xc = 0;
    signed char yc = 0;
    signed char p = 0;

    if ( x >= LCD_X_RES || y >= LCD_Y_RES) return OUT_OF_BORDER;

    yc = radius;
    p = 3 - (radius<<1);
    while (xc <= yc)
    {
        Lcd_pixel(x + xc, y + yc, mode);
        Lcd_pixel(x + xc, y - yc, mode);
        Lcd_pixel(x - xc, y + yc, mode);
        Lcd_pixel(x - xc, y - yc, mode);
        Lcd_pixel(x + yc, y + xc, mode);
        Lcd_pixel(x + yc, y - xc, mode);
        Lcd_pixel(x - yc, y + xc, mode);
        Lcd_pixel(x - yc, y - xc, mode);
        if (p < 0) p += (xc++ << 2) + 6;
            else p += ((xc++ - yc--)<<2) + 10;
    }

    // ��������� ����� ��������� ����
    UpdateLcd = TRUE;
    return OK;
}


/*
 * ���                   :  Lcd_rect  (rectangle)
 * ��������              :  ������ ���� ����������� �������������
 * ��������(�)           :  baseX  -> ���������� ���������� x (������ ����� ����)
 *                          baseY  -> ���������� ���������� y (������ ����� ����)
 *                          height -> ������ (� ��������)
 *                          width  -> ������ (� ��������)
 *                          mode   -> Off, On ��� Xor. ������ enum � n5110.h
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t Lcd_rect ( uint8_t baseX, uint8_t baseY, uint8_t height, uint8_t width, LcdPixelMode mode )
{
    uint8_t tmpIdxX,tmpIdxY,tmp;

    uint8_t response;

    // �������� ������
    if ( ( baseX >= LCD_X_RES) || ( baseY >= LCD_Y_RES) ) return OUT_OF_BORDER;

    if ( height > baseY )
        tmp = 0;
    else
        tmp = baseY - height + 1;

    // ��������� �����
    for ( tmpIdxY = tmp; tmpIdxY <= baseY; tmpIdxY++ )
    {
        for ( tmpIdxX = baseX; tmpIdxX < (baseX + width); tmpIdxX++ )
        {
            response = Lcd_pixel( tmpIdxX, tmpIdxY, mode );
            if(response)
                return response;

        }
    }

    // ��������� ����� ��������� ����
    UpdateLcd = TRUE;
    return OK;
}



/*
 * ���                   :  Lcd_rect_empty
 * ��������              :  ������ ������������� �������������
 * ��������(�)           :  x1    -> ���������� ���������� x ������ �������� ����
 *                          y1    -> ���������� ���������� y ������ �������� ����
 *                          x2    -> ���������� ���������� x ������� ������� ����
 *                          y2    -> ���������� ���������� y ������� ������� ����
 *                          mode  -> Off, On ��� Xor. ������ enum � n5110.h
 * ������������ �������� :  ������ ������������ �������� � n5110lcd.h
 */
uint8_t Lcd_rect_empty ( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LcdPixelMode mode )
{
    uint8_t tmpIdx;

    // �������� ������
    if ( ( x1 >= LCD_X_RES) ||  ( x2 >= LCD_X_RES) || ( y1 >= LCD_Y_RES) || ( y2 >= LCD_Y_RES) )
        return OUT_OF_BORDER;

    if ( ( x2 > x1 ) && ( y2 > y1 ) )
    {
        // ������ �������������� �����
        for ( tmpIdx = x1; tmpIdx <= x2; tmpIdx++ )
        {
            Lcd_pixel( tmpIdx, y1, mode );
            Lcd_pixel( tmpIdx, y2, mode );
        }

        // ������ ������������ �����
        for ( tmpIdx = y1; tmpIdx <= y2; tmpIdx++ )
        {
            Lcd_pixel( x1, tmpIdx, mode );
            Lcd_pixel( x2, tmpIdx, mode );
        }

        // ��������� ����� ��������� ����
        UpdateLcd = TRUE;
    }
    return OK;
}

void SPI_Send(unsigned char x){
	
	unsigned char mask = 0;
	char i = 8;
	
	mask = 0x80;
	
	while (i)
	{
		i--;
		// ��������� ������ MOSI
		if (x & mask)
		   S_MOSI;
		else
			R_MOSI;

		S_CLK;
		for (int h = 0; h<10; h++);
		R_CLK;
		
   	mask >>= 1;
   }
	

	
}
