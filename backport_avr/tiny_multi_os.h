/*notizen: reg: 0xe1:
  0 normal
  1:


*/

#define __NOP__ asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");
#define __NOP__2  asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");
//asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");asm volatile("nop");

/* CPKI AttinyGfxApi & TinyMultiOs, Preview Version 0.9.3b1
  see
  https://www.youtube.com/watch?v=WNJQXsJqSbM
  Copyright (c) 2002
  Görg Pflug & CPKI Gmbh, www.cpki.de . All rights reserved.
  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the CPKI Gmbh, Görg Pflug and its contributors.”
  Neither the name of the Cpki GmbH nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE Görg Pflug, CPKI Gmbh AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE


*/

#define swapu8(x,y) { u8 tmp=x;x=y;y=tmp; }

static unsigned char matrix_vfd[16];

#ifndef _API_DEFINED
#define _API_DEFINED

#ifndef F_CPU
#define F_CPU 16000000
#warning "CPU Clock: assuming 16mhz"
#endif
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __AVR__
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/interrupt.h>
#include <string.h>
#include <avr/power.h>
#include <util/delay.h>
#endif

#ifdef __AVR__
#define s8 signed char
#define u8 unsigned char
#define s16 signed int
#define u16 unsigned int
#define ns_counter s8
#define n_counter u8
#else
#define ns_counter int
#define n_counter unsigned int
#define s8 signed char
#define u8 unsigned char
#define s16 signed short
#define u16 unsigned short
#endif


#define SSD1306_ADDRESS 0x78
#ifndef __AVR__

#define PIN_SDA GPIO_NUM_21
#define PIN_SCL GPIO_NUM_22

#else

#define I2C_PORT PORTD
  #define I2C_PIN PIND
  #define I2C_DDR DDRD
  #define PIN_SDA  1
  #define PIN_SCL 0
#endif



static u8 _engine_flags_and_frame_counter = 0;



// helper function, Calculate Skip Counter from x,y coord
static inline int GfxApiPosition(unsigned char x, unsigned char y)
{
  return x * 64 + y;
}

#ifdef ENABLE_LINEDRAWING
u8 _gfx_linepos = 0;
#ifdef ENABLE_TRIANGLES
u8 _gfx_tripos = 0;
#endif
#ifdef ENABLE_CIRCLES
u8 _gfx_circlepos = 0;
#endif


#ifndef NR_TRIS
#define NR_TRIS 0
#endif
#ifndef NR_CIRCLES
#define NR_CIRCLES 0
#endif
static unsigned char _gfx_points_of_lines[4 * NR_LINES + 8 * NR_TRIS + 4 * NR_CIRCLES];

// Start storage of Line Points
static void GfxApiBeginLines()
{
  _gfx_linepos = 0;
}
#ifdef ENABLE_TRIANGLES
static void GfxApiBeginTriangles()
{
  _gfx_tripos = 0;
}
#endif
#ifdef ENABLE_CIRCLES
static void GfxApiBeginCircles()
{
  _gfx_circlepos = 0;
}
#endif

void reorder_lines()
{
  u8 i;
  for (i = 0; i < _gfx_linepos; i += 4)
  {
    if (_gfx_points_of_lines[i] > _gfx_points_of_lines[i + 2])
    {
      swapu8(_gfx_points_of_lines[i], _gfx_points_of_lines[i + 2]);
      swapu8(_gfx_points_of_lines[i + 1], _gfx_points_of_lines[i + 3]);
    }
  }
}

static void GfxApiStoreLinePoint(unsigned char x1, unsigned char y1)
{
  _gfx_points_of_lines[_gfx_linepos] = x1;
  _gfx_points_of_lines[_gfx_linepos + 1] = y1;
  _gfx_linepos += 2;
}

#ifdef ENABLE_CIRCLES
static void GfxApiStoreCircle(unsigned char x1, unsigned char y1, unsigned char radius, unsigned char pattern)
{
#ifndef ENABLE_TRIANGLES
#define _gfx_tripos 0a
#endif
  _gfx_points_of_lines[_gfx_circlepos + _gfx_linepos + _gfx_tripos] = x1;
  _gfx_points_of_lines[_gfx_circlepos + 1 + _gfx_linepos + _gfx_tripos] = y1;
  _gfx_points_of_lines[_gfx_circlepos + 2 + _gfx_linepos + _gfx_tripos] = radius;
  _gfx_points_of_lines[_gfx_circlepos + 3 + _gfx_linepos + _gfx_tripos] = pattern;
  _gfx_circlepos += 4;

#ifndef ENABLE_TRIANGLES
#undef _gfx_tripos
#endif
}
#endif
#ifdef ENABLE_TRIANGLES
static void GfxApiStoreTrianglePoint(unsigned char x1, unsigned char y1)
{
  _gfx_points_of_lines[_gfx_linepos + _gfx_tripos] = x1;
  _gfx_points_of_lines[_gfx_linepos + 1 + _gfx_tripos] = y1;
  _gfx_tripos += 2;
}
#endif
#endif
#ifdef ENABLE_LINEDRAWING
static unsigned char _cur_seg = 0;
#endif



/// define fehlt xxxxxxx
u8 _ssd1306_drive_counter = 1, _ssd1306_drive_speed = 4;


void ApiIntToHex(u16 in, u8 *out);
u8 ApiCharToFontIndex(u8 c);

typedef struct GfxApiSprite
{
  u16 SkipCounter;
  u16 readpos_byte;
  u8 sprite_height;
  u8 Color;
} GfxApiSprite;

typedef struct GfxApiBargraph
{
  u8 bargraph_type[8];
  u8 bargraph_value[8];
  s8 bargraph_falloff[8];
  u8 bargraph_falloff_change[8];
} GfxApiBargraph;


static u8 GfxApiReadSprite(GfxApiSprite *s);

typedef struct GfxApiCompressedLayer
{
  u16 SkipCounter;
  s8 PixelValue;
  u8 Bitpos;
  const u8 * BytePos;
  u8 Brightness;
} GfxApiCompressedLayer;

static void os_gfx_start_display_transfer();
static void os_i2c_start (void);

static void delay5nop()
{
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
  asm volatile ("nop");
}

#ifndef __AVR__
#include <driver/gpio.h>

// Set SCL pin to low (push-pull mode)
void os_i2c_scl_low() {
  GPIO.out_w1tc = (1 << PIN_SCL);
  GPIO.enable_w1ts = (1 << PIN_SCL);
  GPIO.pin[PIN_SCL].pad_driver = 0;
  __NOP__2
}

// Set SDA pin to low (push-pull mode)
void os_i2c_sda_low() {
  GPIO.out_w1tc = (1 << PIN_SDA);
  GPIO.enable_w1ts = (1 << PIN_SDA);
  GPIO.pin[PIN_SDA].pad_driver = 0;
  __NOP__2
}
void os_i2c_sda_input() {
  gpio_set_direction(PIN_SDA, GPIO_MODE_INPUT);
  gpio_set_pull_mode(PIN_SDA, GPIO_PULLUP_ONLY);
  __NOP__2
}

void os_i2c_sda_output() {
  gpio_set_direction(PIN_SDA, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(PIN_SDA, GPIO_FLOATING);
  __NOP__2
}
// Set SDA pin to high (open-drain mode)
void os_i2c_sda_high() {
  GPIO.pin[PIN_SDA].pad_driver = 1;
  GPIO.out_w1ts = (1 << PIN_SDA);
  GPIO.enable_w1ts = (1 << PIN_SDA);
  __NOP__2
}

// Set SCL pin to high (open-drain mode)
void os_i2c_scl_high() {
  GPIO.pin[PIN_SCL].pad_driver = 1;
  GPIO.out_w1ts = (1 << PIN_SCL);
  GPIO.enable_w1ts = (1 << PIN_SCL);
  __NOP__2
}

// Set SCL pin to low (open-drain mode)
void os_i2c_scl_low_ppull() {
  GPIO.pin[PIN_SCL].pad_driver = 1;
  GPIO.out_w1tc = (1 << PIN_SCL);
  GPIO.enable_w1ts = (1 << PIN_SCL);
  __NOP__2
}

// Set SCL pin to high (open-drain mode)
void os_i2c_scl_high_ppull() {
  GPIO.pin[PIN_SCL].pad_driver = 1;
  GPIO.out_w1ts = (1 << PIN_SCL);
  GPIO.enable_w1ts = (1 << PIN_SCL);
  __NOP__2
}

// Set SDA pin to low (open-drain mode)
void os_i2c_sda_low_ppull() {
  GPIO.pin[PIN_SDA].pad_driver = 1;
  GPIO.out_w1tc = (1 << PIN_SDA);
  GPIO.enable_w1ts = (1 << PIN_SDA);
  __NOP__2
}

// Set SDA pin to high (open-drain mode)
void os_i2c_sda_high_ppull() {
  GPIO.pin[PIN_SDA].pad_driver = 1;
  GPIO.out_w1ts = (1 << PIN_SDA);
  GPIO.enable_w1ts = (1 << PIN_SDA);
  __NOP__2
}
#else

#define os_i2c_scl_low()  do{ I2C_DDR |= (1<<PIN_SCL);} while (0)
#define os_i2c_sda_low()  do{ I2C_DDR |= (1<<PIN_SDA);} while(0)
#define os_i2c_sda_high() do {I2C_DDR &= ~(1<<PIN_SDA);} while(0)
#define os_i2c_scl_high()   do{ I2C_DDR &= ~(1<<PIN_SCL); } while(0)

#define os_i2c_scl_low_ppull() do{ I2C_PORT &= ~(1<<PIN_SCL);} while (0)
#define os_i2c_scl_high_ppull() do{ I2C_PORT |= (1<<PIN_SCL);} while (0)

#define os_i2c_sda_low_ppull() do{ I2C_PORT &= ~(1<<PIN_SDA);} while (0)
#define os_i2c_sda_high_ppull() do{ I2C_PORT |= (1<<PIN_SDA);} while (0)
#endif


/*
//deactivate push pull bus driving....for both scl and sda not needed on the ssd1306
#define os_i2c_scl_low_ppull os_i2c_scl_low
#define os_i2c_scl_high_ppull os_i2c_scl_high

#define os_i2c_sda_low_ppull os_i2c_sda_low
#define os_i2c_sda_high_ppull os_i2c_sda_high*/
//*/
void os_i2c_init (void);
static void os_i2c_write (const u8 *buf, u8 len);

#define __POS_IDX 3
#define __POS_FLIPB 0
#define __POS_SEG 0


const u8 dither_temp3[] __attribute__((progmem)) =
{


  0x19, 0xc0, 0x2a,
  //frame:1
  0x86, 0x20, 0x95,
  //frame:2
  0x60, 0x1f, 0x40,

};

static u8 frame_counter_3;


#ifdef ENABLE_CONSOLE
#define FONT
#endif
#ifdef ENABLE_SECOND_CONSOLE
#define FONT
#endif
#ifdef FONT
const u8 os_font[] __attribute__((progmem)) = {
#ifdef REPLACE_FONT
#include REPLACE_FONT
#else
#ifdef ENABLE_FONT_BASIC
  // source https://raw.githubusercontent.com/dhepper/font8x8/master/font8x8_basic.h , public domain
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 95, 95, 6, 0, 0, 0, 3, 3, 0, 3, 3, 0, 0, 20, 127, 127, 20, 127, 127, 20, 0, 36, 46, 107, 107, 58, 18, 0, 0, 70, 102, 48, 24, 12, 102, 98, 0, 48, 122, 79, 93, 55, 122, 72, 0, 4, 7, 3, 0, 0, 0, 0, 0, 0, 28, 62, 99, 65, 0, 0, 0, 0, 65, 99, 62, 28, 0, 0, 0, 8, 42, 62, 28, 28, 62, 42, 8, 8, 8, 62, 62, 8, 8, 0, 0, 0, 128, 224, 96, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 96, 96, 0, 0, 0, 0, 96, 48, 24, 12, 6, 3, 1, 0, 62, 127, 113, 89, 77, 127, 62, 0, 64, 66, 127, 127, 64, 64, 0, 0, 98, 115, 89, 73, 111, 102, 0, 0, 34, 99, 73, 73, 127, 54, 0, 0, 24, 28, 22, 83, 127, 127, 80, 0, 39, 103, 69, 69, 125, 57, 0, 0, 60, 126, 75, 73, 121, 48, 0, 0, 3, 3, 113, 121, 15, 7, 0, 0, 54, 127, 73, 73, 127, 54, 0, 0, 6, 79, 73, 105, 63, 30, 0, 0, 0, 0, 102, 102, 0, 0, 0, 0, 0, 128, 230, 102, 0, 0, 0, 0, 8, 28, 54, 99, 65, 0, 0, 0, 36, 36, 36, 36, 36, 36, 0, 0, 0, 65, 99, 54, 28, 8, 0, 0, 2, 3, 81, 89, 15, 6, 0, 0, 62, 127, 65, 93, 93, 31, 30, 0, 124, 126, 19, 19, 126, 124, 0, 0, 65, 127, 127, 73, 73, 127, 54, 0, 28, 62, 99, 65, 65, 99, 34, 0, 65, 127, 127, 65, 99, 62, 28, 0, 65, 127, 127, 73, 93, 65, 99, 0, 65, 127, 127, 73, 29, 1, 3, 0, 28, 62, 99, 65, 81, 115, 114, 0, 127, 127, 8, 8, 127, 127, 0, 0, 0, 65, 127, 127, 65, 0, 0, 0, 48, 112, 64, 65, 127, 63, 1, 0, 65, 127, 127, 8, 28, 119, 99, 0, 65, 127, 127, 65, 64, 96, 112, 0, 127, 127, 14, 28, 14, 127, 127, 0, 127, 127, 6, 12, 24, 127, 127, 0, 28, 62, 99, 65, 99, 62, 28, 0, 65, 127, 127, 73, 9, 15, 6, 0, 30, 63, 33, 113, 127, 94, 0, 0, 65, 127, 127, 9, 25, 127, 102, 0, 38, 111, 77, 89, 115, 50, 0, 0, 3, 65, 127, 127, 65, 3, 0, 0, 127, 127, 64, 64, 127, 127, 0, 0, 31, 63, 96, 96, 63, 31, 0, 0, 127, 127, 48, 24, 48, 127, 127, 0, 67, 103, 60, 24, 60, 103, 67, 0, 7, 79, 120, 120, 79, 7, 0, 0, 71, 99, 113, 89, 77, 103, 115, 0, 0, 127, 127, 65, 65, 0, 0, 0, 1, 3, 6, 12, 24, 48, 96, 0, 0, 65, 65, 127, 127, 0, 0, 0, 8, 12, 6, 3, 6, 12, 8, 0, 128, 128, 128, 128, 128, 128, 128, 128, 0, 0, 3, 7, 4, 0, 0, 0, 32, 116, 84, 84, 60, 120, 64, 0, 65, 127, 63, 72, 72, 120, 48, 0, 56, 124, 68, 68, 108, 40, 0, 0, 48, 120, 72, 73, 63, 127, 64, 0, 56, 124, 84, 84, 92, 24, 0, 0, 72, 126, 127, 73, 3, 2, 0, 0, 152, 188, 164, 164, 248, 124, 4, 0, 65, 127, 127, 8, 4, 124, 120, 0, 0, 68, 125, 125, 64, 0, 0, 0, 96, 224, 128, 128, 253, 125, 0, 0, 65, 127, 127, 16, 56, 108, 68, 0, 0, 65, 127, 127, 64, 0, 0, 0, 124, 124, 24, 56, 28, 124, 120, 0, 124, 124, 4, 4, 124, 120, 0, 0, 56, 124, 68, 68, 124, 56, 0, 0, 132, 252, 248, 164, 36, 60, 24, 0, 24, 60, 36, 164, 248, 252, 132, 0, 68, 124, 120, 76, 4, 28, 24, 0, 72, 92, 84, 84, 116, 36, 0, 0, 0, 4, 62, 127, 68, 36, 0, 0, 60, 124, 64, 64, 60, 124, 64, 0, 28, 60, 96, 96, 60, 28, 0, 0, 60, 124, 112, 56, 112, 124, 60, 0, 68, 108, 56, 16, 56, 108, 68, 0, 156, 188, 160, 160, 252, 124, 0, 0, 76, 100, 116, 92, 76, 100, 0, 0, 8, 8, 62, 119, 65, 65, 0, 0, 0, 0, 0, 119, 119, 0, 0, 0, 65, 65, 119, 62, 8, 8, 0, 0, 2, 3, 1, 3, 2, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
#define FONT_ASCII
#define HAVE_FONT
#endif
#ifndef HAVE_FONT  // minimal font 0-9,A-Z
  0, 0, 0, 0, 0, 0, 0, 0, 0, 60, 98, 82, 74, 70, 60, 0, 0, 0, 68, 66, 126, 64, 64,
  0, 0, 100, 82, 82, 82, 82, 76, 0, 0, 36, 66, 66, 74, 74, 52, 0, 0, 48, 40, 36,
  126, 32, 32, 0, 0, 46, 74, 74, 74, 74, 50, 0, 0, 60, 74, 74, 74, 74, 48, 0, 0,
  2, 2, 98, 18, 10, 6, 0, 0, 52, 74, 74, 74, 74, 52, 0, 0, 12, 82, 82, 82, 82, 60, 0, 0,
  124, 18, 18, 18, 18, 124, 0, 0, 126, 74, 74, 74, 74, 52, 0, 0, 60, 66, 66, 66, 66,
  36, 0, 0, 126, 66, 66, 66, 36, 24, 0, 0, 126, 74, 74, 74, 74, 66, 0, 0, 126, 10,
  10, 10, 10, 2, 0, 0, 60, 66, 66, 82, 82, 52, 0, 0, 126, 8, 8, 8, 8, 126, 0, 0, 0,
  66, 66, 126, 66, 66, 0, 0, 48, 64, 64, 64, 64, 62, 0, 0, 126, 8, 8, 20, 34,
  64, 0, 0, 126, 64, 64, 64, 64, 64, 0, 0, 126, 4, 8, 8, 4, 126, 0, 0, 126, 4, 8, 16,
  32, 126, 0, 0, 60, 66, 66, 66, 66, 60, 0, 0, 126, 18, 18, 18, 18, 12, 0, 0, 60, 66, 82, 98,
  66, 60, 0, 0, 126, 18, 18, 18, 50, 76, 0, 0, 36, 74, 74, 74, 74, 48, 0, 2, 2, 2, 126, 2, 2,
  2, 0, 0, 62, 64, 64, 64, 64, 62, 0, 0, 30, 32, 64, 64, 32, 30, 0, 0, 62, 64, 32, 32, 64,
  62, 0, 0, 66, 36, 24, 24, 36, 66, 0, 2, 4, 8, 112, 8, 4, 2, 0, 0, 66, 98, 82, 74, 70, 66, 0,
#undef HAVE_FONT
#undef FONT
#endif

#ifdef ENABLE_USERFONT
#include "userfont.h"
#endif
#endif
};
#endif

static void os_i2c_start (void)
{
  os_i2c_scl_high();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  os_i2c_sda_low();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  os_i2c_scl_low();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  os_i2c_sda_high();
}




static void os_i2c_stop (void)
{
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  os_i2c_sda_low();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;

  os_i2c_scl_high();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  os_i2c_sda_high();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
}


static u8 os_i2c_write_byte (u8 byte)
{
  u8 bitpos = 0x80;

  do {
    os_i2c_scl_low();
    if (byte & bitpos)
      os_i2c_sda_high();
    else
      os_i2c_sda_low();

    os_i2c_scl_high();
    bitpos >>= 1;

  } while (bitpos);
  os_i2c_scl_low();

  os_i2c_sda_high();
  os_i2c_scl_high();
  __NOP__;
  __NOP__;
  __NOP__;
#ifdef __AVR__
  u8 ack = (I2C_PIN & (1 << PIN_SDA)) ? 1 : 0;
#else
  u8 ack = (GPIO.in >> PIN_SDA) & 0x01;
#endif

  os_i2c_scl_low();
  os_i2c_sda_low();
  return ack;
}

#ifdef __AVR__

static inline u8 os_i2c_read_byte (u8 ack)
{
  os_i2c_scl_low();
  os_i2c_sda_high();

  u8 in = 0;
  u8 bitpos = 0x80;
  do {
    os_i2c_scl_high();
    __NOP__;
    __NOP__;
    __NOP__;
    __NOP__;
    __NOP__;
    __NOP__;
#ifdef __AVR__
    if (I2C_PIN & (1 << PIN_SDA))
      in |= bitpos;
#else
    if (GPIO.in & (1 << PIN_SDA))
      in |= bitpos;
#endif

    bitpos >>= 1;
    os_i2c_scl_low();
    __NOP__;
    __NOP__;

  } while (bitpos);

  if (ack)os_i2c_sda_low();

  os_i2c_scl_high();

  return in;
}
#else

static inline uint8_t os_i2c_read_byte(uint8_t ack) {
  os_i2c_scl_low();
  os_i2c_sda_high();
  os_i2c_sda_input();

  uint8_t in = 0;
  uint8_t bitpos = 0x80;

  do {
    os_i2c_scl_high();
    __NOP__;
    __NOP__;
    __NOP__;
    __NOP__;
    __NOP__;
    __NOP__;

    if (gpio_get_level(PIN_SDA))
      in |= bitpos;

    bitpos >>= 1;
    os_i2c_scl_low();
    __NOP__;
    __NOP__;
  } while (bitpos);

  if (ack)
    os_i2c_sda_low_ppull();
  else
    os_i2c_sda_high();

  os_i2c_scl_high();
  os_i2c_sda_output();

  return in;
}


#endif

void os_i2c_init (void)
{
/*#ifndef ENABLE_WIRE

   gpio_set_direction(PIN_SDA, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(PIN_SDA, GPIO_FLOATING);
 gpio_set_direction(PIN_SCL, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(PIN_SCL, GPIO_FLOATING);

#ifdef __AVR__*/
  I2C_PORT = 0;//_BV(PIN_SDA) | _BV(PIN_SCL);
//#endif
  //I2C_DDR  = _BV(PIN_SDA) | _BV(PIN_SCL);
  os_i2c_scl_high();
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");

  os_i2c_sda_high();
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");
               asm volatile("nop");

/*#else
  Wire.begin();
  Wire.setClock(5*400000UL);
#endif*/
}

void xos_i2c_init (void)
{

  I2C_PORT = _BV(PIN_SDA) | _BV(PIN_SCL);
  I2C_DDR  = _BV(PIN_SDA) | _BV(PIN_SCL);
  os_i2c_scl_high();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  os_i2c_sda_high();
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;
  __NOP__;

}

static void os_i2c_write (const u8 *buf, u8 len)
{
  os_i2c_start();
  os_i2c_write_byte(SSD1306_ADDRESS);
  while (len--)
    os_i2c_write_byte(*buf++);
  os_i2c_stop();
}

// 24c512 support
static void eeprom_connect(u8 address, u16 from_addr)
{
  for (;;)
  {
    os_i2c_start();
    u8 ack = os_i2c_write_byte(address);
    if (!ack)break;
    os_i2c_stop();
    //    about 150 loops when called in sequence
  }
  os_i2c_write_byte(from_addr >> 8);
  os_i2c_write_byte(from_addr);

}
void eeprom_i2c_write(u8 address, u16 from_addr, u8 data) {
  eeprom_connect(address, from_addr);
  os_i2c_write_byte(data);
  os_i2c_stop();
}

static void eeprom_i2c_read(u8 address, int from_addr, u8 *dst, u8 len) {
  eeprom_connect(address, from_addr);

  os_i2c_stop();
  os_i2c_start();
  os_i2c_write_byte(address | 1);

  u8 i;
  for (i = len; i; i--)
    *dst = os_i2c_read_byte(1), dst++;

  os_i2c_scl_low();  // scl must be lowered after read, read lowers scl at its start, this saves cycles because the ack can be "processed" (=ignored) by processing the outer for loop.
  os_i2c_stop();
}


/*
  FUSES =
  {
  .low = LFUSE_DEFAULT,
  .high = HFUSE_DEFAULT,//(FUSE_BOOTSZ0 & FUSE_BOOTSZ1 & FUSE_EESAVE & FUSE_SPIEN ),
  .extended = FUSE_SELFPRGEN,
  };
*/
const u8 os_conv_hex[] __attribute__((progmem)) = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

void ApiIntToHex(u16 in, u8 *out)
{
  out[0] = pgm_read_byte(&os_conv_hex[(in >> 12) & 0xf]);
  out[1] = pgm_read_byte(&os_conv_hex[(in >> 8) & 0xf]);
  out[2] = pgm_read_byte(&os_conv_hex[(in >> 4) & 0xf]);
  out[3] = pgm_read_byte(&os_conv_hex[in & 0xf]);
}

u8 ApiCharToFontIndex(u8 c)
{
#ifdef FONT_ASCII
  return c - 0x20;
#else
  if (c >= 'a' && c <= 'z')return 11 + c - 'a';
  if (c >= 'A' && c <= 'Z')return 11 + c - 'A';
  if (c >= '0' && c <= '9')return 1 + c - '0';
  return 0;
#endif
}


// convert a C-String to Font Indices, can be used to use sprintf directly in the console buffer
// example:
// sprintf(myconsole,"hello world");
// GfxApiConvertStringToFontIdx(myconsole);
void GfxApiConvertStringToFontIdx(u8 *s)
{
  while (*s)
  {
    *s = ApiCharToFontIndex(*s);
    s++;
  }
}
#define SystemServer_WriteToScreen GfxApiWriteToConsole
// for compatibility until all demos are fixed
extern void GfxApiWriteToConsole(const char *txt, u8 *screen, u8 x, u8 y)
{
  int p = x + 16 * y;
  int l = strlen(txt);
  int i;
  for (i = 0; i < l; i++)screen[(i + p) & 0x7f] = ApiCharToFontIndex(txt[i]);
}
#ifdef ENABLE_LINEDRAWING
void _reorder_lines()
{
  u8 i;
  for (i = 0; i < _gfx_linepos; i += 4)
  {
    if (_gfx_points_of_lines[i] > _gfx_points_of_lines[i + 2])
    {
      swapu8(_gfx_points_of_lines[i], _gfx_points_of_lines[i + 2]);
      swapu8(_gfx_points_of_lines[i + 1], _gfx_points_of_lines[i + 3]);
    }
  }
}
#endif

void os_init_ssd1306 (void)
{
  // brigher screen init:
#ifndef ENABLE_DARKER_SCREEN
  const u8 init1306[] = {
    0, 0xe4, 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x0, 0x40, 0x8D, 0x14, 0x20, 0x01, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40,  0xA4, 0xA6, 0xAF
  };
#else
  // darker screen init:
  const u8 init1306[] = {
    0x0, 0x20, 1 /*vertical mode*/, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0x0, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 0x20, 0x8D, 0x14, 0xAF
  };
#endif
  os_i2c_write(init1306, sizeof(init1306));
}


static void GfxApiShowFrame(u8 grayscale_zoom)   // use mux:1
{
  const u8 init1306[] = {0,0xa5,
                         0xd6, grayscale_zoom
                         , 0xa8, 63, //zoom:1, mux 64
                         0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, // nop
                         0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3,
                         0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, // nop
                         0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3,
                         0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, // nop
                         0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3,
                         
                         
                         0xa8, 1,0xa4  // mux:1
                        };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiSetDisplayZoom (u8 brightness)
{
  const u8 init1306[] = {
    0,
    0xd6, brightness
  };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiWriteVRam (u16 adr, u8 val)
{
  const u8 setPositionCmd[] = {
    0x00,                    // Co = 0, D/C# = 0
    0x20, 0b10,
    0xB0 + (adr >> 7) ,   // Set page address
    0x00 | (adr & 0x0f) , // Set column lower nibble
    0x10 | ((adr >> 4) & 0xf)  // Set column higher nibble
  };
  const u8 dta[] = {0x40, val};
  os_i2c_write(setPositionCmd, sizeof(setPositionCmd));
  os_i2c_write(dta, sizeof(dta));
}
void ___pset(int x, int y)
{
  int row = y / 8;
  u16 b = 3 << (y - row * 8);
  y++;
  //b|=1<<(y&7);
  GfxApiWriteVRam(x + row * 128, b);
}


void GfxApiStartVectorScope(u8 x_zoom)
{
  int i;
  for (i = 0; i < 1024; i++)GfxApiWriteVRam(i, 0);
  for (i = 2; i < 64; i++)
  {
    if (x_zoom == 0)
      ___pset(i + 32, i);


    if (x_zoom == 2)
    {
      ___pset(i + 32, i);
      ___pset(i / 2, i);
      ___pset(i / 2 + 96, i);

    }


    if (x_zoom == 1)
    {
      ___pset(i * 2, i);
      ___pset(i * 2 + 1, i);
    }
  }
}


// 0 loop pixels
// 1 repeat pixels
// 2 make new pixels black    
// 3 make new pixels white

void GfxApiSetDisplaySetContentScroll (u8 direction, u8 clear_mode, u8 x_start, u8 row_start, u8 x_end,  u8 row_end)
{
  const u8 init1306[] = {
    0,// 0xd6, 1, 0xda, fuzz_d2
    0x2c + (direction?1:0), 0, row_start, clear_mode, row_end, x_start, x_end/*
  // 132 nop
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,

    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,


    //0xfc//,0xa5


    // e1: 1 parameter kein effekt

    /*
      ,0,
      2,  // row
      fuzz_d2,  // pattern to fill
      4,  // row
      32  // xstart
      ,64 // xend*/
  };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiSetDisplayDrawRect (u8 direction, u8 x_start, u8 row_start, u8 x_end,  u8 row_end, u8 clear_pixels)
{
  // 8 schwarz permanent?
  // 16

  // xd2, 4 dunkler, saft?
  // 8 schwarz
  //16,32 sw
  const u8 init1306[] = {
    0,// 0xd6, 1, 0xda, fuzz_d2
    0x24, direction, row_start, clear_pixels, row_end, x_start, x_end,
    /*
      // 132 nop
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,

        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
    */
  };
  //*/
  os_i2c_write(init1306, sizeof(init1306));
}






void GfxApiSetBrightnessEx (u8 brightness, u8 br1, u8 br2)
{
  //brightness=brightness|(brightness<<4);
  u8 v1 = br1; //brightness>>5;
  v1 <<= 4;

  u8 v2 = br2; //brightness>>4;
  v2 += 1;
  if (v2 > 15)v2 = 15; // better temporal dither?
  v2 |= v2 << 4;
  const u8 init1306[] = {
    0,
    0xd9, v2, 0xdb, v1, 0x81, brightness
  };
  os_i2c_write(init1306, sizeof(init1306));
}
void GfxApiSetDisplayUndocumentedD2 (u8 par)
{
  const u8 init1306[] = {
    0,
    0xd2, par//((brightness | 1 ) )&~16
  };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiConsoleSetAttribute(u8 hb)
{
  _engine_flags_and_frame_counter &= 0x7f;
  if (hb)_engine_flags_and_frame_counter |= 0x80;
}
// set the brightness of the screen 0 to 15
/*void GfxApiSetFreq (u8 freq)
{

  u16 a = (1 + freq / 2) << 4;
  a |= 15 - freq;
  a = freq;
  a = 0xf0;
}*/

void GfxApiSetFreq (u8 freq)
{  
  const u8 init1306[]={
    0,
    0xd5,freq
  };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiSetPrecharge(u8 a)
{
  // freq=0;
  const u8 init1306[] = {0,
                         0xd9, a //,0xdb,b
                         //    0xa2,freq,   //scrolly
                        };
  os_i2c_write(init1306, sizeof(init1306));
}


// power 0 to 5  , 0 off
void GfxApiVectoscopeSetOutputVoltage(u8 a)
{
  // freq=0;
  if(a>4)a=4;
  const u8 init1306[] = {
    0, 0x8d, "\x10\x15\x14\x94\x95"[a]
    //    0xa2,freq,   //scrolly
  };
  os_i2c_write(init1306, sizeof(init1306));
}
// set the brightness of the screen 0 to 15
void GfxApiSetBrightness (u8 brightness)
{
  brightness = brightness | (brightness << 4);
  u8 v1 = brightness >> 5;
  v1 <<= 4;
  u8 v2 = brightness >> 4;
  v2 |= v2 << 4;
  const u8 init1306[] = {
    0,
    0xd9, v2, 0xdb, v1, 0x81, brightness
  };
  os_i2c_write(init1306, sizeof(init1306));
}



void GfxApiDisplaySetChargePump(u8 a)
{
  // freq=0;
  const u8 init1306[] = {
    0, 0x8d, a
    //    0xa2,freq,   //scrolly
  };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiDisplaySetRegP1(u8 a,u8 b)
{
  // freq=0;
  const u8 init1306[] = {
    0, a,b
    //    0xa2,freq,   //scrolly
  };
  os_i2c_write(init1306, sizeof(init1306));
}

void GfxApiSetVcom(u8 a)
{
  // freq=0;
  const u8 init1306[] = {
    0, 0xdb, a
  };
  os_i2c_write(init1306, sizeof(init1306));
}


void GfxApiSetDisplayEnable (u8 e)
{
  const u8 init1306[] = {
    0,
    0x0ae + (e ? 1 : 0)
  };
  os_i2c_write(init1306, sizeof(init1306));
}
void GfxApiFlipY(u8 flip_y)
{
  const u8 init1306[] = {
    0,
    flip_y ? 0xc0 : 0xc8
  };
  os_i2c_write(init1306, sizeof(init1306));
}
void GfxApiSetInvert (u8 invert)
{
  const u8 init1306[] = {
    0,
    0xa6 + (invert ? 1 : 0)
  };
  os_i2c_write(init1306, sizeof(init1306));
}
void GfxApiSetStartline (u8 startline)
{
  const u8 init1306[] = {
    0,
    0x40 + (startline & 63)
  };
  os_i2c_write(init1306, sizeof(init1306));
}
void GfxApiSetDisplayOffset (u8 startline)
{
  const u8 init1306[] = {
    0,
    0xd3, startline
  };
  os_i2c_write(init1306, sizeof(init1306));
}



void GfxApiSetDisplayMux (u8 e)
{
  const u8 init1306[] = {
    0,
    0x0a8,  e
  };
  os_i2c_write(init1306, sizeof(init1306));
}

void GfxApiSetScrollArea(u8 a, u8 b)
{
  const u8 init1306[] = {
    0,
    0x0a3,  a, b
  };
  os_i2c_write(init1306, sizeof(init1306));

}



void GfxApiSetDisplayOn (u8 e)
{
  const u8 init1306[] = {
    0,
    0x0a4 + e
  };
  os_i2c_write(init1306, sizeof(init1306));
}

#ifdef CONSOLE_ENABLE_CURSOR
static u8 cursor_pos = 0;
#ifdef CONSOLE_ENABLE_CURSOR_BLINK
#ifndef CONSOLE_CURSOR_BLINK_MASK
#define CONSOLE_CURSOR_BLINK_MASK 1
#endif
#endif
#endif


#ifdef CONSOLE_ENABLE_PRINTF
static u8 * _printf_console;
#ifndef CONSOLE_ENABLE_CURSOR
static u8 cursor_pos = 0;
#endif

static void GfxApiAttachConsole(u8*console)
{
  _printf_console = console;
}

int console_putc(char c, FILE *file) {
  // console[1]++;//=10;
  if (c != '\n')
  {
    _printf_console[cursor_pos] = ApiCharToFontIndex(c) | (_engine_flags_and_frame_counter & 0x80);
    cursor_pos++;
  }
  else cursor_pos = (cursor_pos & 0xf0) + 0x10;

  if (cursor_pos >= 128)
  {
    u8 i;
    for (i = 16; i < 128; i++)_printf_console[i - 16] = _printf_console[i];
    for (i = 128 - 16; i < 128; i++)_printf_console[i] = 0;
    cursor_pos -= 16;
  }
  return c;
}
#ifndef __AVR__

int console_putc_esp32(_reent* r, void* file, const char* ptr, int len) {
  for (int i = 0; i < len; i++) {
    console_putc(ptr[i], stdout);
  }

  // Return the number of characters written
  return len;
}
#endif


#endif



#if defined(CONSOLE_ENABLE_CURSOR) || defined(CONSOLE_ENABLE_PRINTF)
void GfxApiGotoXY(u8 x, u8 y)
{
  cursor_pos = (x & 15) + (y << 4);
}


#endif

#ifndef ENABLE_ARDUINO_SUPPORT
extern void MainTask();
void setup()
{
  cli();
  /*
    I2C_DDR=0;
    //  I2C_PORT=255;  // input + pullup
    //only attiny85 PRR=0b1100;   // disable timer, adc
    MCUCR|=0x80; // disable brownout
    ACSR|=0x80;
  */
  os_i2c_init();
  os_init_ssd1306();
#ifdef ENABLE_ATTINY_POWER_MANAGER
  MCUSR = 0;
  wdt_enable(WDTO_120MS);
  WDTCR |= 1 << WDIE;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
#endif
  sei();

#ifdef CONSOLE_ENABLE_PRINTF
#ifdef __AVR__
  fdevopen(&console_putc, 0);
#else
  stdout->_write = console_putc_esp32;
  stdout->_flags = __SWR;
  setvbuf(stdout, NULL, _IONBF, 0);


#endif

#endif

  MainTask();
}
#else
void streamgfx_init_arduino()
{
  os_i2c_init();
  os_init_ssd1306();
#ifdef CONSOLE_ENABLE_PRINTF
  fdevopen(&console_putc, 0);
#endif
}
#endif

#ifdef ENABLE_ATTINY_POWER_MANAGER
ISR(WDT_vect)
{
  WDTCR |= 1 << WDIE;
}
u16 bat_volt = 5200;
u8 low_power_screen_disable = 0;
s8 _br = 15;
#define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))

int _manage_battery() {
  static u8 phase = 0;
  if (phase == 1)
  {
    PRR &= ~1; // enable adc....
    phase++;
    return bat_volt;
  }
  if (phase == 2)
  {
    ADMUX =
      (0 << REFS1) | // Sets ref. voltage to VCC, bit 1
      (0 << REFS0) | // Sets ref. voltage to VCC, bit 0
      (1 << MUX3)  | // use Vbg as input, MUX bit 3
      (1 << MUX2)  | // use Vbg as input, MUX bit 2
      (0 << MUX1)  | // use Vbg as input, MUX bit 1
      (0 << MUX0);   // use Vbg as input, MUX bit 0
    ADCSRA =
      (1 << ADEN)  | // enable ADC
      (1 << ADPS2) | // set prescaler to 64, bit 2
      (1 << ADPS1) | // set prescaler to 64, bit 1
      (0 << ADPS0);  // set prescaler to 64, bit 0
    phase++;
    return bat_volt;
  }
  if (phase == 3)
  {
    ADCSRA &= ~(1 << ADEN);
    ACSR |= (1 << ACD);
    ADCSRA |= (1 << ADEN);
    phase++;
    return bat_volt;
  }
  if (phase == 4)
  {
    ADCSRA |= (1 << ADSC); // start ADC measurement
    while ( ADCSRA & (1 << ADSC) ); // wait till conversion complete
    uint16_t adc = ADC;
    // clear the ADIF bit by writing 1 to it
    ADCSRA |= (1 << ADIF);
    // disable the ADC
    ADCSRA &= ~(1 << ADEN);
    phase++;
    bat_volt = 1000.0f * 1024.0f * 1.1f / (float)adc;
    return bat_volt;
  }
  PRR |= 1;
  if (bat_volt < 2000)
  {
    if (!low_power_screen_disable)
    {
      GfxApiSetDisplayEnable(0);

      const u8 disable_charge_pump[] = {
        0,
        0x8d,
        0x10
      };
      os_i2c_write(disable_charge_pump, sizeof(disable_charge_pump));

      clock_prescale_set(clock_div_256);
    }
    low_power_screen_disable = 10;
  }
  else
  {
    if (bat_volt < POWER_MANAGER_LOW_POWER)
    {

      if (bat_volt < POWER_MANAGER_LOW_POWER - 50)
        _br -= 1;
      if (_br < 0)_br = 0;
      //  sleep_cpu();
      //  os_i2c_init();  // bring usi back....
    }
    else
    {
      if (bat_volt > POWER_MANAGER_LOW_POWER + 50)
      {
        _br++;
        if (_br > 15)_br = 15;
      }
    }
    _br = 15;
    GfxApiSetBrightness(_br);

    if (low_power_screen_disable)
    {
      sleep_cpu();
      if (bat_volt > 3300)
      {
        sleep_cpu();
        low_power_screen_disable--;
        if (!low_power_screen_disable)
        {
          const u8 reset[] = {
            0,
            0xe4 // undocumented command, said to help startup if display is "confused"
          };
          clock_prescale_set(clock_div_1);

          _delay_ms(500);
          for (int i = 0; i < 18; i++)
            os_i2c_stop();

          os_init_ssd1306();
        }
      }
    }
  }
  phase++;
  if (phase == 10)phase = 0;
}
#endif
#endif  //api functions,fonts, power manager

#ifdef DISPLAYFUNC
/*
   todo: optimierung aufbau an pixel x=128 y=row 7 beginnen, i2c_dataout statisch machen, clock_i2c abschaffen... kürzerer code....
   layer in if moven, layersum "statisch für den if" machen, layersum nur berechnen wenn auch nötig


*/



#include <math.h>
#include <stdlib.h>
//#define ENABLE_TRIANGLES
/* CPKI AttinyStreamGfxApi & TinyMultiOs, Version 0.9.3b1
  see
  https://www.youtube.com/watch?v=WNJQXsJqSbM
  Copyright (c) 2022
  Görg Pflug & CPKI Gmbh, www.cpki.de . All rights reserved.
  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
  Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the CPKI Gmbh, Görg Pflug and its contributors.”
  Neither the name of the Cpki GmbH nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY Görg Pflug, CPKI Gmbh AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
  Version:
  0.8.1 faster
  0.8   Filled Triangle drawing
  0.5   faster, support functions, Consistent API, engine-interleaved I2C, propper line-drawing-api
    much better 3D-Performance
  0.4   Prelimary Support for Arduino-Wire, performance worse than on attiny85
  0.3   Compatible with C++, compiles in Microchip Studio and Arduino, most samples still untested
    added .INO example for Arduino
  0.2   Multiple Consoles
  0.2.1 ASCII Font Support
    define: ENABLE_FONT_BASIC
    "Low Quality" Halftoning, saves 64 bytes, faster, for some use cases "low quality" halftoning might even look better
    define: ENABLE_LOW_QUALITY_HALFTONE
    Halftoning gets disabled Automatically when not using layers or pixel callback, faster, saves 64 byte

*/

#pragma GCC push_options
#ifndef DISABLE_OFAST
#pragma GCC optimize ("Ofast")
#endif
#ifndef ENABLE_LAYERS
#ifndef PIXEL_CALLBACK
#define DISABLE_HALFTONE
#endif
#endif

#ifndef GfxApiLayerGetNextByteInlineHack
#define GfxApiLayerGetNextByteInlineHack



static void os_i2c_write_byte_fast (u8 byte)
{
  u8 bitpos = 0x80;

  do {
    os_i2c_scl_low();
    if (byte & bitpos)
      os_i2c_sda_high();
    else
      os_i2c_sda_low();

    os_i2c_scl_high();
    bitpos >>= 1;
  } while (bitpos);
  os_i2c_scl_low();

  os_i2c_sda_high();
  os_i2c_scl_high();;
  __NOP__;
  os_i2c_scl_low();
  //    __NOP__;

  os_i2c_sda_low();
}


static const u8 _bit_set[] PROGMEM = {1, 2, 4, 8, 16, 32, 64, 128};

static const u8 _ordered_dither_matrix_simulated_annealing[] PROGMEM =
{
  // hacked dither pattern, does not contain 0 and 63, optimized by hillclimbing
  55, 11, 21, 52, 14, 24, 61, 18,
  48, 19, 5, 45, 27, 15, 43, 7,
  13, 59, 30, 9, 56, 33, 22, 62,
  26, 41, 1, 36, 47, 3, 51, 38,
  28, 39, 53, 17, 40, 60, 1, 35,
  57, 2, 23, 32, 12, 49, 25, 10,
  16, 50, 62, 6, 37, 20, 54, 31,
  4, 34, 44, 29, 58, 42, 8, 46,
  /*
    13,48,19,56,6,22,62,41,
    30,24,43,0,36,51,10,31,
    2,53,11,58,15,27,44,17,
    40,42,21,38,47,8,34,54,
    14,63,49,4,23,61,20,5,
    45,35,9,59,28,1,50,32,
    18,52,60,16,55,12,25,57,
    29,7,39,26,33,46,37,3,

    /*
    // this seems to be the best matrix for vector drawing
    52,10,59,2,36,9,34,3,
    20,38,23,49,19,62,22,48,
    61,5,54,11,43,4,40,12,
    25,42,17,46,27,57,18,45,
    53,1,58,32,15,33,50,8,
    21,31,39,7,63,0,30,37,
    6,56,13,47,16,44,14,60,
    41,24,35,29,55,26,51,28,
  */
};  // ordered dither matrix generated by simulated annealing


// gcc is too stupid to inline this function..... because it is compiled on O3...
// so split.... but we want to define it only once, because the display function can potentially be included multiple times
// one should check this for the sprites too, the compiler pushed loads of variables.


static void os_i2c_stop_fast (void)
{
  os_i2c_sda_low();
  __NOP__;
  os_i2c_scl_high();
  __NOP__;
  os_i2c_sda_high();
  __NOP__;
}


static void os_i2c_start_fast (void)
{
  os_i2c_scl_high();
  __NOP__;
  os_i2c_sda_low();
  __NOP__;
  os_i2c_scl_low();
  __NOP__;
  os_i2c_sda_high();
}

static inline void os_restart_display_transfer()
{
  os_i2c_start_fast();
  os_i2c_write_byte_fast(SSD1306_ADDRESS);
  os_i2c_write_byte_fast(0x40);
}


static void os_gfx_start_display_transfer()
{
  const u8 init1306[] = {
    0, 0x21, 0, 0x7f, 0xb7, 0xf, ((127 >> 4) & 0xf) | 0x10
#ifdef __SCREENMODE_MUX
    , 0xa8, __SCREENMODE_MUX
#endif
  };
  os_i2c_write(init1306, sizeof(init1306));
  os_restart_display_transfer();
}
static inline void os_i2c_write_fast (const u8 *buf, u8 len)
{
  os_i2c_start_fast();
  os_i2c_write_byte_fast(SSD1306_ADDRESS);
  while (len--)
    os_i2c_write_byte_fast(*buf++);
  os_i2c_stop_fast();
}


static void GfxApiVectorscopeDisplay(s8 x, u8 y)
{
  if (x < 0)return;
  if (y < 0)return;
  if (x > 63)return;
  if (y > 63)return;

  static s8 last_x;
  const u8 move_line[] = {0,
                          0xa8, 1, 0xd3, (y & 63), 0x40 + (x & 63)
                         };
  const u8 park_line[] = {0,
                          0xa8, 1, 0x40, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3,
                          0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3,
                          0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3, 0xe3,
                         };
  //      if(abs(last_x-y)<2)
  os_i2c_write_fast(move_line, sizeof(move_line));
  //    else
  //    os_i2c_write_fast(park_line, sizeof(park_line));
  //    last_x=y;
}


static void GfxApiVectorscopeDisplay2(s8 x, u8 y, s8 brightness)
{
  static s8 last_x;
  const u8 move_line[] = {0,
                          0xa8, 1, 0xd3, (y & 63), 0x40 + (x & 63)
                         };
  const u8 move_line2[] = {0,
                           0xd1, 15, 0xa8, 1, 0xd3, (y & 63), 0x40 + (x & 63), 0xd1, 0

                          };
  if (abs(x - last_x) < 2)
    os_i2c_write_fast(move_line, sizeof(move_line));
  else
    os_i2c_write_fast(move_line2, sizeof(move_line2));

  last_x = x;
}



void GfxApiVectorScopeLine(int x1, int y1, int x2, int y2, int brightness, int off_after) {
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;
  int pixel_nr = 0;
  while (1) {
    GfxApiVectorscopeDisplay(x1, y1);
    if (!pixel_nr)
    {
      GfxApiVectoscopeSetOutputVoltage(brightness);
    }
    pixel_nr++;
    off_after--;
    if (!off_after)GfxApiVectoscopeSetOutputVoltage(0);

    if (x1 == x2 && y1 == y2) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x1 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y1 += sy;
    }
  }
  GfxApiVectoscopeSetOutputVoltage(0);
}


void GfxApiVectorScopeLine2(int x1, int y1, int x2, int y2) {
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  os_i2c_start();
  os_i2c_write_byte(SSD1306_ADDRESS);
  os_i2c_write_byte(0x0);  // command mode
  i2c_counter += 2;
  while (1) {
    if ((x1 >= 0) && (x1 < 64) && (y1 >= 0) && (y1 < 64))

    {
      static int last_x, last_y;
      if (x1 != last_x) {
        os_i2c_write_byte_fast(0xd3);
        os_i2c_write_byte_fast(x1 & 63);
        i2c_counter += 2;
      }
      if (y1 != last_y)       {
        os_i2c_write_byte_fast(0x40 + (y1 & 63));
        i2c_counter++;
      }
      last_x = x1;
      last_y = y1;
    }
    if (x1 == x2 && y1 == y2) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x1 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y1 += sy;
    }
  }
  os_i2c_stop();
}

u8 add_40h = 0;

void vectoscopeHline(s16 x1, s16 y1, s16 len, u8 c)
{
  if (y1 < 0)return;
  if (y1 > 63)return;
  if (x1 > 127)return;
  if (x1 < 0)len += x1, x1 = 0;
  if (len < 0)return;
  if (x1 + len > 127)
  {
    len -= x1 + len - 127;
  }

  static int last_xstart;
  static int last_xend;
  static int last_y;


  static u8 row = 0;

  // 0x24,direction,row_start,clear_pixels,row_end,x_start,x_end,
  const u8 cmd1[] = {
    0x24, 0,
    row,
    0/* clear*/,
    row,
    0, 127,
    //      0xe3,0xe3,0xe3,0xe3  , // clear line
  };
  const u8 cmd1b[] = {

    0x24, 0, row, c/*set*/, row, x1, x1 + len,
  };// set line
  const u8 cmd2[] = {

    0xd3, y1 & 63,
    //   0x40//+62
    // , 0xe3,0xe3  // move line
  };
  if ((last_xstart != x1) || (last_xend != x1 + len))
  {
    if ((last_xstart != x1) && (last_xend != x1 + len))
    {
      if ((x1 > last_xstart) || (x1 + len < last_xend)) // the new line to be drawn needs to be cleared, if the white area has not expanded
        for (u8 i = 0; i < sizeof(cmd1); i++)
        {
          os_i2c_write_byte_fast(cmd1[i]);//+(y1&63));
          i2c_counter++;
        }

      for (u8 i = 0; i < sizeof(cmd1b); i++)
      {
        os_i2c_write_byte_fast(cmd1b[i]);//+(y1&63));
        i2c_counter++;
      }
    }
    else   // one is identical
    {
      if (last_xstart == x1) // change at the right side
      {
        if (x1 + len < last_xend) // contract
        {
          const u8 contract_cmd[] = {
            0x24, 0,
            row,
            0/* clear*/,
            row,
            x1 + len + 1, 127,
            //      0xe3,0xe3,0xe3,0xe3  , // clear line
          };
          for (u8 i = 0; i < sizeof(contract_cmd); i++)
          {
            os_i2c_write_byte_fast(contract_cmd[i]);//+(y1&63));
            i2c_counter++;
          }

        }
        else
        {
          for (u8 i = 0; i < sizeof(cmd1b); i++)
          {
            os_i2c_write_byte_fast(cmd1b[i]);//+(y1&63));
            i2c_counter++;
          }

        }
      }
      else
      {
        if (x1 > last_xstart) // contract
        {
          const u8 contract_cmd[] = {
            0x24, 0,
            row,
            0/* clear*/,
            row,
            0, x1 - 1,
            //      0xe3,0xe3,0xe3,0xe3  , // clear line
          };
          for (u8 i = 0; i < sizeof(contract_cmd); i++)
          {
            os_i2c_write_byte_fast(contract_cmd[i]);//+(y1&63));
            i2c_counter++;
          }

        }
        else   //expand, just draw it
        {
          for (u8 i = 0; i < sizeof(cmd1b); i++)
          {
            os_i2c_write_byte_fast(cmd1b[i]);//+(y1&63));
            i2c_counter++;
          }

        }
      }

    }


    last_xstart = x1;
    last_xend = x1 + len;
  }
  if (last_y != y1)
  {
    for (u8 i = 0; i < sizeof(cmd2); i++)
    {
      os_i2c_write_byte_fast(cmd2[i]);//+(y1&63));
      i2c_counter++;
    }
    last_y = y1;
  }

  i2c_counter += 2;
}


void VectoscopeTriangle(s16 x1, s16 y1, s16 x2, s16 y2, s16 x3, s16 y3, u8 c)
{
  cli();

  if (x2 < 0)return;
  if (x2 > 126)return;
  if (y2 < 0)return;
  if (y2 > 62)return;


  if (x1 < 0)return;
  if (x1 > 126)return;
  if (y1 < 0)return;
  if (y1 > 62)return;


  if (x1 < 0)return;
  if (x1 > 126)return;
  if (y1 < 0)return;
  if (y1 > 62)return;
  os_i2c_start();
  os_i2c_write_byte(SSD1306_ADDRESS);
  os_i2c_write_byte(0x0);  // command mode
  os_i2c_write_byte(0x40);  // command mode

  s16 t1x, t2x, y, minx, maxx, t1xp, t2xp;
  u8 change = 0;
  //const u8 terminate_line = _cur_seg + 7;

  //note: x and y are swapped to get better organization of the line

  int i;
  //for(i=20;i<40;i++)vectoscopeHline(30,i,20,0);
  //return;
  minx = y1;
  if (y2 < minx)minx = y2;
  if (y3 < minx)minx = y3;
  maxx = y1;
  if (y2 > maxx)maxx = y2;
  if (y3 > maxx)maxx = y3;


  s16 signx1, signx2, dx1, dy1, dx2, dy2;
  s16 e1, e2;
  // Sort vertices
  if (y1 > y2)
  {
    swapu8(y1, y2);
    swapu8(x1, x2);
  }
  if (y1 > y3)
  {
    swapu8(y1, y3);
    swapu8(x1, x3);
  }
  if (y2 > y3)
  {
    swapu8(y2, y3);
    swapu8(x2, x3);
  }

  t1x = x1;
  t2x = x1;

  y = y1; // Starting points

  dx1 = (s8)(x2 - x1);
  if (dx1 < 0)
  {
    dx1 = -dx1;
    signx1 = -1;
  }
  else
    signx1 = 1;
  dy1 = (s8)(y2 - y1);

  dx2 = (s8)(x3 - x1);
  if (dx2 < 0)
  {
    dx2 = -dx2;
    signx2 = -1;
  }
  else signx2 = 1;

  dy2 = (s8)(y3 - y1);

  if (dy1 > dx1)
  { // swap values
    swapu8(dx1, dy1);
    change |= 1; // = true;
  }
  if (dy2 > dx2)
  { // swap values
    swapu8(dy2, dx2);
    change |= 2; // = true;
  }

  e2 = (u8)(dx2 >> 1);
  // Flat top, just process the second half
  if (y1 == y2) goto next;

  e1 = (u8)(dx1 >> 1);

  for (u8 i = 0; i < dx1;)
  {
    t1xp = 0; t2xp = 0;
    if (t1x < t2x)
    {
      minx = t1x;
      maxx = t2x;
    }
    else
    {
      minx = t2x;
      maxx = t1x;
    }
    // process first line until y value is about to change
    while (i < dx1)
    {
      i++;
      e1 += dy1;
      while (e1 >= dx1)
      {
        e1 -= dx1;
        if (change & 1)
          t1xp = signx1;
        else
          goto next1;
      }
      if (change & 1) break;

      t1x += signx1;
    }
    // Move line
next1:
    // process second line until y value is about to change
    while (1)

    {
      e2 += dy2;
      while (e2 >= dx2) {
        e2 -= dx2;
        if (change & 2)
          t2xp = signx2;
        else
          goto next2;
      }
      if (change & 2)
        break;
      t2x += signx2;
    }
next2:

    if (minx > t1x) minx = t1x;
    if (minx > t2x) minx = t2x;
    if (maxx < t1x) maxx = t1x;
    if (maxx < t2x) maxx = t2x;
    //*/
    vectoscopeHline(minx, y, maxx - minx, c);
    //   x_vline(minx, y, maxx, c, linebuffer);  // Draw line from min to max points found on the y
    //   if (y == terminate_line)return;
    // Now increase y
    if (!(change & 1)) t1x += signx1;
    t1x += t1xp;
    if (!(change & 2)) t2x += signx2;
    t2x += t2xp;
    y += 1;
    if (y == y2) break;

  }
next:
  // Second half
  dx1 = (s8)(x3 - x2);
  if (dx1 < 0)
  {
    dx1 = -dx1;
    signx1 = -1;
  }
  else
    signx1 = 1;
  dy1 = (s8)(y3 - y2);
  t1x = x2;

  if (dy1 > dx1)
  { // swap values
    swapu8(dy1, dx1);
    change |= 1; // = true;
  }
  else
    change &= 2; //false;

  e1 = (u8)(dx1 >> 1);

  for (u8 i = 0; i <= dx1; i++)
  {
    t1xp = 0; t2xp = 0;
    if (t1x < t2x)
    {
      minx = t1x;
      maxx = t2x;
    }
    else
    {
      minx = t2x;
      maxx = t1x;
    }
    // process first line until y value is about to change
    while (i < dx1) {
      e1 += dy1;
      while (e1 >= dx1)
      {
        e1 -= dx1;
        if (change & 1)
        {
          t1xp = signx1;
          break;
        }
        else
          goto next3;
      }
      if (change & 1)
        break;

      t1x += signx1;
      if (i < dx1) i++;
    }
next3:
    // process second line until y value is about to change
    while (t2x != x3)
    {
      e2 += dy2;
      while (e2 >= dx2)
      {
        e2 -= dx2;
        if (change & 2)
          t2xp = signx2;
        else
          goto next4;
      }
      if (change & 2)
        break;

      t2x += signx2;
    }
next4:

    if (minx > t1x) minx = t1x;
    if (minx > t2x) minx = t2x;
    if (maxx < t1x) maxx = t1x;
    if (maxx < t2x) maxx = t2x;
    //  */
    vectoscopeHline(minx, y, maxx - minx, c);

    //    x_vline(minx, y, maxx, c, linebuffer);  // Draw line from min to max points found on the y
    //    if (y == terminate_line)return;
    // Now increase y
    if (!(change & 1)) t1x += signx1;
    t1x += t1xp;
    if (!(change & 2)) t2x += signx2;
    t2x += t2xp;
    y += 1;
    if (y > y3) return;
  }
}



void xxxGfxApiVectorScopeLine2(int x1, int y1, int x2, int y2) {
  int dx = abs(x2 - x1);
  int dy = abs(y2 - y1);
  int sx = (x1 < x2) ? 1 : -1;
  int sy = (y1 < y2) ? 1 : -1;
  int err = dx - dy;

  os_i2c_start();
  os_i2c_write_byte(SSD1306_ADDRESS);
  os_i2c_write_byte(0x0);  // command mode
  i2c_counter += 2;
  while (1) {
    if ((x1 >= 0) && (x1 < 64) && (y1 >= 0) && (y1 < 64))
    {
      static int last_x, last_y;
      static u8 cur_row = 0;
      if (y1 != last_y)
      {
        // 0x24,direction,row_start,clear_pixels,row_end,x_start,x_end,
        const u8 cmd[] = {
          0x24, 0, 0, 0/*clear*/, 0, 0, 127, 0xe3, 0xe3, //0xe3,0xe3,0xe3,0xe3,
          //0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,

          0x24, 0, 0, 1/*set*/, 0, y1, y1 + 1, 0xe3, //0xe3,0xe3,0xe3,0xe3,0xe3,

          //0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,0xe3,
        };
        for (u8 i = 0; i < sizeof(cmd); i++)
          os_i2c_write_byte_fast(cmd[i]);//+(y1&63));
        i2c_counter++;
      }
      if (x1 != last_x)
      {
        os_i2c_write_byte_fast(0xd3);
        os_i2c_write_byte_fast(x1 & 63);
        i2c_counter += 2;
      }

      last_x = x1;
      last_y = y1;
    }
    if (x1 == x2 && y1 == y2) {
      break;
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x1 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y1 += sy;
    }
  }
  os_i2c_stop();
}



static inline void  __attribute__((always_inline)) GfxApiLayerGetNextByte (GfxApiCompressedLayer *l)
{

  if (l->SkipCounter)
  {
    l->SkipCounter--;
    return;// l->PixelValue;
  }
  u8 r = pgm_read_byte(l->BytePos);
  l->BytePos++;
  u8 color = (r >> 5);
  static const u8 brightness_transform[] =
  {
#define __MUL (0.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (1.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (2.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (3.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (4.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (5.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (6.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL (7.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),
#undef __MUL

    // subtraktion
#define __MUL -(0.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(1.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(2.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(3.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(4.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(5.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(6.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),

#define __MUL -(7.0/7.0)
    __MUL*((0 << 3) | 0),
    __MUL*((1 << 3) | 1),
    __MUL*((2 << 3) | 2),
    __MUL*((3 << 3) | 3),
    __MUL*((4 << 3) | 4),
    __MUL*((5 << 3) | 5),
    __MUL*((6 << 3) | 6),
    __MUL*((7 << 3) | 7),
#undef __MUL

  };

  l->PixelValue = brightness_transform[(l->Brightness & (7 << 3)) | color];
  u8 rept = r & 31;
  l->SkipCounter = rept;
  if (rept != 31)return;// l->PixelValue;
  for (;;)
  {
    u8 add = pgm_read_byte(l->BytePos);
    l->BytePos++;
    l->SkipCounter += add;
    if (add != 255)return;// color;
  }
}


#ifdef ENABLE_LINEDRAWING

static unsigned char _need_clear = 1;
// Bresenham
static inline void _line(u8 *_x0, u8 *_y0, u8 x1, u8 y1, unsigned char *linebuffer) {
  if (x1 < _cur_seg) return ; // rechts ist kleiner als das akute segment....raus...die linie
  u8 x0 = *_x0;
  const u8 curseg_plus_7 = _cur_seg + 7;
  if (x0 > curseg_plus_7) return ; // links grösser als cur seg... raus...

  _need_clear = 1;

  // kann das segment nicht schneiden
  const s8 dx =  x1 - x0; // x1 is allways >= x0, no abs needed
  u8 y0 = *_y0;
  s8 dy = y0 - y1;
  s8 sy;

  if (y0 < y1)
  {
    sy = 1;
  }
  else
  {
    dy = -dy;
    sy = -1;
  }

  s8 err = dx + dy;
  s8 e2;   // s8 ist hier potentiell ein bug wegen überlauf......

  const u8 dasboot = 96;
  y0 -= dasboot;
  y1 -= dasboot;

  for (;;)
  {
    if (((x0 & 0xf8) == _cur_seg) && ((y0 & ((u8)~63)) == 0)) //return;//y-clipping
    {
      const u8 linep = (u8)y0 / (u8)8 + (x0 & (u8)7) * (u8)8;
      linebuffer[linep] |= pgm_read_byte(&_bit_set[y0 & 7]); //bit;
    }

    if ((x0 == x1) && (y0 == y1))
    {
      return;
    }
    -    e2 = 2 * err;
    if (e2 <= dx)
    {
      err += dx;
      y0 += sy;
    }
    if (e2 >= dy)
    {
      err += dy;
      x0++;

      if (x0 > curseg_plus_7)
      {
        *_x0 = x0;
        *_y0 = y0 + 96;

        return;
      }
    }
  }

}

static void _vline(int y0, int x0, int y1, u8 c, u8 *linebuffer) {
  if ((x0 & 0xf8) != _cur_seg)return;
  y0 -= 96;
  if (y0 < 0)y0 = 0;
  if (y0 > 63)y0 = 63;

  y1 -= 96;
  if (y1 < 0)y1 = 0;
  if (y1 > 63)y1 = 63;

  if (y0 > y1)swapu8(y0, y1);
  _need_clear = 1;
  x0 &= 7;
  u8 xx0 = x0;
  x0 *= 8;
  // optimize!
  u8 bit = 1 << (y0 & 7);

  u8 y = y0;
  const u8 ye = y1;
  u8 linep = y0 / 8;
  linep += x0;

  u16 addr = (u16)&_ordered_dither_matrix_simulated_annealing[xx0 * 8];

#ifndef ENABLE_VECTOR_ERRORDIFFUSION
  linebuffer += linep;

  u8 cache = *linebuffer;

  for (;;)
  {
    cache &= ~bit;
    if (pgm_read_byte(addr + (y & 7)) <= c)
      cache |= bit;
    bit <<= 1;
    if (!bit)
    {
      *linebuffer = cache;
      bit = 1, linebuffer++;
      cache = *linebuffer;
    }
    y++;
    if (y > ye)
    {
      *linebuffer = cache;
      return;
    }
  }
#else

  s8 error;
  s8 *e_left = &linebuffer[64 + y];
  linebuffer += linep;
  u8 cache = *linebuffer;

  cache &= ~bit;
  if (pgm_read_byte(addr + (y & 7)) <= c)
  {
    error = c - 63;
    cache |= bit;
  } else error = c;
  error >>= 1;
  *e_left = error;

  y++;
  if (y > ye)
  {
    *linebuffer = cache;
    return;
  }
  bit <<= 1;
  if (!bit) {
    *linebuffer = cache;
    bit = 1, linebuffer++;
    cache = *linebuffer;
  }

  e_left++;

  for (;;)
  {
    //*linebuffer&=~bit;
    cache &= ~bit;


    error += c + *e_left;
    if (error > 31)
    {
      error -= 63;
      //*linebuffer|=bit;
      cache |= bit;
    }
    error >>= 1;
    *e_left = error;

    y++;
    if (y > ye)
    {
      *linebuffer = cache;
      return;
    }

    bit <<= 1;
    if (!bit) {
      *linebuffer = cache;
      bit = 1, linebuffer++;
      cache = *linebuffer;
    }
    e_left++;
  } //*/
#endif
}


#ifdef ENABLE_CIRCLES
static inline void drawCircle_y(u8 yc, u8 xc, u8 y, u8 x, u8 *linebuffer, u8 pat)
{

  //  _vline(xc-x,yc+y,xc+x,pat,linebuffer);
  //    _vline(xc-x,yc-y,xc+x,pat,linebuffer);
  _vline(xc + y, yc + x, xc - y, pat, linebuffer);
  _vline(xc + y, yc - x, xc - y, pat, linebuffer);

}
static inline void drawCircle_x(u8 yc, u8 xc, u8 y, u8 x, u8 *linebuffer, u8 pat)
{

  _vline(xc - x, yc + y, xc + x, pat, linebuffer);
  _vline(xc - x, yc - y, xc + x, pat, linebuffer);
  //  _vline(xc+y,yc+x,xc-y,pat,linebuffer);
  //  _vline(xc+y,yc-x,xc-y,pat,linebuffer);

}

static inline void fillCircle(u8 xc, u8 yc, u8 r, u8 pat, u8*linebuffer)
{
  if (((xc - r) & 0xf8) > _cur_seg + 7) return ; // links grösser als cur seg... raus...
  if (((xc + r) & 0xf8) < _cur_seg) return ; // rechts ist kleiner als das akute segment....raus...die linie
  // kann das segment nicht schneiden
  _need_clear = 1;
  s8 x = 0, y = r;
  s8  d = 3 - 2 * r;
  drawCircle_x(xc, yc, x, y, linebuffer, pat);
  drawCircle_y(xc, yc, x, y, linebuffer, pat);

  while (y >= x)
  {
    x++;
    drawCircle_x(xc, yc, x, y, linebuffer, pat);

    if (d > 0)
    {
      y--;
      // hier die y abhängigen vlines zeichnen, nicht mit draw circle...

      d = d + 4 * (x - y) + 10;
    }
    else
      d = d + 4 * x + 6;
    drawCircle_y(xc, yc, x, y, linebuffer, pat);
    // drawCircle2(xc, yc, x, y,linebuffer,pat);
  }
}
#endif

#ifdef ENABLE_TRIANGLES
// triangle code was once based on https://www.avrfreaks.net/sites/default/files/triangles.c , https://www.avrfreaks.net/forum/algorithm-draw-filled-triangle

static void fillTriangle(u8 x1, u8 y1, u8 x2, u8 y2, u8 x3, u8 y3, u8 c, u8*linebuffer) {
  u8 t1x, t2x, y, minx, maxx, t1xp, t2xp;
  u8 change = 0;
  const u8 terminate_line = _cur_seg + 7;

  //note: x and y are swapped to get better organization of the line


  minx = y1;
  if (y2 < minx)minx = y2;
  if (y3 < minx)minx = y3;
  maxx = y1;
  if (y2 > maxx)maxx = y2;
  if (y3 > maxx)maxx = y3;
  if ((minx & 0xf8) > _cur_seg + 7) return ; // links grösser als cur seg... raus...
  if ((maxx & 0xf8) < _cur_seg) return ; // rechts ist kleiner als das akute segment....raus...die linie


  s8 signx1, signx2, dx1, dy1, dx2, dy2;
  u8 e1, e2;
  // Sort vertices
  if (y1 > y2)
  {
    swapu8(y1, y2);
    swapu8(x1, x2);
  }
  if (y1 > y3)
  {
    swapu8(y1, y3);
    swapu8(x1, x3);
  }
  if (y2 > y3)
  {
    swapu8(y2, y3);
    swapu8(x2, x3);
  }

  t1x = x1;
  t2x = x1;

  y = y1; // Starting points

  dx1 = (s8)(x2 - x1);
  if (dx1 < 0)
  {
    dx1 = -dx1;
    signx1 = -1;
  }
  else
    signx1 = 1;
  dy1 = (s8)(y2 - y1);

  dx2 = (s8)(x3 - x1);
  if (dx2 < 0)
  {
    dx2 = -dx2;
    signx2 = -1;
  }
  else signx2 = 1;

  dy2 = (s8)(y3 - y1);

  if (dy1 > dx1)
  { // swap values
    swapu8(dx1, dy1);
    change |= 1; // = true;
  }
  if (dy2 > dx2)
  { // swap values
    swapu8(dy2, dx2);
    change |= 2; // = true;
  }

  e2 = (u8)(dx2 >> 1);
  // Flat top, just process the second half
  if (y1 == y2) goto next;

  e1 = (u8)(dx1 >> 1);

  for (u8 i = 0; i < dx1;)
  {
    t1xp = 0; t2xp = 0;
    if (t1x < t2x)
    {
      minx = t1x;
      maxx = t2x;
    }
    else
    {
      minx = t2x;
      maxx = t1x;
    }
    // process first line until y value is about to change
    while (i < dx1)
    {
      i++;
      e1 += dy1;
      while (e1 >= dx1)
      {
        e1 -= dx1;
        if (change & 1)
          t1xp = signx1;
        else
          goto next1;
      }
      if (change & 1) break;

      t1x += signx1;
    }
    // Move line
next1:
    // process second line until y value is about to change
    while (1)
    {
      e2 += dy2;
      while (e2 >= dx2) {
        e2 -= dx2;
        if (change & 2)
          t2xp = signx2;
        else
          goto next2;
      }
      if (change & 2)
        break;
      t2x += signx2;
    }
next2:

    if (minx > t1x) minx = t1x;
    if (minx > t2x) minx = t2x;
    if (maxx < t1x) maxx = t1x;
    if (maxx < t2x) maxx = t2x;
    //*/
    _vline(minx, y, maxx, c, linebuffer);  // Draw line from min to max points found on the y
    if (y == terminate_line)return;
    // Now increase y
    if (!(change & 1)) t1x += signx1;
    t1x += t1xp;
    if (!(change & 2)) t2x += signx2;
    t2x += t2xp;
    y += 1;
    if (y == y2) break;

  }
next:
  // Second half
  dx1 = (s8)(x3 - x2);
  if (dx1 < 0)
  {
    dx1 = -dx1;
    signx1 = -1;
  }
  else
    signx1 = 1;
  dy1 = (s8)(y3 - y2);
  t1x = x2;

  if (dy1 > dx1)
  { // swap values
    swapu8(dy1, dx1);
    change |= 1; // = true;
  }
  else
    change &= 2; //false;

  e1 = (u8)(dx1 >> 1);

  for (u8 i = 0; i <= dx1; i++)
  {
    t1xp = 0; t2xp = 0;
    if (t1x < t2x)
    {
      minx = t1x;
      maxx = t2x;
    }
    else
    {
      minx = t2x;
      maxx = t1x;
    }
    // process first line until y value is about to change
    while (i < dx1) {
      e1 += dy1;
      while (e1 >= dx1)
      {
        e1 -= dx1;
        if (change & 1)
        {
          t1xp = signx1;
          break;
        }
        else
          goto next3;
      }
      if (change & 1)
        break;

      t1x += signx1;
      if (i < dx1) i++;
    }
next3:
    // process second line until y value is about to change
    while (t2x != x3)
    {
      e2 += dy2;
      while (e2 >= dx2)
      {
        e2 -= dx2;
        if (change & 2)
          t2xp = signx2;
        else
          goto next4;
      }
      if (change & 2)
        break;

      t2x += signx2;
    }
next4:

    if (minx > t1x) minx = t1x;
    if (minx > t2x) minx = t2x;
    if (maxx < t1x) maxx = t1x;
    if (maxx < t2x) maxx = t2x;
    //  */
    _vline(minx, y, maxx, c, linebuffer);  // Draw line from min to max points found on the y
    if (y == terminate_line)return;
    // Now increase y
    if (!(change & 1)) t1x += signx1;
    t1x += t1xp;
    if (!(change & 2)) t2x += signx2;
    t2x += t2xp;
    y += 1;
    if (y > y3) return;
  }
}
#endif
#endif
#endif


static void DISPLAYFUNC (
  GfxApiCompressedLayer * layers,
  GfxApiSprite * Sprites,
  u8 * screen,
  u8 line_enable_flags
#ifdef ENABLE_CONSOLE0_FONT_SWITCHING
  , int font_adress_console_0_in
#endif
#ifdef ENABLE_SECOND_CONSOLE
  , u8 * screen2,
  s16 zoomX,
  s16 zoomY,
  s16 scrollX,
  s16 scrollY
#endif
#ifdef ENABLE_SECOND_CONSOLE_AFFINE
  , s16 zoomX2,
  s16 zoomY2
#endif
#ifdef ENABLE_BARGRAPH
  , GfxApiBargraph *bargraph
#endif

#ifdef ENABLE_VIDEO_VQ
  , const u8  * const vq_array
  , const u8 * const vq_tiles
#endif
)
{


  u8 pattern = 0;
#ifdef CONSOLE_ENABLE_CHESSBOARD_DITHER
  +0x55
#endif
  ;

  u8 pattern_55 = 0x55;
  if (_engine_flags_and_frame_counter & 1)pattern = ~pattern, pattern_55 = ~pattern_55;

#ifdef ENABLE_ATTINY_POWER_MANAGER
  if (low_power_screen_disable)
  {
    _manage_battery();
    return;
  }
#endif
  static  u8 _i2c_data_out;
  u8 i2c_data_out = _i2c_data_out;
  //u8 must_float_pin=i2c_data_out&0x80;
#ifdef ENABLE_LINEDRAWING
  u8 linebuffer[64
#ifdef ENABLE_VECTOR_ERRORDIFFUSION
                + 64
#endif
               ];
  u8 linebuffer_pos = 0;
  memset(linebuffer, 0, sizeof(linebuffer));
  reorder_lines();
#endif
#ifdef ENABLE_LAYERS
  u16 layers_constant_counter = 0;
#endif
#ifdef ENABLE_SPRITES
  u16 sprites_constant_counter = 0;
#endif
  u8 x_pos_screen;
  os_gfx_start_display_transfer();

#ifndef DISABLE_HALFTONE
#ifndef ENABLE_LOW_QUALITY_HALFTONE
#ifndef ENABLE_ORDERED_DITHERING
  s8 error_right[64];
  memset(error_right, 0, sizeof(error_right));
#endif
#endif
  s8 layersum = 0;
#endif

  u8 jitter = _engine_flags_and_frame_counter & 0x3;

#ifdef ENABLE_SECOND_CONSOLE
  s16 second_console_x = scrollX;
#endif
#ifdef ENABLE_SECOND_CONSOLE_AFFINE
  s16 second_console_y0 = scrollY;
#endif

  for (x_pos_screen = 0; x_pos_screen < 128; x_pos_screen++
#ifdef ENABLE_SECOND_CONSOLE
       , second_console_x += zoomX
#endif
#ifdef ENABLE_SECOND_CONSOLE_AFFINE
                             , second_console_y0 += zoomY2
#endif
      )
  {


#ifdef ENABLE_LINEDRAWING


    if ((x_pos_screen & 7) == 0)
    { // clear linebuffer for 8x64 block and fill with lines
      linebuffer_pos = 0;
      //       os_i2c_stop();
      //u8 rd;
      //  eeprom_i2c_read(0x50 << 1, 0, &rd, 1);
      //   os_i2c_restart_display_transfer();

      if (_need_clear)
      {
        _need_clear = 0;
        memset(linebuffer, 0, 64);
        //   _clear_start=64,_clear_end=0;
      }
      _cur_seg = (x_pos_screen & 0xf8) + 64;

      for (u8 i = 0; i < _gfx_linepos; i += 4)
        _line(&_gfx_points_of_lines[i], &_gfx_points_of_lines[i + 1], _gfx_points_of_lines[i + 2], _gfx_points_of_lines[i + 3], linebuffer);
#ifdef ENABLE_TRIANGLES
      for (u8 i = _gfx_linepos; i < _gfx_linepos + _gfx_tripos; i += 8)
      {
        fillTriangle(_gfx_points_of_lines[i], _gfx_points_of_lines[i + 1], _gfx_points_of_lines[i + 2], _gfx_points_of_lines[i + 3], _gfx_points_of_lines[i + 4], _gfx_points_of_lines[i + 5], _gfx_points_of_lines[i + 6], linebuffer);
      }
#endif
#ifdef ENABLE_CIRCLES

#ifndef ENABLE_TRIANGLES
#define _gfx_tripos 0
#endif
      for (u8 i = _gfx_linepos + _gfx_tripos; i < _gfx_linepos + _gfx_circlepos + _gfx_tripos; i += 4)
      {
        fillCircle(_gfx_points_of_lines[i], _gfx_points_of_lines[i + 1], _gfx_points_of_lines[i + 2], _gfx_points_of_lines[i + 3], linebuffer);
      }
#ifndef ENABLE_TRIANGLES
#undef _gfx_tripos
#endif
#endif
    }
#endif
#if defined(ENABLE_CONSOLE) || defined (CONSOLE_ENABLE_CURSOR) || defined (ENABLE_SEGMENTED)
    u8 charpos = x_pos_screen >> 3;
#endif

#ifdef ENABLE_VIDEO_VQ
    u8 charpos_vq = x_pos_screen >> 3;
#endif


#ifdef ENABLE_SECOND_CONSOLE
#ifndef ENABLE_SECOND_CONSOLE_AFFINE
    u8 charpos_second_con = (second_console_x >> 11);
    //  if(charpos_second_con>SECOND_CONSOLE_LINE_LENGTH)charpos_second_con=SECOND_CONSOLE_LINE_LENGTH-1;
    s16 second_console_y = scrollY;
#else
    s16 second_console_y = second_console_y0;
    s16 second_console_x_affine = second_console_x;
#endif
#endif
    u8 or_bit = 1;
#ifndef ENABLE_ORDERED_DITHERING
    s8 propagte_error = 0;
#endif
    u8 block_8_px = 0;
    u8 y_pos_screen;
#ifdef ENABLE_ORDERED_DITHERING
    u8 ordered_dither_y = 0;
    u8 ordered_dither_x = (x_pos_screen & 7) << 3;
#endif

    pattern = ~pattern;
    const u8 subpos_x = x_pos_screen & 7;



      //#if defined (ENABLE_SEGMENTED) || defined (ENABLE_BARGRAPH)
      u8 row = 0;
    //#endif
    u8 fwd[8];
    memset(fwd, 0, sizeof(fwd));


  Serial.println("a3");

    for (y_pos_screen = 0; y_pos_screen < 64; y_pos_screen++)
    {
      __NOP__;
      __NOP__;

      if (i2c_data_out & 0x80)
      {
        os_i2c_sda_high_ppull();
#ifdef __AVR__
        if (or_bit == 128)
        {
          os_i2c_sda_high();
          os_i2c_sda_low_ppull();

        }
#endif
      }
      else
      {
        os_i2c_sda_low_ppull();

      }
      i2c_data_out <<= 1;

      __NOP__;
      __NOP__;
      os_i2c_scl_high_ppull();
      __NOP__;
      __NOP__;

#ifndef DISABLE_HALFTONE
#ifndef PIXEL_CALLBACK
      s8 background_pixel = 0;
#else
      s8 background_pixel = 0;
      //if(y_pos_screen<32)      // for 3 color mode.
      background_pixel = PIXEL_CALLBACK (x_pos_screen, y_pos_screen);
#endif
      //layersum=background_pixel;
#endif
#ifdef ENABLE_LAYERS
      if (!layers_constant_counter)
      {
        u8 i;
        for (i = 0; i < NR_LAYERS; i++)
        {
          GfxApiLayerGetNextByte(&layers[i]);
          //  Serial.println(tmp);
          //  layers[i].PixelValue = tmp;
        }
        //                if(!layers_constant_counter)
        {
          u16 min = layers[0].SkipCounter;
          u8 i;
          for (i = 1; i < NR_LAYERS; i++)
          {
            if (layers[i].SkipCounter < min)min = layers[i].SkipCounter;
          }
          if (min)
          {
            layers_constant_counter = min;
            for (i = 0; i < NR_LAYERS; i++)
              layers[i].SkipCounter -= layers_constant_counter;
          }
        }
        layersum = 0;
        for (u8 i = 0; i < NR_LAYERS; i++)
        {
          if (layers[i].Brightness & 2)
          {
            if (layers[i].PixelValue)
              layersum = layers[i].PixelValue;
          }
          else
          {
            if (layers[i].Brightness & 1)
            {
              layersum += layers[i].PixelValue;
            }
            else
            {
              layersum -= layers[i].PixelValue;
            }
          }
        }
      }
      else layers_constant_counter--;
      // change to implement different combination logic for the layers
#endif

#ifndef DISABLE_HALFTONE
#ifdef ENABLE_LAYERS

      background_pixel += layersum;
#endif
      if (background_pixel > 63)background_pixel = 63;
      if (background_pixel < 0)background_pixel = 0;


      //    if(layersum<0)layersum=0;
      //    if(layersum>63)layersum=63;
#ifndef ENABLE_ORDERED_DITHERING
#ifndef ENABLE_LOW_QUALITY_HALFTONE
      propagte_error += background_pixel + error_right[y_pos_screen];
#else
      propagte_error += background_pixel;
#endif
      if (propagte_error > 31)block_8_px |= or_bit, propagte_error -= 63;

      /*
         200/4=50
         255/4=63
         50/3=16
         50-16=34
         nach 54

      */
      /*
          //  if (propagte_error > 21*2)
            if (propagte_error > 53)
            {
              block_8_px |= or_bit;
              fwd[row^4] |= or_bit;
              propagte_error-=63;
            }
            else
              if (propagte_error > 34)
              {
                 block_8_px |= or_bit;
                 propagte_error-=50;
              }//*/
#ifndef ENABLE_LOW_QUALITY_HALFTONE
      propagte_error /= 2;
      error_right[y_pos_screen] = propagte_error;
#endif
#endif
#endif

#ifdef ENABLE_ORDERED_DITHERING
      if (pgm_read_byte(&_ordered_dither_matrix_simulated_annealing[ordered_dither_y + ordered_dither_x]) > layersum)block_8_px |= or_bit;
      ordered_dither_y++;
#endif
      /*
        #ifdef ENABLE_SPRITES
        if(!sprites_constant_counter)
        {
        u8 i;
        for(i=0;i<NR_SPRITES;i++)
        if(GfxApiReadSprite(&Sprites[i]))block_8_px|=or_bit;
        }else sprites_constant_counter--;
        #endif
      */
#ifdef ENABLE_SECOND_CONSOLE
#ifndef ENABLE_SECOND_CONSOLE_AFFINE
      if ((y_pos_screen >= SECOND_CONSOLE_LINE_START) && ((y_pos_screen) < SECOND_CONSOLE_LINE_END))
      {
        u8 the_char = (screen2[charpos_second_con + SECOND_CONSOLE_LINE_LENGTH * (((second_console_y >> 8)) >> 3)]);

        const u8 font_block8 = pgm_read_byte(&os_font[((int)the_char << 3) + ( (second_console_x >> 8) & 7)]);
        if (font_block8 & (1 << ((second_console_y >> 8) & 7))) //todo pgm read
          block_8_px |= or_bit;
        //        if(y_pos_screen&1)
        //        block_8_px|=or_bit;

        second_console_y += zoomY;
      }
#else
      if ((y_pos_screen >= SECOND_CONSOLE_LINE_START) && ((y_pos_screen) < SECOND_CONSOLE_LINE_END))
      {
        u8 charpos_second_con = (second_console_x_affine >> 11);
        if (charpos_second_con > SECOND_CONSOLE_LINE_LENGTH)charpos_second_con = SECOND_CONSOLE_LINE_LENGTH - 1;
        u8 the_char = (screen2[charpos_second_con + (SECOND_CONSOLE_LINE_LENGTH) * (((second_console_y >> 8)) >> 3)]);
        const u8 font_block8 = pgm_read_byte(&os_font[((int)the_char << 3) + ( (second_console_x_affine >> 8) & 7)]);
        if (font_block8 & (1 << ((second_console_y >> 8) & 7)))
          block_8_px |= or_bit;
        second_console_y += zoomY;
        second_console_x_affine += zoomX2;
      }
#endif
#endif
      /*
        #ifdef ENABLE_LINEDRAWING
            if(linebuffer[y_pos_screen]&line_cur_bit)block_8_px|=or_bit;;
        #endif
      */
      or_bit <<= 1;
      if (or_bit == 0)
      {
#ifdef ENABLE_LINEDRAWING
        block_8_px |= linebuffer[linebuffer_pos];
        linebuffer_pos++;
#endif

        os_i2c_scl_low_ppull();

        os_i2c_sda_high();
        //os_i2c_sda_low_ppull();
        asm volatile ("nop");   // :-(

#ifdef ENABLE_ORDERED_DITHERING
        ordered_dither_y = 0;
#endif
        or_bit = 1;
        os_i2c_scl_high_ppull();

// todo testen
#ifdef ENABLE_VIDEO_VQ
        if (vq_array)
        {
          u8 the_char_1 = vq_array[charpos_vq];
          u8 the_char_2 = vq_array[charpos_vq + 16];

          u8 tmp;
          if (y_pos_screen >= 32)
          {
            tmp = pgm_read_byte(&vq_tiles[the_char_1 * 8 + subpos_x]) & 0xf;
            tmp |= pgm_read_byte(&vq_tiles[the_char_2 * 8 + subpos_x]) << 4;
          }
          else
          {
            tmp = pgm_read_byte(&vq_tiles[the_char_1 * 8 + subpos_x]) >> 4;
            tmp |= pgm_read_byte(&vq_tiles[the_char_2 * 8 + subpos_x]) & 0xf0;
          }
          block_8_px |= tmp;
          charpos_vq = (charpos_vq + 32) & 0x7f;
        }
#endif
        ///////xxxxxx

#ifdef ENABLE_CONSOLE
        if (line_enable_flags & pgm_read_byte(_bit_set + row))
        {
          u8 the_char = screen[charpos];

          u8 bright = (the_char & 0x80); //?1:0;
          the_char &= 0x7f;

          u8 bits = pgm_read_byte(&os_font[((int)the_char << 3) + subpos_x]);



#ifdef CONSOLE_ENABLE_CURSOR

          if (charpos == cursor_pos)
#ifdef CONSOLE_ENABLE_CURSOR_BLINK
            //if(_engine_flags_and_frame_counter&1)//CONSOLE_CURSOR_BLINK_MASK)
            if (_engine_flags_and_frame_counter & CONSOLE_CURSOR_BLINK_MASK)
#endif
              bits = ~bits;
#endif
          if (bright)fwd[row ^ 4] = bits;
          block_8_px |= bits;
          charpos += 16;

        }
#endif


        block_8_px |= fwd[row]; // or in second plane

        // todo sprites umbauen....
#ifdef ENABLE_SPRITES
        static char vor;
        block_8_px |= vor;
        vor = 0;
        if (sprites_constant_counter < 8)
        {
          u8 i;
          for (i = 0; i < NR_SPRITES; i++)
          {
            if (Sprites[i].SkipCounter < 8)
            {
              u16 tmp = pgm_read_byte(Sprites[i].readpos_byte++);
              tmp <<= Sprites[i].SkipCounter;
              u8 pat = Sprites[i].Color ? pattern : 255;
              block_8_px |= tmp & pat;
              vor |= (tmp >> 8)&pat;
              Sprites[i].SkipCounter += 64 - 8;
              Sprites[i].sprite_height--;
              if (!Sprites[i].sprite_height)
                Sprites[i].SkipCounter = 0x1fff;
            } else Sprites[i].SkipCounter -= 8;
          }
          u16 min = Sprites[0].SkipCounter;
          for (i = 1; i < NR_SPRITES; i++)
          {
            if (Sprites[i].SkipCounter < min)min = Sprites[i].SkipCounter;
          }
          if (min)
          {
            sprites_constant_counter = min - (min & 7);
            for (i = 0; i < NR_SPRITES; i++)
              Sprites[i].SkipCounter -= sprites_constant_counter;
          }


        }
        else sprites_constant_counter -= 8;
#endif

        row++;

        os_i2c_scl_low_ppull();
        os_i2c_sda_low();
        i2c_data_out = block_8_px;
        block_8_px = 0;
        _ssd1306_drive_counter--;
#ifndef DISABLE_DRIVER
#ifdef SCREEN_MODE_DRIVER
        if (vsync_flag)
          //        if( !_ssd1306_drive_counter)
        {
          vsync_flag = 0;
          os_i2c_stop();
          SCREEN_MODE_DRIVER();
          _ssd1306_drive_counter = _ssd1306_drive_speed;
          //  drive_gray();
        }
#endif
#endif
      }  // ende 8 pixel block
      os_i2c_scl_low_ppull();

    }   // ende y schleife
  }   // ende x schleife
  os_i2c_write_byte(i2c_data_out);
  _i2c_data_out = i2c_data_out;
  os_i2c_stop();
#ifdef ENABLE_ATTINY_POWER_MANAGER
  _manage_battery();
#endif
  u8 tmp = _engine_flags_and_frame_counter + 1;
  tmp &= 0x7f;
  _engine_flags_and_frame_counter &= 0x80;
  _engine_flags_and_frame_counter |= tmp;
}
#ifdef DISABLE_HALFTONE
#undef DISABLE_HALFTONE
#endif
#undef STRINGNIZE
#undef STRINGNIZE_NX

#pragma GCC pop_options
#endif
