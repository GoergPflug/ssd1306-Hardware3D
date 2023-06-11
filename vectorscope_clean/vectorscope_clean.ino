#pragma GCC optimize("O2,unsafe-math-optimizations,no-math-errno,associative-math,reciprocal-math,fast-math")   // "fast-math" helps auto-vectorize loops

#define ENABLE_ARDUINO_SUPPORT
#ifndef __AVR__
#include "soc/rtc_wdt.h"
#include "esp_system.h"
#include "esp32-hal.h"
#endif
int i2c_counter;
#ifndef __AVR__

#include "esp_timer.h"
#endif
int fuzz_d2 = 255;
int fra = 0;
int bro = 0;
int tri_size = 0;
#define TIMER_FREQUENCY 12000

//hw_timer_t * timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static unsigned int qqqx123 = 0;
#define CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD 1
#define TDIV (150)
//#define SCREEN_MODE_DRIVER drive_128x64_vsync
//#define __SCREENMODE_MUX 63
#define DISABLE_DRIVER
#define CONSOLE_ENABLE_PRINTF


//#define ENABLE_VIDEO_VQ
static volatile unsigned char vsync_flag = 0;
unsigned char mux_0_countdown = 0;
#include <stdlib.h>
#include <stdfix.h>
#ifdef __AVR__
#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#endif
#define DISABLE_OFAST
///////////////////////////////////////////////////////////////////////////////
//Gfx-Api Configuration
///////////////////////////////////////////////////////////////////////////////
#define DISPLAYFUNC Display

#include "tiny_multi_os.h"

void loop()
{
  Display(0, 0, 0, 0); //console,cat_tiles+chunk*128*8);
  
 
  GfxApiSetDisplayUndocumentedD2(4);
  GfxApiStartVectorScope(1);
  
  float counter=0;
  int error=0;
  for(;;)
  {
    GfxApiVectorscopeDisplay(sin(counter)*27+31,cos(counter*1.01)*27+31);

    int level=(abs(8*2.5*(1+sin(counter*0.1))))+error;

    int olevel=level/8;
    error=level-8*olevel;   //     4 3 
    GfxApiVectoscopeSetOutputVoltage(olevel);
    counter+=0.01;
  }
}

void setup() {
  streamgfx_init_arduino();
  sei();//allow interrupts

  loop();
}
