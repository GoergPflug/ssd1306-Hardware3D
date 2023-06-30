int sintab[256];


int frames=0;
#pragma GCC optimize ("Ofast")

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
//#define ENABLE_SPRITES
unsigned char calc_diagram(unsigned char x, unsigned char y);
#define PIXEL_CALLBACK calc_diagram
//#define ENABLE_LAYERS

#define ENABLE_CONSOLE
#define NR_LAYERS 1
#define NR_SPRITES 0
//#define SUBTRACT_LAYER0
// rrr #define CONSOLE_SIZE_Y 1
//#define CONSOLE_LINE_START (0*8)




#include "tiny_multi_os.h"
//#include <avr/wdt.h>



unsigned char counter = 0;

unsigned char lenna[]=
{

 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,48,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,14,68,25,1,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,7,7,7,7,7,7,7,7,7,3,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,13,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,17,23,23,23,23,
23,23,23,23,23,23,23,23,49,68,61,23,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,2,0,0,0,10,22,37,23,23,23,23,23,23,23,23,23,21,8,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,20,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,36,36,36,36,
36,36,36,36,36,36,36,36,36,85,68,71,61,17,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,6,14,2,0,3,17,44,94,36,36,36,36,36,36,36,36,36,29,21,4,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,38,3,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,41,45,45,
45,45,45,45,45,45,45,45,45,45,45,98,68,79,107,49,9,0,0,0,0,0,0,0,0,0,0,
0,0,0,1,11,57,100,107,37,56,66,66,49,102,62,47,47,47,47,47,47,47,47,47,
47,28,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,1,2,17,68,21,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,
1,1,1,0,0,0,50,87,102,102,102,102,102,102,102,102,102,102,102,102,115,94,
85,142,195,114,51,43,34,30,30,28,25,25,25,25,25,25,25,49,79,91,88,104,107,
112,68,68,73,156,144,144,144,144,144,144,144,140,144,144,183,69,39,28,22,
16,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,19,20,20,0,0,21,25,45,62,78,
104,128,52,15,3,0,0,0,0,0,0,0,0,0,0,1,8,8,8,13,23,39,42,49,54,54,54,54,
54,54,54,54,54,46,13,3,0,38,93,185,213,213,213,213,213,213,213,213,213,
213,213,208,120,98,165,225,198,147,116,98,92,83,80,75,75,75,75,75,75,75,
75,71,75,114,227,231,164,79,118,157,210,213,213,213,213,213,213,213,195,
213,213,191,105,79,75,76,71,51,36,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,26,34,
42,58,58,58,36,24,61,75,83,95,107,131,160,88,73,47,31,37,41,49,52,60,71,
79,90,98,112,130,144,154,161,164,170,174,174,174,174,174,174,174,174,174,
174,174,76,40,24,2,10,38,96,199,242,250,250,250,250,250,250,250,250,250,
248,194,100,206,162,141,106,94,84,73,67,68,65,64,67,67,68,63,67,62,68,92,
199,240,244,120,70,87,182,250,250,249,249,244,241,235,227,215,235,235,217,
103,74,68,69,71,90,154,46,5,0,0,0,0,0,0,0,0,0,0,0,0,1,29,55,52,53,51,50,
51,83,59,64,62,68,61,56,68,103,98,94,71,52,54,69,76,85,93,105,114,143,148,
178,196,210,234,253,255,255,255,255,255,255,255,255,255,255,255,255,255,
63,33,14,0,0,8,33,82,186,242,255,255,255,255,255,255,255,255,255,247,161,
144,138,102,94,82,72,68,63,68,69,57,63,61,68,70,67,68,74,143,122,228,170,
71,73,109,206,244,253,242,242,245,239,239,218,238,239,241,204,106,74,68,
68,68,87,162,146,117,16,0,0,0,0,0,0,0,0,0,0,0,32,53,46,48,42,49,45,49,51,
53,60,64,68,61,47,68,83,73,103,80,63,48,62,73,74,85,93,104,112,135,146,
173,192,207,230,250,255,255,255,255,255,255,255,255,255,255,255,255,51,
10,0,0,0,0,7,33,83,181,245,255,255,255,255,255,255,255,255,251,220,115,
109,94,83,72,66,62,62,62,66,63,72,68,68,69,73,68,67,71,91,216,158,71,100,
112,186,234,248,252,252,249,251,240,220,229,251,232,183,105,74,68,68,68,
77,136,131,156,112,35,11,0,0,0,0,0,0,0,0,19,48,51,43,42,54,43,55,43,54,
53,62,62,68,70,63,68,73,67,93,95,77,61,49,64,74,76,87,93,108,113,142,153,
178,196,206,231,249,255,255,255,255,255,255,255,255,255,255,255,48,0,0,
0,0,0,0,3,34,71,183,241,253,255,255,255,255,255,246,187,134,174,92,84,70,
67,53,52,68,63,67,67,68,92,73,68,68,73,67,69,76,179,132,81,109,112,155,
231,253,255,255,248,255,238,235,239,241,228,205,165,69,68,60,58,68,124,
135,110,158,87,69,11,0,0,0,0,0,0,9,44,48,50,49,43,56,51,50,51,52,51,46,
57,72,68,69,73,67,62,74,102,93,69,51,52,69,74,78,90,96,107,118,140,152,
177,196,202,232,248,255,255,255,255,255,255,255,255,255,255,48,0,0,0,0,
0,0,0,3,27,66,165,236,253,255,255,255,255,174,69,102,172,98,78,66,57,55,
53,60,64,61,61,67,81,111,66,65,68,64,66,64,125,89,68,80,93,128,231,255,
255,255,255,250,235,241,244,232,236,235,191,77,70,74,47,71,123,122,107,
70,65,126,61,0,0,0,0,0,0,27,14,22,44,43,42,48,66,49,49,55,53,38,45,63,68,
68,76,65,62,64,85,103,80,62,48,57,70,74,78,91,95,108,116,142,150,174,193,
202,224,248,255,255,255,255,255,255,255,255,255,48,0,0,0,0,0,0,0,0,4,27,
67,161,235,255,255,255,255,174,64,97,157,110,72,56,51,51,53,53,59,63,72,
63,63,78,65,65,63,71,68,70,78,70,68,78,43,67,226,255,255,255,255,231,240,
239,230,232,236,235,179,76,87,85,83,86,130,113,76,78,57,89,140,16,0,0,0,
0,12,31,19,15,44,47,42,46,51,49,45,51,53,70,51,61,70,68,88,62,62,62,71,
97,95,69,59,46,62,74,74,82,91,99,108,125,142,154,179,192,203,226,246,255,
255,255,255,255,255,255,255,48,0,0,0,0,0,0,0,0,0,2,24,61,149,234,253,255,
255,174,64,97,142,73,61,51,56,51,56,52,58,57,61,66,67,68,67,68,72,63,68,
68,68,68,68,61,50,54,216,253,255,252,245,233,237,236,224,227,223,224,119,
80,109,81,78,87,83,84,68,69,86,60,164,94,1,0,0,0,21,28,19,15,35,45,55,41,
43,58,48,41,51,60,57,60,57,62,71,89,62,62,67,85,103,81,69,51,51,65,74,74,
86,91,98,108,122,141,156,176,193,205,231,244,254,255,255,255,255,255,255,
48,0,0,0,0,0,0,0,0,0,0,0,23,50,145,227,250,247,197,127,168,98,62,55,51,
51,51,48,46,53,55,61,61,63,64,61,61,61,62,61,61,61,61,61,61,61,126,255,
248,247,239,234,255,249,239,229,237,217,195,82,65,83,148,104,102,67,71,
64,65,93,74,91,165,21,0,0,11,9,28,28,15,30,39,43,46,48,51,55,56,56,51,57,
57,57,62,64,97,62,62,62,80,95,98,77,62,48,52,70,74,74,88,91,102,108,123,
142,154,179,195,206,226,243,255,255,255,255,255,255,48,0,0,0,0,0,0,0,0,
0,0,0,2,21,56,136,242,215,204,204,145,121,66,51,51,46,51,48,48,51,53,55,
63,61,51,40,40,47,70,59,58,50,45,38,33,30,167,249,245,240,245,248,242,242,
238,233,237,236,156,71,54,64,116,171,84,68,66,68,59,65,102,72,137,57,0,
0,6,4,28,28,15,10,7,16,53,46,51,51,52,52,51,53,53,53,56,67,68,48,40,40,
44,51,57,49,43,38,35,43,45,45,49,51,53,58,60,70,70,138,181,195,205,227,
243,255,255,255,255,255,48,0,0,0,0,0,0,0,0,0,0,0,0,1,18,47,225,140,62,105,
100,128,65,51,50,55,41,51,48,50,45,54,53,63,56,30,34,44,84,64,66,54,42,
36,24,20,97,228,245,245,243,243,236,236,221,209,204,192,109,37,42,49,64,
68,57,57,58,67,63,65,65,72,84,110,0,0,0,26,17,28,25,6,20,12,45,45,21,7,
23,34,35,35,35,35,35,37,47,45,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
28,28,28,28,28,119,161,177,196,201,228,242,253,255,255,255,48,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,8,206,81,14,84,73,53,31,54,45,49,55,55,49,46,51,66,55,
57,64,40,39,38,61,66,66,59,44,38,27,21,30,114,235,240,242,242,242,233,217,
204,181,186,167,45,34,34,49,60,57,61,76,54,63,70,67,65,68,112,0,0,1,32,
17,28,26,21,28,28,36,34,0,8,12,17,34,34,34,34,34,34,37,72,41,41,41,41,41,
41,38,36,37,34,34,34,30,28,28,28,28,28,28,119,142,158,179,192,202,223,240,
252,255,255,48,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,179,55,67,60,21,0,34,54,54,
49,48,48,68,62,51,53,51,57,64,51,49,40,45,44,57,62,48,41,28,22,20,127,191,
203,203,203,203,195,193,193,166,186,186,87,34,42,41,48,53,53,89,46,65,68,
72,68,68,99,0,0,3,27,30,13,29,25,28,13,28,5,0,0,13,26,34,34,34,34,34,34,
37,73,21,21,21,21,21,21,19,16,18,15,15,15,34,28,28,28,28,28,28,110,133,
142,158,179,198,202,223,238,255,255,44,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,124,
37,85,12,0,0,43,54,50,45,45,50,46,51,45,51,56,57,64,51,51,45,36,43,32,29,
34,34,28,18,13,49,180,118,118,118,118,118,118,118,116,118,118,59,10,4,2,
32,51,56,67,71,61,61,66,68,68,84,0,0,11,27,23,6,17,26,28,27,18,7,0,0,1,
3,3,3,3,3,3,3,3,52,0,0,0,0,0,0,0,0,0,0,0,0,34,28,30,30,30,40,43,106,109,
131,142,155,179,196,204,228,238,252,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,100,
30,0,0,0,0,68,54,44,48,51,53,41,37,46,49,51,57,67,53,51,46,42,51,20,0,0,
0,0,0,0,51,159,43,43,43,43,43,43,43,43,43,43,33,6,0,0,25,55,52,53,65,56,
65,58,68,68,36,0,0,12,25,29,41,22,24,20,47,22,22,4,0,0,0,0,0,0,0,0,0,0,
19,0,0,0,0,0,0,0,0,0,0,0,0,39,35,45,45,45,45,45,92,108,108,136,142,156,
181,195,205,226,241,36,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,81,29,0,0,0,0,83,
54,44,48,55,66,43,34,38,49,51,57,71,53,51,46,42,51,20,0,0,0,0,0,18,68,99,
33,33,33,33,33,33,33,33,33,33,21,1,0,0,25,50,48,48,57,57,69,68,72,59,42,
0,0,12,23,29,40,28,43,9,26,28,28,22,23,30,39,43,46,57,66,73,82,96,96,96,
73,36,0,0,0,0,0,0,0,0,0,45,45,45,49,56,56,56,91,98,108,114,137,144,162,
180,195,204,230,36,0,0,0,0,0,0,0,0,0,0,0,0,0,0,12,73,30,0,0,0,0,87,54,53,
53,50,77,62,60,35,47,45,59,78,57,58,49,42,51,20,0,0,0,0,0,71,67,48,19,19,
19,19,19,19,19,19,19,19,7,0,0,0,29,53,47,50,51,61,66,63,76,56,43,0,0,12,
24,28,28,28,54,14,22,28,38,27,39,42,54,55,51,83,82,81,108,109,110,110,128,
130,126,72,22,4,0,0,0,0,0,63,63,63,64,65,65,65,91,91,97,108,109,139,142,
159,176,196,210,29,0,0,0,0,0,0,0,0,0,0,0,0,0,1,15,68,33,1,0,0,0,82,54,47,
51,50,51,83,96,48,49,51,64,78,67,57,49,44,52,20,0,0,0,0,0,31,106,34,5,5,
5,5,5,5,5,5,5,5,0,0,0,3,35,47,51,51,54,58,64,64,75,65,43,0,0,12,27,28,28,
28,45,40,22,31,37,5,24,48,51,70,53,82,64,61,104,104,108,108,108,101,108,
126,121,73,20,0,0,0,0,74,74,74,74,74,74,74,87,91,91,101,108,119,138,142,
157,183,196,29,0,0,0,0,0,0,0,0,0,0,0,0,0,3,23,91,37,9,0,0,0,67,54,48,51,
48,46,51,61,63,51,51,64,77,67,62,55,41,53,20,0,0,0,0,0,2,107,6,0,0,0,0,
0,0,0,0,0,0,0,0,0,17,47,48,51,51,57,52,60,70,68,63,44,0,0,6,27,32,28,28,
30,52,29,38,52,23,12,42,51,63,68,79,104,68,103,108,86,81,108,89,108,88,
101,125,117,16,0,0,0,66,66,66,66,66,66,66,81,86,91,92,105,108,116,142,142,
166,187,21,0,0,0,0,0,0,0,0,0,0,0,0,0,7,33,168,61,12,0,0,0,39,49,50,50,51,
46,56,52,60,54,45,64,93,62,71,57,41,59,26,0,0,15,45,48,48,52,47,44,44,41,
36,31,26,25,15,9,4,10,29,58,53,40,39,44,53,53,55,62,66,64,63,43,0,0,3,27,
42,33,28,26,31,30,39,72,45,8,43,50,55,62,81,69,71,103,96,73,47,103,107,
108,116,76,108,137,146,41,0,0,45,45,53,58,58,58,58,76,77,88,91,91,105,108,
114,140,142,167,17,0,0,0,0,0,0,0,0,0,0,0,0,0,8,38,210,78,12,0,0,0,33,54,
47,44,51,54,52,65,60,54,49,63,96,69,64,64,51,64,27,0,2,52,114,121,121,121,
121,121,121,116,99,89,60,56,55,55,106,118,116,75,46,39,44,36,54,51,52,55,
59,58,62,74,0,0,0,20,49,39,28,25,17,22,29,66,61,28,38,48,55,61,90,64,60,
94,102,101,61,76,106,91,132,110,108,108,157,134,19,0,40,37,37,40,45,45,
45,68,74,77,91,91,92,106,108,122,142,155,17,0,0,0,0,0,0,0,0,0,0,0,0,0,8,
38,219,77,12,0,0,0,29,49,30,53,50,51,64,96,58,49,50,63,106,64,72,84,73,
62,35,20,34,111,212,235,235,235,235,235,235,226,204,184,167,155,148,148,
100,95,65,49,46,49,47,44,52,51,52,52,53,60,57,79,39,0,2,22,50,33,26,44,
22,26,20,35,43,38,35,44,50,62,124,75,52,93,102,130,86,77,107,79,111,111,
108,108,120,198,94,0,34,28,28,29,32,32,32,66,74,74,81,91,91,94,108,108,
122,145,12,0,0,0,0,0,0,0,0,0,0,0,0,0,6,30,138,52,10,0,0,0,42,55,36,46,51,
65,69,105,64,54,51,64,122,77,73,93,96,69,27,0,0,16,126,222,255,255,255,
252,255,239,224,220,204,216,188,183,51,49,43,45,38,38,48,57,38,52,47,57,
59,66,59,39,46,0,0,0,30,29,25,23,26,28,27,33,28,34,42,39,50,57,94,82,73,
89,98,101,84,102,118,112,107,85,87,108,108,150,199,35,34,28,28,28,28,28,
28,64,73,74,74,84,91,91,93,107,108,122,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,17,
55,23,1,0,0,0,38,67,70,44,57,74,108,88,101,52,52,72,137,79,71,68,136,67,
27,0,0,0,21,116,222,255,255,250,249,237,226,205,201,217,192,182,47,51,38,
44,38,44,51,58,65,54,54,54,57,64,64,27,0,0,0,0,12,12,27,25,20,24,24,28,
28,31,31,38,50,53,66,68,79,85,96,103,96,94,86,108,102,87,61,104,108,114,
211,97,34,28,28,28,28,28,28,60,67,74,74,74,87,91,91,97,108,114,10,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,6,27,10,0,0,0,0,47,64,110,52,71,96,122,68,135,51,
64,86,139,85,77,72,118,69,32,0,0,0,0,17,122,220,255,255,242,235,212,192,
180,195,193,178,66,51,44,54,44,50,51,44,58,48,51,45,53,60,64,15,0,0,0,0,
1,0,6,7,21,27,20,21,21,15,14,27,34,34,37,39,44,56,66,95,93,57,52,101,106,
116,65,99,97,102,152,156,39,28,28,28,28,28,28,51,62,72,74,74,76,88,91,91,
97,108,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,11,1,0,0,0,0,47,54,58,63,95,124,
168,92,117,64,68,88,146,85,85,79,132,80,45,16,7,6,0,0,6,95,207,251,230,
225,221,180,177,201,194,186,96,57,46,64,61,44,52,47,56,46,51,49,51,56,38,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,21,27,28,38,51,58,51,71,114,165,
83,102,119,85,113,157,50,28,28,28,32,32,32,42,53,62,73,74,74,79,89,91,91,
102,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,47,54,51,54,139,160,190,
103,100,66,68,88,146,85,85,85,156,104,81,48,29,15,0,0,0,8,95,200,227,217,
207,203,191,196,190,180,134,73,53,50,81,66,69,56,52,52,51,47,51,47,6,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,17,28,28,29,72,92,55,111,117,
85,106,114,130,108,121,75,36,36,36,45,45,45,48,46,56,65,73,74,74,80,91,
91,97,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,47,54,51,52,105,142,
111,112,113,68,81,91,126,85,85,106,232,163,84,78,15,0,0,0,0,0,9,88,188,
205,194,206,186,201,181,169,165,97,71,63,44,109,85,59,50,51,51,51,50,27,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,28,28,28,60,140,77,107,
102,103,107,102,120,108,108,97,58,58,58,58,58,58,54,45,46,61,68,74,74,74,
83,91,92,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,45,63,56,54,66,92,
82,93,141,89,113,140,145,85,85,105,255,235,126,69,12,0,0,0,0,0,0,0,74,161,
200,190,179,180,170,170,181,139,98,83,67,107,118,74,54,54,51,49,24,0,0,
0,0,0,0,0,0,0,0,0,15,0,0,0,0,0,0,0,0,0,0,0,0,0,28,28,28,32,116,100,107,
74,104,71,71,106,108,108,107,62,62,62,62,62,62,63,46,45,53,62,72,74,74,
74,87,95,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,68,64,50,46,75,
74,127,126,140,184,197,176,112,108,83,255,255,169,48,12,0,0,0,0,0,0,0,4,
66,163,193,191,177,167,167,161,176,130,133,113,102,144,86,60,73,54,34,0,
0,0,0,0,0,0,0,0,0,0,0,24,0,0,0,0,0,0,0,0,0,0,0,0,0,28,28,28,28,69,108,111,
115,99,71,56,100,104,104,110,52,52,52,62,62,62,67,57,46,46,56,63,73,74,
74,75,89,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,68,71,68,47,52,68,
92,208,169,208,201,210,217,149,114,72,230,255,136,39,12,0,0,0,0,0,0,0,0,
3,51,145,181,170,166,167,155,161,147,139,110,91,101,64,63,94,64,41,7,0,
0,0,0,0,0,0,0,0,25,41,102,27,10,0,0,0,0,0,0,0,0,0,0,0,28,28,28,28,45,104,
88,103,104,96,57,95,116,106,106,31,39,44,48,48,48,68,62,54,45,46,61,67,
74,74,74,82,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,34,65,92,70,49,64,
91,164,250,169,159,135,177,239,122,69,53,198,255,136,39,12,0,0,0,0,0,0,
0,0,0,0,29,115,175,164,169,165,153,130,130,90,73,58,56,49,48,51,63,42,7,
25,5,0,0,0,0,0,0,3,44,57,0,0,0,0,0,0,0,0,0,0,0,0,10,28,28,28,28,61,98,106,
85,108,129,67,82,108,108,94,28,28,28,35,39,39,74,68,62,47,45,52,60,68,74,
74,78,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,49,94,122,84,120,118,182,
239,255,172,117,92,157,252,183,54,142,255,255,136,39,12,0,0,0,0,0,0,0,0,
0,0,0,31,94,141,156,169,170,148,121,90,74,60,56,46,45,44,48,55,35,66,89,
26,15,18,22,30,37,32,77,15,0,0,0,0,0,0,0,0,0,0,0,0,25,37,37,37,37,77,99,
112,106,108,161,79,82,108,108,95,28,28,28,28,28,28,75,73,63,57,46,45,54,
63,70,74,77,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,22,59,106,170,137,239,
234,248,253,253,157,89,136,185,254,246,164,243,255,255,136,39,12,0,0,0,
0,0,0,0,0,0,0,0,0,18,67,127,143,143,136,110,71,61,57,52,48,45,44,42,42,
46,44,116,88,73,74,75,64,61,49,55,55,56,56,61,81,81,103,101,0,0,0,1,24,
44,46,46,46,47,97,100,108,102,108,130,112,85,108,108,76,28,28,28,29,29,
41,87,80,69,62,54,45,46,61,63,74,75,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,43,50,106,158,199,255,253,245,241,240,206,154,245,251,248,245,255,247,
255,255,136,39,12,0,0,0,0,0,0,0,0,0,0,0,0,0,14,49,128,134,118,134,98,68,
73,60,47,44,39,39,39,31,43,63,89,78,68,70,59,55,53,74,79,88,90,99,101,101,
113,130,22,25,32,45,64,62,62,62,63,73,88,102,112,114,108,108,112,103,108,
117,55,30,33,33,45,45,45,91,88,75,64,62,47,45,51,60,67,72,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,42,32,129,255,255,255,243,249,255,242,228,252,
250,251,249,241,255,251,255,245,136,39,12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,
50,106,108,105,107,80,72,59,49,45,46,35,36,32,32,45,65,62,58,60,65,65,69,
76,82,82,82,93,96,96,96,82,67,76,97,95,71,66,62,65,72,79,81,69,87,117,104,
104,108,108,108,94,46,40,45,45,45,45,45,95,98,81,71,63,55,45,45,54,62,64,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,18,43,133,255,255,242,251,255,255,
253,217,249,253,255,255,253,253,255,240,249,136,39,12,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,5,38,95,80,70,62,58,51,55,51,50,42,36,32,31,47,51,51,43,48,
48,43,45,57,59,64,63,61,68,68,74,64,46,49,52,54,50,64,71,72,72,76,81,58,
50,78,106,95,86,107,107,49,45,45,46,46,46,56,59,98,107,93,80,69,61,54,45,
46,61,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,18,44,174,253,255,255,
255,255,255,255,234,242,255,255,255,255,241,251,255,255,136,39,12,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,27,70,65,60,53,51,56,71,49,39,35,31,30,35,49,
47,51,51,48,48,49,63,53,57,70,59,66,64,74,83,32,32,34,39,45,53,59,67,47,
71,101,90,71,49,86,103,84,103,91,9,50,50,62,64,66,66,66,99,108,103,88,75,
64,62,47,45,51,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,18,44,172,250,
249,255,255,255,255,255,239,232,255,255,255,255,251,247,255,255,136,39,
12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,24,53,59,53,53,48,87,57,41,37,33,
30,33,54,40,48,48,47,48,53,76,51,55,59,65,66,59,52,121,54,38,32,32,47,51,
60,69,60,50,96,127,106,74,54,102,119,100,46,0,69,69,69,71,74,74,74,101,
108,108,95,81,69,62,55,45,45,54,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
18,44,172,249,236,253,252,254,255,255,252,245,248,255,248,255,255,246,255,
255,136,39,11,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,21,45,53,47,47,63,93,
45,34,31,33,66,51,51,44,49,46,48,41,54,42,48,58,63,62,87,54,74,43,30,40,
28,38,48,60,81,106,54,76,98,128,131,74,98,112,85,3,0,75,75,75,75,75,75,
75,94,107,108,104,90,74,65,61,54,45,51,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,18,44,172,242,228,242,242,252,255,255,255,251,243,255,255,255,255,
241,255,255,136,39,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,12,38,47,44,
75,77,54,39,36,70,58,45,61,42,40,53,52,47,45,53,40,61,55,64,99,73,55,28,
28,51,35,31,53,56,55,81,72,80,80,93,143,133,103,95,24,0,0,97,97,91,91,91,
91,91,91,98,108,108,96,83,74,64,62,47,45,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,18,44,172,248,218,248,249,255,255,255,255,241,241,252,249,255,
242,244,255,255,136,39,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,12,35,
50,82,45,50,47,64,80,54,35,37,36,38,44,78,49,47,74,53,55,53,69,64,80,43,
24,24,66,56,35,45,61,53,58,72,74,78,84,98,138,94,42,0,0,0,129,129,101,101,
101,93,91,91,92,105,108,105,89,80,70,62,56,49,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,18,44,172,250,233,224,241,250,248,253,231,231,253,248,245,
240,252,249,245,197,85,28,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
9,30,46,42,34,41,98,74,54,34,34,34,34,39,43,76,53,55,59,51,45,54,65,74,
52,35,18,40,63,36,49,60,79,52,68,71,80,82,94,83,33,2,0,0,0,64,140,133,142,
136,111,110,87,90,97,108,108,98,90,74,65,61,61,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,18,44,172,250,250,204,213,245,253,248,249,255,255,255,248,
255,238,201,162,112,42,14,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,7,22,39,35,57,126,74,54,34,34,34,34,35,35,61,56,46,48,53,51,57,68,73,
77,40,26,28,28,29,42,53,88,61,66,76,76,69,61,25,0,0,0,0,0,0,10,62,113,143,
155,155,83,81,91,105,108,108,97,83,71,62,67,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,18,44,172,240,240,228,197,250,253,253,255,255,255,255,240,210,
172,123,82,53,25,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,
21,36,97,127,74,54,34,34,34,34,35,35,68,57,53,50,53,51,51,62,64,71,54,50,
45,41,54,51,64,64,71,56,49,36,20,9,0,0,0,0,0,0,0,0,0,6,36,89,149,100,86,
99,105,125,130,110,100,87,78,72,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
8,32,108,181,181,181,153,180,182,182,182,182,182,182,170,121,88,58,40,31,
11,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,15,116,125,74,
50,28,28,28,32,34,35,67,56,37,43,42,47,47,49,59,55,17,5,4,4,5,4,6,6,6,4,
2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,12,9,8,9,9,12,13,10,9,8,8,6,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,18,53,96,96,96,74,88,96,96,96,96,96,96,88,
53,39,32,26,18,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,133,110,52,17,0,0,0,15,19,23,54,72,17,8,0,0,15,28,38,51,16,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,22,8,34,0,25,11,0,0,17,37,5,0,28,15,29,24,18,
24,24,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,32,45,45,45,43,43,45,
45,45,45,45,45,43,33,27,19,13,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,49,109,8,0,0,0,0,0,0,0,14,63,13,6,0,0,0,6,20,54,43,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,45,18,51,0,42,8,0,0,36,51,4,0,45,39,
46,41,14,50,50,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,17,29,29,29,
27,27,29,29,29,29,29,29,27,19,12,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,27,82,0,0,0,0,0,0,0,0,0,26,0,0,0,0,0,0,0,36,
65,17,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,33,13,33,0,41,20,0,0,27,62,9,0,
47,18,48,26,0,55,26,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,15,15,
15,14,14,15,15,15,15,15,15,13,5,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,71,0,0,0,0,0,0,0,0,0,17,0,0,0,0,0,0,0,13,
20,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,25,34,22,45,9,32,21,6,0,24,37,10,0,
33,33,33,24,13,32,21,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,3,
3,3,3,3,3,3,3,3,3,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,38,60,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,4,15,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,17,13,11,19,6,14,14,4,0,6,21,3,0,12,11,11,9,9,
9,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
24,25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,13,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

}
;
unsigned char calc_diagram(unsigned char xx, unsigned char yy)
{
  lenna[xx+128*yy]/4;
}


void rotatePoints(const double* points, int numPoints, double angle, double* rotatedPoints) {
  int i;
  double cosAngle = cos(angle);
  double sinAngle = sin(angle);

  for (i = 0; i < numPoints; i += 2) {
    double x = points[i];
    double y = points[i + 1];

    // Apply rotation transformation
    double rotatedX = x * cosAngle - y * sinAngle;
    double rotatedY = x * sinAngle + y * cosAngle;

    rotatedPoints[i] = rotatedX;
    rotatedPoints[i + 1] = rotatedY;
  }
}

double spaceship[] =
{
#define _M 2.5
  0, 0 + _M,        -1, -2 + _M,
  -1, -2 + _M,      -1, -3 + _M,
  -1, -3 + _M,      -1, -4 + _M,
  -1, -4 + _M,      -2, -6 + _M,
  -2, -6 + _M,      -0, -5 + _M,

  0, 0 + _M,      1, -2 + _M,
  1, -2 + _M,      1, -3 + _M,
  1, -3 + _M,      1, -4 + _M,
  1, -4 + _M,      2, -6 + _M,
  2, -6 + _M,      0, -5 + _M,



#define _ASX 5
#define _ASY 5
  0 + _ASX, 0 + _ASY, 1.2 + _ASX, 0.2 + _ASY,
  1.2 + _ASX, 0.2 + _ASY, 1.3 + _ASX, 1 + _ASY,
  1.3 + _ASX, 1 + _ASY, 1.5 + _ASX, 2 + _ASY,
  1.5 + _ASX, 2 + _ASY, -0.2 + _ASX, 1.1 + _ASY,
  -0.2 + _ASX, 1.1 + _ASY, 0 + _ASX, 0 + _ASY,


#define _ASX -5
#define _ASY -1.3
  0 + _ASX, 0 + _ASY, 1.2 + _ASX, 0.2 + _ASY,
  1.2 + _ASX, 0.2 + _ASY, 1.3 + _ASX, 1 + _ASY,
  1.3 + _ASX, 1 + _ASY, 1.5 + _ASX, 2 + _ASY,
  1.5 + _ASX, 2 + _ASY, -0.2 + _ASX, 1.1 + _ASY,
  -0.2 + _ASX, 1.1 + _ASY, 0 + _ASX, 0 + _ASY,


#define _ASX -0.2
#define _ASY 4
  0 + _ASX, 0 + _ASY, 1.2 + _ASX, 0.2 + _ASY,
  1.2 + _ASX, 0.2 + _ASY, 1.3 + _ASX, 1 + _ASY,
  1.3 + _ASX, 1 + _ASY, 1.5 + _ASX, 2 + _ASY,
  1.5 + _ASX, 2 + _ASY, -0.2 + _ASX, 1.1 + _ASY,
  -0.2 + _ASX, 1.1 + _ASY, 0 + _ASX, 0 + _ASY,


};


typedef struct myPoint {
  double x;
  double y;
} myPoint;
void rotatePoints(myPoint* points, int numPoints, myPoint center, double angle);

void rotatePoints(myPoint* points, int numPoints, myPoint center, double angle)
{
  float sinAngle = sin(angle);
  float cosAngle = cos(angle);

  for (int i = 0; i < numPoints; i++) {
    float translatedX = points[i].x - center.x;
    float translatedY = points[i].y - center.y;

    points[i].x = translatedX * cosAngle - translatedY * sinAngle + center.x;
    points[i].y = translatedX * sinAngle + translatedY * cosAngle + center.y;
  }
}



#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define NUM_OF_INDICES 12
#define SCREEN_CENTER_X  (SCREEN_WIDTH / 2)
#define SCREEN_CENTER_Y  (SCREEN_HEIGHT / 2)
#define OBJ_SCALE (float)2500
#define CAMERA_DISTANCE 15

float objX = 0;
float objY = 0;
float objZ = 10;
float rotationX = .2;
float rotationY = .3;

 
static void rotate(float x, float y, float angle,float s,float c, float *r1, float *r2)
{
  *r1 = x * c - y * s;
  *r2 = y * c + x * s;
}
static  const u8 tri_color[12]={// 0,0,0,0,0,0,
50, 63, 40, 63, 50, 40,
40, 50, 50, 63, 63, 40,
                   };
typedef struct triangle
{
  u8 x1,y1,x2,y2,x3,y3,c,z; 
}triangle;

uint16_t sinTable16[] = {
  0,
1145, 2289, 3435, 4572, 5716, 6853, 7989, 9125, 10255, 11385,
12508, 13631, 14745, 15859, 16963, 18067, 19165, 20253, 21342, 22417,
23489, 24553, 25610, 26659, 27703, 28731, 29755, 30773, 31777, 32772,
33756, 34734, 35697, 36649, 37594, 38523, 39445, 40350, 41247, 42131,
42998, 43856, 44701, 45528, 46344, 47147, 47931, 48708, 49461, 50205,
50933, 51646, 52342, 53022, 53686, 54334, 54969, 55579, 56180, 56760,
57322, 57866, 58394, 58908, 59399, 59871, 60327, 60768, 61184, 61584,
61969, 62330, 62677, 63000, 63304, 63593, 63858, 64108, 64334, 64545,
64731, 64903, 65049, 65177, 65289, 65377, 65449, 65501, 65527, 65535,
65535
};
#include <math.h>




extern "C" void printtriangle(char* txt,int x1,int y1,int x2,int y2, int x3,int y3)
{
  char tmp[256];
  sprintf(tmp,"triangle %s : %d %d %d %d %d %d\r\n",x1,y1,x2,y2,x3,y3);
  Serial.println(tmp);
  
}



extern "C" void printvectori(char* txt,int aa,int bb,int cc)
{
  
  char tmp[256];
  sprintf(tmp,"triangle %s :( %d, %d, %d, )\r\n",txt,aa,bb,cc);
  Serial.println(tmp);
  
}

extern "C" 
void printvectorf(char* txt,float a,float b, float c)
{
  int aa=a*100;
  int bb=b*100;
  int cc=c*100;
  
  char tmp[256];
  sprintf(tmp,"triangle %s :( %d.2d, %d.2d, %d.2d, )\r\n",txt,aa,bb,cc);
  Serial.println(tmp);
  
}

extern "C" void printstr(char* txt)
{
  
  char tmp[256];
  sprintf(tmp,"str:%s\r\n",txt);
  Serial.println(tmp);
  
}


extern "C" void printfloat(float f)
{
  
  char tmp[256];
  int a=f*100;
  sprintf(tmp,"str:%d\r\n",a);
  Serial.println(tmp);
  
}







extern "C" 

unsigned char TransformPointD(int x,int y, int z,void *matrix,int *xx,int *yy)
 ;
extern "C" 
void print_matrix(void *m);

extern "C" void setCamera(int ex,int ey,int ez,int cx,int cy, int cz);

extern "C"
void * GfxApiSetCameraLookat();
extern "C" void print_matrix(void *);
// bugs... assembler .... extern "C" void GfxApiSetCameraLookatFloat(float eyeX,float eyeY,float eyeZ,float centerX,float centerY,float centerZ);

const int vertices[] = {
  -1.0f,-1.0f,-1.0f, 
  -1.0f,-1.0f, 1.0f,
  -1.0f, 1.0f, 1.0f, 
  1.0f, 1.0f,-1.0f, 
  -1.0f,-1.0f,-1.0f,
  -1.0f, 1.0f,-1.0f, 
  1.0f,-1.0f, 1.0f,
  -1.0f,-1.0f,-1.0f,
  1.0f,-1.0f,-1.0f,
  1.0f, 1.0f,-1.0f,
  1.0f,-1.0f,-1.0f,
  -1.0f,-1.0f,-1.0f,
  -1.0f,-1.0f,-1.0f,
  -1.0f, 1.0f, 1.0f,
  -1.0f, 1.0f,-1.0f,
  1.0f,-1.0f, 1.0f,
  -1.0f,-1.0f, 1.0f,
  -1.0f,-1.0f,-1.0f,
  -1.0f, 1.0f, 1.0f,
  -1.0f,-1.0f, 1.0f,
  1.0f,-1.0f, 1.0f,
  1.0f, 1.0f, 1.0f,
  1.0f,-1.0f,-1.0f,
  1.0f, 1.0f,-1.0f,
  1.0f,-1.0f,-1.0f,
  1.0f, 1.0f, 1.0f,
  1.0f,-1.0f, 1.0f,
  1.0f, 1.0f, 1.0f,
  1.0f, 1.0f,-1.0f,
  -1.0f, 1.0f,-1.0f,
  1.0f, 1.0f, 1.0f,
  -1.0f, 1.0f,-1.0f,
  -1.0f, 1.0f, 1.0f, 
  1.0f, 1.0f, 1.0f,
  -1.0f, 1.0f, 1.0f,
  1.0f,-1.0f, 1.0f,
/*
  -5,-1,-5,
  -5,-1,5,
  5,-1,-5,

  
  5,-1,5,
  -5,-1,5,
  5,-1,-5,*/
//  */
};

const int groundplane[]={
  
  -5,-1,-5,
  -5,-1,5,
  5,-1,-5,

  
  5,-1,5,
  -5,-1,5,
  5,-1,-5,
};


void render_object(int *vertices,int count,int SC, int xpos,int ypos,int zpos,void *matrix,GfxApiRenderBuffer *rend)
{
  u8 cnt=3;
  int x[3],y[3],x_out,y_out;
    for (int i = 0; i <count ; i+=3)
  {
    int x1, y1, z1;
    x1 = SC*vertices[i + 0]+xpos;
    y1 = SC*vertices[i + 1]+ypos;
    z1 = SC*vertices[i + 2]+zpos;

  
    TransformPointD(x1,y1,z1,matrix,&x_out,&y_out);

    cnt--;
    
    x[cnt]=x_out;
    y[cnt]=y_out;
    if(!cnt)
    {
   // Perform hidden surface removal by checking triangle orientation
            int dx1 = x[1] - x[0];
            int  dy1 = y[1] - y[0];
            int  dx2 = x[2] - x[0];
            int  dy2 = y[2] - y[0];
            int cross_product = dx1 * dy2 - dy1 * dx2;

         //   if (cross_product < 0) 
            {
//          
               RenderbufferTriangle(x[0],y[0],x[1],y[1],x[2],y[2],rend);
          
                sei();
            }
      
      cnt=3;
    }
  }
}

void Calc3D() {
  
  static GfxApiRenderBuffer rend;

  static u8 once=1;
  if(once)
  {
    memset (&rend,255,sizeof(rend)); // initalize renderbuffer
  }
  static int dir=10;
  static int dir2=5;
  sei();
  static int eyex=0;
  static int eyey=300;
  static int eyez;

static int cntf2;
cntf2+=3;
static float SC2=20;
  eyex=sintab[cntf2&0xff]
  *SC2;
  
  eyez=sintab[(cntf2+64)&0xff]*SC2;
 
  setCamera(eyex,eyey,eyez,
  100,64,0);
if(SC2>6)SC2-=.1;

  static void *matrix ;
  matrix=GfxApiSetCameraLookat();
  /*
  print_matrix(matrix);

  int x_out,y_out;
  TransformPointD(-2*128,-1*128,-1*128,matrix,&x_out,&y_out);

return;
*/if (eyey>300)dir2=-dir2;

  if (eyey<200)dir2=-dir2;
  static u8 b1,b2,b3,b4;
  static int ccc=500;
  if(ccc>0)ccc--;
  else{
  b1++;
  b2+=2;
  b3+=3;
  b4+=5;
  }
static int scaler_c=600,scaler=128;

static int h2=100*16,h1=120*16;
if(h1)h1-=16;
if(h2)h2-=16; else scaler_c--;
if(scaler_c<0)scaler--;
if(!scaler)
{
  eyey=300;
  h2=100*16,h1=120*16;
  scaler_c=600,scaler=128,SC2=30,ccc=500;
}
  
  render_object(vertices, sizeof(vertices)/2,scaler,-256,0,-256,matrix,&rend);
 GfxApiFlushRenderbuffer(&rend,1);

  render_object(vertices, sizeof(vertices)/2,scaler,256,h1+abs(sintab[b2]),-256,matrix,&rend);
 GfxApiFlushRenderbuffer(&rend,2);


  render_object(vertices, sizeof(vertices)/2,scaler,-256,h2+abs(sintab[b3]),256,matrix,&rend);
 GfxApiFlushRenderbuffer(&rend,3);

//  render_object(vertices, sizeof(vertices)/2,scaler,256,0,256,matrix,&rend);
// GfxApiFlushRenderbuffer(&rend,4);


render_object(groundplane, sizeof(groundplane)/2,scaler,0,0,0,matrix,&rend);
 GfxApiFlushRenderbuffer(&rend,1);

 

/*  u8 cnt=3;  
  int x[3],y[3];
 // Serial.println(sizeof(vertices)/sizeof(vertices[0]));
  for (int i = 0; i <sizeof(vertices)/sizeof(int) ; i+=3)
  {
    #undef SC
    #define SC (128)
    
    int x1, y1, z1;
    x1 = SC*vertices[i + 0];
    y1 = SC*vertices[i + 1];
    z1 = SC*vertices[i + 2];

  
    TransformPointD(x1,y1,z1,matrix,&x_out,&y_out);

    cnt--;
    
    x[cnt]=x_out;
    y[cnt]=y_out;
    if(!cnt)
    {
   // Perform hidden surface removal by checking triangle orientation
            int dx1 = x[1] - x[0];
            int  dy1 = y[1] - y[0];
            int  dx2 = x[2] - x[0];
            int  dy2 = y[2] - y[0];
            int cross_product = dx1 * dy2 - dy1 * dx2;

         //   if (cross_product < 0) 
            {
//                RenderbufferTriangle(x[0],y[0],x[1],y[1],x[2],y[2],&rend);
                tri++;
                
                
                if(tri>12)
                {
                    RenderbufferTriangle(x[0],y[0],x[1],y[1],x[2],y[2],&rend);
              
//            memset (&rend,255,sizeof(rend)); // initalize renderbuffer
 
              //    GfxApiFlushRenderbuffer(&rend,1);
                }else 
                if (cross_product < 0) 
  
               VectoscopeTriangle(x[0],y[0],x[1],y[1],x[2],y[2],1);
          
                sei();
            }
      
      cnt=3;
      os_i2c_stop();
    }
  }*/

  /* RenderbufferTriangle(0,0,63,31,-63,31,&rend);
             
   RenderbufferTriangle(0,-10,63,31,-63,31,&rend);
    */         
      os_i2c_stop();
  
 // rend.top=31;
 // rend.bottom=60;
//  for(int i=31;i<60;i++)rend.xstart[i]=i,rend.xend[i]=i+60;
  GfxApiFlushRenderbuffer(&rend,7);
  
      os_i2c_stop();
}





u8 invert_byte(u8 b)
{
  int i,o=0;
  for(i=0;i<8;i++)
  {
    if(b&(1<<i))o|=1<<(7-i);
  }
  return o;
}
/*
void timingcheck()
{

  Display(0, 0, 0, 0); //console,cat_tiles+chunk*128*8);

  
  GfxApiSetDisplayUndocumentedD2(4);
  GfxApiSetDisplayMux(63);  
static u8 pcharge,pcharge2,dcharge,dcharge2;
Serial.begin(9600);
for(;;)
{

  sei();
  //delay(20);
 GfxApiSetDisplayMux(63);
//  GfxApiShowFrame(1);
  _delay_us(30);
  GfxApiSetDisplayOn(1);

  _delay_us(30);
  GfxApiSetDisplayOn(0);

  
 if(Serial.available()){
  delay(20);
  u8 e= Serial.parseInt(); 
  if(e==1)pcharge=Serial.parseInt(); 
  if(e==2)dcharge=Serial.parseInt();
  if(e==3)pcharge2=Serial.parseInt(); 
  if(e==4)dcharge2=Serial.parseInt();

  Serial.print ("pcharge ");
  Serial.println(pcharge);
  Serial.println(pcharge2);
  Serial.print ("dcharge ");
  Serial.println(dcharge);
  Serial.println(dcharge2);
  GfxApiDisplaySetRegP1(0xd1,dcharge);
  
}
}
}
*/
#include <math.h>
void play_video()
{
   Serial.begin(9600);
 
  for(;;){
 
//  timingcheck();
  u16 br = 0;
  GfxApiCompressedLayer Overlay;

//void GfxApiSetDisplaySetContentScroll (u8 direction, u8 clear_mode, u8 x_start, u8 row_start, u8 x_end,  u8 row_end)

  Display(&Overlay, 0, 0, 0); //console,cat_tiles+chunk*128*8);
  
//GfxApiSetDisplaySetContentScroll(0,0,0,0,127,0);
GfxApiSetDisplayUndocumentedD2(4);
 // GfxApiStartVectorScope(1);


  float counter=0;
  int x=0;

  int coord[256];
  for (int i=0;i<256;i++)coord[i]=rand()&63;
  u8 cho=0;
  u8 pcharge=0,pcharge2=0,dcharge=0,dcharge2=0;
int i;

  Display(0, 0, 0, 0); //console,cat_tiles+chunk*128*8);

    GfxApiSetDisplayMux(1);
   sei();
   frames=0;

for(;;){
  long long start=millis();
//  for(int i=0;;i++)
    Calc3D();

  long long end=millis();
    int _time=end-start;
 
  
    int fps=1000/_time;

    //if((cnt&0xf)==0)
    if(fps<27)
      __3d_accept_error++;else if(__3d_accept_error>1) __3d_accept_error--;
      if(__3d_accept_error>30)__3d_accept_error=30;
//    Serial.println(_time);


  }
  
  sei();
  
  }
}

void loop()
{
  for (;;)
    play_video();

}


#ifndef __AVR__

// Callback function to be executed by the timer
void timer_callback() {
  vsync_flag = 1;
  // Add your code here
  // This function will be called at the specified frequency
}



void setup_timer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &timer_callback, true);
  timerAlarmWrite(timer, 1000000 / 1, true);
  timerAlarmEnable(timer);

}

void stop_timer() {
  // Stop and delete the timer
}
#endif

void setup() {
  streamgfx_init_arduino();
  int i;  cli();//stop all interrupts
  /*
    // turn on CTC mode
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCCR1B |= (1 << WGM12);
    // Set CS11 bit for prescaler 8
    TCCR1B |= (1 << CS11);

    //initialize counter value to 0;
    TCNT1  = 0;

    // set timer count for 50Hz increments
    OCR1A = 39999/TDIV;// = (16*10^6) / (50*8) - 1

    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  */
  sei();//allow interrupts
//  setup_timer();
  tri_size = 30;

  for(i=0;i<256;i++)
  {
    float a=i/256.0;
    a*=6.28318530718;
    sintab[i]=128*sin(a);
  }
   
  loop();
}
/* clock:
      for(;;)
  {
   static u16 cnt;
   cnt+=1;

   fuzz_d2=cnt>>4;

   if((cnt&0xff)==0)Serial.println(cnt);

//                VectoscopeTriangle(20,20,40,40,10,60,1);
      u8 x=sin(counter)*28+31;
      u8 y=cos(counter)*28+31;



      u8 x2=sin(counter*2.2)*28+31;

      u8 y2=cos(counter*2.2)*28+31;


      u8 x3=sin(counter*1.5)*28+31;

      u8 y3=sin(counter*1.6)*28+31;

  int xx=0;
  {
   GfxApiVectorScopeLine(31,31,x,y,3,5);

   GfxApiVectorScopeLine(31,31,x2,y2,1,5);

  }
  ;
   counter+=0.01;
     adder+=0.0001;
     }
  }
*/
