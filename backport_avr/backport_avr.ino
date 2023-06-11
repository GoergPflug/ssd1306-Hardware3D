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



unsigned char calc_diagram(unsigned char xx, unsigned char yy)
{
  return yy;//lenna[xx+128*yy]/4;
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

 const float vertices[] = {
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
  1.0f,-1.0f, 1.0f
};

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

void Calc3D() {
//  GfxApiBeginTriangles();
  const float srx= sin(rotationX);
  const float crx=cos(rotationX);

  const float sry= sin(rotationY);
  const float cry= cos(rotationY);


  float z_sort=0;
  u8 cnt=3,cnt2=0;

  int x[3],y[3];
  
  for (int i = 0; i < 3*12; i++)
  {
    u8 a = i*3;
    float x1, y1, z1;
    x1 = vertices[a + 0];
    y1 = vertices[a + 1];
    z1 = vertices[a + 2];

    rotate(x1, z1, rotationY,sry,cry,&x1,&z1);
    rotate(y1, z1, rotationX,srx,crx,&y1,&z1);

    x1 = (float)(x1 * OBJ_SCALE);
    y1 = (float)(y1 * OBJ_SCALE);

    x1 += objX;
    y1 += objY;
    z1 += objZ;
    float inverse=1.0/(z1 * CAMERA_DISTANCE);
    x1 *=inverse;
    y1 *=inverse;

    x1 += SCREEN_CENTER_X;
    y1 += SCREEN_CENTER_Y;
    z_sort+=z1*7;
  //  GfxApiStoreTrianglePoint(x1, y1);// , x2, y2);
    cnt--;
    x[cnt]=x1;
    y[cnt]=y1;
    if(!cnt)
    {
   // Perform hidden surface removal by checking triangle orientation
            float dx1 = x[1] - x[0];
            float dy1 = y[1] - y[0];
            float dx2 = x[2] - x[0];
            float dy2 = y[2] - y[0];
            float cross_product = dx1 * dy2 - dy1 * dx2;

            if (cross_product > 0) 
            {
                // Call the triangle function only for visible triangles
                VectoscopeTriangle(x[0],y[0],x[1],y[1],x[2],y[2],0);
           //     yield();
            }
      
      cnt=3;
      os_i2c_stop();
      cnt2++;
      z_sort=0;
    }
  }
  rotationX=3;// += .051;
  //if(rotationX>2*M_PI)rotationX-=2*M_PI;
  rotationY=2;// += .05;
  //if(rotationY>2*M_PI)rotationX-=2*M_PI;
  
}




void drawCube(double angle1, double angle2, int slow) {
  const double size = 20.0;
  const double pi = 3.14159;

  // Convert angles to radians
  double rad1 = angle1 * pi / 180.0;
  double rad2 = angle2 * pi / 180.0;

  // Calculate cosines and sines
  double c1 = cos(rad1);
  double s1 = sin(rad1);
  double c2 = cos(rad2);
  double s2 = sin(rad2);

  // Calculate the vertices of the cube
  double vertices[8][3] = {
    { -size, -size, -size},
    {size, -size, -size},
    {size, size, -size},
    { -size, size, -size},
    { -size, -size, size},
    {size, -size, size},
    {size, size, size},
    { -size, size, size}
  };

  // Connect the vertices to form the cube
  int edges[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7}
  };

  // Rotate the cube around the x-axis
  for (int i = 0; i < 8; i++) {
    double y = vertices[i][1];
    double z = vertices[i][2];
    vertices[i][1] = y * c1 - z * s1;
    vertices[i][2] = y * s1 + z * c1;
  }

  // Rotate the cube around the y-axis
  for (int i = 0; i < 8; i++) {
    double x = vertices[i][0];
    double z = vertices[i][2];
    vertices[i][0] = x * c2 + z * s2;
    vertices[i][2] = -x * s2 + z * c2;
  }

  // Draw the lines of the cube
  for (int i = 0; i < 12; i++) {
    int v1 = edges[i][0];
    int v2 = edges[i][1];
    int x1 = (int)vertices[v1][0];
    int y1 = (int)vertices[v1][1];
    int x2 = (int)vertices[v2][0];
    int y2 = (int)vertices[v2][1];
    GfxApiVectorScopeLine(x1 + 31, y1 + 31, x2 + 31, y2 + 31, 4, -1);
  }
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

void play_video()
{
  u16 br = 0;
  Serial.begin(9600);
  GfxApiCompressedLayer Overlay;

  Serial.println("a2");
//void GfxApiSetDisplaySetContentScroll (u8 direction, u8 clear_mode, u8 x_start, u8 row_start, u8 x_end,  u8 row_end)

  Display(&Overlay, 0, 0, 0); //console,cat_tiles+chunk*128*8);
  
  Serial.println("back");
//GfxApiSetDisplaySetContentScroll(0,0,0,0,127,0);
GfxApiSetDisplayUndocumentedD2(4);
  for(;;)
  {
    GfxApiSetDisplayMux(1);
    VectoscopeTriangle(0,0,10,10,5,15,1);
    
  }

// zoom image  // fire:
 sei(); 
  for(;;)
  {
    static int c;
    c++;

  Serial.println("a2");
    u8 d=rand()&1;
    
    
    // GfxApiSetDisplaySetContentScroll(d,(d?2:1),invert_byte(c)&127,0,127,7);
//    __NOP__2
  //  GfxApiSetDisplaySetContentScroll(rand()&1,3,rand(),rand(),rand(),7);
//__NOP__2
GfxApiSetInvert(1);
    delay(10);
 // GfxApiSetDisplayUndocumentedD2(4);
  for(;;);

  //GfxApiShowFrame(0);
//GfxApiSetDisplaySetContentScroll(0,0,0,0,127,7);
 

//  GfxApiDisplaySetRegP1(0xd1,(c));

  delay(6);
  
 // GfxApiDisplaySetRegP1(0xa9,c>>4);
 // GfxApiDisplaySetRegP1(0xd1,0);

  //GfxApiSetPrecharge(c>>1);
  
 //GfxApiDisplaySetRegP1(0xd1,c);
//GfxApiSetDisplayUndocumentedD2((c>>4));
  

//GfxApiSetInvert(0);
//GfxApiDisplaySetRegP1(0xd4,0);
  
// GfxApiDisplaySetRegP1(0xd1,~c);

//GfxApiSetDisplayUndocumentedD2(0);
  
//  GfxApiDisplaySetRegP1(0xa9,0xf);
  
//  GfxApiDisplaySetRegP1(0xa9,0xf);
//GfxApiSetDisplayOn(0);

// rand()&0xf|:
// 0b1    || flackernde linien , dünn
// 0b10   || black
// 0b11   || black
// 0b101  || flackern weniger dünn
// 0b111  || black -> bit 2 killt das display
// 0b1000 || black

// |  0x10 black

 
// 0 loop pixels
// 1 repeat pixels
// 2 make new pixels black    
// 3 make new pixels white

  }



  int s_pos = 0;
  static unsigned char console[16 * 8];
  //GfxApiSetFreqTemporal(0);
  int chunk = 0;
  int frames_total = 100;
  unsigned long next_frame = millis();
  int milis_frame = 1000 / 25;
  next_frame += milis_frame;
  int drawn = 0;

  u8 cc[128];

  for (u8 i = 0; i < 128; i++)cc[i] = 0; //((i)%37)|((i&1)?128:0);


  for (;;)
  {
    for (int frame = 0; frame < 100; frame++)
    {
      int i;
      {


        // GfxApiSetFreq(0xF0);
        //  GfxApiFlipY(1);
        //GfxApiSetFreq(0x0f);
        //        (4);
        for (;;) {
          static u8 cnt;
          cnt++;
          static u8 u = 10;
          qqqx123++;
          if (1)
            GfxApiAttachConsole(cc);

          GfxApiConsoleSetAttribute(1);
          for (i = 1; i < 4; i++)
          {
            GfxApiGotoXY(0, i);
            printf("row:%d", i);

          }


  Display(&Overlay, 0, cc, 15); //console,cat_tiles+chunk*128*8);
    delay(5000); // Delay for 5 seconds (5000 milliseconds)
  printf("change\r\n");
  Display(&Overlay, 0, cc, 15); //console,cat_tiles+chunk*128*8);
int trans[] = {134, 132, 128, 6, 4, 0};
//  ((brightness | 1 ) )&~16
// 134: 10000110 = 6?
// 132: 10000100 = 5?
// 128 nur stripes?
// 6=7 
// 4=5
// 0 no change
// 1 no change
// 2 no change
// 3 no change
// 4 image a slight bit darker / motion blur
// 5 image a slight bit darker / motion blur
// 6 dark
// 7: dark  
// 8 no change
// 9 no change
// 10 no change
// 11 no change
// 12 image a bit darker / motion blur?
// 13 image a bit darker / motion blur?
// 14 dark
// 15 dark
// 23 dark glowing random lines from top to bottom, something of the driver gooes off?
// 31 dark??
// 16 to 31 bright glowing random lines from top to bottom, something of the driver gooes off?
// 32 dark
// 36 
// 55 dark
// bit 4 set: there never seems to be anythhing on screen related to vram? turn vram / connection to it off?
// this holds for: 
// GfxApiSetDisplayUndocumentedD2(128|4|64|16);
// 128 stripes
// 64 ? flag? unused kein effekt?
//GfxApiSetDisplaySetScrollArea(0,0,0,127,4,3);
//GfxApiSetDisplayStopScroll();
if(0)for(;;){

  
  GfxApiSetDisplayUndocumentedD2(0);
    delay(1000);
  GfxApiSetDisplayUndocumentedD2(4);
    Display(&Overlay, 0, cc, 15); //console,cat_tiles+chunk*128*8);

  

  //if(0)
  for(int r=0;r<1000;r++)
  for(i=0;i<64;i++)
  {
    GfxApiSetStartline(i);
//    GfxApiSetDisplayScroll(0,0,0,0,127,4);
  }
  if(0){
    GfxApiWriteVRam(128*4+i+1,255);
    
    GfxApiWriteVRam(128*4+i,0);
    delay(10);
  }
  //GfxApiSetDisplayStartScroll();
    //delay(500);
//  GfxApiSetDisplayStopScroll();
  if(0)  for(int i=1;i<5;i++)
    {
//       GfxApiSetCharge(i);
       delay(1000);
    }
}

                
 

          //
          static int ii = 0;
          for (;;) {
            static u8 cnt;
            cnt++;
            ///  GfxApiSetCharge(cnt);
            for (i = 0; i < 1; i++)
            {
//              timerAlarmWrite(timer, 1000000 / (1), true);
//              GfxApiGotoXY(0, 2); printf("%d\r\n%d", i, scan);
              fuzz_d2 = i;
              //GfxApiSetRegD2(i);
              ii++;
              fflush(stdout);
              //  GfxApiSetCharge(i);
              // GfxApiSetSlowDrive(i);
              // GfxApiSetFreq(i);
              //  GfxApiSetRegA9(i);
              //      GfxApiSetRegF2(0);

              //  GfxApiSetRegF2(1<<5);
              //GfxApiSetRegFD(0x16);
              //  GfxApiSetRegFD(0x12^(1<<(i&7)));
              //  GfxApiSetRegF2(0);
              //GfxApiSetRegD2(i);
//              xxxxxbr = 255;
              //GfxApiSetRegFE(i);
              //  fflush(stdout);
              //   GfxApiSetDisplayScroll(i);

              //    GfxApiSetStartline(i);
              //    GfxApiSetDisplayOffset(i);
              //  GfxApiSetFreq(0xf0);

              //      GfxApiSetScrollArea(0,2);
              int r;

              //   GfxApiSetDisplayScroll(0);
              yield();
              Display(&Overlay, 0, cc, 15); //console,cat_tiles+chunk*128*8);

              uint64_t startTime;

              Display(&Overlay, 0, cc, 15); //console,cat_tiles+chunk*128*8);

              static u16 cnt;
              cnt++;
              //    GfxApiSetCharge(255);

              for (;;)
              {

                //  cnt&=0xf;
                /*  GfxApiSetDisplayScroll(0,0,0,120,7,1);
                  GfxApiSetDisplayScroll(0,0,0,120,6,1);
                  GfxApiSetDisplayScroll(0,0,0,120,5,1);
                  GfxApiSetDisplayScroll(0,0,0,120,4,1);
                  GfxApiSetDisplayScroll(0,0,0,120,3,1);
                  GfxApiSetDisplayScroll(0,0,0,120,2,1);
                  GfxApiSetDisplayScroll(0,0,0,120,1,1);
                  GfxApiSetDisplayDrawRect(0,64,0,127,7,1);
                */
  
                //     GfxApiSetDisplayScrollC();
                double counter = 0, adder = 0;

                int slow = 0;

                GfxApiStartVectorScope(0);
                // for(;;);
                //  poke_init();
                float cdiv = 1.0 / 16;
                GfxApiSetFreq(0xf0);
                for (;;)
                {
                  static u16 cnt;
                  cnt += 1;

                  fuzz_d2 = cnt >> 4;


                  //GfxApiVectorScopeLine(31,31,63,63,3,5);

                  int xx = 0;
                  static float cccn;
                  cccn += 0.03;

                  float scale = 1 + sin(cccn);
                  scale *= 2;
                  scale += 3;
                  double rotated[256];
                  static double rot;
                  rot += 0.05;

            //      rotatePoints(spaceship, sizeof(spaceship) / (sizeof(spaceship[0]) * 1), rot, rotated);


                  static u8 bbb, bbb2;

                  {

                    if (Serial.available()) {
                      int data = Serial.parseInt();
                      if (Serial.read() == '\n') {
                        // Successfully read an integer from serial
                        Serial.print("Received integer: ");
                        Serial.println(data);
                      } else {
                        // Invalid data format
                        Serial.println("Invalid data format");
                      }
                      //GfxApiSetDisplayUndocumentedD2(data);
                      //  for(i=0;i<sizeof(draw_mode)/8;i+=2)draw_mode[i*2]=0x94,draw_mode[i*2+1]=data;
                    }
                  }
//        Serial.print("i2c cmds: ");
  //                      Serial.println(i2c_counter
//                        );
          i2c_counter=0;
          
//    
{
  static double counter;
  counter+=0.01;           
/*   
 *    
 */
 u8 x=sin(counter)*28+31;
      u8 y=cos(counter*1.1)*25+31;
  
   u8 x1=sin(counter*1.23)*28+31;
      u8 y1=cos(counter*1.21)*25+31;
  
   u8 x2=sin(counter*1.3)*28+31;
      u8 y2=cos(counter*1.3)*25+31;
   VectoscopeTriangle(y,x1,y1,x2,y2,x,1);
   os_i2c_stop();
   for(int c=0, x=-10;x<11;x+=20,c++)
   {
    GfxApiVectoscopeSetOutputVoltage(c?5:1);
   VectoscopeTriangle(x,x,x1,y1,x2,y2,c+1);
   }
   os_i2c_stop();




//    for(int x=-10;x<11;x+=10)
  // VectoscopeTriangle(x,x,x1,y1,x2,y2,1);
/*tri: 65
07:34:17.666 -> 10
07:34:17.666 -> 102
07:34:17.666 -> 46
07:34:17.666 -> 25
07:34:17.666 -> 44
*/
                    GfxApiSetDisplayUndocumentedD2(4);
  
// rtc_wdt_feed();  // Reset the watchdog timer
//    rtc_wdt_disable();  // Disable the watchdog
    
    //VectoscopeTriangle(65,10,102, 46,25,44,0);
//cli();
//for(;;)
//Calc3D();
sei();   
    //  VectoscopeTriangle(c*25+10,10,c*25+20,20,c*25+10,30,c+1);
     
    
      
}
                  if (1)
                    for (i = 0; i < 25; i++)
                    {
                      //dmode[i*2]
                      GfxApiSetDisplayUndocumentedD2(4);
                    GfxApiSetDisplayMux(1);
//for(int xq=0;xq<1;xq++)
int xq=0;
//                      xxxGfxApiVectorScopeLine2(31 +xq+ rotated[i * 4]*scale, 31 + rotated[1 + i * 4]*scale, 31+xq + rotated[i * 4 + 2]*scale, 31 + rotated[i * 4 + 3]*scale);//,
                                          //  dmode[i * 2], 100);

                    }
                  static int subc;
                  //GfxApiSetBlink(counter+subc);

                  //    GfxApiVectorscopeDisplay(x,y);
                  //   GfxApiVectorScopeLine(31,31,x2,y2,1,5);


                  ;
                  counter += 0.05;
                  if (counter == 64)counter = 0, subc++;
                  adder += 0.0001;
                }
              }
              /*
                for(;;)
                {
                if(vsync_flag)
                {
                //    SCREEN_MODE_DRIVER();
                //    GfxApiSetDisplayScroll(0);
                  vsync_flag=0;
                }
                }
              */
              //  GfxApiSetDisplayMux(63);
              //  ets_delay_us(150);

              //  GfxApiSetDisplayMux(0);


            }


          }
          //           GfxApiSetBrightness(cnt>>4);



        }
        {

          int i;
#define RSTART 100
          static int qqq = 256;

          qqq--;
          if (qqq < 100)qqq = 512;

          long long s = millis();
          GfxApiSetDisplayOn(0);
          GfxApiSetInvert(0);
          {
            static u8 xi = 31;


            GfxApiSetInvert(0);
            GfxApiSetDisplayOn(0);

          }
        }
        //     while(millis()<next_frame)
        ;
        //   Display(0, 0, 0,0,console,cat_tiles);

        //  GfxApiSetBrightness(rand()&0xf);
        //      GfxApiSetFreq(0xa0);
        //      GfxApiSetDisplayMux(0);

        drawn++;
      }
      next_frame += milis_frame;
      frames_total--;
      if (!frames_total)
      {
        //        Serial.println(drawn);
        return;
      }

    }
    chunk++;
    s_pos = 0;
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
