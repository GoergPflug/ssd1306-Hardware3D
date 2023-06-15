/*
 * GccApplication18_sqrt_test.c
 *
 * Created: 15.06.2023 12:47:10
 * Author : Lenovo
 */ 

#pragma GCC optimize ("Ofast")
//#pragma GCC optimize ("Ofast")   708 for 2.0k
//       Os 418......
// debug 420

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <stdfix.h>

#include "asm_sqrt.h"

#include <ctype.h>

typedef union fixedunion
{
  unsigned char b[2];
  unsigned int i;
  short accum sa;
}fixedunion;
short accum bsqrt(short accum xx) {
  
  unsigned int *ip=(unsigned int *)&xx;
  uint16_t asuint =*ip;  // 7 bit
  __uint24 x=*ip;
  x*=128;
  __uint24 res = 0;
  __uint24 bit = ((__uint24)1) << 22; // The second-to-top bit is set: 1 << 30 for 32 bits

  // "bit" starts at the highest power of four <= the argument.
  while (bit > x) {
    bit >>= 2;
  }

  while (bit != 0) {
    if (x >= res + bit) {
      x -= res + bit;
      res = (res >> 1) + bit;
    }
    else {
      res >>= 1;
    }
    bit >>= 2;
  }
  fixedunion f;
  f.i=res;
  return f.sa;
}
int testroot(void)
{ 
 static volatile short accum c;
 static int a;
 a+=sqrt_29_large(0x200*128);
//  c+=bsqrt(2);
}



#define DTYP short accum

typedef struct {
  DTYP x;
  DTYP y;
  DTYP z;
} Vector3;

typedef struct {
  DTYP x;
  DTYP y;
  DTYP z;
  DTYP w;
} Vector4;


typedef struct {
  DTYP m[4][4];
} Matrix4;


static Matrix4 view_matrix;

void lookAtRH(Vector3 eye, Vector3 center, Vector3 up) {
  Vector3 f = {
    center.x - eye.x,
    center.y - eye.y,
    center.z - eye.z
  };
  DTYP fLength = 1.0k/bsqrt(f.x * f.x + f.y * f.y + f.z * f.z);
  f.x *= fLength;
  f.y *= fLength;
  f.z *= fLength;

  Vector3 s = {
    f.z,
    0.0f,
    -f.x
  };
  DTYP sLength = 1.0k/bsqrt(s.x * s.x + s.y * s.y + s.z * s.z);
  s.x *= sLength;
  s.y *= sLength;
  s.z *= sLength;

  Vector3 u = {
    s.y * f.z - s.z * f.y,
    s.z * f.x - s.x * f.z,
    s.x * f.y - s.y * f.x
  };

  view_matrix.m[0][0] = s.x;
  view_matrix.m[1][0] = s.y;
  view_matrix.m[2][0] = s.z;
  view_matrix.m[0][1] = u.x;
  view_matrix.m[1][1] = u.y;
  view_matrix.m[2][1] = u.z;
  view_matrix.m[0][2] = -f.x;
  view_matrix.m[1][2] = -f.y;
  view_matrix.m[2][2] = -f.z;
  view_matrix.m[3][0] = -s.x * eye.x - s.y * eye.y - s.z * eye.z;
  view_matrix.m[3][1] = -u.x * eye.x - u.y * eye.y - u.z * eye.z;
  view_matrix.m[3][2] = f.x * eye.x + f.y * eye.y + f.z * eye.z;
  view_matrix.m[0][3] = 0.0f;
  view_matrix.m[1][3] = 0.0f;
  view_matrix.m[2][3] = 0.0f;
  view_matrix.m[3][3] = 1.0f;

}
Vector3 transformPoint(Matrix4 matrix, Vector3 point) {
    Vector4 result;
    result.x = matrix.m[0][0] * point.x + matrix.m[1][0] * point.y + matrix.m[2][0] * point.z + matrix.m[3][0];
    result.y = matrix.m[0][1] * point.x + matrix.m[1][1] * point.y + matrix.m[2][1] * point.z + matrix.m[3][1];
    result.z = matrix.m[0][2] * point.x + matrix.m[1][2] * point.y + matrix.m[2][2] * point.z + matrix.m[3][2];
    result.w = matrix.m[0][3] * point.x + matrix.m[1][3] * point.y + matrix.m[2][3] * point.z + matrix.m[3][3];

    Vector3 transformedPoint;

    short accum divider=1;//1.0k/ result.w;
    
    transformedPoint.x = result.x *divider;
    transformedPoint.y = result.y *divider;
//    transformedPoint.z = result.z / result.w;

    return transformedPoint;
}

void test_ook() {
  Vector3 eye = { 0.0f, 0.0f, 0.0f };
  Vector3 center = { 2.0f, 0.0f, 0.0f };
  Vector3 up = { 0.0f, 1.0f, 0.0f };
  
  lookAtRH(eye, center, up);
}
 static  Vector3 transformedPoint;

unsigned int * test_transform()
{
  
static   Vector3 point = { 3.0f, 1.0f, -1.0f }; // Expected result: (0.00, 0.00, 0.00)

 transformedPoint= transformPoint(view_matrix, point);

  return (int*)&transformedPoint;
}

void xcmain(char *obuf) {
  Vector3 eye = { 0.0f, 0.0f, 0.0f };
  Vector3 center = { 2.0f, 0.0f, 0.0f };
  Vector3 up = { 0.0f, 1.0f, 0.0f };
  
  lookAtRH(eye, center, up);

  // Example point
  Vector3 point = { 3.0f, 1.0f, -1.0f }; // Expected result: (0.00, 0.00, 0.00)

  // Transform the point
  Vector3 transformedPoint = transformPoint(view_matrix, point);

  // Print the transformed point and the expected result
  int a,b,c;
  
  a=100*  transformedPoint.x,b=100* transformedPoint.y, 100* transformedPoint.z;

  //accum aa=sqrt_fixed_sa(8.0);

 // int aaa=aa*10;


  sprintf(obuf,"Point transformed: %d, %d, %d",a,b,c);
 // printf("Expected result:   0.00, 0.00, 0.00\n");

}
