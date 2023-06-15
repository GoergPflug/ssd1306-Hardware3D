/*
 * GccApplication18_sqrt_test.c
 *
 * Created: 15.06.2023 12:47:10
 * Author : Lenovo
 */ 

#pragma GCC optimize ("Os")
//#pragma GCC optimize ("Ofast")   708 for 2.0k
//       Os 418......
// debug 420

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <stdfix.h>


#include <ctype.h>




void printstr(char* txt);
void printvectorf(char* txt,float a,float b, float c);

typedef struct s8coord
{
  char x,y;
} s8coord;

typedef union GfxApiCoord
{
  unsigned int asinteger;
  s8coord coord;
} GfxApiCoord;


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
/*
 * 
 * 
 * 
 template<typename T, qualifier Q>
  GLM_FUNC_QUALIFIER mat<4, 4, T, Q> lookAtRH(vec<3, T, Q> const& eye, vec<3, T, Q> const& center, vec<3, T, Q> const& up)
  {
x    vec<3, T, Q> const f(normalize(center - eye));
    vec<3, T, Q> const s(normalize(cross(f, up)));
    vec<3, T, Q> const u(cross(s, f));

    mat<4, 4, T, Q> Result(1);
    Result[0][0] = s.x;
    Result[1][0] = s.y;
    Result[2][0] = s.z;
    Result[0][1] = u.x;
    Result[1][1] = u.y;
    Result[2][1] = u.z;
    Result[0][2] =-f.x;
    Result[1][2] =-f.y;
    Result[2][2] =-f.z;
    Result[3][0] =-dot(s, eye);
    Result[3][1] =-dot(u, eye);
    Result[3][2] = dot(f, eye);
    return Result;
  }

 * 
 */
volatile  static volatile Vector3 cross( Vector3 a,  Vector3 b) {
 volatile Vector3 result;
  result.x = a.y * b.z - a.z * b.y;
  result.y = a.z * b.x - a.x * b.z;
  result.z = a.x * b.y - a.y * b.x;
  return result;
}


volatile
Matrix4  lookAtRH(Vector3 eye, Vector3 center, Vector3 up) {
//Matrix4 * view_matrix=&vmatrix;
volatile Vector3 f = {
  center.x - eye.x,
  center.y - eye.y,
  center.z - eye.z
};
float fLength = 1.0f / bsqrt(f.x * f.x + f.y * f.y + f.z * f.z);
f.x *= fLength;
f.y *= fLength;
f.z *= fLength;

volatile Vector3 s = cross(f, up);
float sLength = 1.0f / bsqrt(s.x * s.x + s.y * s.y + s.z * s.z);
s.x *= sLength;
s.y *= sLength;
s.z *= sLength;

const volatile Vector3 u = cross(s, f);

volatile Matrix4 view_matrix;
memset(&view_matrix, 0, sizeof(view_matrix));

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

return view_matrix;

return view_matrix;
//*/
//  vmatrix=view_matrix;
}
Vector3 transformPoint(Matrix4 matrix, Vector3 point) {
    Vector4 result;
#ifdef DEBUG_TRANSFORM
    printvectorf("trans in:", point.x,point.y,point.z);
#endif
    result.x = matrix.m[0][0] * point.x + matrix.m[1][0] * point.y + matrix.m[2][0] * point.z + matrix.m[3][0];
    result.y = matrix.m[0][1] * point.x + matrix.m[1][1] * point.y + matrix.m[2][1] * point.z + matrix.m[3][1];
    result.z = matrix.m[0][2] * point.x + matrix.m[1][2] * point.y + matrix.m[2][2] * point.z + matrix.m[3][2];
    result.w = matrix.m[0][3] * point.x + matrix.m[1][3] * point.y + matrix.m[2][3] * point.z + matrix.m[3][3];
#ifdef DEBUG_TRANSFORM
    char row[256];
printstr("matrix:");
for(int i=0;i<4;i++){

 int   a=100*matrix.m[0][i];
    
int    b=100*matrix.m[1][i];
    
 int   c=100*matrix.m[2][i];
    
 int   d=100*matrix.m[3][i];
    
    sprintf(row,"%d    %d   %d  %d  %d",a,b,c,d);
    printstr(row);
}
#endif


volatile    Vector3 transformedPoint;

    short accum divider=1.0k/ result.w;
    
    transformedPoint.x = result.x *divider;
    transformedPoint.y = result.y *divider;
    transformedPoint.z = result.z *divider;
//#ifdef DEBUG_TRANSFORM
    printvectorf("matrix mul output:", transformedPoint.x,transformedPoint.y,transformedPoint.z);
//#endif
    return transformedPoint;
}

  Vector3 eye = { 0,0,0};
  Vector3 center = { 0,0,0};
  Vector3 up = { 0.0f, 1.0f, 0.0f };


void set_eye(int x,int y,int z)
{
  eye.x=1;
  eye.y=0;
  eye.z=1;
}

void * GfxApiSetCameraLookat() {
static  Matrix4 view_matrix;
view_matrix= lookAtRH(eye, center, up);

return (void*)&view_matrix;

}
 volatile static  Vector3 transformedPoint;

GfxApiCoord TransformPointD(float x,float y, float z,void *matrix)
  {
    volatile
  Matrix4 *m;
  m=(Matrix4*)matrix;
volatile  Vector3 point = { x,y,z}; // Expected result: (0.00, 0.00, 0.00)
 printvectorf("point to transform:", point.x,point.y,point.z);

  
  transformedPoint= transformPoint(*m, point);
//#ifdef DEBUG_TRANSFORM
  printvectorf("transformed point:", transformedPoint.x,transformedPoint.y,transformedPoint.z);
//#endif
    volatile
  int a=transformedPoint.x;
  volatile
  int b=transformedPoint.y;
  int aa=a,bb=b;


  GfxApiCoord res;
  res.coord.x=a;
  res.coord.y=b;
#ifdef DEBUG_TRANSFORM
  char tmp[32];  
  sprintf(tmp,"outscr %4x",res.asinteger);
  printstr(tmp);
#endif
}
/*    
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

}*/



void print_matrix(void *m)
{
  printstr("matrix");

  Matrix4 * view_matrix=m;
  for(int y=0;y<4;y++)
  {
      
      
      int a=100*view_matrix->m[0][y];
      int b=100*view_matrix->m[1][y];
      int c=100*view_matrix->m[2][y];
      int d=100*view_matrix->m[3][y];
      char tmp[100];
      sprintf(tmp,"row: %d  | %d  %d  %d  %d",y,a,b,c,d);
      printstr(tmp);
  }
  
}
