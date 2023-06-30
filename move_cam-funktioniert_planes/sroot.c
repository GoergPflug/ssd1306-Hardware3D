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

#include <stdfix.h>


#include <ctype.h>


int from_sa(short accum in)
{
  accum c=in;
  
  return c*128;
}

void printstr(char* txt);
void printvectorf(char* txt,float a,float b, float c);



typedef union fixedunion
{
  unsigned char b[2];
  unsigned int i;
  short accum sa;
}fixedunion;
short accum bsqrt(short accum xx) {
  
  unsigned int *ip=(unsigned int *)&xx;
  unsigned int asuint =*ip;  // 7 bit
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

  static void cross( Vector3 *a,  Vector3 *b, Vector3* result ) {
  
  result->x = a->y * b->z - a->z * b->y;
  result->y = a->z * b->x - a->x * b->z;
  result->z = a->x * b->y - a->y * b->x;
  
}

static Matrix4 view_matrix;
  
static void  lookAtRH(Vector3 eye, Vector3 center, Vector3 up) {
//Matrix4 * view_matrix=&vmatrix;
  volatile Vector3 f;
  volatile Vector3 s;   // fixes gcc compiler problem...
  short accum Length;
  
  volatile  Vector3 u; 
  f.x=  center.x - eye.x,
  f.y= center.y - eye.y,
  f.z= center.z - eye.z;
  
  Length = 1.0k / bsqrt(f.x * f.x + f.y * f.y + f.z * f.z);
  f.x *= Length;
  f.y *= Length;
  f.z *= Length;
  
  cross(&f, &up,&s);
  Length = 1.0k / bsqrt(s.x * s.x + s.y * s.y + s.z * s.z);
  s.x *= Length;
  s.y *= Length;
  s.z *= Length;
  
  cross(&s, &f,&u);
  
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
  view_matrix.m[0][3] = 0.0k;
  view_matrix.m[1][3] = 0.0k;
  view_matrix.m[2][3] = 0.0k;
  view_matrix.m[3][3] = 1.0k;
 }

void printfloat(float f);
 
static Vector3 transformPoint(Matrix4 matrix, Vector3 point) {
short accum result_x,result_y,result_z;
   
    result_x = matrix.m[0][0] * point.x + matrix.m[1][0] * point.y + matrix.m[2][0] * point.z + matrix.m[3][0]*1.0k;  // all
    result_y = matrix.m[0][1] * point.x + matrix.m[1][1] * point.y + matrix.m[2][1] * point.z + matrix.m[3][1]*1.0k;
   
    result_z = matrix.m[0][2] * point.x ;
        result_z          += matrix.m[1][2] * point.y ;
        result_z      += matrix.m[2][2] * point.z ;
      result_z              += matrix.m[3][2]*1.0k;
   
    short accum w =(matrix.m[0][3] * point.x + matrix.m[1][3] * point.y + matrix.m[2][3] * point.z  + matrix.m[3][3]*1.0k);
    Vector3 v={result_x,result_y,result_z};
    return v;
}

  Vector3 eye = { 0,0,0};
  Vector3 center = { 0,0,0};
  Vector3 up = { 0.0f, 1.0f, 0.0f };
  Vector3 world_center ={0,0,0};

void setCamera(int ex,int ey,int ez,int cx,int cy, int cz)
{
  fixedunion f;

  f.i=ex;
  eye.x=f.sa;
  f.i=ey;
  eye.y=f.sa;
  f.i=ez;
  eye.z=f.sa;


  f.i=cx;
  center.x=f.sa;
  f.i=cy; 
  center.y=f.sa,
  f.i=cz;
  center.z=f.sa;
  
}
void * GfxApiSetCameraLookat() {
  lookAtRH(eye, center, up);
  
  return (void*)&view_matrix;
}
unsigned char TransformPointD(int x,int y, int z,void *matrix,int *xx,int *yy)
  {
//    unsigned char is_valid=1;
  Matrix4 *m;
  m=(Matrix4*)matrix;
  Vector3 point;
  int *ip=(int*)&point;
  ip[0]=x;ip[1]=y;ip[2]=z;

  point.x+=world_center.x;
  point.z+=world_center.y;
  point.y+=world_center.z;
 
  Vector3 transformedPoint=transformPoint(*m, point);

  short accum inverse=127.0k/(transformedPoint.z-10k);

  transformedPoint.x*=inverse;
  
  transformedPoint.y*=inverse;


  *xx=transformedPoint.x;
  *yy=transformedPoint.y;
  return transformedPoint.z<0?0:1;
}


void print_matrix(void *m)
{
  return;
  printstr("matrix");

  Matrix4 * view_matrix=m;
  for(int y=0;y<4;y++)
  {
      
      
      int a=100*view_matrix->m[y][0];
      int b=100*view_matrix->m[y][1];
      int c=100*view_matrix->m[y][2];
      int d=100*view_matrix->m[y][3];
      char tmp[100];
      sprintf(tmp,"row: %d  | %d  %d  %d  %d",y,a,b,c,d);
      printstr(tmp);
  }
  
}
