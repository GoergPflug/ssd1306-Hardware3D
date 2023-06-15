#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <stdfix.h>

#define DTYP short accum
#include <avr/io.h>


#include <avr/io.h>
#include <avr/io.h>

float babylonian_sqrt(float n) {
  float x = n;
  float y = 1.0;
  int i;
  for (i = 0; i < 4; i++) {
    x = (x + y) / 2;
    y = n / x;
  }
  return x;
}

#include <stdint.h>

int32_t sqrt24(int32_t x) {
    int32_t result;
    asm volatile (
        "movw r24,%1\n"
        "movw r16,%2\n"
        "call sqrt24\n"
        "movw %0,r24\n"
        : "=r" (result)
        : "r" ((int16_t)(x >> 8)), "r" ((int16_t)(x & 0xFFFF))
    );
    return result;
}

asm (
    ".global sqrt24\n"
    "sqrt24:\n"
    "mov r16,r23\n"
    "mov r17,r24\n"
    "mov r18,r25\n"
    "ldi r19,12\n"
    "clr r25\n"
    "sqrt_loop:\n"
    "lsl r16\n"
    "rol r17\n"
    "rol r18\n"
    "subi r16,-128\n"
    "sbci r17,-1\n"
    "sbci r18,-1\n"
    "brcc sqrt_skip\n"
    "addi r16,128\n"
    "adci r17,0\n"
    "adci r18,0\n"
    "sqrt_skip:\n"
    "rol r25\n"
    "rol r24\n"
    "rol r23\n"
    "subi r16,-128\n"
    "sbci r17,-1\n"
    "sbci r18,-1\n"
    "brcc sqrt_done\n"
    "ori r25,1\n"
    "sqrt_done:\n"
    "dec r19\n"
    "brne sqrt_loop\n"
    "ret\n");



void cmain(char *obuf) {

  float f=root(2);

  int a=f*100;
  
	sprintf(obuf,"Point transformed: %d",a);
	printf("Expected result:   0.00, 0.00, 0.00\n");

}
