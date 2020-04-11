/* 
  From https://raw.githubusercontent.com/KhronosGroup/OpenGL-Registry/
      d62c37dde0a40148aecc9e9701ba0ae4ab83ee22/extensions/EXT/
       EXT_texture_shared_exponent.txt
*/

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define __LITTLE_ENDIAN  1
#define __BIG_ENDIAN     2

#ifdef _WIN32
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif

#define RGB9E5_EXPONENT_BITS          5
#define RGB9E5_MANTISSA_BITS          9
#define RGB9E5_EXP_BIAS               15
#define RGB9E5_MAX_VALID_BIASED_EXP   31

#define MAX_RGB9E5_EXP               (RGB9E5_MAX_VALID_BIASED_EXP - RGB9E5_EXP_BIAS)
#define RGB9E5_MANTISSA_VALUES       (1<<RGB9E5_MANTISSA_BITS)
#define MAX_RGB9E5_MANTISSA          (RGB9E5_MANTISSA_VALUES-1)
#define MAX_RGB9E5                   (((float)MAX_RGB9E5_MANTISSA)/RGB9E5_MANTISSA_VALUES * (1<<MAX_RGB9E5_EXP))
#define EPSILON_RGB9E5               ((1.0/RGB9E5_MANTISSA_VALUES) / (1<<RGB9E5_EXP_BIAS))

typedef struct {
#ifdef __BYTE_ORDER
#if __BYTE_ORDER == __BIG_ENDIAN
  unsigned int negative:1;
  unsigned int biasedexponent:8;
  unsigned int mantissa:23;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
  unsigned int mantissa:23;
  unsigned int biasedexponent:8;
  unsigned int negative:1;
#endif
#endif
} BitsOfIEEE754;

typedef union {
  unsigned int raw;
  float value;
  BitsOfIEEE754 field;
} float754;

typedef struct {
#ifdef __BYTE_ORDER
#if __BYTE_ORDER == __BIG_ENDIAN
  unsigned int biasedexponent:RGB9E5_EXPONENT_BITS;
  unsigned int b:RGB9E5_MANTISSA_BITS;
  unsigned int g:RGB9E5_MANTISSA_BITS;
  unsigned int r:RGB9E5_MANTISSA_BITS;
#elif __BYTE_ORDER == __LITTLE_ENDIAN
  unsigned int r:RGB9E5_MANTISSA_BITS;
  unsigned int g:RGB9E5_MANTISSA_BITS;
  unsigned int b:RGB9E5_MANTISSA_BITS;
  unsigned int biasedexponent:RGB9E5_EXPONENT_BITS;
#endif
#endif
} BitsOfRGB9E5;

typedef union {
  unsigned int raw;
  BitsOfRGB9E5 field;
} rgb9e5;

float ClampRange_for_rgb9e5(float x)
{
  if (x > 0.0) {
    if (x >= MAX_RGB9E5) {
      return MAX_RGB9E5;
    } else {
      return x;
    }
  } else {
    /* NaN gets here too since comparisons with NaN always fail! */
    return 0.0;
  }
}

float MaxOf3(float x, float y, float z)
{
  if (x > y) {
    if (x > z) {
      return x;
    } else {
      return z;
    }
  } else {
    if (y > z) {
      return y;
    } else {
      return z;
    }
  }
}

/* Ok, FloorLog2 is not correct for the denorm and zero values, but we
   are going to do a max of this value with the minimum rgb9e5 exponent
   that will hide these problem cases. */
int FloorLog2(float x)
{
  float754 f;

  f.value = x;
  return (f.field.biasedexponent - 127);
}

int Max(int x, int y)
{
  if (x > y) {
    return x;
  } else {
    return y;
  }
}

rgb9e5 float3_to_rgb9e5(const float rgb[3])
{
  rgb9e5 retval;
  float maxrgb;
  int rm, gm, bm;
  float rc, gc, bc;
  int exp_shared;
  double denom;

  rc = ClampRange_for_rgb9e5(rgb[0]);
  gc = ClampRange_for_rgb9e5(rgb[1]);
  bc = ClampRange_for_rgb9e5(rgb[2]);

  maxrgb = MaxOf3(rc, gc, bc);
  exp_shared = Max(-RGB9E5_EXP_BIAS-1, FloorLog2(maxrgb)) + 1 + RGB9E5_EXP_BIAS;
  assert(exp_shared <= RGB9E5_MAX_VALID_BIASED_EXP);
  assert(exp_shared >= 0);
  /* This pow function could be replaced by a table. */
  denom = pow(2, exp_shared - RGB9E5_EXP_BIAS - RGB9E5_MANTISSA_BITS);

  maxm = (int) floor(maxrgb / denom + 0.5);
  if (maxm == MAX_RGB9E5_MANTISSA+1) {
    denom *= 2;
    exp_shared += 1;
    assert(exp_shared <= RGB9E5_MAX_VALID_BIASED_EXP);
  } else {
    assert(maxm <= MAX_RGB9E5_MANTISSA);
  }

  rm = (int) floor(rc / denom + 0.5);
  gm = (int) floor(gc / denom + 0.5);
  bm = (int) floor(bc / denom + 0.5);

  assert(rm <= MAX_RGB9E5_MANTISSA);
  assert(gm <= MAX_RGB9E5_MANTISSA);
  assert(bm <= MAX_RGB9E5_MANTISSA);
  assert(rm >= 0);
  assert(gm >= 0);
  assert(bm >= 0);

  retval.field.r = rm;
  retval.field.g = gm;
  retval.field.b = bm;
  retval.field.biasedexponent = exp_shared;

  return retval;
}

void rgb9e5_to_float3(rgb9e5 v, float retval[3])
{
  int exponent = v.field.biasedexponent - RGB9E5_EXP_BIAS - RGB9E5_MANTISSA_BITS;
  float scale = (float) pow(2, exponent);

  retval[0] = v.field.r * scale;
  retval[1] = v.field.g * scale;
  retval[2] = v.field.b * scale;
}

