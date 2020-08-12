#ifndef __FIX_H__
#define __FIX_H__

#include <stdint.h>

// fixed point data type in 1.31 format (Q31)
typedef int32_t fix_t;

// min/max values
#define FIX_MAX  (0x7fffffff)
#define FIX_MIN  (0x80000000)

// check if sign bit is set
#define FIX_POSITIVE(q)  ( (q & FIX_MIN) == 0 )

// conversion functions between fixed point and float/integer
fix_t fix_to_fix(float f);
float fix_to_float(fix_t q);

// arithmetic functions (uses saturating)
fix_t fix_add_sat(fix_t q1, fix_t q2);
fix_t fix_sub_sat(fix_t q1, fix_t q2);
fix_t fix_mul_sat(fix_t q1, fix_t q2);
fix_t fix_abs_sat(fix_t q);

#endif // __FIX_H__
