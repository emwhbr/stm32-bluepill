#include <stdio.h>

#include "fix.h"

//////////////////////////////////////////////////////////////

// Q31, 31 fractional bits
#define QN  31

// 64-bit fractional data type in 1.63 format (Q63)
typedef int64_t q63_t;

#define TOFIX(f) ( (int32_t)( (f) * (float)(1UL<<(QN))) )
#define TOFLT(q) ( (float)(q) / (float)(1UL<<(QN)) )

//////////////////////////////////////////////////////////////

static inline fix_t sat_q63_to_fix(q63_t x)
{
   return ( (fix_t)(x >> 32) != ((fix_t)x >> QN) ?
      (FIX_MAX ^ ((fix_t)(x >> 63))) : (fix_t)x );
}

//////////////////////////////////////////////////////////////

fix_t fix_to_fix(float f)
{
   float temp = f * (float)(1UL<<(QN));

   // round
   temp += (f > 0 ? 0.5f : -0.5f);

   // saturate
   return sat_q63_to_fix((q63_t)temp);
}

//////////////////////////////////////////////////////////////

float fix_to_float(fix_t q)
{
   return TOFLT(q);
}

//////////////////////////////////////////////////////////////

fix_t fix_add_sat(fix_t q1, fix_t q2)
{
   return sat_q63_to_fix((q63_t)q1 + (q63_t)q2);
}

//////////////////////////////////////////////////////////////

fix_t fix_sub_sat(fix_t q1, fix_t q2)
{
   return sat_q63_to_fix((q63_t)q1 - (q63_t)q2);
}

//////////////////////////////////////////////////////////////

fix_t fix_mul_sat(fix_t q1, fix_t q2)
{
   return sat_q63_to_fix( ((q63_t)q1 * (q63_t)q2) >> QN );
}

//////////////////////////////////////////////////////////////

fix_t fix_abs_sat(fix_t q)
{
   if (FIX_POSITIVE(q))
   {
      return q;
   }
   else
   {
      return fix_sub_sat(0, q);
   }
}
