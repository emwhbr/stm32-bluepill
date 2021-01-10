#ifndef __DEBUG_PIN_H__
#define __DEBUG_PIN_H__

#include <stdbool.h>

typedef enum
{
   DEBUG_PIN1,
   DEBUG_PIN2,
   DEBUG_PIN3,
   DEBUG_PIN4
} debug_pin_t;

void debug_pin_init(void);

void debug_pin_set_value(debug_pin_t pin, bool value);

#define DBG_P1_HI debug_pin_set_value(DEBUG_PIN1, true)
#define DBG_P1_LO debug_pin_set_value(DEBUG_PIN1, false)

#define DBG_P2_HI debug_pin_set_value(DEBUG_PIN2, true)
#define DBG_P2_LO debug_pin_set_value(DEBUG_PIN2, false)

#define DBG_P3_HI debug_pin_set_value(DEBUG_PIN3, true)
#define DBG_P3_LO debug_pin_set_value(DEBUG_PIN3, false)

#define DBG_P4_HI debug_pin_set_value(DEBUG_PIN4, true)
#define DBG_P4_LO debug_pin_set_value(DEBUG_PIN4, false)

#endif // __DEBUG_PIN_H__
