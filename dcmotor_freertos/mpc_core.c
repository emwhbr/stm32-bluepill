#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "mpc_core.h"
#include "pid_ctrl.h"
#include "motor.h"
#include "adc.h"
#include "application_freertos_prio.h"

/////////////////////////////////////////////////////////////

// GPIO support
#define DBG_GPIO_RCC      RCC_GPIOB
#define DBG_GPIO_PORT     GPIOB
#define DBG_GPIO_PIN_POS  GPIO7  // Debug signal, actual position within acceptable limits
#define DBG_GPIO_PIN_PID  GPIO8  // Debug signal, time to execute PID calculations

#define DBG_POS_HI  gpio_set(DBG_GPIO_PORT,   DBG_GPIO_PIN_POS)
#define DBG_POS_LO  gpio_clear(DBG_GPIO_PORT, DBG_GPIO_PIN_POS)
#define DBG_POS_TG  gpio_toggle(DBG_GPIO_PORT, DBG_GPIO_PIN_POS)

#define DBG_PID_HI  gpio_set(DBG_GPIO_PORT,   DBG_GPIO_PIN_PID)
#define DBG_PID_LO  gpio_clear(DBG_GPIO_PORT, DBG_GPIO_PIN_PID)
#define DBG_PID_TG  gpio_toggle(DBG_GPIO_PORT, DBG_GPIO_PIN_PID)

// Experiments shows that:
//   - Encoder starts to pulse  6ms after duty change from zero to 50%.
//   - Encoder starts to pulse 10ms after duty change from zero to 100%.
//   - Encoder stops to pulse 180ms after duty change from 100% to zero.

// PID parameters
#define P_GAIN  (  8.40f)
#define I_GAIN  (  0.05f)
#define D_GAIN  ( 48.20f)

#define DUTY_MIN  (-1.0f * motor_pwm_max_duty())
#define DUTY_MAX  (motor_pwm_max_duty())

// motor control state definitions
enum motor_ctrl_state
{
   MOTOR_CTRL_STATE_RESET,
   MOTOR_CTRL_STATE_CLOCKWISE,
   MOTOR_CTRL_STATE_ANTICLOCKWISE,
   MOTOR_CTRL_STATE_STOP
};

// commands sent to PID task
enum pid_task_cmd
{
   PID_TASK_CMD_ENABLE,
   PID_TASK_CMD_DISABLE
};

// Period time of PID task
#define PID_TASK_PERIOD_TIME_MS  (5) // F = 1000 / Tms = 200Hz

// internal driver data (state and synchronization)
struct motor_ctrl_dev
{
   struct pid_ctrl       pid;
   bool                  pid_enable;
   QueueHandle_t         xPIDCmdQueue;
   TaskHandle_t          xPIDTask;
   enum motor_ctrl_state current_motor_ctrl_state;
};

static struct motor_ctrl_dev *g_dev = NULL;

// internal functions
static void mpc_core_gpio_init(void);

static void mpc_core_motor_ctrl_state(float pid_output,
                                      bool *motor_clockwise,
                                      uint16_t *motor_duty);

static void mpc_core_pid_task(__attribute__((unused))void * pvParameters);

/////////////////////////////////////////////////////////////

void mpc_core_initialize(bool zero_shaft)
{
   if (g_dev == NULL)
   {
      mpc_core_gpio_init();

      g_dev = pvPortMalloc(sizeof(struct motor_ctrl_dev));

      // create the PID task command queue
      g_dev->xPIDCmdQueue = xQueueCreate(1, sizeof(enum pid_task_cmd));

      // create the PID task
      xTaskCreate(mpc_core_pid_task, "PID", 250, NULL, TASK_PID_PRIO, &g_dev->xPIDTask);

      g_dev->pid_enable = false;
      g_dev->current_motor_ctrl_state = MOTOR_CTRL_STATE_RESET;
   }

   // disable PID controller
   if (g_dev->pid_enable)
   {
      enum pid_task_cmd cmd = PID_TASK_CMD_DISABLE;
      xQueueSend(g_dev->xPIDCmdQueue, (const void *) &cmd, portMAX_DELAY);

      g_dev->pid_enable = false;
      g_dev->current_motor_ctrl_state = MOTOR_CTRL_STATE_RESET;
   }

   // stop motor
   motor_ctrl(true, 0);

   // set new origo
   if (zero_shaft)
   {
      motor_zero_shaft_position();
   }

   uint32_t now_deg = (motor_get_shaft_position() * 360) / motor_shaft_max_position();

   printf("INIT: deg : %03lu\n", now_deg);

   // intialize PID controller
   pid_ctrl_initialize(&g_dev->pid, P_GAIN, I_GAIN, D_GAIN);
   pid_ctrl_set_output_limits(&g_dev->pid, DUTY_MIN, DUTY_MAX);
   pid_ctrl_set_command_position(&g_dev->pid, 0.0);

   DBG_POS_HI;
   DBG_PID_HI;
}

/////////////////////////////////////////////////////////////

void mpc_core_calibrate(bool zero_shaft)
{
   // disable PID controller
   if (g_dev->pid_enable)
   {
      enum pid_task_cmd cmd = PID_TASK_CMD_DISABLE;
      xQueueSend(g_dev->xPIDCmdQueue, (const void *) &cmd, portMAX_DELAY);

      g_dev->pid_enable = false;
      g_dev->current_motor_ctrl_state = MOTOR_CTRL_STATE_RESET;
   }

   // stop motor
   motor_ctrl(true, 0);

   // set new origo
   if (zero_shaft)
   {
      motor_zero_shaft_position();
   }

   // re-initialize PID controller
   pid_ctrl_initialize(&g_dev->pid, P_GAIN, I_GAIN, D_GAIN);
   pid_ctrl_set_output_limits(&g_dev->pid, DUTY_MIN, DUTY_MAX);

   // update PID set-point, based on ADC value
   uint16_t adc_val = adc_get_value();
   if (adc_val < 125)
   {
      // < 100mV
      adc_val = 0;
   }
   else if ( (adc_val >= 125) && (adc_val <= 3725) )
   {
      // linear range, 100mV - 3000mV
      adc_val = ((adc_val-125) * ADC_MAX_VALUE) / 3600;
   }
   else
   {
      adc_val = ADC_MAX_VALUE;
   }

   uint16_t command_pos = (adc_val * motor_shaft_max_position()) / ADC_MAX_VALUE;
   uint32_t command_deg = (command_pos * 360) / motor_shaft_max_position();
   uint32_t now_deg = (motor_get_shaft_position() * 360) / motor_shaft_max_position();

   printf("CALIB: s-pos : %04u / %04u, s-deg : %03lu, deg : %03lu\n",
          command_pos, motor_shaft_max_position(), command_deg, now_deg);

   pid_ctrl_set_command_position(&g_dev->pid, command_pos);

   DBG_POS_HI;
   DBG_PID_HI;
}

/////////////////////////////////////////////////////////////

void mpc_core_position(void)
{
   // enable PID controller
   if (!g_dev->pid_enable)
   {
      enum pid_task_cmd cmd = PID_TASK_CMD_ENABLE;
      xQueueSend(g_dev->xPIDCmdQueue, (const void *) &cmd, portMAX_DELAY);
      g_dev->pid_enable = true;
      DBG_POS_HI;
      DBG_PID_HI;
   }

   bool neg = false;
   float pos_error = g_dev->pid.m_pos_error;
   if (pos_error < 0)
   {
      neg = true;
      pos_error *= -1.0f;
   }
   printf("POS: %c%04u\n", (neg ? '-' : '+'), (uint16_t)pos_error);
}

/////////////////////////////////////////////////////////////

static void mpc_core_gpio_init(void)
{
   rcc_periph_clock_enable(DBG_GPIO_RCC);

   gpio_set_mode(
      DBG_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      DBG_GPIO_PIN_POS | DBG_GPIO_PIN_PID);

   DBG_POS_HI;
   DBG_PID_HI;
}

/////////////////////////////////////////////////////////////

static void mpc_core_motor_ctrl_state(float pid_output,
                                      bool *motor_clockwise,
                                      uint16_t *motor_duty)
{
   enum motor_ctrl_state next_state = g_dev->current_motor_ctrl_state;

   // determine motor duty and direction
   if (pid_output >= 0.0f)
   {
      *motor_clockwise = true;
      *motor_duty = pid_output;
   }
   else {
      *motor_clockwise = false;
      *motor_duty = (-1.0f * pid_output);
   }

   // determine next state and limit output if necessary
   switch (g_dev->current_motor_ctrl_state)
   {
   case MOTOR_CTRL_STATE_RESET:
      if (*motor_clockwise)
      {
         next_state = MOTOR_CTRL_STATE_CLOCKWISE;
      }
      else
      {
         next_state = MOTOR_CTRL_STATE_ANTICLOCKWISE;
      }
      break;
   case MOTOR_CTRL_STATE_CLOCKWISE:
      // changed direction requires stop-state
      if (*motor_clockwise == false)
      {
         *motor_duty = 0;
         next_state = MOTOR_CTRL_STATE_STOP;
      }
      break;
   case MOTOR_CTRL_STATE_ANTICLOCKWISE:
      // changed direction requires stop-state
      if (*motor_clockwise == true)
      {
         *motor_duty = 0;
         next_state = MOTOR_CTRL_STATE_STOP;
      }
      break;
   case MOTOR_CTRL_STATE_STOP:
      *motor_duty = 0;
      next_state = MOTOR_CTRL_STATE_RESET;
      break;
   }

   // check if transition to new state
   if (next_state != g_dev->current_motor_ctrl_state)
   {
      g_dev->current_motor_ctrl_state = next_state;
   }
}

/////////////////////////////////////////////////////////////

static void mpc_core_pid_task(__attribute__((unused))void * pvParameters)
{
   uint16_t shaft_pos;
   float pid_output;

   bool motor_clockwise;
   uint16_t motor_duty;

   float dbg_pos_error;

   enum pid_task_cmd cmd = PID_TASK_CMD_DISABLE;
   bool pid_enable = false;

   TickType_t xLastWakeTime;

   // ---------------------------------
   // PID controller loop
   // ---------------------------------
   while (1)
   {
      // check if PID controller is enabled
      if (pid_enable)
      {
         // poll command queue
         if (xQueueReceive(g_dev->xPIDCmdQueue,
                           &cmd,
                           0) == pdPASS)
         {
            pid_enable = (cmd == PID_TASK_CMD_DISABLE ? false : true);
         }
      }
      else
      {
         // wait for command
         xQueueReceive(g_dev->xPIDCmdQueue, &cmd, portMAX_DELAY);
         pid_enable = (cmd == PID_TASK_CMD_DISABLE ? false : true);
         if (pid_enable)
         {
            xLastWakeTime = xTaskGetTickCount();
         }
      }

      if (!pid_enable)
      {
         continue; // wait for command to enable
      }
      else
      {
         // get current shaft position
         shaft_pos = motor_get_shaft_position();

         DBG_PID_LO; // start PID calculations

         // update PID
         pid_output = pid_ctrl_update(&g_dev->pid, shaft_pos);

         DBG_PID_HI; // stop PID calculations

         //calculate new motor output, limited if necessary
         mpc_core_motor_ctrl_state(pid_output,
                                   &motor_clockwise,
                                   &motor_duty);

         // apply new output
         motor_ctrl(motor_clockwise, motor_duty);

         // indicate position error with debug pin
         dbg_pos_error = g_dev->pid.m_pos_error;
         if (dbg_pos_error < 0.0f)
         {
            dbg_pos_error *= -1.0f;
         }
         if ((int)dbg_pos_error < 16)
         {
            DBG_POS_LO; // within +/- 16 points, +/-1%
         }
         else
         {
            DBG_POS_HI; // outside allowed position error
         }

         vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PID_TASK_PERIOD_TIME_MS));
      }
   }
}
