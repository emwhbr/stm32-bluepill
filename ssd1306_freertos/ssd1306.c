#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include <FreeRTOS.h>
#include <task.h>

#include "ssd1306.h"

//
// Implementation notes:
// FreeRTOS must have been enabled before using this driver.
//
// References:
// [1] Referece manual STM32F103xx, RM0008
//     https://www.st.com
//
// [2] SSD1306 Advance information
//     http://www.solomon-systech.com
// 
// [3] UG-2864HSWEG01 128x64 Evaluation Kit User Guide
//     https://www.adafruit.com
//     James_Wang@univision.com.tw
//

/////////////////////////////////////////////////////////////

// GPIO
#define SSD1306_GPIO_RCC       RCC_GPIOC
#define SSD1306_GPIO_PORT      GPIOC
#define SSD1306_GPIO_PIN_RST   GPIO15  // Reset signal pin
#define SSD1306_GPIO_PIN_DC    GPIO14  // Data/Command control pin

#define SSD1306_RST_HI  gpio_set(SSD1306_GPIO_PORT, SSD1306_GPIO_PIN_RST)
#define SSD1306_RST_LO  gpio_clear(SSD1306_GPIO_PORT, SSD1306_GPIO_PIN_RST)

#define SSD1306_DC_HI  gpio_set(SSD1306_GPIO_PORT, SSD1306_GPIO_PIN_DC)
#define SSD1306_DC_LO  gpio_clear(SSD1306_GPIO_PORT, SSD1306_GPIO_PIN_DC)

// SPI
#define SSD1306_SPI       SPI1
#define SSD1306_SPI_RCC   RCC_SPI1
#define SSD1306_SPI_PORT  GPIOA
#define SSD1306_SPI_NSS   GPIO4
#define SSD1306_SPI_SCK   GPIO5
#define SSD1306_SPI_MOSI  GPIO7

// internal functions
static void ssd1306_gpio_init(void);
static void ssd1306_hw_reset(void);
static void ssd1306_spi_init(void);
static void ssd1306_spi_write(uint8_t val);
static void ssd1306_write_command(uint8_t cmd);
static void ssd1306_write_data(uint8_t data);

/////////////////////////////////////////////////////////////
// Fundamental Command Table
/////////////////////////////////////////////////////////////
#define SSD1306_SET_CONTRAST                             0x81

#define SSD1306_DISPLAY_ON_RAM                           0xA4
#define SSD1306_DISPLAY_ON_IGNORE                        0xA5

#define SSD1306_DISPLAY_NORMAL                           0xA6
#define SSD1306_DISPLAY_INVERT                           0xA7

#define SSD1306_DISPLAY_OFF                              0xAE
#define SSD1306_DISPLAY_ON                               0xAF

/////////////////////////////////////////////////////////////
// Scrolling Command Table
/////////////////////////////////////////////////////////////
#define SSD1306_DEACTIVATE_SCROLL                        0x2E

/////////////////////////////////////////////////////////////
// Address Setting Command Table
/////////////////////////////////////////////////////////////
#define SSD1306_SET_MEMORY_MODE                          0x20
#define SSD1306_MEMORY_MODE_HORIZONTAL                   0x00
#define SSD1306_MEMORY_MODE_VERTICAL                     0x01
#define SSD1306_MEMORY_MODE_PAGE                         0x02

// Only for horizontal or vertical addressing mode
#define SSD1306_COLUMN_ADDR                              0x21
#define SSD1306_SET_PAGE_ADDR                            0x22

// Only for page addressing mode
#define SSD1306_SET_LO_COL_START                         0x00 // low nibble  : 0x00 - 0xFF
#define SSD1306_SET_HI_COL_START                         0x10 // high nibble : 0x10 - 0x1F
#define SSD1306_SET_PAGE_START                           0xB0

/////////////////////////////////////////////////////////////
// Hardware Configuration Command Table
/////////////////////////////////////////////////////////////
#define SSD1306_SET_STARTLINE                            0x40

#define SSD1306_SEG_REMAP_NORMAL                         0xA0
#define SSD1306_SEG_REMAP_INV                            0xA1

#define SSD1306_SET_MULTIPLEX                            0xA8

#define SSD1306_COM_SCAN_DIR_INC                         0xC0
#define SSD1306_COM_SCAN_DIR_DEC                         0xC8

#define SSD1306_SET_DISPLAY_OFFSET                       0xD3

#define SSD1306_SET_COM_PINS                             0xDA

/////////////////////////////////////////////////////////////
// Timing & Driving Scheme Setting Command Table
/////////////////////////////////////////////////////////////
#define SSD1306_SET_DISPLAY_CLOCKDIV                     0xD5
#define SSD1306_SET_PRECHARGE_PERIOD                     0xD9

#define SSD1306_SET_VCOM_DESELECT                        0xDB

#define SSD1306_NOP                                      0xE3

/////////////////////////////////////////////////////////////
// Charge Pump Command Table
/////////////////////////////////////////////////////////////
#define SSD1306_CHARGEPUMP                               0x8D
#define SSD1306_CHARGEPUMP_ON                            0x14
#define SSD1306_CHARGEPUMP_OFF                           0x10

// Initialization command sequence
static const uint8_t ssd1306_init_commands[]=
{
   // Fundamental Commands
   SSD1306_DISPLAY_OFF,

   SSD1306_SET_CONTRAST, 0x7F,
   SSD1306_DISPLAY_NORMAL,
   SSD1306_DISPLAY_ON_RAM,

   // Scrolling
   SSD1306_DEACTIVATE_SCROLL,

   // Addressing
   SSD1306_SET_MEMORY_MODE, SSD1306_MEMORY_MODE_PAGE,

   // Hardware Configuration
   SSD1306_SEG_REMAP_INV,
   SSD1306_SET_MULTIPLEX,      0x3F,
   SSD1306_COM_SCAN_DIR_DEC,
   SSD1306_SET_DISPLAY_OFFSET, 0x00,
   SSD1306_SET_COM_PINS,       0x12,

   // Timing & Driving Scheme Setting
   SSD1306_SET_DISPLAY_CLOCKDIV, 0x80,
   SSD1306_SET_PRECHARGE_PERIOD, 0x22,
   SSD1306_SET_VCOM_DESELECT,    0x20,

   // Charge Pump
   SSD1306_CHARGEPUMP, SSD1306_CHARGEPUMP_ON,

   // Turn on screen
   SSD1306_DISPLAY_ON,
};

/////////////////////////////////////////////////////////////

static void ssd1306_gpio_init(void)
{
   rcc_periph_clock_enable(SSD1306_GPIO_RCC);

   gpio_set_mode(
      SSD1306_GPIO_PORT,
      GPIO_MODE_OUTPUT_2_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      SSD1306_GPIO_PIN_RST | SSD1306_GPIO_PIN_DC);

   SSD1306_RST_HI;
   SSD1306_DC_HI;
}

/////////////////////////////////////////////////////////////

static void ssd1306_hw_reset(void)
{
   // according to ref[2] : reset shall be kept low for at least 3us
   SSD1306_RST_LO;
   vTaskDelay(pdMS_TO_TICKS(1));
   SSD1306_RST_HI;

   // according to ref[2] : SEG/COM will be on after 100ms
   vTaskDelay(pdMS_TO_TICKS(100));
}

/////////////////////////////////////////////////////////////

static void ssd1306_spi_init(void)
{
   rcc_periph_clock_enable(SSD1306_SPI_RCC);

   spi_reset(SSD1306_SPI);

   gpio_set_mode(
      SSD1306_SPI_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
      SSD1306_SPI_NSS | SSD1306_SPI_SCK | SSD1306_SPI_MOSI
   );

   // SPI1 uses APB2 bus,, we assume SYSCLK = 72MHz, APB2 prescaler = 1
   // SPI clock cycle according to ref[2] : 10ns => Fmax = 10 MHz
   //
   // SPI prescaler    SPI-frequency (MHz)
   //     64               1.125
   //     32               2.250
   //     16               4.500
   //      8               9.000
   spi_init_master(
      SSD1306_SPI,
      SPI_CR1_BAUDRATE_FPCLK_DIV_8,
      SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1,
      SPI_CR1_DFF_8BIT,
      SPI_CR1_MSBFIRST
   );

   spi_set_bidirectional_transmit_only_mode(SSD1306_SPI);
   spi_disable_software_slave_management(SSD1306_SPI);
   spi_enable_ss_output(SSD1306_SPI);
}

/////////////////////////////////////////////////////////////

static void ssd1306_spi_write(uint8_t val)
{
   spi_enable(SSD1306_SPI);

   spi_send(SSD1306_SPI, val);

   while (SPI_SR(SSD1306_SPI) & SPI_SR_BSY);

   spi_disable(SSD1306_SPI);
}

/////////////////////////////////////////////////////////////

static void ssd1306_write_command(uint8_t cmd)
{
   SSD1306_DC_LO;
   ssd1306_spi_write(cmd);
}

/////////////////////////////////////////////////////////////

static void ssd1306_write_data(uint8_t data)
{
   SSD1306_DC_HI;
   ssd1306_spi_write(data);

   //vTaskDelay(pdMS_TO_TICKS(1));
}

/////////////////////////////////////////////////////////////

void ssd1306_init(void)
{
   // initialize hardware
   ssd1306_gpio_init();
   ssd1306_spi_init();

   // reset OLED
   ssd1306_hw_reset();

   // initialization sequence
   for (size_t i=0; i < sizeof(ssd1306_init_commands); ++i)
   {
      ssd1306_write_command(ssd1306_init_commands[i]);
   }
}

/////////////////////////////////////////////////////////////

void ssd1306_set_contrast(uint8_t val)
{
   ssd1306_write_command(SSD1306_SET_CONTRAST);
   ssd1306_write_command(val);
}

/////////////////////////////////////////////////////////////

void ssd1306_invert(uint8_t val)
{
   if (val)
      ssd1306_write_command(SSD1306_DISPLAY_INVERT);
   else
      ssd1306_write_command(SSD1306_DISPLAY_NORMAL);
}

/////////////////////////////////////////////////////////////

void ssd1306_set_pixel(uint8_t *buf, uint16_t buf_w, uint16_t x, uint16_t y, SSD1306_COLOR color)
{
   uint16_t byte_idx = x + (y / 8) * buf_w;

   switch(color)
   {
      case SSD1306_COLOR_WHITE:
         buf[byte_idx] |=  (1 << (y & 7));
         break;
      case SSD1306_COLOR_BLACK:
         buf[byte_idx] &= ~(1 << (y & 7));
         break;
      case SSD1306_COLOR_SWAP:
         buf[byte_idx] ^=  (1 << (y & 7));
         break;
   }
}

/////////////////////////////////////////////////////////////

SSD1306_COLOR ssd1306_get_pixel(const uint8_t *buf, uint16_t buf_w, uint16_t x, uint16_t y)
{
   uint8_t byte = buf[x + (y / 8) * buf_w];

   return (byte & (1 << (y & 7)) ? SSD1306_COLOR_WHITE : SSD1306_COLOR_BLACK);
}

/////////////////////////////////////////////////////////////

void ssd1306_flush(const uint8_t *buf,
                   uint16_t x1, uint16_t y1,
                   uint16_t x2, uint16_t y2)
{
   uint8_t page_start = y1 / 8;
   uint8_t page_end   = y2 / 8;

   for(uint8_t page=page_start; page <= page_end; page++)
   {
      // set start page and column
      ssd1306_write_command(SSD1306_SET_PAGE_START | page);
      ssd1306_write_command(SSD1306_SET_LO_COL_START | (x1 & 0x0f) ) ;
      ssd1306_write_command(SSD1306_SET_HI_COL_START | ((x1 >> 4) & 0x0f) );

      // send data for page
      for(uint16_t col=x1; col <= x2; col++)
      {
         ssd1306_write_data(*buf++);
      }
    }
}
