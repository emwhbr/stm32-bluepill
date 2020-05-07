#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "enc28j60.h"
#include "enc28j60_defs.h"
#include "dwt_delay.h"

//
// Implementation notes:
// TBD
//
// References:
// [1] ENC28J60 Data sheet, DS39662E
//     https://www.microchip.com
//
// [2] ENC28J60 Rev.B7 Silicon Errata, DS80349C
//     https://www.microchip.com
// 
// [3] ENC28J60-H User's manual, ENC28J60-H.pdf
//     https://www.olimex.com
//

/////////////////////////////////////////////////////////////

// GPIO support
#define ENC28J60_GPIO_RCC       RCC_GPIOA
#define ENC28J60_GPIO_PORT      GPIOA
#define ENC28J60_GPIO_PIN_RST   GPIO11  // Reset signal
#define ENC28J60_GPIO_PIN_INT   GPIO12  // Interrupt signal

#define ENC28J60_RST_HI  gpio_set(ENC28J60_GPIO_PORT,   ENC28J60_GPIO_PIN_RST)
#define ENC28J60_RST_LO  gpio_clear(ENC28J60_GPIO_PORT, ENC28J60_GPIO_PIN_RST)

// SPI support
#define ENC28J60_SPI            SPI2
#define ENC28J60_SPI_RCC        RCC_SPI2
#define ENC28J60_SPI_GPIO_RCC   RCC_GPIOB
#define ENC28J60_SPI_GPIO_PORT  GPIOB
#define ENC28J60_SPI_NSS        GPIO12
#define ENC28J60_SPI_SCK        GPIO13
#define ENC28J60_SPI_MISO       GPIO14
#define ENC28J60_SPI_MOSI       GPIO15
#define ENC28J60_SPI_DR         SPI2_DR

#define ENC28J60_SPI_NSS_HI  gpio_set(ENC28J60_SPI_GPIO_PORT,    ENC28J60_SPI_NSS)
#define ENC28J60_SPI_NSS_LO  gpio_clear(ENC28J60_SPI_GPIO_PORT,  ENC28J60_SPI_NSS)

#define ENC28J60_SPI_CMD_RCR  (0b00000000) // read control register
#define ENC28J60_SPI_CMD_RBM  (0b00111010) // read buffer memory
#define ENC28J60_SPI_CMD_WCR  (0b01000000) // write control register
#define ENC28J60_SPI_CMD_WBM  (0b01111010) // write buffer memory
#define ENC28J60_SPI_CMD_BFS  (0b10000000) // bit field set
#define ENC28J60_SPI_CMD_BFC  (0b10100000) // bit field clear 
#define ENC28J60_SPI_CMD_SRC  (0b11111111) // soft system reset

// global variables
static uint16_t g_current_bank = ENC28J60_BANK0;

// internal functions
static void enc28j60_gpio_init(void);
static void enc28j60_spi_init(void);

static void enc28j60_bfs_eth_register(uint16_t addr, uint8_t mask);
static void enc28j60_bfc_eth_register(uint16_t addr, uint8_t mask);
static void enc28j60_set_bank(uint16_t addr);
static uint8_t enc28j60_read_ctrl_register(uint16_t addr);
static void enc28j60_write_ctrl_register(uint16_t addr, uint8_t data);

static uint16_t enc28j60_read_phy_register(uint8_t addr);
static void enc28j60_write_phy_register(uint8_t addr, uint16_t data);

/////////////////////////////////////////////////////////////

static void enc28j60_gpio_init(void)
{
   rcc_periph_clock_enable(ENC28J60_GPIO_RCC);

   gpio_set_mode(
      ENC28J60_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      ENC28J60_GPIO_PIN_RST);

   ENC28J60_RST_HI;
}

/////////////////////////////////////////////////////////////

static void enc28j60_spi_init(void)
{
   rcc_periph_clock_enable(ENC28J60_SPI_GPIO_RCC);
   rcc_periph_clock_enable(ENC28J60_SPI_RCC);

   spi_reset(ENC28J60_SPI);

   gpio_set_mode(
      ENC28J60_SPI_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
      ENC28J60_SPI_SCK | ENC28J60_SPI_MOSI
   );

   gpio_set_mode(
      ENC28J60_SPI_GPIO_PORT,
      GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_FLOAT,
      ENC28J60_SPI_MISO
   );

   // use software controlled chip select
   gpio_set_mode(
      ENC28J60_SPI_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      ENC28J60_SPI_NSS
   );

   // SPI2 uses APB1 bus,, we assume SYSCLK = 72MHz, APB1 prescaler = 2, APB1 = 36MHz
   // SPI clock cycle according to ref[1] : Fmax = 20 MHz
   //
   // SPI prescaler    SPI-frequency (MHz)
   //     64               0.5625
   //     32               1.1250
   //     16               2.2500
   //      8               4.5000
   //      4               9.0000
   //      2              18.0000

   spi_init_master(
      ENC28J60_SPI,
      SPI_CR1_BAUDRATE_FPCLK_DIV_4,
      SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1,
      SPI_CR1_DFF_8BIT,
      SPI_CR1_MSBFIRST
   );

   spi_set_full_duplex_mode(ENC28J60_SPI);
   spi_disable_software_slave_management(ENC28J60_SPI);
   spi_enable(ENC28J60_SPI);

   ENC28J60_SPI_NSS_HI;
}

/////////////////////////////////////////////////////////////

static uint8_t enc28j60_spi_xfer(uint8_t data)
{
   uint8_t rx_data;

   while (!(SPI_SR(ENC28J60_SPI) & SPI_SR_TXE));
   SPI_DR(ENC28J60_SPI) = data;

   while (!(SPI_SR(ENC28J60_SPI) & SPI_SR_RXNE));
   rx_data = SPI_DR(ENC28J60_SPI);

   while (SPI_SR(ENC28J60_SPI) & SPI_SR_BSY);

   return rx_data;
}

/////////////////////////////////////////////////////////////

static void enc28j60_bfs_eth_register(uint16_t addr, uint8_t mask)
{
   // Note!
   // This function is only applicable on ETH registers.

   ENC28J60_SPI_NSS_LO;

   enc28j60_spi_xfer(ENC28J60_SPI_CMD_BFS | ENC28J60_CTRL_ADR(addr));
   enc28j60_spi_xfer(mask);

   ENC28J60_SPI_NSS_HI;
}

/////////////////////////////////////////////////////////////

static void enc28j60_bfc_eth_register(uint16_t addr, uint8_t mask)
{
   // Note!
   // This function is only applicable on ETH registers.

   ENC28J60_SPI_NSS_LO;

   enc28j60_spi_xfer(ENC28J60_SPI_CMD_BFC | ENC28J60_CTRL_ADR(addr)); 
   enc28j60_spi_xfer(mask);

   ENC28J60_SPI_NSS_HI;
}

/////////////////////////////////////////////////////////////

static void enc28j60_set_bank(uint16_t addr)
{
   // no need to switch bank for a common register
   if ( (ENC28J60_CTRL_REG(addr) & ENC28J60_CTRL_REG_ALL) ==  ENC28J60_CTRL_REG_ALL ) 
   {
      return;
   }

   uint16_t bank = ENC28J60_CTRL_BNK(addr);

   // only switch bank when necessary
   if (bank != g_current_bank)
   {
      switch (bank)
      {
         case ENC28J60_BANK0:
            enc28j60_bfc_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_BSEL1 | ENC28J60_ECON1_BSEL0);
            break;
         case ENC28J60_BANK1:
            enc28j60_bfc_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_BSEL1 | ENC28J60_ECON1_BSEL0);
            enc28j60_bfs_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_BSEL0);
            break;
         case ENC28J60_BANK2:
            enc28j60_bfc_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_BSEL1 | ENC28J60_ECON1_BSEL0);
            enc28j60_bfs_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_BSEL1);
            break;
         case ENC28J60_BANK3:
            enc28j60_bfs_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_BSEL1 | ENC28J60_ECON1_BSEL0);
            break;
         default:
            break;
      }

      g_current_bank = bank;
   }
}

/////////////////////////////////////////////////////////////

static uint8_t enc28j60_read_ctrl_register(uint16_t addr)
{
   enc28j60_set_bank(addr);

   ENC28J60_SPI_NSS_LO;

   enc28j60_spi_xfer(ENC28J60_SPI_CMD_RCR | ENC28J60_CTRL_ADR(addr));

   // control registers of type MAC and MII requires an extar dummy byte
   if ( (ENC28J60_CTRL_REG(addr) == ENC28J60_CTRL_REG_MAC) ||
        (ENC28J60_CTRL_REG(addr) == ENC28J60_CTRL_REG_MII) )
   {
      enc28j60_spi_xfer(0);
   }

   uint8_t reg = enc28j60_spi_xfer(0);

   ENC28J60_SPI_NSS_HI;

   return reg;
}

/////////////////////////////////////////////////////////////

static void enc28j60_write_ctrl_register(uint16_t addr, uint8_t data)
{
   enc28j60_set_bank(addr);

   ENC28J60_SPI_NSS_LO;

   enc28j60_spi_xfer(ENC28J60_SPI_CMD_WCR | ENC28J60_CTRL_ADR(addr));
   enc28j60_spi_xfer(data);

   ENC28J60_SPI_NSS_HI;
}

/////////////////////////////////////////////////////////////

static uint16_t enc28j60_read_phy_register(uint8_t addr)
{
   enc28j60_write_ctrl_register(ENC28J60_MIREGADR, addr);
   enc28j60_write_ctrl_register(ENC28J60_MICMD, ENC28J60_MICMD_MIIRD);
   dwt_delay(11); // 10.24us according to ref[1], section 3.3.1

   while ( (enc28j60_read_ctrl_register(ENC28J60_MISTAT) & ENC28J60_MISTAT_BUSY) == ENC28J60_MISTAT_BUSY ) ;

   enc28j60_write_ctrl_register(ENC28J60_MICMD, 0);

   uint16_t phy_reg;
   phy_reg = enc28j60_read_ctrl_register(ENC28J60_MIRDL);
   phy_reg |= (enc28j60_read_ctrl_register(ENC28J60_MIRDH) << 8);

   return phy_reg;
}

/////////////////////////////////////////////////////////////

static void enc28j60_write_phy_register(uint8_t addr, uint16_t data)
{
   enc28j60_write_ctrl_register(ENC28J60_MIREGADR, addr);

   enc28j60_write_ctrl_register(ENC28J60_MIWRL, data & 0x00ff);
   enc28j60_write_ctrl_register(ENC28J60_MIWRH, (data & 0xff00) >> 8);
   dwt_delay(11); // 10.24us according to ref[1], section 3.3.2

   while ( (enc28j60_read_ctrl_register(ENC28J60_MISTAT) & ENC28J60_MISTAT_BUSY) == ENC28J60_MISTAT_BUSY ) ;
}

/////////////////////////////////////////////////////////////

void enc28j60_init(void)
{
   // initialize hardware
   enc28j60_gpio_init();
   enc28j60_spi_init();

   // system reset according to ref[1] section 11.2
   ENC28J60_RST_LO;
   dwt_delay(1);  // tRSTLOW > 400ns
   ENC28J60_RST_HI;
   dwt_delay(50);

   // wait for oscillator startup
   while ((enc28j60_read_ctrl_register(ENC28J60_ESTAT) & ENC28J60_ESTAT_CLKRDY) != ENC28J60_ESTAT_CLKRDY) ;

   // Microchip Organizationally Unique Identifier (OUI).
   // This is a sanity test that SPI works ok and chip responds as expected.
   uint16_t phid1 = enc28j60_read_phy_register(ENC28J60_PHID1);
   uint16_t phid2 = enc28j60_read_phy_register(ENC28J60_PHID2);
   if ( !( (phid1 == 0x0083) && (phid2 == 0x1400) ) )
   {
      // TBD: how to signal error here?
      return;
   }

   // TBD: This is only for test (LEDA-green=slow, LEDB-yellow=fast)
   enc28j60_write_phy_register(ENC28J60_PHLCON,
                               ENC28J60_PHLCON_LACFG(0xb) | ENC28J60_PHLCON_LBCFG(0xa));
}
