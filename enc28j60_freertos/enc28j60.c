#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include "libopencm3/cm3/nvic.h"

#include <FreeRTOS.h>
#include <semphr.h>

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
#define ENC28J60_GPIO_PIN_DBG   GPIO8   // Debug signal

#define ENC28J60_RST_HI  gpio_set(ENC28J60_GPIO_PORT,   ENC28J60_GPIO_PIN_RST)
#define ENC28J60_RST_LO  gpio_clear(ENC28J60_GPIO_PORT, ENC28J60_GPIO_PIN_RST)

#define ENC28J60_DBG_HI  gpio_set(ENC28J60_GPIO_PORT,   ENC28J60_GPIO_PIN_DBG)
#define ENC28J60_DBG_LO  gpio_clear(ENC28J60_GPIO_PORT, ENC28J60_GPIO_PIN_DBG)

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

// SPI DMA support
#define ENC28J60_SPI_DMA_RCC  RCC_DMA1
#define ENC28J60_SPI_DMA_CTL  DMA1
#define ENC28J60_SPI_DMA_CHN  DMA_CHANNEL5
#define ENC28J60_SPI_DMA_IRQ  NVIC_DMA1_CHANNEL5_IRQ

#define ENC28J60_SPI_DMA_PERIPHERAL_ADDR ((uint32_t)&ENC28J60_SPI_DR)

static volatile SemaphoreHandle_t xSpiDmaDoneSem = NULL;

// instruction set
#define ENC28J60_SPI_CMD_RCR  (0b00000000) // read control register
#define ENC28J60_SPI_CMD_RBM  (0b00111010) // read buffer memory
#define ENC28J60_SPI_CMD_WCR  (0b01000000) // write control register
#define ENC28J60_SPI_CMD_WBM  (0b01111010) // write buffer memory
#define ENC28J60_SPI_CMD_BFS  (0b10000000) // bit field set
#define ENC28J60_SPI_CMD_BFC  (0b10100000) // bit field clear 
#define ENC28J60_SPI_CMD_SRC  (0b11111111) // soft system reset

// receive and transmit buffers
#define ENC28J60_RX_BUFFER_START  0x0000  // RX : 6KB
#define ENC28J60_RX_BUFFER_END    0x17FF
#define ENC28J60_TX_BUFFER_START  0x1800  // TX : 2KB
#define ENC28J60_TX_BUFFER_END    0x1FFF

// helper macros
#define ENC28J60_ETH_MAX_FRAME_SIZE  1518

#define MSB(value) (((value) & 0xff00) >> 8)
#define LSB(value)  ((value) & 0x00ff)

// global variables
static uint16_t g_current_bank = ENC28J60_BANK0;

// internal functions
static void enc28j60_gpio_init(void);
static void enc28j60_spi_init(void);
static void enc28j60_spi_dma_init(void);
static void enc28j60_spi_tx_dma_start(const uint8_t *buf, uint16_t size);

static void enc28j60_bfs_eth_register(uint16_t addr, uint8_t mask);
static void enc28j60_bfc_eth_register(uint16_t addr, uint8_t mask);
static void enc28j60_set_bank(uint16_t addr);
static uint8_t enc28j60_read_ctrl_register(uint16_t addr);
static void enc28j60_write_ctrl_register(uint16_t addr, uint8_t data);

static uint16_t enc28j60_read_phy_register(uint8_t addr);
static void enc28j60_write_phy_register(uint8_t addr, uint16_t data);

static void enc28j60_write_eth_buffer(const uint8_t *buf, size_t len);

/////////////////////////////////////////////////////////////

static void enc28j60_gpio_init(void)
{
   rcc_periph_clock_enable(ENC28J60_GPIO_RCC);

   gpio_set_mode(
      ENC28J60_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      ENC28J60_GPIO_PIN_RST | ENC28J60_GPIO_PIN_DBG);

   ENC28J60_RST_HI;
   ENC28J60_DBG_HI;
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
      SPI_CR1_BAUDRATE_FPCLK_DIV_2,
      SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_1,
      SPI_CR1_DFF_8BIT,
      SPI_CR1_MSBFIRST
   );

   spi_set_full_duplex_mode(ENC28J60_SPI);
   spi_disable_software_slave_management(ENC28J60_SPI);
   spi_enable(ENC28J60_SPI);

   ENC28J60_SPI_NSS_HI;

   enc28j60_spi_dma_init();
}

/////////////////////////////////////////////////////////////

static void enc28j60_spi_dma_init(void)
{
   rcc_periph_clock_enable(ENC28J60_SPI_DMA_RCC);

   dma_channel_reset(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);

   dma_set_peripheral_address(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, ENC28J60_SPI_DMA_PERIPHERAL_ADDR);
   dma_disable_memory_increment_mode(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);
   dma_set_peripheral_size(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, DMA_CCR_PSIZE_8BIT);

   dma_set_read_from_memory(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);
   dma_enable_memory_increment_mode(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);
   dma_set_memory_size(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, DMA_CCR_MSIZE_8BIT);

   dma_set_priority(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, DMA_CCR_PL_HIGH);
   dma_enable_transfer_complete_interrupt(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);

   nvic_set_priority(ENC28J60_SPI_DMA_IRQ, 0);
   nvic_enable_irq(ENC28J60_SPI_DMA_IRQ);

   // create sempahore used for DMA synchronization
   if (xSpiDmaDoneSem == NULL)
   {
      xSpiDmaDoneSem = xSemaphoreCreateBinary();
   }
}

/////////////////////////////////////////////////////////////

void dma1_channel5_isr(void)
{
   // check if DMA transfer complete
   if (dma_get_interrupt_flag(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, DMA_TCIF))
   {
      dma_clear_interrupt_flags(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, DMA_TCIF);

      // wait until last SPI transfer is completed
      while (SPI_SR(ENC28J60_SPI) & SPI_SR_BSY);
      spi_clean_disable(ENC28J60_SPI);
      ENC28J60_SPI_NSS_HI;

      spi_disable_tx_dma(ENC28J60_SPI);
      dma_disable_channel(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);

      // signal DMA done
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xSemaphoreGiveFromISR(xSpiDmaDoneSem, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
}

/////////////////////////////////////////////////////////////

static void enc28j60_spi_tx_dma_start(const uint8_t *buf, uint16_t size)
{
   // configure the DMA
   dma_set_memory_address(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, (uint32_t)buf);
   dma_set_number_of_data(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN, size);

   // start the DMA transfer
   dma_enable_channel(ENC28J60_SPI_DMA_CTL, ENC28J60_SPI_DMA_CHN);
   spi_enable_tx_dma(ENC28J60_SPI);
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

   enc28j60_write_ctrl_register(ENC28J60_MIWRL, LSB(data));
   enc28j60_write_ctrl_register(ENC28J60_MIWRH, MSB(data));
   dwt_delay(11); // 10.24us according to ref[1], section 3.3.2

   while ( (enc28j60_read_ctrl_register(ENC28J60_MISTAT) & ENC28J60_MISTAT_BUSY) == ENC28J60_MISTAT_BUSY ) ;
}

/////////////////////////////////////////////////////////////

static void enc28j60_write_eth_buffer(const uint8_t *buf, size_t len)
{
   ENC28J60_SPI_NSS_LO;

   enc28j60_spi_xfer(ENC28J60_SPI_CMD_WBM);
   enc28j60_spi_xfer(0x00); // packet control byte

   enc28j60_spi_tx_dma_start(buf, len);

   // wait until DMA completed
   xSemaphoreTake(xSpiDmaDoneSem, portMAX_DELAY);

   spi_enable(ENC28J60_SPI);
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

   // define the receive buffer, the transmit buffer is considered to be the rest
   enc28j60_write_ctrl_register(ENC28J60_ERXSTL, LSB(ENC28J60_RX_BUFFER_START));
   enc28j60_write_ctrl_register(ENC28J60_ERXSTH, MSB(ENC28J60_RX_BUFFER_START));
   enc28j60_write_ctrl_register(ENC28J60_ERXNDL, LSB(ENC28J60_RX_BUFFER_END));
   enc28j60_write_ctrl_register(ENC28J60_ERXNDH, MSB(ENC28J60_RX_BUFFER_END));

   // MAC => Half-Duplex mode
   // - enable automatic padding to at least 64 bytes
   // - always append a valid CRC
   // - check frame length
   enc28j60_write_ctrl_register(ENC28J60_MACON3,
                                ENC28J60_MACON3_PADCFG(3) | ENC28J60_MACON3_TXCRCEN | ENC28J60_MACON3_FRMLNEN);

   // MAC will wait indefinitely for it to become free
   enc28j60_write_ctrl_register(ENC28J60_MACON4, ENC28J60_MACON4_DEFER);

   // maximum frame length that will be permitted to be received or transmitted
   enc28j60_write_ctrl_register(ENC28J60_MAMXFLL, LSB(ENC28J60_ETH_MAX_FRAME_SIZE));
   enc28j60_write_ctrl_register(ENC28J60_MAMXFLH, MSB(ENC28J60_ETH_MAX_FRAME_SIZE));

   // configure Back-to-Back Inter-Packet Gap
   enc28j60_write_ctrl_register(ENC28J60_MABBIPG, 0x12);

   // configure the Non-Back-to-Back Inter-Packet Gap
   enc28j60_write_ctrl_register(ENC28J60_MAIPGL, 0x12);
   enc28j60_write_ctrl_register(ENC28J60_MAIPGH, 0x0C);

   // retransmission and collison window 
   //enc28j60_write_ctrl_register(ENC28J60_MACLCON1, xx);
   //enc28j60_write_ctrl_register(ENC28J60_MACLCON2, xx);

   // PHY => Half-Duplex mode
   // - prevent automatic loopback of the data which is transmitted
   enc28j60_write_phy_register(ENC28J60_PHCON1, 0x0000);
   enc28j60_write_phy_register(ENC28J60_PHCON2, ENC28J60_PHCON2_HDLDIS);

   /*
   // LEDs
   // - LEDA (green) slow
   // - LEDB-(yellow) fast
   enc28j60_write_phy_register(ENC28J60_PHLCON,
                               ENC28J60_PHLCON_LACFG(0xb) | ENC28J60_PHLCON_LBCFG(0xa));
                               */
   // LEDs
   // - LEDA (green)  - transmit and receive activity (stretchable)
   // - LEDB-(yellow) - link status
   enc28j60_write_phy_register(ENC28J60_PHLCON,
                               ENC28J60_PHLCON_LACFG(0b0111) | \
                               ENC28J60_PHLCON_LBCFG(0b0100) | \
                               ENC28J60_PHLCON_LFRQ(0)       | \
                               ENC28J60_PHLCON_STRCH);
}

/////////////////////////////////////////////////////////////

void enc28j60_test_send_packet(const uint8_t *buf, size_t len)
{
   // errata workaround, reset transmit logic
   enc28j60_bfs_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_TXRST);
   enc28j60_bfc_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_TXRST);
   enc28j60_bfc_eth_register(ENC28J60_EIR, ENC28J60_EIR_TXIF | ENC28J60_EIR_TXERIF);

   // point to the first byte in the data payload
   enc28j60_write_ctrl_register(ENC28J60_EWRPTL, LSB(ENC28J60_TX_BUFFER_START));
   enc28j60_write_ctrl_register(ENC28J60_EWRPTH, MSB(ENC28J60_TX_BUFFER_START));

   // copy data to transmit buffer
   ENC28J60_DBG_LO;
   enc28j60_write_eth_buffer(buf, len);
   ENC28J60_DBG_HI;

   // transmit buffer location
   enc28j60_write_ctrl_register(ENC28J60_ETXSTL, LSB(ENC28J60_TX_BUFFER_START));
   enc28j60_write_ctrl_register(ENC28J60_ETXSTH, MSB(ENC28J60_TX_BUFFER_START));
   enc28j60_write_ctrl_register(ENC28J60_ETXNDL, LSB(ENC28J60_TX_BUFFER_START + len));
   enc28j60_write_ctrl_register(ENC28J60_ETXNDH, MSB(ENC28J60_TX_BUFFER_START + len));

   // clear interrupt flags and enable interrupts
   enc28j60_bfc_eth_register(ENC28J60_EIR, ENC28J60_EIR_TXIF | ENC28J60_EIR_TXERIF);
   enc28j60_bfs_eth_register(ENC28J60_EIE, ENC28J60_EIE_TXIE | ENC28J60_EIE_TXERIE);

   // start transmission
   enc28j60_bfs_eth_register(ENC28J60_ECON1, ENC28J60_ECON1_TXRTS);

   // wait for done
   while ( (enc28j60_read_ctrl_register(ENC28J60_EIR) &
            (ENC28J60_EIR_TXIF | ENC28J60_EIR_TXERIF)) == 0 ) ;

   // clear interrupt flags and disable interrupts
   enc28j60_bfc_eth_register(ENC28J60_EIE, ENC28J60_EIE_TXIE | ENC28J60_EIE_TXERIE);
   enc28j60_bfc_eth_register(ENC28J60_EIR, ENC28J60_EIR_TXIF | ENC28J60_EIR_TXERIF);
}
