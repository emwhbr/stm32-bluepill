#ifndef __ENC28J60_DEFS_H__
#define __ENC28J60_DEFS_H__

// control register memory banks
 #define ENC28J60_BANK0  0x0000
 #define ENC28J60_BANK1  0x0100
 #define ENC28J60_BANK2  0x0200
 #define ENC28J60_BANK3  0x0300

//////////////////////////////////////////////////////////////////////////
//                   CONTROL REGISTERS
//////////////////////////////////////////////////////////////////////////

// different types of control registers
#define ENC28J60_CTRL_REG_ETH  0x0000  // ETH
#define ENC28J60_CTRL_REG_MAC  0x1000  // MAC
#define ENC28J60_CTRL_REG_MII  0x2000  // MII
#define ENC28J60_CTRL_REG_ALL  0x4000  // Common to all banks

// mask values for a complete control register address
#define ENC28J60_CTRL_MASK_REG  0xF000
#define ENC28J60_CTRL_MASK_BNK  0x0F00
#define ENC28J60_CTRL_MASK_ADR  0x001F

// helper macros to extract parts of a complete register address
#define ENC28J60_CTRL_REG(addr)  (addr & ENC28J60_CTRL_MASK_REG)
#define ENC28J60_CTRL_BNK(addr)  (addr & ENC28J60_CTRL_MASK_BNK)
#define ENC28J60_CTRL_ADR(addr)  (addr & ENC28J60_CTRL_MASK_ADR)

// control registers, common to all banks
#define ENC28J60_EIE        (ENC28J60_CTRL_REG_ETH | ENC28J60_CTRL_REG_ALL | ENC28J60_BANK0 | 0x1B)
#define ENC28J60_EIR        (ENC28J60_CTRL_REG_ETH | ENC28J60_CTRL_REG_ALL | ENC28J60_BANK0 | 0x1C)
#define ENC28J60_ESTAT      (ENC28J60_CTRL_REG_ETH | ENC28J60_CTRL_REG_ALL | ENC28J60_BANK0 | 0x1D)
#define ENC28J60_ECON2      (ENC28J60_CTRL_REG_ETH | ENC28J60_CTRL_REG_ALL | ENC28J60_BANK0 | 0x1E)
#define ENC28J60_ECON1      (ENC28J60_CTRL_REG_ETH | ENC28J60_CTRL_REG_ALL | ENC28J60_BANK0 | 0x1F)

// control registers, bank 0
#define ENC28J60_ERDPTL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x00)
#define ENC28J60_ERDPTH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x01)
#define ENC28J60_EWRPTL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x02)
#define ENC28J60_EWRPTH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x03)
#define ENC28J60_ETXSTL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x04)
#define ENC28J60_ETXSTH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x05)
#define ENC28J60_ETXNDL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x06)
#define ENC28J60_ETXNDH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x07)
#define ENC28J60_ERXSTL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x08)
#define ENC28J60_ERXSTH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x09)
#define ENC28J60_ERXNDL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x0A)
#define ENC28J60_ERXNDH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x0B)
#define ENC28J60_ERXRDPTL   (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x0C)
#define ENC28J60_ERXRDPTH   (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x0D)
#define ENC28J60_ERXWRPTL   (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x0E)
#define ENC28J60_ERXWRPTH   (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x0F)
#define ENC28J60_EDMASTL    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x10)
#define ENC28J60_EDMASTH    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x11)
#define ENC28J60_EDMANDL    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x12)
#define ENC28J60_EDMANDH    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x13)
#define ENC28J60_EDMADSTL   (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x14)
#define ENC28J60_EDMADSTH   (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x15)
#define ENC28J60_EDMACSL    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x16)
#define ENC28J60_EDMACSH    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK0 | 0x17)

// control registers, bank 1
#define ENC28J60_EHT0       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x00)
#define ENC28J60_EHT1       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x01)
#define ENC28J60_EHT2       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x02)
#define ENC28J60_EHT3       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x03)
#define ENC28J60_EHT4       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x04)
#define ENC28J60_EHT5       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x05)
#define ENC28J60_EHT6       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x06)
#define ENC28J60_EHT7       (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x07)
#define ENC28J60_EPMM0      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x08)
#define ENC28J60_EPMM1      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x09)
#define ENC28J60_EPMM2      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x0A)
#define ENC28J60_EPMM3      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x0B)
#define ENC28J60_EPMM4      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x0C)
#define ENC28J60_EPMM5      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x0D)
#define ENC28J60_EPMM6      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x0E)
#define ENC28J60_EPMM7      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x0F)
#define ENC28J60_EPMCSL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x10)
#define ENC28J60_EPMCSH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x11)
#define ENC28J60_EPMOL      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x14)
#define ENC28J60_EPMOH      (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x15)
#define ENC28J60_EWOLIE     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x16)
#define ENC28J60_EWOLIR     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x17)
#define ENC28J60_ERXFCON    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x18)
#define ENC28J60_EPKTCNT    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK1 | 0x19)

// control registers, bank 2
#define ENC28J60_MACON1      (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x00)
#define ENC28J60_MACON2      (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x01)
#define ENC28J60_MACON3      (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x02)
#define ENC28J60_MACON4      (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x03)
#define ENC28J60_MABBIPG     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x04)
#define ENC28J60_MAIPGL      (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x06)
#define ENC28J60_MAIPGH      (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x07)
#define ENC28J60_MACLCON1    (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x08)
#define ENC28J60_MACLCON2    (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x09)
#define ENC28J60_MAMXFLL     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x0A)
#define ENC28J60_MAMXFLH     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x0B)
#define ENC28J60_MAPHSUP     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK2 | 0x0D)
#define ENC28J60_MICON       (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x11)
#define ENC28J60_MICMD       (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x12)
#define ENC28J60_MIREGADR    (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x14)
#define ENC28J60_MIWRL       (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x16)
#define ENC28J60_MIWRH       (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x17)
#define ENC28J60_MIRDL       (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x18)
#define ENC28J60_MIRDH       (ENC28J60_CTRL_REG_MII | ENC28J60_BANK2 | 0x19)

// control registers, bank 3
#define ENC28J60_MAADR5     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK3 | 0x00)
#define ENC28J60_MAADR6     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK3 | 0x01)
#define ENC28J60_MAADR3     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK3 | 0x02)
#define ENC28J60_MAADR4     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK3 | 0x03)
#define ENC28J60_MAADR1     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK3 | 0x04)
#define ENC28J60_MAADR2     (ENC28J60_CTRL_REG_MAC | ENC28J60_BANK3 | 0x05)
#define ENC28J60_EBSTSD     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x06)
#define ENC28J60_EBSTCON    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x07)
#define ENC28J60_EBSTCSL    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x08)
#define ENC28J60_EBSTCSH    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x09)
#define ENC28J60_MISTAT     (ENC28J60_CTRL_REG_MII | ENC28J60_BANK3 | 0x0A)
#define ENC28J60_EREVID     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x12)
#define ENC28J60_ECOCON     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x15)
#define ENC28J60_EFLOCON    (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x17)
#define ENC28J60_EPAUSL     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x18)
#define ENC28J60_EPAUSH     (ENC28J60_CTRL_REG_ETH | ENC28J60_BANK3 | 0x19)

// control register ESTAT, Ethernet Status Register
#define ENC28J60_ESTAT_INT      (1 << 7)
#define ENC28J60_ESTAT_BUFFER   (1 << 6)  
#define ENC28J60_ESTAT_LATECOL  (1 << 4)
#define ENC28J60_ESTAT_RXBUSY   (1 << 2)
#define ENC28J60_ESTAT_TXABRT   (1 << 1)
#define ENC28J60_ESTAT_CLKRDY   (1 << 0)

// control register ECON1, Ethernet Control Register 1
#define ENC28J60_ECON1_TXRST    (1 << 7)
#define ENC28J60_ECON1_RXRST    (1 << 6)
#define ENC28J60_ECON1_DMAST    (1 << 5)
#define ENC28J60_ECON1_CSUMEN   (1 << 4)
#define ENC28J60_ECON1_TXRTS    (1 << 3)
#define ENC28J60_ECON1_RXEN     (1 << 2)
#define ENC28J60_ECON1_BSEL1    (1 << 1)
#define ENC28J60_ECON1_BSEL0    (1 << 0)

// control register ECON2, Ethernet Control Register 2
#define ENC28J60_ECON2_AUTOINC  (1 << 7)
#define ENC28J60_ECON2_PKTDEC   (1 << 6)
#define ENC28J60_ECON2_PWRSV    (1 << 5)
#define ENC28J60_ECON2_VRPS     (1 << 3)

// control register EIE, Ethernet Interupt Enable Register
#define ENC28J60_EIE_INTIE      (1 << 7)
#define ENC28J60_EIE_PKTIE      (1 << 6)
#define ENC28J60_EIE_DMAIE      (1 << 5)
#define ENC28J60_EIE_LINKIE     (1 << 4)
#define ENC28J60_EIE_TXIE       (1 << 3)
#define ENC28J60_EIE_WOLIE      (1 << 2)
#define ENC28J60_EIE_TXERIE     (1 << 1)
#define ENC28J60_EIE_RXERIE     (1 << 0)

// control register EIR, Ethernet Interrupt Request (flag) Register
#define ENC28J60_EIR_PKTIF      (1 << 6)
#define ENC28J60_EIR_DMAIF      (1 << 5)
#define ENC28J60_EIR_LINKIF     (1 << 4)
#define ENC28J60_EIR_TXIF       (1 << 3)
#define ENC28J60_EIR_WOLIF      (1 << 2)
#define ENC28J60_EIR_TXERIF     (1 << 1)
#define ENC28J60_EIR_RXERIF     (1 << 0)

// control register ERXFCON, Ehterner Receive Filter Control Register
#define NC28J60_ERXFCON_UCEN    (1 << 7)
#define NC28J60_ERXFCON_ANDOR   (1 << 6)
#define NC28J60_ERXFCON_CRCEN   (1 << 5)
#define NC28J60_ERXFCON_PMEN    (1 << 4)
#define NC28J60_ERXFCON_MPEN    (1 << 3)
#define NC28J60_ERXFCON_HTEN    (1 << 2)
#define NC28J60_ERXFCON_MCEN    (1 << 1)
#define NC28J60_ERXFCON_BCEN    (1 << 0)

// control register MICMD, MII Command Register
#define ENC28J60_MICMD_MIISCAN  (1 << 1)
#define ENC28J60_MICMD_MIIRD    (1 << 0)

// control register MISTAT, MII Status Register
#define ENC28J60_MISTAT_NVALID  (1 << 2)
#define ENC28J60_MISTAT_SCAN    (1 << 1)
#define ENC28J60_MISTAT_BUSY    (1 << 0)

// control register MACON1, MAC Control Register 1
#define ENC28J60_MACON1_TXPAUS   (1 << 3)
#define ENC28J60_MACON1_RXPAUS   (1 << 2)
#define ENC28J60_MACON1_PASSALL  (1 << 1)
#define ENC28J60_MACON1_MARXEN   (1 << 0)

// control register MACON3, MAC Control Register 3
#define ENC28J60_MACON3_PADCFG2  (1 << 7)
#define ENC28J60_MACON3_PADCFG1  (1 << 6)
#define ENC28J60_MACON3_PADCFG0  (1 << 5)
#define ENC28J60_MACON3_TXCRCEN  (1 << 4)
#define ENC28J60_MACON3_PHDRLEN  (1 << 3)
#define ENC28J60_MACON3_HFRMLEN  (1 << 2)
#define ENC28J60_MACON3_FRMLNEN  (1 << 1)
#define ENC28J60_MACON3_FULDPX   (1 << 0)

#define ENC28J60_MACON3_PADCFG(x) ((x) << 5)

// control register MACON4, MAC Control Register 4
#define ENC28J60_MACON4_DEFER    (1 << 6)
#define ENC28J60_MACON4_BPEN     (1 << 5)
#define ENC28J60_MACON4_NOBKOFF  (1 << 4)
#define ENC28J60_MACON4_LONGPRE  (1 << 1)
#define ENC28J60_MACON4_PUREPRE  (1 << 0)

//////////////////////////////////////////////////////////////////////////
//                   PHY REGISTERS
//////////////////////////////////////////////////////////////////////////

// phy registers
#define ENC28J60_PHCON1   0x00
#define ENC28J60_PHSTAT1  0x01
#define ENC28J60_PHID1    0x02
#define ENC28J60_PHID2    0x03
#define ENC28J60_PHCON2   0x10
#define ENC28J60_PHSTAT2  0x11
#define ENC28J60_PHIE     0x12
#define ENC28J60_PHIR     0x13
#define ENC28J60_PHLCON   0x14

// phy register, PHCON1
#define ENC28J60_PHCON1_PRST      (1 << 15)
#define ENC28J60_PHCON1_PLOOPBK   (1 << 14)
#define ENC28J60_PHCON1_PPWRSV    (1 << 11)
#define ENC28J60_PHCON1_PDPXMD    (1 << 8)

// phy register, PHSTAT1
#define ENC28J60_PHSTAT1_PFDPX    (1 << 12)
#define ENC28J60_PHSTAT1_PHDPX    (1 << 11)
#define ENC28J60_PHSTAT1_LLSTAT   (1 << 2)
#define ENC28J60_PHSTAT1_JBSTAT   (1 << 1)

// phy register, PHCON2
#define ENC28J60_PHCON2_FRCLINK   (1 << 14)
#define ENC28J60_PHCON2_TXDIS     (1 << 13)
#define ENC28J60_PHCON2_JABBER    (1 << 10)
#define ENC28J60_PHCON2_HDLDIS    (1 << 8)

// phy register, PHSTAT2
#define ENC28J60_PHSTAT2_TXSTAT   (1 << 13)
#define ENC28J60_PHSTAT2_RXSTAT   (1 << 12)
#define ENC28J60_PHSTAT2_COLSTAT  (1 << 11)
#define ENC28J60_PHSTAT2_LSTAT    (1 << 10)
#define ENC28J60_PHSTAT2_DPXSTAT  (1 << 9)
#define ENC28J60_PHSTAT2_PLRITY   (1 << 4)

// phy register, PHIE
#define ENC28J60_PHIE_PLNKIE      (1 << 4)
#define ENC28J60_PHIE_PGEIE       (1 << 1)

// phy register, PHIR
#define ENC28J60_PHIR_PLNKIF      (1 << 4)
#define ENC28J60_PHIR_PGIF        (1 << 2)

// phy register,, PHLCON
#define ENC28J60_PHLCON_LACFG3    (1 << 11)
#define ENC28J60_PHLCON_LACFG2    (1 << 10)
#define ENC28J60_PHLCON_LACFG1    (1 << 9)
#define ENC28J60_PHLCON_LACFG0    (1 << 8)
#define ENC28J60_PHLCON_LBCFG3    (1 << 7)
#define ENC28J60_PHLCON_LBCFG2    (1 << 6)
#define ENC28J60_PHLCON_LBCFG1    (1 << 5)
#define ENC28J60_PHLCON_LBCFG0    (1 << 4)
#define ENC28J60_PHLCON_LFRQ1     (1 << 3)
#define ENC28J60_PHLCON_LFRQ0     (1 << 2)
#define ENC28J60_PHLCON_STRCH     (1 << 1)

#define ENC28J60_PHLCON_LACFG(x)  ((x) << 8)
#define ENC28J60_PHLCON_LBCFG(x)  ((x) << 4)
#define ENC28J60_PHLCON_LFRQ(x)   ((x) << 2)

#endif
