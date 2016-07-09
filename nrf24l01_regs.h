#define l01_config 0x00
#define L01_MASK_RX_DR  (1<<6)
#define L01_MASK_TX_DS  (1<<5)
#define L01_MASK_MAX_RT (1<<4)
#define L01_EN_CRC      (1<<3)
#define L01_CRCO        (1<<2)
#define L01_PWR_UP      (1<<1)
#define L01_PRIM_RX     (1<<0)

#define l01_en_aa 0x01
#define L01_ENAA_P5 (1<<5)
#define L01_ENAA_P4 (1<<4)
#define L01_ENAA_P3 (1<<3)
#define L01_ENAA_P2 (1<<2)
#define L01_ENAA_P1 (1<<1)
#define L01_ENAA_P0 (1<<0)

#define l01_en_rxaddr 0x02
#define L01_ERX_P5 (1<<5)
#define L01_ERX_P4 (1<<4)
#define L01_ERX_P3 (1<<3)
#define L01_ERX_P2 (1<<2)
#define L01_ERX_P1 (1<<1)
#define L01_ERX_P0 (1<<0)

#define l01_setup_aw 0x03
#define L01_AW3 0x01
#define L01_AW4 0x02
#define L01_AW5 0x03

#define l01_setup_retr 0x04
#define L01_ARD_MASK 0xf0
#define L01_ARC_MASK 0x0f

#define l01_rf_ch 0x05
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)
#define L01_ (1<<)

#define l01_rf_setup 0x06
#define L01_CONT_WAVE  (1<<7)
#define L01_RF_DR_LOW  (1<<5)
#define L01_PLL_LOCK   (1<<4)
#define L01_RF_DR_HIGH (1<<3)
#define L01_RF_PWR_MASK 0x06
#define L01_RF_PWR_18 0x00
#define L01_RF_PWR_12 0x02
#define L01_RF_PWR_6  0x04
#define L01_RF_PWR_0  0x06
#define L01_RF_DR_MASK (L01_RF_DR_LOW|L01_RF_DR_HIGH)
#define L01_RF_DR_1M   (0)
#define L01_RF_DR_2M   (L01_RF_DR_HIGH)
#define L01_RF_DR_250K (L01_RF_DR_LOW)

#define l01_status 0x07
#define L01_RX_DR   (1<<6)
#define L01_TX_DS   (1<<5)
#define L01_MAX_RT  (1<<4)
#define L01_RX_P_NO_MASK 0x0e
#define L01_RX_P_NO_OFS 1
#define L01_TX_FULL (1<<0)

#define l01_observe_tx 0x08
#define L01_PLOS_CNT_MASK 0xf0
#define L01_ARC_CNT_MASK  0x0f

#define l01_rpd 0x09
#define L01_RPD  (1<<0)

#define l01_rx_addr_p0 0x0a
#define l01_rx_addr_p1 0x0b
#define l01_rx_addr_p2 0x0c
#define l01_rx_addr_p3 0x0d
#define l01_rx_addr_p4 0x0e
#define l01_rx_addr_p5 0x0f

#define l01_tx_addr 0x10

#define l01_rx_pw_p0 0x11
#define l01_rx_pw_p1 0x12
#define l01_rx_pw_p2 0x13
#define l01_rx_pw_p3 0x14
#define l01_rx_pw_p4 0x15
#define l01_rx_pw_p5 0x16

#define l01_fifo_status 0x17
#define L01_TX_REUSE (1<<6)
#define L01_TX_FULL  (1<<5)
#define L01_TX_EMPTY (1<<4)
#define L01_RX_FULL  (1<<1)
#define L01_RX_EMPTY (1<<0)

#define l01_dynpd 0x1c
#define L01_DPL_P5 (1<<5)
#define L01_DPL_P4 (1<<4)
#define L01_DPL_P3 (1<<3)
#define L01_DPL_P2 (1<<2)
#define L01_DPL_P1 (1<<1)
#define L01_DPL_P0 (1<<0)

#define l01_feature 0x1d
#define L01_EN_DPL     (1<<2)
#define L01_EN_ACK_PAY (1<<1)
#define L01_EN_DYN_ACK (1<<0)

#define l01_r_register          0x00
#define l01_w_register          0x20
#define l01_r_rx_payload        0x61
#define l01_w_rx_payload        0xa0
#define l01_flush_rx            0xe2
#define l01_flush_tx            0xe1
#define l01_reuse_tx_pl         0xe3
#define l01_r_rx_pl_wid         0x60
#define l01_w_ack_payload       0xa8
#define l01_w_tx_payload_no_ack 0xb0
#define l01_nop                 0xff









