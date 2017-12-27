#ifndef REF_H_
#define REF_H_

typedef __INT32_TYPE__ int32_t ;
typedef __UINT32_TYPE__ uint32_t ;
typedef __INT8_TYPE__ int8_t ;
typedef __UINT8_TYPE__ uint8_t ;
#define     __IO    volatile
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_UE_Pos                    (0U)
#define USART_CR1_UE_Msk                    (0x1U << USART_CR1_UE_Pos)         /*!< 0x00000001 */
#define USART_CR1_UE                        USART_CR1_UE_Msk                   /*!< USART Enable */
#define USART_CR1_UESM_Pos                  (1U)
#define USART_CR1_UESM_Msk                  (0x1U << USART_CR1_UESM_Pos)       /*!< 0x00000002 */
#define USART_CR1_UESM                      USART_CR1_UESM_Msk                 /*!< USART Enable in STOP Mode */
#define USART_CR1_RE_Pos                    (2U)
#define USART_CR1_RE_Msk                    (0x1U << USART_CR1_RE_Pos)         /*!< 0x00000004 */
#define USART_CR1_RE                        USART_CR1_RE_Msk                   /*!< Receiver Enable */
#define USART_CR1_TE_Pos                    (3U)
#define USART_CR1_TE_Msk                    (0x1U << USART_CR1_TE_Pos)         /*!< 0x00000008 */
#define USART_CR1_TE                        USART_CR1_TE_Msk                   /*!< Transmitter Enable */
#define USART_CR1_IDLEIE_Pos                (4U)
#define USART_CR1_IDLEIE_Msk                (0x1U << USART_CR1_IDLEIE_Pos)     /*!< 0x00000010 */
#define USART_CR1_IDLEIE                    USART_CR1_IDLEIE_Msk               /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE_Pos                (5U)
#define USART_CR1_RXNEIE_Msk                (0x1U << USART_CR1_RXNEIE_Pos)     /*!< 0x00000020 */
#define USART_CR1_RXNEIE                    USART_CR1_RXNEIE_Msk               /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE_Pos                  (6U)
#define USART_CR1_TCIE_Msk                  (0x1U << USART_CR1_TCIE_Pos)       /*!< 0x00000040 */
#define USART_CR1_TCIE                      USART_CR1_TCIE_Msk                 /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos                 (7U)
#define USART_CR1_TXEIE_Msk                 (0x1U << USART_CR1_TXEIE_Pos)      /*!< 0x00000080 */
#define USART_CR1_TXEIE                     USART_CR1_TXEIE_Msk                /*!< TXE Interrupt Enable */
#define USART_CR1_PEIE_Pos                  (8U)
#define USART_CR1_PEIE_Msk                  (0x1U << USART_CR1_PEIE_Pos)       /*!< 0x00000100 */
#define USART_CR1_PEIE                      USART_CR1_PEIE_Msk                 /*!< PE Interrupt Enable */
#define USART_CR1_PS_Pos                    (9U)
#define USART_CR1_PS_Msk                    (0x1U << USART_CR1_PS_Pos)         /*!< 0x00000200 */
#define USART_CR1_PS                        USART_CR1_PS_Msk                   /*!< Parity Selection */
#define USART_CR1_PCE_Pos                   (10U)
#define USART_CR1_PCE_Msk                   (0x1U << USART_CR1_PCE_Pos)        /*!< 0x00000400 */
#define USART_CR1_PCE                       USART_CR1_PCE_Msk                  /*!< Parity Control Enable */
#define USART_CR1_WAKE_Pos                  (11U)
#define USART_CR1_WAKE_Msk                  (0x1U << USART_CR1_WAKE_Pos)       /*!< 0x00000800 */
#define USART_CR1_WAKE                      USART_CR1_WAKE_Msk                 /*!< Receiver Wakeup method */
#define USART_CR1_M_Pos                     (12U)
#define USART_CR1_M_Msk                     (0x10001U << USART_CR1_M_Pos)      /*!< 0x10001000 */
#define USART_CR1_M                         USART_CR1_M_Msk                    /*!< Word length */
#define USART_CR1_M0_Pos                    (12U)
#define USART_CR1_M0_Msk                    (0x1U << USART_CR1_M0_Pos)         /*!< 0x00001000 */
#define USART_CR1_M0                        USART_CR1_M0_Msk                   /*!< Word length - Bit 0 */
#define USART_CR1_MME_Pos                   (13U)
#define USART_CR1_MME_Msk                   (0x1U << USART_CR1_MME_Pos)        /*!< 0x00002000 */
#define USART_CR1_MME                       USART_CR1_MME_Msk                  /*!< Mute Mode Enable */
#define USART_CR1_CMIE_Pos                  (14U)
#define USART_CR1_CMIE_Msk                  (0x1U << USART_CR1_CMIE_Pos)       /*!< 0x00004000 */
#define USART_CR1_CMIE                      USART_CR1_CMIE_Msk                 /*!< Character match interrupt enable */
#define USART_CR1_OVER8_Pos                 (15U)
#define USART_CR1_OVER8_Msk                 (0x1U << USART_CR1_OVER8_Pos)      /*!< 0x00008000 */
#define USART_CR1_OVER8                     USART_CR1_OVER8_Msk                /*!< Oversampling by 8-bit or 16-bit mode */
#define USART_CR1_DEDT_Pos                  (16U)
#define USART_CR1_DEDT_Msk                  (0x1FU << USART_CR1_DEDT_Pos)      /*!< 0x001F0000 */
#define USART_CR1_DEDT                      USART_CR1_DEDT_Msk                 /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define USART_CR1_DEDT_0                    (0x01U << USART_CR1_DEDT_Pos)      /*!< 0x00010000 */
#define USART_CR1_DEDT_1                    (0x02U << USART_CR1_DEDT_Pos)      /*!< 0x00020000 */
#define USART_CR1_DEDT_2                    (0x04U << USART_CR1_DEDT_Pos)      /*!< 0x00040000 */
#define USART_CR1_DEDT_3                    (0x08U << USART_CR1_DEDT_Pos)      /*!< 0x00080000 */
#define USART_CR1_DEDT_4                    (0x10U << USART_CR1_DEDT_Pos)      /*!< 0x00100000 */
#define USART_CR1_DEAT_Pos                  (21U)
#define USART_CR1_DEAT_Msk                  (0x1FU << USART_CR1_DEAT_Pos)      /*!< 0x03E00000 */
#define USART_CR1_DEAT                      USART_CR1_DEAT_Msk                 /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define USART_CR1_DEAT_0                    (0x01U << USART_CR1_DEAT_Pos)      /*!< 0x00200000 */
#define USART_CR1_DEAT_1                    (0x02U << USART_CR1_DEAT_Pos)      /*!< 0x00400000 */
#define USART_CR1_DEAT_2                    (0x04U << USART_CR1_DEAT_Pos)      /*!< 0x00800000 */
#define USART_CR1_DEAT_3                    (0x08U << USART_CR1_DEAT_Pos)      /*!< 0x01000000 */
#define USART_CR1_DEAT_4                    (0x10U << USART_CR1_DEAT_Pos)      /*!< 0x02000000 */
#define USART_CR1_RTOIE_Pos                 (26U)
#define USART_CR1_RTOIE_Msk                 (0x1U << USART_CR1_RTOIE_Pos)      /*!< 0x04000000 */
#define USART_CR1_RTOIE                     USART_CR1_RTOIE_Msk                /*!< Receive Time Out interrupt enable */
#define USART_CR1_EOBIE_Pos                 (27U)
#define USART_CR1_EOBIE_Msk                 (0x1U << USART_CR1_EOBIE_Pos)      /*!< 0x08000000 */
#define USART_CR1_EOBIE                     USART_CR1_EOBIE_Msk                /*!< End of Block interrupt enable */
#define USART_CR1_M1_Pos                    (28U)
#define USART_CR1_M1_Msk                    (0x1U << USART_CR1_M1_Pos)         /*!< 0x10000000 */
#define USART_CR1_M1                        USART_CR1_M1_Msk                   /*!< Word length - Bit 1 */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADDM7_Pos                 (4U)
#define USART_CR2_ADDM7_Msk                 (0x1U << USART_CR2_ADDM7_Pos)      /*!< 0x00000010 */
#define USART_CR2_ADDM7                     USART_CR2_ADDM7_Msk                /*!< 7-bit or 4-bit Address Detection */
#define USART_CR2_LBDL_Pos                  (5U)
#define USART_CR2_LBDL_Msk                  (0x1U << USART_CR2_LBDL_Pos)       /*!< 0x00000020 */
#define USART_CR2_LBDL                      USART_CR2_LBDL_Msk                 /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE_Pos                 (6U)
#define USART_CR2_LBDIE_Msk                 (0x1U << USART_CR2_LBDIE_Pos)      /*!< 0x00000040 */
#define USART_CR2_LBDIE                     USART_CR2_LBDIE_Msk                /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos                  (8U)
#define USART_CR2_LBCL_Msk                  (0x1U << USART_CR2_LBCL_Pos)       /*!< 0x00000100 */
#define USART_CR2_LBCL                      USART_CR2_LBCL_Msk                 /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA_Pos                  (9U)
#define USART_CR2_CPHA_Msk                  (0x1U << USART_CR2_CPHA_Pos)       /*!< 0x00000200 */
#define USART_CR2_CPHA                      USART_CR2_CPHA_Msk                 /*!< Clock Phase */
#define USART_CR2_CPOL_Pos                  (10U)
#define USART_CR2_CPOL_Msk                  (0x1U << USART_CR2_CPOL_Pos)       /*!< 0x00000400 */
#define USART_CR2_CPOL                      USART_CR2_CPOL_Msk                 /*!< Clock Polarity */
#define USART_CR2_CLKEN_Pos                 (11U)
#define USART_CR2_CLKEN_Msk                 (0x1U << USART_CR2_CLKEN_Pos)      /*!< 0x00000800 */
#define USART_CR2_CLKEN                     USART_CR2_CLKEN_Msk                /*!< Clock Enable */
#define USART_CR2_STOP_Pos                  (12U)
#define USART_CR2_STOP_Msk                  (0x3U << USART_CR2_STOP_Pos)       /*!< 0x00003000 */
#define USART_CR2_STOP                      USART_CR2_STOP_Msk                 /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0                    (0x1U << USART_CR2_STOP_Pos)       /*!< 0x00001000 */
#define USART_CR2_STOP_1                    (0x2U << USART_CR2_STOP_Pos)       /*!< 0x00002000 */
#define USART_CR2_LINEN_Pos                 (14U)
#define USART_CR2_LINEN_Msk                 (0x1U << USART_CR2_LINEN_Pos)      /*!< 0x00004000 */
#define USART_CR2_LINEN                     USART_CR2_LINEN_Msk                /*!< LIN mode enable */
#define USART_CR2_SWAP_Pos                  (15U)
#define USART_CR2_SWAP_Msk                  (0x1U << USART_CR2_SWAP_Pos)       /*!< 0x00008000 */
#define USART_CR2_SWAP                      USART_CR2_SWAP_Msk                 /*!< SWAP TX/RX pins */
#define USART_CR2_RXINV_Pos                 (16U)
#define USART_CR2_RXINV_Msk                 (0x1U << USART_CR2_RXINV_Pos)      /*!< 0x00010000 */
#define USART_CR2_RXINV                     USART_CR2_RXINV_Msk                /*!< RX pin active level inversion */
#define USART_CR2_TXINV_Pos                 (17U)
#define USART_CR2_TXINV_Msk                 (0x1U << USART_CR2_TXINV_Pos)      /*!< 0x00020000 */
#define USART_CR2_TXINV                     USART_CR2_TXINV_Msk                /*!< TX pin active level inversion */
#define USART_CR2_DATAINV_Pos               (18U)
#define USART_CR2_DATAINV_Msk               (0x1U << USART_CR2_DATAINV_Pos)    /*!< 0x00040000 */
#define USART_CR2_DATAINV                   USART_CR2_DATAINV_Msk              /*!< Binary data inversion */
#define USART_CR2_MSBFIRST_Pos              (19U)
#define USART_CR2_MSBFIRST_Msk              (0x1U << USART_CR2_MSBFIRST_Pos)   /*!< 0x00080000 */
#define USART_CR2_MSBFIRST                  USART_CR2_MSBFIRST_Msk             /*!< Most Significant Bit First */
#define USART_CR2_ABREN_Pos                 (20U)
#define USART_CR2_ABREN_Msk                 (0x1U << USART_CR2_ABREN_Pos)      /*!< 0x00100000 */
#define USART_CR2_ABREN                     USART_CR2_ABREN_Msk                /*!< Auto Baud-Rate Enable*/
#define USART_CR2_ABRMODE_Pos               (21U)
#define USART_CR2_ABRMODE_Msk               (0x3U << USART_CR2_ABRMODE_Pos)    /*!< 0x00600000 */
#define USART_CR2_ABRMODE                   USART_CR2_ABRMODE_Msk              /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define USART_CR2_ABRMODE_0                 (0x1U << USART_CR2_ABRMODE_Pos)    /*!< 0x00200000 */
#define USART_CR2_ABRMODE_1                 (0x2U << USART_CR2_ABRMODE_Pos)    /*!< 0x00400000 */
#define USART_CR2_RTOEN_Pos                 (23U)
#define USART_CR2_RTOEN_Msk                 (0x1U << USART_CR2_RTOEN_Pos)      /*!< 0x00800000 */
#define USART_CR2_RTOEN                     USART_CR2_RTOEN_Msk                /*!< Receiver Time-Out enable */
#define USART_CR2_ADD_Pos                   (24U)
#define USART_CR2_ADD_Msk                   (0xFFU << USART_CR2_ADD_Pos)       /*!< 0xFF000000 */
#define USART_CR2_ADD                       USART_CR2_ADD_Msk                  /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos                   (0U)
#define USART_CR3_EIE_Msk                   (0x1U << USART_CR3_EIE_Pos)        /*!< 0x00000001 */
#define USART_CR3_EIE                       USART_CR3_EIE_Msk                  /*!< Error Interrupt Enable */
#define USART_CR3_IREN_Pos                  (1U)
#define USART_CR3_IREN_Msk                  (0x1U << USART_CR3_IREN_Pos)       /*!< 0x00000002 */
#define USART_CR3_IREN                      USART_CR3_IREN_Msk                 /*!< IrDA mode Enable */
#define USART_CR3_IRLP_Pos                  (2U)
#define USART_CR3_IRLP_Msk                  (0x1U << USART_CR3_IRLP_Pos)       /*!< 0x00000004 */
#define USART_CR3_IRLP                      USART_CR3_IRLP_Msk                 /*!< IrDA Low-Power */
#define USART_CR3_HDSEL_Pos                 (3U)
#define USART_CR3_HDSEL_Msk                 (0x1U << USART_CR3_HDSEL_Pos)      /*!< 0x00000008 */
#define USART_CR3_HDSEL                     USART_CR3_HDSEL_Msk                /*!< Half-Duplex Selection */
#define USART_CR3_NACK_Pos                  (4U)
#define USART_CR3_NACK_Msk                  (0x1U << USART_CR3_NACK_Pos)       /*!< 0x00000010 */
#define USART_CR3_NACK                      USART_CR3_NACK_Msk                 /*!< SmartCard NACK enable */
#define USART_CR3_SCEN_Pos                  (5U)
#define USART_CR3_SCEN_Msk                  (0x1U << USART_CR3_SCEN_Pos)       /*!< 0x00000020 */
#define USART_CR3_SCEN                      USART_CR3_SCEN_Msk                 /*!< SmartCard mode enable */
#define USART_CR3_DMAR_Pos                  (6U)
#define USART_CR3_DMAR_Msk                  (0x1U << USART_CR3_DMAR_Pos)       /*!< 0x00000040 */
#define USART_CR3_DMAR                      USART_CR3_DMAR_Msk                 /*!< DMA Enable Receiver */
#define USART_CR3_DMAT_Pos                  (7U)
#define USART_CR3_DMAT_Msk                  (0x1U << USART_CR3_DMAT_Pos)       /*!< 0x00000080 */
#define USART_CR3_DMAT                      USART_CR3_DMAT_Msk                 /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE_Pos                  (8U)
#define USART_CR3_RTSE_Msk                  (0x1U << USART_CR3_RTSE_Pos)       /*!< 0x00000100 */
#define USART_CR3_RTSE                      USART_CR3_RTSE_Msk                 /*!< RTS Enable */
#define USART_CR3_CTSE_Pos                  (9U)
#define USART_CR3_CTSE_Msk                  (0x1U << USART_CR3_CTSE_Pos)       /*!< 0x00000200 */
#define USART_CR3_CTSE                      USART_CR3_CTSE_Msk                 /*!< CTS Enable */
#define USART_CR3_CTSIE_Pos                 (10U)
#define USART_CR3_CTSIE_Msk                 (0x1U << USART_CR3_CTSIE_Pos)      /*!< 0x00000400 */
#define USART_CR3_CTSIE                     USART_CR3_CTSIE_Msk                /*!< CTS Interrupt Enable */
#define USART_CR3_ONEBIT_Pos                (11U)
#define USART_CR3_ONEBIT_Msk                (0x1U << USART_CR3_ONEBIT_Pos)     /*!< 0x00000800 */
#define USART_CR3_ONEBIT                    USART_CR3_ONEBIT_Msk               /*!< One sample bit method enable */
#define USART_CR3_OVRDIS_Pos                (12U)
#define USART_CR3_OVRDIS_Msk                (0x1U << USART_CR3_OVRDIS_Pos)     /*!< 0x00001000 */
#define USART_CR3_OVRDIS                    USART_CR3_OVRDIS_Msk               /*!< Overrun Disable */
#define USART_CR3_DDRE_Pos                  (13U)
#define USART_CR3_DDRE_Msk                  (0x1U << USART_CR3_DDRE_Pos)       /*!< 0x00002000 */
#define USART_CR3_DDRE                      USART_CR3_DDRE_Msk                 /*!< DMA Disable on Reception Error */
#define USART_CR3_DEM_Pos                   (14U)
#define USART_CR3_DEM_Msk                   (0x1U << USART_CR3_DEM_Pos)        /*!< 0x00004000 */
#define USART_CR3_DEM                       USART_CR3_DEM_Msk                  /*!< Driver Enable Mode */
#define USART_CR3_DEP_Pos                   (15U)
#define USART_CR3_DEP_Msk                   (0x1U << USART_CR3_DEP_Pos)        /*!< 0x00008000 */
#define USART_CR3_DEP                       USART_CR3_DEP_Msk                  /*!< Driver Enable Polarity Selection */
#define USART_CR3_SCARCNT_Pos               (17U)
#define USART_CR3_SCARCNT_Msk               (0x7U << USART_CR3_SCARCNT_Pos)    /*!< 0x000E0000 */
#define USART_CR3_SCARCNT                   USART_CR3_SCARCNT_Msk              /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define USART_CR3_SCARCNT_0                 (0x1U << USART_CR3_SCARCNT_Pos)    /*!< 0x00020000 */
#define USART_CR3_SCARCNT_1                 (0x2U << USART_CR3_SCARCNT_Pos)    /*!< 0x00040000 */
#define USART_CR3_SCARCNT_2                 (0x4U << USART_CR3_SCARCNT_Pos)    /*!< 0x00080000 */
#define USART_CR3_WUS_Pos                   (20U)
#define USART_CR3_WUS_Msk                   (0x3U << USART_CR3_WUS_Pos)        /*!< 0x00300000 */
#define USART_CR3_WUS                       USART_CR3_WUS_Msk                  /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define USART_CR3_WUS_0                     (0x1U << USART_CR3_WUS_Pos)        /*!< 0x00100000 */
#define USART_CR3_WUS_1                     (0x2U << USART_CR3_WUS_Pos)        /*!< 0x00200000 */
#define USART_CR3_WUFIE_Pos                 (22U)
#define USART_CR3_WUFIE_Msk                 (0x1U << USART_CR3_WUFIE_Pos)      /*!< 0x00400000 */
#define USART_CR3_WUFIE                     USART_CR3_WUFIE_Msk                /*!< Wake Up Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_FRACTION_Pos          (0U)
#define USART_BRR_DIV_FRACTION_Msk          (0xFU << USART_BRR_DIV_FRACTION_Pos)     /*!< 0x0000000F */
#define USART_BRR_DIV_FRACTION              USART_BRR_DIV_FRACTION_Msk               /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_MANTISSA_Pos          (4U)
#define USART_BRR_DIV_MANTISSA_Msk          (0xFFFU << USART_BRR_DIV_MANTISSA_Pos)   /*!< 0x0000FFF0 */
#define USART_BRR_DIV_MANTISSA              USART_BRR_DIV_MANTISSA_Msk               /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos                  (0U)
#define USART_GTPR_PSC_Msk                  (0xFFU << USART_GTPR_PSC_Pos)      /*!< 0x000000FF */
#define USART_GTPR_PSC                      USART_GTPR_PSC_Msk                 /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_GT_Pos                   (8U)
#define USART_GTPR_GT_Msk                   (0xFFU << USART_GTPR_GT_Pos)       /*!< 0x0000FF00 */
#define USART_GTPR_GT                       USART_GTPR_GT_Msk                  /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define USART_RTOR_RTO_Pos                  (0U)
#define USART_RTOR_RTO_Msk                  (0xFFFFFFU << USART_RTOR_RTO_Pos)  /*!< 0x00FFFFFF */
#define USART_RTOR_RTO                      USART_RTOR_RTO_Msk                 /*!< Receiver Time Out Value */
#define USART_RTOR_BLEN_Pos                 (24U)
#define USART_RTOR_BLEN_Msk                 (0xFFU << USART_RTOR_BLEN_Pos)     /*!< 0xFF000000 */
#define USART_RTOR_BLEN                     USART_RTOR_BLEN_Msk                /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define USART_RQR_ABRRQ_Pos                 (0U)
#define USART_RQR_ABRRQ_Msk                 (0x1U << USART_RQR_ABRRQ_Pos)            /*!< 0x00000001 */
#define USART_RQR_ABRRQ                     USART_RQR_ABRRQ_Msk                      /*!< Auto-Baud Rate Request */
#define USART_RQR_SBKRQ_Pos                 (1U)
#define USART_RQR_SBKRQ_Msk                 (0x1U << USART_RQR_SBKRQ_Pos)            /*!< 0x00000002 */
#define USART_RQR_SBKRQ                     USART_RQR_SBKRQ_Msk                      /*!< Send Break Request */
#define USART_RQR_MMRQ_Pos                  (2U)
#define USART_RQR_MMRQ_Msk                  (0x1U << USART_RQR_MMRQ_Pos)             /*!< 0x00000004 */
#define USART_RQR_MMRQ                      USART_RQR_MMRQ_Msk                       /*!< Mute Mode Request */
#define USART_RQR_RXFRQ_Pos                 (3U)
#define USART_RQR_RXFRQ_Msk                 (0x1U << USART_RQR_RXFRQ_Pos)            /*!< 0x00000008 */
#define USART_RQR_RXFRQ                     USART_RQR_RXFRQ_Msk                      /*!< Receive Data flush Request */
#define USART_RQR_TXFRQ_Pos                 (4U)
#define USART_RQR_TXFRQ_Msk                 (0x1U << USART_RQR_TXFRQ_Pos)            /*!< 0x00000010 */
#define USART_RQR_TXFRQ                     USART_RQR_TXFRQ_Msk                      /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define USART_ISR_PE_Pos                    (0U)
#define USART_ISR_PE_Msk                    (0x1U << USART_ISR_PE_Pos)         /*!< 0x00000001 */
#define USART_ISR_PE                        USART_ISR_PE_Msk                   /*!< Parity Error */
#define USART_ISR_FE_Pos                    (1U)
#define USART_ISR_FE_Msk                    (0x1U << USART_ISR_FE_Pos)         /*!< 0x00000002 */
#define USART_ISR_FE                        USART_ISR_FE_Msk                   /*!< Framing Error */
#define USART_ISR_NE_Pos                    (2U)
#define USART_ISR_NE_Msk                    (0x1U << USART_ISR_NE_Pos)         /*!< 0x00000004 */
#define USART_ISR_NE                        USART_ISR_NE_Msk                   /*!< Noise detected Flag */
#define USART_ISR_ORE_Pos                   (3U)
#define USART_ISR_ORE_Msk                   (0x1U << USART_ISR_ORE_Pos)        /*!< 0x00000008 */
#define USART_ISR_ORE                       USART_ISR_ORE_Msk                  /*!< OverRun Error */
#define USART_ISR_IDLE_Pos                  (4U)
#define USART_ISR_IDLE_Msk                  (0x1U << USART_ISR_IDLE_Pos)       /*!< 0x00000010 */
#define USART_ISR_IDLE                      USART_ISR_IDLE_Msk                 /*!< IDLE line detected */
#define USART_ISR_RXNE_Pos                  (5U)
#define USART_ISR_RXNE_Msk                  (0x1U << USART_ISR_RXNE_Pos)       /*!< 0x00000020 */
#define USART_ISR_RXNE                      USART_ISR_RXNE_Msk                 /*!< Read Data Register Not Empty */
#define USART_ISR_TC_Pos                    (6U)
#define USART_ISR_TC_Msk                    (0x1U << USART_ISR_TC_Pos)         /*!< 0x00000040 */
#define USART_ISR_TC                        USART_ISR_TC_Msk                   /*!< Transmission Complete */
#define USART_ISR_TXE_Pos                   (7U)
#define USART_ISR_TXE_Msk                   (0x1U << USART_ISR_TXE_Pos)        /*!< 0x00000080 */
#define USART_ISR_TXE                       USART_ISR_TXE_Msk                  /*!< Transmit Data Register Empty */
#define USART_ISR_LBDF_Pos                  (8U)
#define USART_ISR_LBDF_Msk                  (0x1U << USART_ISR_LBDF_Pos)       /*!< 0x00000100 */
#define USART_ISR_LBDF                      USART_ISR_LBDF_Msk                 /*!< LIN Break Detection Flag */
#define USART_ISR_CTSIF_Pos                 (9U)
#define USART_ISR_CTSIF_Msk                 (0x1U << USART_ISR_CTSIF_Pos)      /*!< 0x00000200 */
#define USART_ISR_CTSIF                     USART_ISR_CTSIF_Msk                /*!< CTS interrupt flag */
#define USART_ISR_CTS_Pos                   (10U)
#define USART_ISR_CTS_Msk                   (0x1U << USART_ISR_CTS_Pos)        /*!< 0x00000400 */
#define USART_ISR_CTS                       USART_ISR_CTS_Msk                  /*!< CTS flag */
#define USART_ISR_RTOF_Pos                  (11U)
#define USART_ISR_RTOF_Msk                  (0x1U << USART_ISR_RTOF_Pos)       /*!< 0x00000800 */
#define USART_ISR_RTOF                      USART_ISR_RTOF_Msk                 /*!< Receiver Time Out */
#define USART_ISR_EOBF_Pos                  (12U)
#define USART_ISR_EOBF_Msk                  (0x1U << USART_ISR_EOBF_Pos)       /*!< 0x00001000 */
#define USART_ISR_EOBF                      USART_ISR_EOBF_Msk                 /*!< End Of Block Flag */
#define USART_ISR_ABRE_Pos                  (14U)
#define USART_ISR_ABRE_Msk                  (0x1U << USART_ISR_ABRE_Pos)       /*!< 0x00004000 */
#define USART_ISR_ABRE                      USART_ISR_ABRE_Msk                 /*!< Auto-Baud Rate Error */
#define USART_ISR_ABRF_Pos                  (15U)
#define USART_ISR_ABRF_Msk                  (0x1U << USART_ISR_ABRF_Pos)       /*!< 0x00008000 */
#define USART_ISR_ABRF                      USART_ISR_ABRF_Msk                 /*!< Auto-Baud Rate Flag */
#define USART_ISR_BUSY_Pos                  (16U)
#define USART_ISR_BUSY_Msk                  (0x1U << USART_ISR_BUSY_Pos)       /*!< 0x00010000 */
#define USART_ISR_BUSY                      USART_ISR_BUSY_Msk                 /*!< Busy Flag */
#define USART_ISR_CMF_Pos                   (17U)
#define USART_ISR_CMF_Msk                   (0x1U << USART_ISR_CMF_Pos)        /*!< 0x00020000 */
#define USART_ISR_CMF                       USART_ISR_CMF_Msk                  /*!< Character Match Flag */
#define USART_ISR_SBKF_Pos                  (18U)
#define USART_ISR_SBKF_Msk                  (0x1U << USART_ISR_SBKF_Pos)       /*!< 0x00040000 */
#define USART_ISR_SBKF                      USART_ISR_SBKF_Msk                 /*!< Send Break Flag */
#define USART_ISR_RWU_Pos                   (19U)
#define USART_ISR_RWU_Msk                   (0x1U << USART_ISR_RWU_Pos)        /*!< 0x00080000 */
#define USART_ISR_RWU                       USART_ISR_RWU_Msk                  /*!< Receive Wake Up from mute mode Flag */
#define USART_ISR_WUF_Pos                   (20U)
#define USART_ISR_WUF_Msk                   (0x1U << USART_ISR_WUF_Pos)        /*!< 0x00100000 */
#define USART_ISR_WUF                       USART_ISR_WUF_Msk                  /*!< Wake Up from stop mode Flag */
#define USART_ISR_TEACK_Pos                 (21U)
#define USART_ISR_TEACK_Msk                 (0x1U << USART_ISR_TEACK_Pos)      /*!< 0x00200000 */
#define USART_ISR_TEACK                     USART_ISR_TEACK_Msk                /*!< Transmit Enable Acknowledge Flag */
#define USART_ISR_REACK_Pos                 (22U)
#define USART_ISR_REACK_Msk                 (0x1U << USART_ISR_REACK_Pos)      /*!< 0x00400000 */
#define USART_ISR_REACK                     USART_ISR_REACK_Msk                /*!< Receive Enable Acknowledge Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define USART_ICR_PECF_Pos                  (0U)
#define USART_ICR_PECF_Msk                  (0x1U << USART_ICR_PECF_Pos)       /*!< 0x00000001 */
#define USART_ICR_PECF                      USART_ICR_PECF_Msk                 /*!< Parity Error Clear Flag */
#define USART_ICR_FECF_Pos                  (1U)
#define USART_ICR_FECF_Msk                  (0x1U << USART_ICR_FECF_Pos)       /*!< 0x00000002 */
#define USART_ICR_FECF                      USART_ICR_FECF_Msk                 /*!< Framing Error Clear Flag */
#define USART_ICR_NCF_Pos                   (2U)
#define USART_ICR_NCF_Msk                   (0x1U << USART_ICR_NCF_Pos)        /*!< 0x00000004 */
#define USART_ICR_NCF                       USART_ICR_NCF_Msk                  /*!< Noise detected Clear Flag */
#define USART_ICR_ORECF_Pos                 (3U)
#define USART_ICR_ORECF_Msk                 (0x1U << USART_ICR_ORECF_Pos)      /*!< 0x00000008 */
#define USART_ICR_ORECF                     USART_ICR_ORECF_Msk                /*!< OverRun Error Clear Flag */
#define USART_ICR_IDLECF_Pos                (4U)
#define USART_ICR_IDLECF_Msk                (0x1U << USART_ICR_IDLECF_Pos)     /*!< 0x00000010 */
#define USART_ICR_IDLECF                    USART_ICR_IDLECF_Msk               /*!< IDLE line detected Clear Flag */
#define USART_ICR_TCCF_Pos                  (6U)
#define USART_ICR_TCCF_Msk                  (0x1U << USART_ICR_TCCF_Pos)       /*!< 0x00000040 */
#define USART_ICR_TCCF                      USART_ICR_TCCF_Msk                 /*!< Transmission Complete Clear Flag */
#define USART_ICR_LBDCF_Pos                 (8U)
#define USART_ICR_LBDCF_Msk                 (0x1U << USART_ICR_LBDCF_Pos)      /*!< 0x00000100 */
#define USART_ICR_LBDCF                     USART_ICR_LBDCF_Msk                /*!< LIN Break Detection Clear Flag */
#define USART_ICR_CTSCF_Pos                 (9U)
#define USART_ICR_CTSCF_Msk                 (0x1U << USART_ICR_CTSCF_Pos)      /*!< 0x00000200 */
#define USART_ICR_CTSCF                     USART_ICR_CTSCF_Msk                /*!< CTS Interrupt Clear Flag */
#define USART_ICR_RTOCF_Pos                 (11U)
#define USART_ICR_RTOCF_Msk                 (0x1U << USART_ICR_RTOCF_Pos)      /*!< 0x00000800 */
#define USART_ICR_RTOCF                     USART_ICR_RTOCF_Msk                /*!< Receiver Time Out Clear Flag */
#define USART_ICR_EOBCF_Pos                 (12U)
#define USART_ICR_EOBCF_Msk                 (0x1U << USART_ICR_EOBCF_Pos)      /*!< 0x00001000 */
#define USART_ICR_EOBCF                     USART_ICR_EOBCF_Msk                /*!< End Of Block Clear Flag */
#define USART_ICR_CMCF_Pos                  (17U)
#define USART_ICR_CMCF_Msk                  (0x1U << USART_ICR_CMCF_Pos)       /*!< 0x00020000 */
#define USART_ICR_CMCF                      USART_ICR_CMCF_Msk                 /*!< Character Match Clear Flag */
#define USART_ICR_WUCF_Pos                  (20U)
#define USART_ICR_WUCF_Msk                  (0x1U << USART_ICR_WUCF_Pos)       /*!< 0x00100000 */
#define USART_ICR_WUCF                      USART_ICR_WUCF_Msk                 /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define USART_RDR_RDR_Pos                   (0U)
#define USART_RDR_RDR_Msk                   (0x1FFU << USART_RDR_RDR_Pos)            /*!< 0x000001FF */
#define USART_RDR_RDR                       USART_RDR_RDR_Msk                        /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define USART_TDR_TDR_Pos                   (0U)
#define USART_TDR_TDR_Msk                   (0x1FFU << USART_TDR_TDR_Pos)            /*!< 0x000001FF */
#define USART_TDR_TDR                       USART_TDR_TDR_Msk                        /*!< TDR[8:0] bits (Transmit Data value) */

/*********************RCC Define*********************/
typedef struct
{
  __IO uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
  __IO uint32_t ICSCR;       /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
  __IO uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
  __IO uint32_t PLLCFGR;     /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
  __IO uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 configuration register,                                     Address offset: 0x10 */
  __IO uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 configuration register,                                     Address offset: 0x14 */
  __IO uint32_t CIER;        /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
  __IO uint32_t CIFR;        /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
  __IO uint32_t CICR;        /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
  uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
  __IO uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
  __IO uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
  __IO uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
  __IO uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1,                                    Address offset: 0x38 */
  __IO uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2,                                    Address offset: 0x3C */
  __IO uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x40 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
  __IO uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clocks enable register,                              Address offset: 0x48 */
  __IO uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clocks enable register,                              Address offset: 0x4C */
  __IO uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clocks enable register,                              Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
  __IO uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clocks enable register 1,                            Address offset: 0x58 */
  __IO uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clocks enable register 2,                            Address offset: 0x5C */
  __IO uint32_t APB2ENR;     /*!< RCC APB2 peripheral clocks enable register,                              Address offset: 0x60 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
  __IO uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x68 */
  __IO uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x6C */
  __IO uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
  uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
  __IO uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1, Address offset: 0x78 */
  __IO uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2, Address offset: 0x7C */
  __IO uint32_t APB2SMENR;   /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register, Address offset: 0x80 */
  uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
  __IO uint32_t CCIPR;       /*!< RCC peripherals independent clock configuration register,                Address offset: 0x88 */
  __IO uint32_t RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
  __IO uint32_t BDCR;        /*!< RCC backup domain control register,                                      Address offset: 0x90 */
  __IO uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
} RCC_TypeDef;
/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint16_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  uint16_t  RESERVED2;       /*!< Reserved, 0x12                                                 */
  __IO uint32_t RTOR;        /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint16_t RQR;         /*!< USART Request register,                   Address offset: 0x18 */
  uint16_t  RESERVED3;       /*!< Reserved, 0x1A                                                 */
  __IO uint32_t ISR;         /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;         /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;         /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t  RESERVED4;       /*!< Reserved, 0x26                                                 */
  __IO uint16_t TDR;         /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t  RESERVED5;       /*!< Reserved, 0x2A                                                 */
} USART_TypeDef;

/*********************TIM Define*********************/
typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                            Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  __IO uint32_t OR1;         /*!< TIM option register 1,                    Address offset: 0x50 */
  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  __IO uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  __IO uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  __IO uint32_t OR2;         /*!< TIM option register 2,                    Address offset: 0x60 */
  __IO uint32_t OR3;         /*!< TIM option register 3,                    Address offset: 0x64 */
} TIM_TypeDef;



/*System Control Block (SCB)*/
typedef struct
{
  __IM  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __IOM uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __IOM uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __IOM uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __IOM uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __IOM uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __IOM uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __IOM uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __IOM uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __IOM uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __IOM uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __IOM uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __IOM uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __IOM uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __IM  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __IM  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __IM  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __IM  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __IM  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __IOM uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;


/**
 * @brief STM32L4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Cortex-M4 Non Maskable Interrupt                                */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts    */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                                   */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                                   */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                                   */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                                   */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                                   */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                                   */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                                   */
  ADC1_2_IRQn                 = 18,     /*!< ADC1, ADC2 SAR global Interrupts                                  */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM17 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  DFSDM3_IRQn                 = 42,     /*!< SD Filter 3 global Interrupt                                      */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                              */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                             */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt                            */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  ADC3_IRQn                   = 47,     /*!< ADC3 global  Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  DFSDM0_IRQn                 = 61,     /*!< SD Filter 0 global Interrupt                                      */
  DFSDM1_IRQn                 = 62,     /*!< SD Filter 1 global Interrupt                                      */
  DFSDM2_IRQn                 = 63,     /*!< SD Filter 2 global Interrupt                                      */
  COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                                        */
  LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                                 */
  LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                                 */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global interrupt                                   */
  LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                                                */
  QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                                         */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt                         */
  SAI2_IRQn                   = 75,     /*!< Serial Audio Interface 2 global interrupt                         */
  SWPMI1_IRQn                 = 76,     /*!< Serial Wire Interface 1 global interrupt                          */
  TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt                           */
  LCD_IRQn                    = 78,     /*!< LCD global interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
} IRQn_Type;

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  __IOM uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  __IOM uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RSERVED1[24U];
  __IOM uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  __IOM uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  __IOM uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  __IOM uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  __OM  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __IOM uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __IOM uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __IOM uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __IM  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;

/* SysTick Control / Status Register Definitions */
#define SysTick_CTRL_COUNTFLAG_Pos         16U                                            /*!< SysTick CTRL: COUNTFLAG Position */
#define SysTick_CTRL_COUNTFLAG_Msk         (1UL << SysTick_CTRL_COUNTFLAG_Pos)            /*!< SysTick CTRL: COUNTFLAG Mask */

#define SysTick_CTRL_CLKSOURCE_Pos          2U                                            /*!< SysTick CTRL: CLKSOURCE Position */
#define SysTick_CTRL_CLKSOURCE_Msk         (1UL << SysTick_CTRL_CLKSOURCE_Pos)            /*!< SysTick CTRL: CLKSOURCE Mask */

#define SysTick_CTRL_TICKINT_Pos            1U                                            /*!< SysTick CTRL: TICKINT Position */
#define SysTick_CTRL_TICKINT_Msk           (1UL << SysTick_CTRL_TICKINT_Pos)              /*!< SysTick CTRL: TICKINT Mask */

#define SysTick_CTRL_ENABLE_Pos             0U                                            /*!< SysTick CTRL: ENABLE Position */
#define SysTick_CTRL_ENABLE_Msk            (1UL /*<< SysTick_CTRL_ENABLE_Pos*/)           /*!< SysTick CTRL: ENABLE Mask */

/* SysTick Reload Register Definitions */
#define SysTick_LOAD_RELOAD_Pos             0U                                            /*!< SysTick LOAD: RELOAD Position */
#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFUL /*<< SysTick_LOAD_RELOAD_Pos*/)    /*!< SysTick LOAD: RELOAD Mask */

/* SysTick Current Register Definitions */
#define SysTick_VAL_CURRENT_Pos             0U                                            /*!< SysTick VAL: CURRENT Position */
#define SysTick_VAL_CURRENT_Msk            (0xFFFFFFUL /*<< SysTick_VAL_CURRENT_Pos*/)    /*!< SysTick VAL: CURRENT Mask */

/* SysTick Calibration Register Definitions */
#define SysTick_CALIB_NOREF_Pos            31U                                            /*!< SysTick CALIB: NOREF Position */
#define SysTick_CALIB_NOREF_Msk            (1UL << SysTick_CALIB_NOREF_Pos)               /*!< SysTick CALIB: NOREF Mask */

#define SysTick_CALIB_SKEW_Pos             30U                                            /*!< SysTick CALIB: SKEW Position */
#define SysTick_CALIB_SKEW_Msk             (1UL << SysTick_CALIB_SKEW_Pos)                /*!< SysTick CALIB: SKEW Mask */

#define SysTick_CALIB_TENMS_Pos             0U                                            /*!< SysTick CALIB: TENMS Position */
#define SysTick_CALIB_TENMS_Msk            (0xFFFFFFUL /*<< SysTick_CALIB_TENMS_Pos*/)    /*!< SysTick CALIB: TENMS Mask */

/*@} end of group CMSIS_SysTick */


#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define SCB                 ((SCB_Type *)     SCB_BASE)   /*!< SCB configuration struct */

#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */


#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000U)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00U)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000U)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400U)

#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)

#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000U)
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x0800U)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x1000U)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x1400U)
#define GPIOG_BASE            (AHB2PERIPH_BASE + 0x1800U)
#define GPIOH_BASE            (AHB2PERIPH_BASE + 0x1C00U)
#define GPIOA                 ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)

#define PERIPH_BASE           ((uint32_t)0x40000000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x1000U)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)

#define __NVIC_PRIO_BITS          4       /*!< STM32L4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */


/**
  \brief   Set Interrupt Priority
  \details Sets the priority of an interrupt.
  \note    The priority cannot be set for every core interrupt.
  \param [in]      IRQn  Interrupt number.
  \param [in]  priority  Priority to set.
 */
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    SCB->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
  else
  {
    NVIC->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
  }
}

/* ##################################    SysTick function  ############################################ */
/**
  \ingroup  CMSIS_Core_FunctionInterface
  \defgroup CMSIS_Core_SysTickFunctions SysTick Functions
  \brief    Functions that configure the System.
  @{
 */

#if (__Vendor_SysTickConfig == 0U)

/**
  \brief   System Tick Configuration
  \details Initializes the System Timer and its interrupt, and starts the System Tick Timer.
           Counter is in free running mode to generate periodic interrupts.
  \param [in]  ticks  Number of ticks between two interrupts.
  \return          0  Function succeeded.
  \return          1  Function failed.
  \note    When the variable <b>__Vendor_SysTickConfig</b> is set to 1, then the
           function <b>SysTick_Config</b> is not included. In this case, the file <b><i>device</i>.h</b>
           must contain a vendor-specific implementation of this function.
 */
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > SysTick_LOAD_RELOAD_Msk)
  {
    return (1UL);                                                   /* Reload value impossible */
  }

  SysTick->LOAD  = (uint32_t)(ticks - 1UL);                         /* set reload register */
  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                         /* Enable SysTick IRQ and SysTick Timer */
  return (0UL);                                                     /* Function successful */
}

#endif

/*@} end of CMSIS_Core_SysTickFunctions */

//static inline void delay_us (int micros) {
//	/* Go to clock cycles */
//	micros *= (4000000 / 1000000) / 5;
//
//	/* Wait till done */
//	while (micros--);
//
//}

static inline void delay_us(int n_in){
	asm("push {r0}\r\n"
			"mov r0, r0\r\n"
			"LOOP_US:\r\n"
			"nop\r\n"
			"subs r0, #1\r\n"
			"BGT LOOP_US\r\n"
			"POP {r0}\r\n"
			:: "r" (n_in));

}

static inline void delay_ms(int n){
	asm("push {r0}\r\n"
			"mov r0, %0\r\n"
			"LOOP:\r\n"
			"subs r0, #1\r\n"
			"BGT LOOP\r\n"
			"POP {r0}\r\n"
			:: "r" (n*1333));
}



#endif /* REF_H_ */
