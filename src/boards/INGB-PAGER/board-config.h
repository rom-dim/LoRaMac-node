/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
 *
 * \copyright TODO
 *
 * \code TODO
  *
 * \endcode
 *
 * \author  Eugen Geier TODO (Ing. Buero....)
 * 
 * \author   Dmitirj Belousov
 *
  */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#if defined( SX1262MBXDAS )
#define BOARD_TCXO_WAKEUP_TIME                      5
#else
#define BOARD_TCXO_WAKEUP_TIME                      0
#endif

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PF_13

#define RADIO_MOSI                                  PA_7
#define RADIO_MISO                                  PA_6
#define RADIO_SCLK                                  PA_5

#if defined( SX1262MBXCAS )

#define RADIO_NSS                                   PB_0
#define RADIO_BUSY                                  PF_12
#define RADIO_DIO_1                                 PF_14

//#define RADIO_ANT_SWITCH_POWER                      PA_9 /* INGB? fix ? */
#define RADIO_FREQ_SEL                              PI11 /* LRW_BAND_SEL NET? */
//#define RADIO_XTAL_SEL                              PB_0 #INGB?
//#define RADIO_DEVICE_SEL                            PA_4 #INGB?

#define LED_1                                       PA_1
#define LED_2                                       PA_2

// Debug pins definition.
//#define RADIO_DBG_PIN_TX                            PB_6
//#define RADIO_DBG_PIN_RX                            PC_7

#endif
/*
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define SWCLK                                       PA_14
#define SWDAT                                       PA_13*/

#define UART_TX                                     PB_6
#define UART_RX                                     PB_7

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
