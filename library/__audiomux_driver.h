/*
    __audiomux_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __audiomux_driver.h
@brief    AudioMUX Driver
@mainpage AudioMUX Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   AUDIOMUX
@brief      AudioMUX Click Driver
@{

| Global Library Prefix | **AUDIOMUX** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Oct 2018.**      |
| Developer             | **Nemanja Medakovic**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _AUDIOMUX_H_
#define _AUDIOMUX_H_

/** 
 * @macro T_AUDIOMUX_P
 * @brief Driver Abstract type 
 */
#define T_AUDIOMUX_P    const uint8_t*
#define T_AUDIOMUX_RETVAL     uint8_t

/** @defgroup AUDIOMUX_COMPILE Compilation Config */              /** @{ */

//  #define   __AUDIOMUX_DRV_SPI__                            /**<     @macro __AUDIOMUX_DRV_SPI__  @brief SPI driver selector */
   #define   __AUDIOMUX_DRV_I2C__                            /**<     @macro __AUDIOMUX_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __AUDIOMUX_DRV_UART__                           /**<     @macro __AUDIOMUX_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup AUDIOMUX_VAR Variables */                           /** @{ */

/** Device Slave Address */
extern const uint8_t _AUDIOMUX_DEVICE_ADDR            ;

/** Address auto increment */
extern const uint8_t _AUDIOMUX_AUTO_INCR              ;

/** Memory Addresses */
extern const uint8_t _AUDIOMUX_INPUT_SEL_ADDR         ;
extern const uint8_t _AUDIOMUX_INPUT_GAIN_ADDR        ;
extern const uint8_t _AUDIOMUX_SURROUND_ADDR          ;
extern const uint8_t _AUDIOMUX_VOLUME_LEFT_ADDR       ;
extern const uint8_t _AUDIOMUX_VOLUME_RIGHT_ADDR      ;
extern const uint8_t _AUDIOMUX_TREBLE_BASS_ADDR       ;
extern const uint8_t _AUDIOMUX_OUTPUT_ADDR            ;
extern const uint8_t _AUDIOMUX_BASS_ALC_ADDR          ;

/** Settings for Input Selection and MIC */
extern const uint8_t _AUDIOMUX_IN1_EN                 ;
extern const uint8_t _AUDIOMUX_IN2_EN                 ;
extern const uint8_t _AUDIOMUX_IN3_EN                 ;
extern const uint8_t _AUDIOMUX_IN4_EN                 ;
extern const uint8_t _AUDIOMUX_MIC_GAIN_14DB          ;
extern const uint8_t _AUDIOMUX_MIC_GAIN_10DB          ;
extern const uint8_t _AUDIOMUX_MIC_GAIN_6DB           ;
extern const uint8_t _AUDIOMUX_MIC_GAIN_0DB           ;
extern const uint8_t _AUDIOMUX_MIC_OFF                ;
extern const uint8_t _AUDIOMUX_MIC_ON                 ;

/** Settings for Mute Input */
extern const uint8_t _AUDIOMUX_MUTE_INPUT_ON          ;
extern const uint8_t _AUDIOMUX_MUTE_INPUT_OFF         ;

/** Settings for Input Gain Selection */
extern const uint8_t _AUDIOMUX_IN_GAIN_0DB            ;
extern const uint8_t _AUDIOMUX_IN_GAIN_2DB            ;
extern const uint8_t _AUDIOMUX_IN_GAIN_4DB            ;
extern const uint8_t _AUDIOMUX_IN_GAIN_6DB            ;
extern const uint8_t _AUDIOMUX_IN_GAIN_8DB            ;
extern const uint8_t _AUDIOMUX_IN_GAIN_10DB           ;
extern const uint8_t _AUDIOMUX_IN_GAIN_12DB           ;
extern const uint8_t _AUDIOMUX_IN_GAIN_14DB           ;

/** Surround and Mixing Settings */
extern const uint8_t _AUDIOMUX_SURROUND_MODE_ON       ;
extern const uint8_t _AUDIOMUX_SURROUND_MODE_OFF      ;
extern const uint8_t _AUDIOMUX_SURROUND_GAIN_0DB      ;
extern const uint8_t _AUDIOMUX_SURROUND_GAIN_6DB      ;
extern const uint8_t _AUDIOMUX_SURROUND_GAIN_9DB      ;
extern const uint8_t _AUDIOMUX_SURROUND_GAIN_12DB     ;
extern const uint8_t _AUDIOMUX_MIX_INV_100            ;
extern const uint8_t _AUDIOMUX_MIX_INV_50             ;
extern const uint8_t _AUDIOMUX_MIX_INV_25             ;
extern const uint8_t _AUDIOMUX_MIX_0                  ;
extern const uint8_t _AUDIOMUX_MIX_NONINV_100         ;
extern const uint8_t _AUDIOMUX_MIX_NONINV_75          ;
extern const uint8_t _AUDIOMUX_MIX_NONINV_50          ;
extern const uint8_t _AUDIOMUX_MIX_MUTE               ;
extern const uint8_t _AUDIOMUX_BUFF_GAIN_0            ;
extern const uint8_t _AUDIOMUX_BUFF_GAIN_6DB          ;

/** Settings for Volume */
extern const uint8_t _AUDIOMUX_VOL1_0DB               ;
extern const uint8_t _AUDIOMUX_VOL1_1DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_2DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_3DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_4DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_5DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_6DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_7DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_8DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL1_16DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL1_24DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL1_32DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL1_40DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL1_48DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL1_56DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL2_0DB               ;
extern const uint8_t _AUDIOMUX_VOL2_8DB_NEG           ;
extern const uint8_t _AUDIOMUX_VOL2_16DB_NEG          ;
extern const uint8_t _AUDIOMUX_VOL2_24DB_NEG          ;

/** Settings for Treble and Bass */
extern const uint8_t _AUDIOMUX_TRE_BASS_14DB_NEG      ;
extern const uint8_t _AUDIOMUX_TRE_BASS_12DB_NEG      ;
extern const uint8_t _AUDIOMUX_TRE_BASS_10DB_NEG      ;
extern const uint8_t _AUDIOMUX_TRE_BASS_8DB_NEG       ;
extern const uint8_t _AUDIOMUX_TRE_BASS_6DB_NEG       ;
extern const uint8_t _AUDIOMUX_TRE_BASS_4DB_NEG       ;
extern const uint8_t _AUDIOMUX_TRE_BASS_2DB_NEG       ;
extern const uint8_t _AUDIOMUX_TRE_BASS_0DB           ;
extern const uint8_t _AUDIOMUX_TRE_BASS_14DB          ;
extern const uint8_t _AUDIOMUX_TRE_BASS_12DB          ;
extern const uint8_t _AUDIOMUX_TRE_BASS_10DB          ;
extern const uint8_t _AUDIOMUX_TRE_BASS_8DB           ;
extern const uint8_t _AUDIOMUX_TRE_BASS_6DB           ;
extern const uint8_t _AUDIOMUX_TRE_BASS_4DB           ;
extern const uint8_t _AUDIOMUX_TRE_BASS_2DB           ;

/** Settings for Mute Output */
extern const uint8_t _AUDIOMUX_MUTE_OUTPUT_ON         ;
extern const uint8_t _AUDIOMUX_MUTE_OUTPUT_OFF        ;

/** Settings for Bass ALC Mode */
extern const uint8_t _AUDIOMUX_ALC_MODE_ON            ;
extern const uint8_t _AUDIOMUX_DETECTOR_ON            ;
extern const uint8_t _AUDIOMUX_RELEASE_CURR           ;
extern const uint8_t _AUDIOMUX_TIME_RES_12K5          ;
extern const uint8_t _AUDIOMUX_TIME_RES_25K           ;
extern const uint8_t _AUDIOMUX_TIME_RES_50K           ;
extern const uint8_t _AUDIOMUX_TIME_RES_100K          ;
extern const uint8_t _AUDIOMUX_THRSH_700MVRMS         ;
extern const uint8_t _AUDIOMUX_THRSH_485MVRMS         ;
extern const uint8_t _AUDIOMUX_THRSH_320MVRMS         ;
extern const uint8_t _AUDIOMUX_THRSH_170MVRMS         ;
extern const uint8_t _AUDIOMUX_MODE1_FIXED_RES        ;
extern const uint8_t _AUDIOMUX_MODE2_ADAPTIVE         ;

/** Returned values for AudioMUX */
extern const uint8_t _AUDIOMUX_OK                     ;
extern const uint8_t _AUDIOMUX_ADDR_ERR               ;
extern const uint8_t _AUDIOMUX_NBYTES_ERR             ;
extern const uint8_t _AUDIOMUX_IN_SEL_ERR             ;
extern const uint8_t _AUDIOMUX_GAIN_ERR               ;
extern const uint8_t _AUDIOMUX_STATE_ERR              ;
extern const uint8_t _AUDIOMUX_VOL_ERR                ;
extern const uint8_t _AUDIOMUX_TRE_BASS_ERR           ;

                                                                       /** @} */
/** @defgroup AUDIOMUX_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup AUDIOMUX_INIT Driver Initialization */              /** @{ */

#ifdef   __AUDIOMUX_DRV_SPI__
void audiomux_spiDriverInit(T_AUDIOMUX_P gpioObj, T_AUDIOMUX_P spiObj);
#endif
#ifdef   __AUDIOMUX_DRV_I2C__
void audiomux_i2cDriverInit(T_AUDIOMUX_P gpioObj, T_AUDIOMUX_P i2cObj, uint8_t slave);
#endif
#ifdef   __AUDIOMUX_DRV_UART__
void audiomux_uartDriverInit(T_AUDIOMUX_P gpioObj, T_AUDIOMUX_P uartObj);
#endif

                                                                       /** @} */
/** @defgroup AUDIOMUX_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Single Write function
 *
 * @param[in] regAddr  Address where data be written
 * @param[in] dataIn  Data to be written
 *
 * @returns 0 - OK, 1 - Wrong address
 *
 * Function writes one byte data to the register.
 */
T_AUDIOMUX_RETVAL audiomux_writeSingleByte( uint8_t regAddr, uint8_t dataIn );

/**
 * @brief Multiple Write function
 *
 * @param[in] startAddr  Address which from data writing be started
 * @param[in] dataIn  Memory which from data be written
 * @param[in] nBytes  Number of bytes to be written
 *
 * @returns 0 - OK, 1 - Wrong address, 2 - Number of bytes is out of range
 *
 * Function writes the desired number of bytes to the registers.
 */
T_AUDIOMUX_RETVAL audiomux_writeBytes( uint8_t startAddr, uint8_t *dataIn, uint8_t nBytes );

/**
 * @brief Single Read function
 *
 * @param[in] regAddr  Address which from data be read
 * @param[out] dataOut  Memory where data be stored
 *
 * @returns 0 - OK, 1 - Wrong address
 *
 * Function reads one byte data from the register.
 */
T_AUDIOMUX_RETVAL audiomux_readSingleByte( uint8_t regAddr, uint8_t *dataOut );

/**
 * @brief Multiple Read function
 *
 * @param[in] startAddr  Address which from data read be started
 * @param[out] dataOut  Memory where data be stored
 * @param[in] nBytes  Number of bytes to be read
 *
 * @returns 0 - OK, 1 - Wrong address, 2 - Number of bytes is out of range
 *
 * Function reads the desired number of bytes from the registers.
 */
T_AUDIOMUX_RETVAL audiomux_readBytes( uint8_t startAddr, uint8_t *dataOut, uint8_t nBytes );

/**
 * @brief Input Selection function
 *
 * @param[in] inputSel  Input channel selection, from 0 (IN1) to 3 (IN4)
 * @param[in] muteIn  1 - Input Mute ON, 0 - Input Mute OFF
 * @param[in] micGain  From 0 (14dB) to 3 (0dB)
 * @param[in] micState  0 - MIC ON, 1 - MIC OFF
 *
 * @returns 0 - OK, 3 - Input Selection Error, 4 - Gain Error, 5 - State Error
 *
 * Function performs a input selection and control.
 */
T_AUDIOMUX_RETVAL audiomux_selectInput( uint8_t inputSel, uint8_t muteIn, uint8_t micGain, uint8_t micState );

/**
 * @brief Input Gain Control function
 *
 * @param[in] inputGain  From 0 (0dB) to 7 (14dB)
 *
 * @returns 0 - OK, 4 - Gain Error
 *
 * Function performs a input gain selection.
 */
T_AUDIOMUX_RETVAL audiomux_setInputGain( uint8_t inputGain );

/**
 * @brief Volume Control function
 *
 * @param[in] volume1  Pre-EQ section volume, 0x00 is highest volume and 0x3F is lowest volume
 * @param[in] volume2  Post-EQ section volume, 0x00 is highest and 0x03 is lowest volume
 * @param[in] sideSel  0x03 - Left volume control, 0x04 - Right volume control
 *
 * @returns 0 - OK, 1 - Wrong side address, 6 - Wrong volume selection
 *
 * Function performs a left or right volume control.
 */
T_AUDIOMUX_RETVAL audiomux_setVolume( uint8_t volume1, uint8_t volume2, uint8_t sideSel );

/**
 * @brief Treble and Bass Control function
 *
 * @param[in] treble  Treble selection from -14dB to 14dB
 * @param[in] bass  Bass selection from -14dB to 14dB
 *
 * @returns 0 - OK, 7 - Treble or Bass Selection Error
 *
 * Function performs a control of the treble and bass for EQ section.
 */
T_AUDIOMUX_RETVAL audiomux_setTrebleBass( uint8_t treble, uint8_t bass );

/**
 * @brief Output Mute function
 *
 * @param[in] state  0 - Output Mute ON, 1 - Output Mute OFF
 *
 * @returns 0 - OK, 5 - Parameter State Error
 *
 * Function performs a control of the output to be Mute ON or Mute OFF.
 */
T_AUDIOMUX_RETVAL audiomux_muteOuput( uint8_t state );

                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_AudioMUX_STM.c
    @example Click_AudioMUX_TIVA.c
    @example Click_AudioMUX_CEC.c
    @example Click_AudioMUX_KINETIS.c
    @example Click_AudioMUX_MSP.c
    @example Click_AudioMUX_PIC.c
    @example Click_AudioMUX_PIC32.c
    @example Click_AudioMUX_DSPIC.c
    @example Click_AudioMUX_AVR.c
    @example Click_AudioMUX_FT90x.c
    @example Click_AudioMUX_STM.mbas
    @example Click_AudioMUX_TIVA.mbas
    @example Click_AudioMUX_CEC.mbas
    @example Click_AudioMUX_KINETIS.mbas
    @example Click_AudioMUX_MSP.mbas
    @example Click_AudioMUX_PIC.mbas
    @example Click_AudioMUX_PIC32.mbas
    @example Click_AudioMUX_DSPIC.mbas
    @example Click_AudioMUX_AVR.mbas
    @example Click_AudioMUX_FT90x.mbas
    @example Click_AudioMUX_STM.mpas
    @example Click_AudioMUX_TIVA.mpas
    @example Click_AudioMUX_CEC.mpas
    @example Click_AudioMUX_KINETIS.mpas
    @example Click_AudioMUX_MSP.mpas
    @example Click_AudioMUX_PIC.mpas
    @example Click_AudioMUX_PIC32.mpas
    @example Click_AudioMUX_DSPIC.mpas
    @example Click_AudioMUX_AVR.mpas
    @example Click_AudioMUX_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __audiomux_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */