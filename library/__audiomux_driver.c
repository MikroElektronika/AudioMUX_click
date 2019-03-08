/*
    __audiomux_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__audiomux_driver.h"
#include "__audiomux_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __AUDIOMUX_DRV_I2C__
static uint8_t _slaveAddress;
#endif

const uint8_t _AUDIOMUX_DEVICE_ADDR                   = 0x44;

const uint8_t _AUDIOMUX_AUTO_INCR                     = 0x10;

const uint8_t _AUDIOMUX_INPUT_SEL_ADDR                = 0x00;
const uint8_t _AUDIOMUX_INPUT_GAIN_ADDR               = 0x01;
const uint8_t _AUDIOMUX_SURROUND_ADDR                 = 0x02;
const uint8_t _AUDIOMUX_VOLUME_LEFT_ADDR              = 0x03;
const uint8_t _AUDIOMUX_VOLUME_RIGHT_ADDR             = 0x04;
const uint8_t _AUDIOMUX_TREBLE_BASS_ADDR              = 0x05;
const uint8_t _AUDIOMUX_OUTPUT_ADDR                   = 0x06;
const uint8_t _AUDIOMUX_BASS_ALC_ADDR                 = 0x07;

const uint8_t _AUDIOMUX_IN1_EN                        = 0x00;
const uint8_t _AUDIOMUX_IN2_EN                        = 0x01;
const uint8_t _AUDIOMUX_IN3_EN                        = 0x02;
const uint8_t _AUDIOMUX_IN4_EN                        = 0x03;
const uint8_t _AUDIOMUX_MIC_GAIN_14DB                 = 0x00;
const uint8_t _AUDIOMUX_MIC_GAIN_10DB                 = 0x01;
const uint8_t _AUDIOMUX_MIC_GAIN_6DB                  = 0x02;
const uint8_t _AUDIOMUX_MIC_GAIN_0DB                  = 0x03;
const uint8_t _AUDIOMUX_MIC_OFF                       = 0x01;
const uint8_t _AUDIOMUX_MIC_ON                        = 0x00;

const uint8_t _AUDIOMUX_MUTE_INPUT_ON                 = 0x01;
const uint8_t _AUDIOMUX_MUTE_INPUT_OFF                = 0x00;

const uint8_t _AUDIOMUX_IN_GAIN_0DB                   = 0x00;
const uint8_t _AUDIOMUX_IN_GAIN_2DB                   = 0x01;
const uint8_t _AUDIOMUX_IN_GAIN_4DB                   = 0x02;
const uint8_t _AUDIOMUX_IN_GAIN_6DB                   = 0x03;
const uint8_t _AUDIOMUX_IN_GAIN_8DB                   = 0x04;
const uint8_t _AUDIOMUX_IN_GAIN_10DB                  = 0x05;
const uint8_t _AUDIOMUX_IN_GAIN_12DB                  = 0x06;
const uint8_t _AUDIOMUX_IN_GAIN_14DB                  = 0x07;

const uint8_t _AUDIOMUX_SURROUND_MODE_ON              = 0x01;
const uint8_t _AUDIOMUX_SURROUND_MODE_OFF             = 0x00;
const uint8_t _AUDIOMUX_SURROUND_GAIN_0DB             = 0x00;
const uint8_t _AUDIOMUX_SURROUND_GAIN_6DB             = 0x02;
const uint8_t _AUDIOMUX_SURROUND_GAIN_9DB             = 0x04;
const uint8_t _AUDIOMUX_SURROUND_GAIN_12DB            = 0x06;
const uint8_t _AUDIOMUX_MIX_INV_100                   = 0x00;
const uint8_t _AUDIOMUX_MIX_INV_50                    = 0x08;
const uint8_t _AUDIOMUX_MIX_INV_25                    = 0x10;
const uint8_t _AUDIOMUX_MIX_0                         = 0x18;
const uint8_t _AUDIOMUX_MIX_NONINV_100                = 0x20;
const uint8_t _AUDIOMUX_MIX_NONINV_75                 = 0x28;
const uint8_t _AUDIOMUX_MIX_NONINV_50                 = 0x30;
const uint8_t _AUDIOMUX_MIX_MUTE                      = 0x38;
const uint8_t _AUDIOMUX_BUFF_GAIN_0                   = 0x40;
const uint8_t _AUDIOMUX_BUFF_GAIN_6DB                 = 0x00;

const uint8_t _AUDIOMUX_VOL1_0DB                      = 0x00;
const uint8_t _AUDIOMUX_VOL1_1DB_NEG                  = 0x01;
const uint8_t _AUDIOMUX_VOL1_2DB_NEG                  = 0x02;
const uint8_t _AUDIOMUX_VOL1_3DB_NEG                  = 0x03;
const uint8_t _AUDIOMUX_VOL1_4DB_NEG                  = 0x04;
const uint8_t _AUDIOMUX_VOL1_5DB_NEG                  = 0x05;
const uint8_t _AUDIOMUX_VOL1_6DB_NEG                  = 0x06;
const uint8_t _AUDIOMUX_VOL1_7DB_NEG                  = 0x07;
const uint8_t _AUDIOMUX_VOL1_8DB_NEG                  = 0x08;
const uint8_t _AUDIOMUX_VOL1_16DB_NEG                 = 0x10;
const uint8_t _AUDIOMUX_VOL1_24DB_NEG                 = 0x18;
const uint8_t _AUDIOMUX_VOL1_32DB_NEG                 = 0x20;
const uint8_t _AUDIOMUX_VOL1_40DB_NEG                 = 0x28;
const uint8_t _AUDIOMUX_VOL1_48DB_NEG                 = 0x30;
const uint8_t _AUDIOMUX_VOL1_56DB_NEG                 = 0x38;
const uint8_t _AUDIOMUX_VOL2_0DB                      = 0x00;
const uint8_t _AUDIOMUX_VOL2_8DB_NEG                  = 0x01;
const uint8_t _AUDIOMUX_VOL2_16DB_NEG                 = 0x02;
const uint8_t _AUDIOMUX_VOL2_24DB_NEG                 = 0x03;

const uint8_t _AUDIOMUX_TRE_BASS_14DB_NEG             = 0x00;
const uint8_t _AUDIOMUX_TRE_BASS_12DB_NEG             = 0x01;
const uint8_t _AUDIOMUX_TRE_BASS_10DB_NEG             = 0x02;
const uint8_t _AUDIOMUX_TRE_BASS_8DB_NEG              = 0x03;
const uint8_t _AUDIOMUX_TRE_BASS_6DB_NEG              = 0x04;
const uint8_t _AUDIOMUX_TRE_BASS_4DB_NEG              = 0x05;
const uint8_t _AUDIOMUX_TRE_BASS_2DB_NEG              = 0x06;
const uint8_t _AUDIOMUX_TRE_BASS_0DB                  = 0x07;
const uint8_t _AUDIOMUX_TRE_BASS_14DB                 = 0x08;
const uint8_t _AUDIOMUX_TRE_BASS_12DB                 = 0x09;
const uint8_t _AUDIOMUX_TRE_BASS_10DB                 = 0x0A;
const uint8_t _AUDIOMUX_TRE_BASS_8DB                  = 0x0B;
const uint8_t _AUDIOMUX_TRE_BASS_6DB                  = 0x0C;
const uint8_t _AUDIOMUX_TRE_BASS_4DB                  = 0x0D;
const uint8_t _AUDIOMUX_TRE_BASS_2DB                  = 0x0E;

const uint8_t _AUDIOMUX_MUTE_OUTPUT_ON                = 0x00;
const uint8_t _AUDIOMUX_MUTE_OUTPUT_OFF               = 0x01;

const uint8_t _AUDIOMUX_ALC_MODE_ON                   = 0x01;
const uint8_t _AUDIOMUX_DETECTOR_ON                   = 0x02;
const uint8_t _AUDIOMUX_RELEASE_CURR                  = 0x04;
const uint8_t _AUDIOMUX_TIME_RES_12K5                 = 0x00;
const uint8_t _AUDIOMUX_TIME_RES_25K                  = 0x08;
const uint8_t _AUDIOMUX_TIME_RES_50K                  = 0x10;
const uint8_t _AUDIOMUX_TIME_RES_100K                 = 0x18;
const uint8_t _AUDIOMUX_THRSH_700MVRMS                = 0x00;
const uint8_t _AUDIOMUX_THRSH_485MVRMS                = 0x20;
const uint8_t _AUDIOMUX_THRSH_320MVRMS                = 0x40;
const uint8_t _AUDIOMUX_THRSH_170MVRMS                = 0x60;
const uint8_t _AUDIOMUX_MODE1_FIXED_RES               = 0x00;
const uint8_t _AUDIOMUX_MODE2_ADAPTIVE                = 0x80;

const uint8_t _AUDIOMUX_OK                            = 0x00;
const uint8_t _AUDIOMUX_ADDR_ERR                      = 0x01;
const uint8_t _AUDIOMUX_NBYTES_ERR                    = 0x02;
const uint8_t _AUDIOMUX_IN_SEL_ERR                    = 0x03;
const uint8_t _AUDIOMUX_GAIN_ERR                      = 0x04;
const uint8_t _AUDIOMUX_STATE_ERR                     = 0x05;
const uint8_t _AUDIOMUX_VOL_ERR                       = 0x06;
const uint8_t _AUDIOMUX_TRE_BASS_ERR                  = 0x07;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __AUDIOMUX_DRV_SPI__

void audiomux_spiDriverInit(T_AUDIOMUX_P gpioObj, T_AUDIOMUX_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __AUDIOMUX_DRV_I2C__

void audiomux_i2cDriverInit(T_AUDIOMUX_P gpioObj, T_AUDIOMUX_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __AUDIOMUX_DRV_UART__

void audiomux_uartDriverInit(T_AUDIOMUX_P gpioObj, T_AUDIOMUX_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

T_AUDIOMUX_RETVAL audiomux_writeSingleByte( uint8_t regAddr, uint8_t dataIn )
{
    uint8_t tempData[ 2 ];

    if (regAddr > _AUDIOMUX_BASS_ALC_ADDR)
    {
        return _AUDIOMUX_ADDR_ERR;
    }

    tempData[ 0 ] = regAddr;
    tempData[ 1 ] = dataIn;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, 2, END_MODE_STOP );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_writeBytes( uint8_t startAddr, uint8_t *dataIn, uint8_t nBytes )
{
    uint8_t tempData[ 9 ];
    uint8_t cnt;

    if (startAddr > _AUDIOMUX_BASS_ALC_ADDR)
    {
        return _AUDIOMUX_ADDR_ERR;
    }
    if (nBytes > (0x08 - startAddr))
    {
        return _AUDIOMUX_NBYTES_ERR;
    }

    tempData[ 0 ] = startAddr | _AUDIOMUX_AUTO_INCR;

    for (cnt = 0; cnt < nBytes; cnt++)
    {
        tempData[ cnt + 1 ] = dataIn[ cnt ];
    }

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, tempData, nBytes + 1, END_MODE_STOP );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_readSingleByte( uint8_t regAddr, uint8_t *dataOut )
{
    uint8_t tempData;

    if (regAddr > _AUDIOMUX_BASS_ALC_ADDR)
    {
        return _AUDIOMUX_ADDR_ERR;
    }

    tempData = regAddr;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, &tempData, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, dataOut, 1, END_MODE_STOP );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_readBytes( uint8_t startAddr, uint8_t *dataOut, uint8_t nBytes )
{
    uint8_t tempData;

    if (startAddr > _AUDIOMUX_BASS_ALC_ADDR)
    {
        return _AUDIOMUX_ADDR_ERR;
    }
    if (nBytes > (0x08 - startAddr))
    {
        return _AUDIOMUX_NBYTES_ERR;
    }

    tempData = startAddr | _AUDIOMUX_AUTO_INCR;

    hal_i2cStart();
    hal_i2cWrite( _slaveAddress, &tempData, 1, END_MODE_RESTART );
    hal_i2cRead( _slaveAddress, dataOut, nBytes, END_MODE_STOP );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_selectInput( uint8_t inputSel, uint8_t muteIn, uint8_t micGain, uint8_t micState )
{
    uint8_t byteIn;

    if (inputSel > 0x03)
    {
        return _AUDIOMUX_IN_SEL_ERR;
    }
    if (micGain > 0x03)
    {
        return _AUDIOMUX_GAIN_ERR;
    }
    if ((micState > 0x01) || (muteIn > 0x01))
    {
        return _AUDIOMUX_STATE_ERR;
    }

    byteIn = micState << 5;
    byteIn |= micGain << 3;
    byteIn |= muteIn << 2;
    byteIn |= inputSel;

    audiomux_writeSingleByte( _AUDIOMUX_INPUT_SEL_ADDR, byteIn );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_setInputGain( uint8_t inputGain )
{
    uint8_t byteIn;

    if (inputGain > 0x07)
    {
        return _AUDIOMUX_GAIN_ERR;
    }

    byteIn = inputGain;

    audiomux_writeSingleByte( _AUDIOMUX_INPUT_GAIN_ADDR, byteIn );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_setVolume( uint8_t volume1, uint8_t volume2, uint8_t sideSel )
{
    uint8_t byteIn;
    uint8_t sideAddr;

    if ((volume1 > 0x3F) || (volume2 > 0x03))
    {
        return _AUDIOMUX_VOL_ERR;
    }
    if ((sideSel != _AUDIOMUX_VOLUME_LEFT_ADDR) && (sideSel != _AUDIOMUX_VOLUME_RIGHT_ADDR))
    {
        return _AUDIOMUX_ADDR_ERR;
    }

    byteIn = volume2 << 6;
    byteIn |= volume1;
    sideAddr = sideSel;

    audiomux_writeSingleByte( sideAddr, byteIn );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_setTrebleBass( uint8_t treble, uint8_t bass )
{
    uint8_t byteIn;

    if ((treble > 0x0F) || (bass > 0x0F))
    {
        return _AUDIOMUX_TRE_BASS_ERR;
    }

    byteIn = bass << 4;
    byteIn |= treble;

    audiomux_writeSingleByte( _AUDIOMUX_TREBLE_BASS_ADDR, byteIn );

    return _AUDIOMUX_OK;
}

T_AUDIOMUX_RETVAL audiomux_muteOuput( uint8_t state )
{
    uint8_t byteIn;

    if (state > 0x01)
    {
        return _AUDIOMUX_STATE_ERR;
    }

    byteIn = state;

    audiomux_writeSingleByte( _AUDIOMUX_OUTPUT_ADDR, byteIn );

    return _AUDIOMUX_OK;
}

/* -------------------------------------------------------------------------- */
/*
  __audiomux_driver.c

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