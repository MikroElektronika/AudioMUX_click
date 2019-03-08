![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# AudioMUX Click

- **CIC Prefix**  : AUDIOMUX
- **Author**      : Nemanja Medakovic
- **Verison**     : 1.0.0
- **Date**        : Oct 2018.

---

### Software Support

We provide a library for the AudioMUX Click on our [LibStock](https://libstock.mikroe.com/projects/view/2626/audiomux-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

Library offers a choice to perform a control of the AudioMUX Click board.
The control consists of input control, output control, gain control, volume control, treble and bass control...
The device also can work in Bass ALC Mode by using this library.
For more details check documentation.

Key functions :

- ``` T_AUDIOMUX_RETVAL audiomux_writeSingleByte( uint8_t regAddr, uint8_t dataIn ) ``` - Function writes one byte data to the register.
- ``` T_AUDIOMUX_RETVAL audiomux_selectInput( uint8_t inputSel, uint8_t muteIn, uint8_t micGain, uint8_t micState ) ``` - Function performs a input selection and control.
- ``` T_AUDIOMUX_RETVAL audiomux_setVolume( uint8_t volume1, uint8_t volume2, uint8_t sideSel ) ``` - Function performs a left or right volume control.
- ``` T_AUDIOMUX_RETVAL audiomux_setTrebleBass( uint8_t treble, uint8_t bass ) ``` - Function performs a control of the treble and bass for EQ section.

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes peripherals and pins.
- Application Initialization - Initializes I2C interface.
- Application Task - (code snippet) - Initializes AudioMUX device to work with the desired configurations and
  shows a message on uart when playing is started.
  Device initialization will be performed only once and after that AudioMUX will work with the same desired configurations.


```.c
void applicationTask()
{
    if (initCheck == 0)
    {
        audiomux_selectInput( _AUDIOMUX_IN1_EN, _AUDIOMUX_MUTE_INPUT_OFF, _AUDIOMUX_MIC_GAIN_14DB, _AUDIOMUX_MIC_OFF );
        audiomux_setInputGain( _AUDIOMUX_IN_GAIN_2DB );
        audiomux_writeSingleByte( _AUDIOMUX_SURROUND_ADDR, _AUDIOMUX_MIX_0 );
        audiomux_setVolume( _AUDIOMUX_VOL1_6DB_NEG, _AUDIOMUX_VOL2_0DB, _AUDIOMUX_VOLUME_LEFT_ADDR );
        audiomux_setVolume( _AUDIOMUX_VOL1_6DB_NEG, _AUDIOMUX_VOL2_0DB, _AUDIOMUX_VOLUME_RIGHT_ADDR );
        audiomux_setTrebleBass( _AUDIOMUX_TRE_BASS_4DB_NEG, _AUDIOMUX_TRE_BASS_14DB );
        audiomux_writeSingleByte( _AUDIOMUX_BASS_ALC_ADDR, _AUDIOMUX_MODE2_ADAPTIVE );
        audiomux_muteOuput( _AUDIOMUX_MUTE_OUTPUT_OFF );
        initCheck = 1;

        mikrobus_logWrite( "Playing", _LOG_TEXT );
        Delay_ms( 1000 );
    }

    if (messCnt < 5)
    {
        mikrobus_logWrite( ". ", _LOG_TEXT );
        Delay_ms( 4000 );
        messCnt++;
    }
}
```

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2626/audiomux-click) page.

Other mikroE Libraries used in the example:

- I2C
- UART

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
