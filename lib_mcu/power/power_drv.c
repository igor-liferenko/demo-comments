/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the Power and clock management driver routines.

//!

//!  This file contains the Power and clock management driver routines.

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//_____ I N C L U D E S ____________________________________________________



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the system configuration definition.

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */





// Compiler switch (do not change these settings)

// Compiler definitions

/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file redefines dedicated IAR EWAVR and GNU GCC keywords

//! in order to ensure that any source file can be processed by these compilers.

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */





/*_____ I N C L U D E S ____________________________________________________*/



/*_____ D E C L A R A T I O N S ____________________________________________*/



// define ASM_INCLUDE in your a51 source code before include of .h file

typedef float Float16;

typedef unsigned char U8 ;
typedef unsigned short U16;
typedef unsigned long U32;
typedef signed char S8 ;
typedef signed short S16;
typedef long S32;



typedef unsigned char Bool;


typedef U8 Status;
typedef Bool Status_bool;



















typedef unsigned char Uchar;


typedef unsigned char Uint8;
typedef unsigned int Uint16;
typedef unsigned long int Uint32;

typedef char Int8;
typedef int Int16;
typedef long int Int32;

typedef unsigned char Byte;
typedef unsigned int Word;
typedef unsigned long int DWord;

typedef union
{
// l changed in dw (double word) because l is used for signed long...
  Uint32 dw;
  Uint16 w[2];
  Uint8 b[4];
} Union32;

typedef union
{
  Uint16 w;
  Uint8 b[2];
} Union16;




typedef char p_uart_ptchar;
typedef char r_uart_ptchar;




/**********************************************************************************/

/* codevision COMPILER (__CODEVISIONAVR__)                                                 */

/**********************************************************************************/


/**********************************************************************************/

/* IAR COMPILER (__IAR_SYSTEMS_ICC__)                                             */

/**********************************************************************************/




/* General purpose defines */

/*#define _ConstType_   __farflash
#define _MemType_
#define _GenericType_ __generic
#define code __farflash
#define xdata
#define idata
#define data*/





/*_____ M A C R O S ________________________________________________________*/

/* little-big endian management */




// U16/U32 endian handlers

// => 16bit: (LSB,MSB), 32bit: (LSW,MSW) or (LSB0,LSB1,LSB2,LSB3) or (MSB3,MSB2,MSB1,MSB0)













// BIG_ENDIAN         => 16bit: (MSB,LSB), 32bit: (MSW,LSW) or (LSB3,LSB2,LSB1,LSB0) or (MSB0,MSB1,MSB2,MSB3)


// Endian converters




// host to network conversion: used for Intel HEX format, TCP/IP, ...

// Convert a 16-bit value from host-byte order to network-byte order

// Standard Unix, POSIX 1003.1g (draft)










// Constants















// define ASM_INCLUDE in your a51 source code before include of .h file





/* Bit and bytes manipulations */









// Take the max between a and b

// Take the min between a and b


// Align on the upper value <val> on a <n> boundary

// i.e. Upper(0, 4)= 4

//      Upper(1, 4)= 4

//      Upper(2, 4)= 4

//      Upper(3, 4)= 4

//      Upper(4, 4)= 8

//      ../..



// Align up <val> on a <n> boundary

// i.e. Align_up(0, 4)= 0

//      Align_up(1, 4)= 4

//      Align_up(2, 4)= 4

//      Align_up(3, 4)= 4

//      Align_up(4, 4)= 4

//      ../..



// Align down <val> on a <n> boundary

// i.e. Align_down(0, 4)= 0

//      Align_down(1, 4)= 0

//      Align_down(2, 4)= 0

//      Align_down(3, 4)= 0

//      Align_down(4, 4)= 4

//      ../..



/*M**************************************************************************
* NAME: Long_call
*----------------------------------------------------------------------------
* PARAMS:
* addr: address of the routine to call
*----------------------------------------------------------------------------
* PURPOSE:
* Call the routine at address addr: generate an Assembly LCALL addr opcode.
*----------------------------------------------------------------------------
* EXAMPLE:
* Long_call(0); // Software reset (if no IT used before)
*----------------------------------------------------------------------------
* NOTE:
* May be used as a long jump opcode in some special cases
*****************************************************************************/



/* {For Langdoc} */


/***********************************************************
 SET_SFR_BIT macro
  parameters
    sfr_reg : defined value in include file for sfr register
    bit_pos : defined value B_XX in include file for particular
              bit of sfr register
    bit_val : CLR / SET
************************************************************/



/***********************************************************
 bit_is_clear macro
  parameters
    PORT     : defined value in include file for sfr register
    POSITION : defined value in include file for particular
              bit of sfr register
  example : if (bit_is_clear(PORTB,PORTB3)) ...
************************************************************/



/***********************************************************
 bit_is_set macro
  parameters
    PORT     : defined value in include file for sfr register
    POSITION : defined value in include file for particular
              bit of sfr register
  example : if (bit_is_set(PORTB,PORTB3)) ...
************************************************************/











/******************************************************************************/

/* GCC COMPILER                                                               */

/******************************************************************************/















//#include <avr/sfr_defs.h>


#include  <avr/interrupt.h>

#include  <avr/pgmspace.h>




/* _COMPILER_H_ */



// Use AVR-GCC library

#include  <avr/io.h>



//! @defgroup global_config Application configuration

//! @{


//!< Scheduler tasks declaration

/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the scheduler configuration definition

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */







/*--------------- SCHEDULER CONFIGURATION --------------*/

// SCHEDULER_(TIMED|TASK|FREE|CUSTOM)









//! _CONF_SCHEDULER_H_



//! Enable or not the ADC usage



// Board defines (do not change these settings)



// Select board



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the low level macros and definition for evk527 board

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */





//_____ I N C L U D E S ____________________________________________________


/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the system configuration definition.

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */







// Because pin JTAG multiplexage

/*This file has been prepared for Doxygen automatic documentation generation.*/

/*! \file *********************************************************************
 *
 * \brief Driver routines to read (no write) datas stored in AVRMega flash
 * These routines can be stored and executed in all flash space.
 *
 * - Compiler:           IAR EWAVR and GNU GCC for AVR
 * - Supported devices:  All AVRMega devices
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 *****************************************************************************/


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */






//_____ I N C L U D E S ______________________________________________________



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the system configuration definition.

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */






//_____ M A C R O S ________________________________________________________


//! \name These macros allow to read a specific device ID of the product.

//! @{

//!< This macro function allows to read device ID1 of the product.

//!< This macro function allows to read device ID2 of the product.

//!< This macro function allows to read device ID3 of the product.

//!< This macro function allows to read the OSCAL byte of the product.

//! @}


//! \name These macros allow to read a specific fuse of the product.

//! @{

//!< @brief This macro function allows to read the low fuse byte of the product.

//!< @brief This macro function allows to read device high fuse byte of the product.

//!< @brief This macro function allows to read extended fuse byte of the product.

//!< @brief This macro function allows to read lock byte of the product.

//! @}


//! \name These macros allow to check bits from extended fuse of the product.

//! @{

//!< @brief Check if the OCD is running

//!< @brief Check if the JTAG interface is enabled

//! @}



//! \name High Fuse Byte

//! @{

// Select Reset Vector

// Select Boot Size

// Select Boot Size

// EEPROM memory is preserved through chip erase

// Watchdog timer always on

// Enable Serial programming and Data Downloading

// Enable JTAG

// Enable OCD

//! @}



//_____ D E C L A R A T I O N S ____________________________________________


//! @brief This macro function allows to read device IDs of the product.

//!

//! @param  add   Address of device ID to read.

//!

//! @return byte  Read value

//!

U8 flash_read_sig(unsigned long adr);

//! @brief This macro function allows to read a fuse byte of the product.

//!

//! @param  add   Address of fuse to read.

//! 

//! @return byte  Read value

//!

U8 flash_read_fuse(unsigned long adr);


// FLASH_DRV_H





//_____ M A C R O S ________________________________________________________


//! @defgroup EVK_527_module evk527

//! This module contains low level hardware abstraction layer for evk527 board

//! @image html evk_527.gif

//! @{



//! @defgroup EVK_527_leds Leds Management

//! Macros to manage Leds on evk527

//! @{































//! @}


//! @defgroup EVK_527_joy Joystick Management

//! Macros to manage Joystick on evk527

//! @note The up,left and right joy are mapped on JTAG pins

//! @{













//! Enable interrupts for switches (no possible)



//! Disable interrupts for switches (no possible)


//! @}



//! @defgroup EVK_527_hwb HWB button management

//! HWB button is connected to PE2 and can also

//! be used as generic push button

//! @{




//! @}









//! EVK 527 ADC Channel Definition

// Enable Single Ended Input on ADC9 pin

// Enable Single Ended Input on ADC0 pin


//!< this define is set in config.h file


//! @defgroup EVK_527_df_evk527 ATMEL Hardware data flash configuration

//! Macros to init the environnement for DF on evk527

//! @{


// Type of DF implemented on evk527.














// set CS# dataflash memories (unselect).



//! @}


//! @defgroup EVK_527_mmc_sd Hardware mmc sd interface configuration

//! Macros to init the environnement for mmc /sd on evk527

//! @{   




// port

// offset



//! @}

//! @}


// TARGET_BOARD==EVK527


// EVK_527_H


//! CPU core frequency in kHz





// -------- END Generic Configuration -------------------------------------


// UART Sample configuration, if we have one ... __________________________





//#define uart_putchar putchar







//! @}


// _CONFIG_H_



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the Power Management low level driver definition

//!

//!  This module allows to configure the different power mode of the AVR core and

//!  also to setup the the internal clock prescaler

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

//! - Supported devices:  ATmega32U4

//!

//! \author               Atmel Corporation: http://www.atmel.com \n

//!                       Support and FAQ: http://support.atmel.no/

//!

//! ***************************************************************************


/* Copyright (c) 2007, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */








#include  <avr/power.h>





//! @defgroup powermode Power management drivers

//!

//! @{


//_____ M A C R O S ________________________________________________________









//! Clear_prescaler.

//!

//! This function reset the internal CPU core clock prescaler

//!

//!

//! @param none

//!

//! @return none.

//!





//! Set_prescaler.

//!

//! This function configure the internal CPU core clock prescaler value

//!

//!

//! @param x: prescaler new value

//!

//! @return none.

//!








//Backward compatibility




//_____ D E C L A R A T I O N ______________________________________________


void set_idle_mode(void);
void set_power_down_mode(void);
void set_adc_noise_reduction_mode(void);
void set_power_save_mode(void);
void set_standby_mode(void);
void set_ext_standby_mode(void);

//! Enter_idle_mode.

//!

//! This function makes the AVR core enter idle mode.

//!

//! @param none

//!

//! @return none.

//!



//! Enter_power_down_mode.

//!

//! This function makes the AVR core enter power down mode.

//!

//! @param none

//!

//! @return none.

//!



//! Enter_adc_noise_reduction_mode.

//!

//! This function makes the AVR core enter adc noise reduction mode.

//!

//! @param none

//!

//! @return none.

//!



//! Enter_power_save_mode.

//!

//! This function makes the AVR core enter power save mode.

//!

//! @param none

//!

//! @return none.

//!



//! Enter_standby_mode.

//!

//! This function makes the AVR core enter standby mode.

//!

//! @param none

//!

//! @return none.

//!



//! Enter_ext_standby_mode.

//!

//! This function makes the AVR core enter extended standby mode.

//!

//! @param none

//!

//! @return none.

//!




//! @}





//! @defgroup clockmode Clock management drivers

//!

//! @{


//_____ M A C R O S ________________________________________________________


// Clock control






// Clock state




// Clock selection




// Clock settings : when using a clock source, only the other clock source setting can be modified

// Set the source setting of the next clock source to use before switching to it





//_____ C L O C K   D E F I N I T I O N S ______________________________________

// Configuration byte defined as SUT<1:0> & CKSEL<3:0> (CKSEL0 is the LSb)


// Interal RC oscillator (frequency between 7.3 and 8.1 MHz)





// External crystal, frequency between 0.3 and 0.9 MHz










// External crystal, frequency between 0.9 and 3 MHz










// External crystal, frequency between 3 and 8 MHz










// External crystal, frequency between 8 and 16 MHz










// External clock






//_____ D E C L A R A T I O N ______________________________________________


void Clock_switch_external(void);
void Clock_switch_internal(void);

//! @}



// _POWER_DRV_H_



//_____ M A C R O S ________________________________________________________



//_____ D E C L A R A T I O N ______________________________________________


//! Set_cpu_prescaler.

//!

//! This function write the CPU prescaler register to a define value

//!

//! @param U8 the precaler value to be written

//!

//! @return none.

//!



//! set_power_down_mode.

//!

//! This function makes the AVR core enter power down mode.

//!

//! @param none

//!

//! @return none.

//!

void set_power_down_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM1)) ;
   {asm("SLEEP");} ;
}



//! set_idle_mode.

//!

//! This function makes the AVR core enter idle mode.

//!

//! @param none

//!

//! @return none.

//!

void set_idle_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)) ;
   {asm("SLEEP");} ;
}

//! set_adc_noise_reduction_mode.

//!

//! This function makes the AVR core enter adc noise reduction mode.

//!

//! @param none

//!

//! @return none.

//!

void set_adc_noise_reduction_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM0)) ;
   {asm("SLEEP");} ;
}

//! set_power_save_mode.

//!

//! This function makes the AVR core enter power save mode.

//!

//! @param none

//!

//! @return none.

//!

void set_power_save_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM1)+(1<<SM0)) ;
   {asm("SLEEP");} ;
}

//! set_standby_mode.

//!

//! This function makes the AVR core enter standby mode.

//!

//! @param none

//!

//! @return none.

//!

void set_standby_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM2)+(1<<SM1)) ;
   {asm("SLEEP");} ;
}

//! set_ext_standby_mode.

//!

//! This function makes the AVR core enter extended standby mode.

//!

//! @param none

//!

//! @return none.

//!

void set_ext_standby_mode(void)
{
   (SMCR=0,SMCR |= (1<<SE)+(1<<SM2)+(1<<SM1)+(1<<SM0)) ;
   {asm("SLEEP");} ;
}




//! Clock_switch_external.

//!

//! This function makes the AVR selects the EXTERNAL clock source (CRYSTAL)

//!

//! @param none

//!

//! @return none.

//!

void Clock_switch_external(void)
{
  (CLKSEL0 |= (1<<EXTE)) ;
  while (! (((CLKSTA&(1<<EXTON)) != 0) ? (1==1) : (0==1) ) );
  (CLKSEL0 |= (1<<CLKS)) ;
  (CLKSEL0 &= ~(1<<RCE)) ;
}


//! Clock_switch_internal.

//!

//! This function makes the AVR selects the INTERNAL clock source (RC)

//!

//! @param none

//!

//! @return none.

//!

void Clock_switch_internal(void)
{
  (CLKSEL0 |= (1<<RCE)) ;
  while (! (((CLKSTA&(1<<RCON)) != 0) ? (1==1) : (0==1) ) );
  (CLKSEL0 &= ~(1<<CLKS)) ;
  (CLKSEL0 &= ~(1<<EXTE)) ;
}

