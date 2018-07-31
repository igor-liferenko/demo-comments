/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file manages the USB task either device/host or both.

//!

//!  The USB task selects the correct USB task (usb_device task or usb_host task

//!  to be executed depending on the current mode available.

//!  According to USB_DEVICE_FEATURE and USB_HOST_FEATURE value (located in conf_usb.h file)

//!  The usb_task can be configured to support USB DEVICE mode or USB Host mode or both

//!  for a dual role device application.

//!  This module also contains the general USB interrupt subroutine. This subroutine is used

//!  to detect asynchronous USB events.

//!  Note:

//!    - The usb_task belongs to the scheduler, the usb_device_task and usb_host do not, they are called

//!      from the general usb_task

//!    - See conf_usb.h file for more details about the configuration of this module

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


//_____  I N C L U D E S ___________________________________________________



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

//! \brief This file contains the possible external configuration of the USB

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





/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the usb definition constant parameters from USB V2.0

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

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





//! \name Vendor Identifiant according by USB org to ATMEL




//! \name Product Identifiant according by ATMEL

//! @{




















//! @}



//! \name Global Class, SubClass & Protocol constants

//! @{

//!< Use to declare a specific interface link at VID-PID

//!< Use to declare a specific interface link at VID-PID




//! @}



//! \name Status constant of device

//! @{



//! @}



//! \name Attribut constant of status device

//! @{






//! @}



//! \name Constants used in Endpoint Descriptor

//! @{







//! @}



//! \name Constants used in setup requests

//! @{


//! \name Requests type (bmRequestTypes)

//! @{


//! \name Data transfer direction

//! bit 7,

//! 0 = Host to device

//! 1 = Device to host

//! @{



//! @}


//! \name Type

//! bit 6 to 5,

//! 0 = Standard

//! 1 = Class

//! 2 = Vendor

//! 3 = Reserved

//! @{




//! @}


//! \name Recipient

//! bit 4 to 0,

//! 0 = device

//! 1 = Interface

//! 2 = Endpoint

//! 3 = Other

//! 4...31 = Reserved

//! @{





//! @}


//! \name Request type used by standard setup request

//! @{

// 0x00

// 0x80

// 0x01

// 0x81

// 0x02

// 0x82

//! @}


//! \name Request type used by specific setup request from class driver

//! @{

// 0x20

// 0xA0

// 0x21

// 0xA1

// 0x22

// 0xA2

// 0x23

// 0xA3

// 0x40

// 0xC0

//! @}

//! @}


//! \name Standard Requests (bRequest)

//! @{














//! @}


//! \name Descriptor types used in several setup requests

//! @{









//! @}


//! \name Feature types for SETUP_X_FEATURE standard request

//! @{







//! @}


//! \name Feature types for SETUP_X_FEATURE standard test request

//! @{






//! @}

//! @}


//! \name OTG descriptor (see OTG_BMATTRIBUTES)

//! @{




//! @}



// _USB_COMMUN_H_



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the usb CDC definition constant parameters

//!

//! - Compiler:           IAR EWAVR and GNU GCC for AVR

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





//! \name Global Class, SubClass & Protocol constants for CDC

//! @{










//! @}


//! \name Specific setup requests from CDC

//! @{





































// Added bNotification codes according cdc spec 1.1 chapter 6.3





//! @}


// _USB_COMMUN_CDC_H_




//! @defgroup usb_general_conf USB application configuration

//!

//! @{



// _________________ USB MODE CONFIGURATION ____________________________

//

//! @defgroup USB_op_mode USB operating modes configuration

//! defines to enable device or host usb operating modes

//! supported by the application

//! @{


//! @brief ENABLE to activate the device software library support

//!

//! Possible values ENABLE or DISABLE



//! @}


// _________________ USB REGULATOR CONFIGURATION _______________________

//

//! @defgroup USB_reg_mode USB regulator configuration

//! @{


//! @brief Enable the internal regulator for USB pads

//!

//! When the application voltage is lower than 3.5V, to optimize power consumption

//! the internal USB pads regulatr can be disabled.


// Possible values ENABLE or DISABLE


//! @}



// _________________ DEVICE MODE CONFIGURATION __________________________


//! @defgroup USB_device_mode_cfg USB device operating mode configuration

//!

//! @{


//!  number of endpoints in the application including control endpoint





//! don't allow remote wake up


//! device will connect directly on reset


//! an USB reset does not reset the CPU




//! @defgroup device_cst_actions USB device custom actions

//!

//! @{

// write here the action to associate to each USB event

// be carefull not to waste time in order not disturbing the functions









//! @}


extern void sof_action(void);
extern void suspend_action(void);
//! @}



//! @}


// _CONF_USB_H_


/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the function declarations

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





//! @defgroup usb_task USB task entry point

//! @{


//_____ I N C L U D E S ____________________________________________________



//_____ M A C R O S ________________________________________________________


//! @defgroup usb_software_evts USB software Events Management

//! Macros to manage USB events detected under interrupt

//! @{










// USB plugged

// USB un-plugged

// USB in device

// USB in host

// USB suspend

// USB wake up

// USB resume

// USB reset

// Host start of frame sent

// Host wakeup detected

// The target device is disconnected

//! @}







//_____ D E C L A R A T I O N S ____________________________________________


extern volatile U16 g_usb_event;
extern U8 g_usb_mode;
extern U8 usb_remote_wup_feature;

/**
 * @brief This function initializes the USB proces.
 *
 *  This function enables the USB controller and init the USB interrupts.
 *  The aim is to allow the USB connection detection in order to send
 *  the appropriate USB event to the operating mode manager.
 *  Depending on the mode supported (HOST/DEVICE/DUAL_ROLE) the function
 *  calls the corespong usb mode initialization function
 *
 *  @param none
 *
 *  @return none
 */

void usb_task_init (void);

/**
 *  @brief Entry point of the USB mamnagement
 *
 *  Depending on the mode supported (HOST/DEVICE/DUAL_ROLE) the function
 *  calls the corespong usb management function
 *
 *  @param none
 *
 *  @return none
*/

void usb_task (void);

extern volatile U8 private_sof_counter;



//! @}


/* _USB_TASK_H_ */



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the USB low level driver definition

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



typedef enum endpoint_parameter{ep_num, ep_type, ep_direction, ep_size, ep_bank, nyet_status} t_endpoint_parameter;

//! @defgroup USB_low_level_drivers USB low level drivers

//! USB low level drivers Module

//! @{


//_____ M A C R O S ________________________________________________________























// USB EndPoint









// UEDATX

// UEBCHX

// UEBCLX

// UEINT

// UHADDR


// USB Pipe

// UPNUM

// UPRST

// UPCFG0X



// UPCFG1X







// UPCFG0X







// UPCFG1X




// Parameters for endpoint configuration

// These define are the values used to enable and configure an endpoint.





//typedef enum ep_type {TYPE_CONTROL, TYPE_BULK, TYPE_ISOCHRONOUS, TYPE_INTERRUPT} e_ep_type;




//typedef enum ep_dir {DIRECTION_OUT, DIRECTION_IN} e_ep_dir;










//typedef enum ep_size {SIZE_8,   SIZE_16,  SIZE_32,  SIZE_64,

//                      SIZE_128, SIZE_256, SIZE_512, SIZE_1024} e_ep_size;




//typedef enum ep_bank {ONE_BANK, TWO_BANKS} e_ep_bank;




//typedef enum ep_nyet {NYET_DISABLED, NYET_ENABLED} e_ep_nyet;









//! @defgroup Endpoints_configuration Configuration macros for endpoints

//! List of the standard macro used to configure pipes and endpoints

//! @{








//! @}


//! @defgroup USB_regulator USB Pads Regulator drivers

//! Turns ON/OFF USB pads regulator

//! @{

//! Enable internal USB pads regulator


//! Disable internal USB pads regulator


//! Check regulator enable bit


//! @}


//! @defgroup gen_usb USB common management drivers

//! These macros manage the USB controller

//! @{


//! Enable USB interface


//! Disable  USB interface


//! Use device full speed mode (default mode)


//! For device mode, force low speed mode










//! Enable VBUS pad


//! Disable VBUS pad



//! Stop internal USB clock in interface (freeze the interface register)






//! returns the USB general interrupts (interrupt enabled)


//! acks the general interrupts (interrupt enabled)














//! @}



//! @defgroup USB_device_driver USB device controller drivers

//! These macros manage the USB Device controller.

//! @{

//! initiates a remote wake-up


//! detaches from USB bus


//! attaches to USB bus


//! test if remote wake-up still running


//! test if the device is detached



//! returns the USB device interrupts (interrupt enabled)


//! acks the USB device interrupts (interrupt enabled)



//! enables remote wake-up interrupt


//! disables remote wake-up interrupt



//! acks remote wake-up


//! tests if remote wake-up still running



//! enables resume interrupt


//! disables resume interrupt



//! acks resume


//! tests if resume occurs



//! enables wake-up interrupt


//! disables wake-up interrupt



//! acks wake-up


//! tests if wake-up occurs



//! enables USB reset interrupt


//! disables USB reset interrupt



//! acks USB reset


//! tests if USB reset occurs



//! enables Start Of Frame Interrupt


//! disables Start Of Frame Interrupt



//! acks Start Of Frame


//! tests if Start Of Frame occurs



//! enables suspend state interrupt


//! disables suspend state interrupt


//! test if suspend interrupt is enabled


//! acks Suspend


//! tests if Suspend state detected



//! enables USB device address


//! disables USB device address


//! test if device is adressed


//! sets the USB device address



//! returns the last frame number


//! tests if a crc error occurs in frame number


//! @}





//! @defgroup usb_gen_ep USB endpoint drivers

//! These macros manage the common features of the endpoints.

//! @{

//! selects the endpoint number to interface with the CPU



//! get the currently selected endpoint number



//! resets the selected endpoint



//! enables the current endpoint


//! enables the STALL handshake for the next transaction


//! resets the data toggle sequence


//! disables the current endpoint


//! disables the STALL handshake


//! selects endpoint interface on CPU


//! tests if the current endpoint is enabled


//! tests if STALL handshake request is running



//! configures the current endpoint


//! configures the current endpoint direction



//! configures the current endpoint size


//! configures the current endpoint number of banks


//! allocates the current configuration in DPRAM memory


//! un-allocates the current configuration in DPRAM memory



//! acks endpoint overflow interrupt


//! acks endpoint underflow memory


//! acks Zero Length Packet received


//! returns data toggle


//! returns the number of busy banks


//! tests if at least one bank is busy


//! tests if current endpoint is configured


//! tests if an overflows occurs


//! tests if an underflow occurs


//! tests if a ZLP has been detected



//! returns the control direction


//! returns the number of the current bank



//! clears FIFOCON bit


//! acks NAK IN received


//! acks NAK OUT received


//! acks receive SETUP


//! acks reveive OUT


//! acks STALL sent


//! acks IN ready


//! Kills last bank


//! tests if endpoint read allowed


//! tests if endpoint write allowed


//! tests if read allowed on control endpoint


//! tests if a NAK has been sent


//! tests if SETUP received


//! tests if OUT received


//! tests if IN ready


//! sends IN


//! sends IN on control endpoint


//! frees OUT bank


//! acks OUT on control endpoint



//! enables flow error interrupt


//! enables NAK IN interrupt


//! enables NAK OUT interrupt


//! enables receive SETUP interrupt


//! enables receive OUT interrupt


//! enables STALL sent interrupt


//! enables IN ready interrupt


//! disables flow error interrupt


//! disables NAK IN interrupt


//! disables NAK OUT interrupt


//! disables receive SETUP interrupt


//! disables receive OUT interrupt


//! disables STALL sent interrupt


//! disables IN ready interrupt



//! returns FIFO byte for current endpoint


//! writes byte in FIFO for current endpoint



//! returns number of bytes in FIFO current endpoint (16 bits)


//! returns number of bytes in FIFO current endpoint (8 bits)



//! tests the general endpoint interrupt flags


//! tests the general endpoint interrupt flags


//! @}




//! wSWAP

//! This macro swaps the U8 order in words.

//!

//! @param x        (U16) the 16 bit word to swap

//!

//! @return         (U16) the 16 bit word x with the 2 bytes swaped





//! Usb_write_word_enum_struc

//! This macro help to fill the U16 fill in USB enumeration struct.

//! Depending on the CPU architecture, the macro swap or not the nibbles

//!

//! @param x        (U16) the 16 bit word to be written

//!

//! @return         (U16) the 16 bit word written




//BIG_ENDIAN



//! @}


//_____ D E C L A R A T I O N ______________________________________________


U8 usb_config_ep (U8, U8);
U8 usb_select_enpoint_interrupt (void);
U16 usb_get_nb_byte_epw (void);
U8 usb_send_packet (U8 , U8*, U8);
U8 usb_read_packet (U8 , U8*, U8);
void usb_halt_endpoint (U8);
void usb_reset_endpoint (U8);
U8 usb_init_device (void);


// _USB_DRV_H_



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief CDC USB Identifers.

//!

//!  This file contains the usb parameters that uniquely identify the

//!  CDC application through descriptor tables.

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





/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief @briefProcess USB device enumeration requests header file.

//!

//!  This file contains the USB endpoint 0 management routines corresponding to

//!  the standard enumeration process (refer to chapter 9 of the USB

//!  specification.

//!  This file calls routines of the usb_specific_request.c file for non-standard

//!  request management.

//!  The enumeration parameters (descriptor tables) are contained in the

//!  usb_descriptors.c file.

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

//! \brief This file contains the function declarations

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





/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief CDC USB Identifers.

//!

//!  This file contains the usb parameters that uniquely identify the

//!  CDC application through descriptor tables.

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


//_____ S T A N D A R D    D E F I N I T I O N S ___________________________


//! @defgroup std_request USB device standard  requests decoding

//! @{




// Device State










//_____ D E C L A R A T I O N ______________________________________________


//! @brief Returns true when device connected and correctly enumerated with an host.

//! The device high level application should tests this before performing any applicative requests







//! @brief This function reads the SETUP request sent to the default control endpoint

//! and calls the appropriate function. When exiting of the usb_read_request

//! function, the device is ready to manage the next request.

//!

//! If the received request is not supported or a none USB standard request, the function

//! will call for custom decoding function in usb_specific_request module.

//!

//! @param none

//!

//! @return none

//! @note list of supported requests:

//! SETUP_GET_DESCRIPTOR

//! SETUP_GET_CONFIGURATION

//! SETUP_SET_ADDRESS

//! SETUP_SET_CONFIGURATION

//! SETUP_CLEAR_FEATURE

//! SETUP_SET_FEATURE

//! SETUP_GET_STATUS

//!

void usb_process_request( void);

//! @brief This function manages the remote wakeup generation to wake up the host controlle.

//!

//! If the received request is not supported or a none USB standard request, the function

//! will call for custom decoding function in usb_specific_request module.

//!

//! @param none

//!

//! @return none

//!

void usb_generate_remote_wakeup(void);

extern U8 usb_configuration_nb;
extern U8 remote_wakeup_feature;

//! @}


// _USB_STANDARD_REQUEST_H_



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the possible external configuration of the USB

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







//_____ U S B    D E F I N E _______________________________________________


// USB Device descriptor


// CDC class

// each configuration has its own sub-class

// each configuration has its own protocol










// CDC CONFIGURATION

// Number of interfaces




// 100 mA


// Interface 0 descriptor




// CDC ACM Com





// Interface 1 descriptor




// CDC ACM Data





// USB Endpoint 3 descriptor

// Interrupt IN



// BULK = 0x02, INTERUPT = 0x03


//ms interrupt pooling from host


// USB Endpoint 1 descriptor

// Bulk IN



// BULK = 0x02, INTERUPT = 0x03




// USB Endpoint 2 descriptor

//Bulk OUT  RX endpoint



// BULK = 0x02, INTERUPT = 0x03







// TBD



















//! Usb Request

typedef struct
{
//!< Characteristics of the request
   U8 bmRequestType;
//!< Specific request
   U8 bRequest;
//!< field that varies according to request
   U16 wValue;
//!< field that varies according to request
   U16 wIndex;
//!< Number of bytes to transfer if Data
   U16 wLength;
} S_UsbRequest;

//! Usb Device Descriptor

typedef struct {
//!< Size of this descriptor in bytes
   U8 bLength;
//!< DEVICE descriptor type
   U8 bDescriptorType;
//!< Binay Coded Decimal Spec. release
   U16 bscUSB;
//!< Class code assigned by the USB
   U8 bDeviceClass;
//!< Sub-class code assigned by the USB
   U8 bDeviceSubClass;
//!< Protocol code assigned by the USB
   U8 bDeviceProtocol;
//!< Max packet size for EP0
   U8 bMaxPacketSize0;
//!< Vendor ID. ATMEL = 0x03EB
   U16 idVendor;
//!< Product ID assigned by the manufacturer
   U16 idProduct;
//!< Device release number
   U16 bcdDevice;
//!< Index of manu. string descriptor
   U8 iManufacturer;
//!< Index of prod. string descriptor
   U8 iProduct;
//!< Index of S.N.  string descriptor
   U8 iSerialNumber;
//!< Number of possible configurations
   U8 bNumConfigurations;
} S_usb_device_descriptor;


//! Usb Configuration Descriptor

typedef struct {
//!< size of this descriptor in bytes
   U8 bLength;
//!< CONFIGURATION descriptor type
   U8 bDescriptorType;
//!< total length of data returned
   U16 wTotalLength;
//!< number of interfaces for this conf.
   U8 bNumInterfaces;
//!< value for SetConfiguration resquest
   U8 bConfigurationValue;
//!< index of string descriptor
   U8 iConfiguration;
//!< Configuration characteristics
   U8 bmAttibutes;
//!< maximum power consumption
   U8 MaxPower;
} S_usb_configuration_descriptor;


//! Usb Interface Descriptor

typedef struct {
//!< size of this descriptor in bytes
   U8 bLength;
//!< INTERFACE descriptor type
   U8 bDescriptorType;
//!< Number of interface
   U8 bInterfaceNumber;
//!< value to select alternate setting
   U8 bAlternateSetting;
//!< Number of EP except EP 0
   U8 bNumEndpoints;
//!< Class code assigned by the USB
   U8 bInterfaceClass;
//!< Sub-class code assigned by the USB
   U8 bInterfaceSubClass;
//!< Protocol code assigned by the USB
   U8 bInterfaceProtocol;
//!< Index of string descriptor
   U8 iInterface;
} S_usb_interface_descriptor;


//! Usb Endpoint Descriptor

typedef struct {
//!< Size of this descriptor in bytes
   U8 bLength;
//!< ENDPOINT descriptor type
   U8 bDescriptorType;
//!< Address of the endpoint
   U8 bEndpointAddress;
//!< Endpoint's attributes
   U8 bmAttributes;
//!< Maximum packet size for this EP
   U16 wMaxPacketSize;
//!< Interval for polling EP in ms
   U8 bInterval;
} S_usb_endpoint_descriptor;


//! Usb Device Qualifier Descriptor

typedef struct {
//!< Size of this descriptor in bytes
   U8 bLength;
//!< Device Qualifier descriptor type
   U8 bDescriptorType;
//!< Binay Coded Decimal Spec. release
   U16 bscUSB;
//!< Class code assigned by the USB
   U8 bDeviceClass;
//!< Sub-class code assigned by the USB
   U8 bDeviceSubClass;
//!< Protocol code assigned by the USB
   U8 bDeviceProtocol;
//!< Max packet size for EP0
   U8 bMaxPacketSize0;
//!< Number of possible configurations
   U8 bNumConfigurations;
//!< Reserved for future use, must be zero
   U8 bReserved;
} S_usb_device_qualifier_descriptor;


//! Usb Language Descriptor

typedef struct {
//!< size of this descriptor in bytes
   U8 bLength;
//!< STRING descriptor type
   U8 bDescriptorType;
//!< language id
   U16 wlangid;
} S_usb_language_id;


//_____ U S B   M A N U F A C T U R E R   D E S C R I P T O R _______________



//struct usb_st_manufacturer

typedef struct {
// size of this descriptor in bytes
   U8 bLength;
// STRING descriptor type
   U8 bDescriptorType;
// unicode characters
   U16 wstring[ 5 ];
} S_usb_manufacturer_string_descriptor;


//_____ U S B   P R O D U C T   D E S C R I P T O R _________________________



//struct usb_st_product

typedef struct {
// size of this descriptor in bytes
   U8 bLength;
// STRING descriptor type
   U8 bDescriptorType;
// unicode characters
   U16 wstring[ 16 ];
} S_usb_product_string_descriptor;


//_____ U S B   S E R I A L   N U M B E R   D E S C R I P T O R _____________



//struct usb_st_serial_number

typedef struct {
// size of this descriptor in bytes
   U8 bLength;
// STRING descriptor type
   U8 bDescriptorType;
// unicode characters
   U16 wstring[ 0x05 ];
} S_usb_serial_number;


/*_____ U S B   C D C  D E S C R I P T O R __________________________________*/


typedef struct
{
   S_usb_configuration_descriptor cfg;
   S_usb_interface_descriptor ifc0;
   U8 CS_INTERFACE[19];
   S_usb_endpoint_descriptor ep3;
   S_usb_interface_descriptor ifc1;
   S_usb_endpoint_descriptor ep1;
   S_usb_endpoint_descriptor ep2;
} S_usb_user_configuration_descriptor;










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



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the Watchdog low level driver definition

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




#include  <avr/io.h>

#include  <avr/wdt.h>



//_____ M A C R O S ________________________________________________________


//! @defgroup wdt_drv Watchdog and reset sytem drivers

//! @{











//For compatibility with Tinyx61 code




//#define Wdt_reset_instruction()   (asm("WDR"))





































//! Wdt_off.

//!

//! This macro stops the hardware watchdog timer.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!






//! wdt_change_16ms.

//!

//! This macro activates the hardware watchdog timer for 16ms timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_32ms.

//!

//! This macro activates the hardware watchdog timer for 32ms timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!




//! wdt_change_64ms.

//!

//! This macro activates the hardware watchdog timer for 64ms timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!






//! wdt_change_32ms.

//!

//! This macro activates the hardware watchdog timer for 125ms timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_250ms.

//!

//! This macro activates the hardware watchdog timer for 250ms timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_500ms.

//!

//! This macro activates the hardware watchdog timer for 500ms timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_1s.

//!

//! This macro activates the hardware watchdog timer for 1s timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!




//! wdt_change_2s.

//!

//! This macro activates the hardware watchdog timer for 2s timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!


//! wdt_change_4s.

//!

//! This macro activates the hardware watchdog timer for 4s timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!




//! wdt_change_8s.

//!

//! This macro activates the hardware watchdog timer for 8s timeout.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!




//! wdt_change_interrupt_16ms.

//!

//! This macro activates the hardware watchdog timer for 16ms interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_32ms.

//!

//! This macro activates the hardware watchdog timer for 32ms interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_64ms.

//!

//! This macro activates the hardware watchdog timer for 64ms interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_125ms.

//!

//! This macro activates the hardware watchdog timer for 125ms interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_250ms.

//!

//! This macro activates the hardware watchdog timer for 250ms interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_500ms.

//!

//! This macro activates the hardware watchdog timer for 500ms interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_1s.

//!

//! This macro activates the hardware watchdog timer for 1s interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_2s.

//!

//! This macro activates the hardware watchdog timer for 2s interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_4s.

//!

//! This macro activates the hardware watchdog timer for 4s interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!



//! wdt_change_interrupt_8s.

//!

//! This macro activates the hardware watchdog timer for 8s interrupt.

//!

//! @warning Interrupts should be disable before call to ensure

//! no timed sequence break.

//!

//! @param none

//!

//! @return none.

//!







//! @}





// _WDT_DRV_H_



/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief @brief This file contains the low level macros and definition for the USB PLL

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


//! @defgroup PLL PLL driver

//! PLL Module

//! @{

//_____ M A C R O S ________________________________________________________


//! @defgroup PLL_macros PLL Macros

//! These functions allow to control the PLL

//! @{




































//! @brief Start the PLL at only 48 MHz, regarding CPU frequency

//! Start the USB PLL with clockfactor

//! clockfactor can be PLLx06 or PLLx03



//! return 1 when PLL locked



//! Test PLL lock bit and wait until lock is set



//! Stop the PLL



//! Select the internal RC as clock source for PLL



//! Select XTAL as clock source for PLL



// Start the PLL in autofactor mode

// regarding FOSC define





//! @}



/*
      Example 
      
   Pll_start_auto();
   Pll_set_hs_tmr_pscal_1dot5();
   Timerhs_clear();
   Timerhs_set_waveform_mode(TIMERHS_WGM_FAST_PWM);
   Timerhs_set_nb_bit(8);
   
   Timerhs_enable_pwm_a_and_na();
   Timerhs_enable_pwm_b_and_nb();
   Timerhs_enable_pwm_d_and_nd();
   
   Timerhs_set_compare_a(dt);
   Timerhs_set_compare_b(dt);
   Timerhs_set_compare_d(dt);
   Timerhs_set_clock(TIMERHS_CLK_BY_1);

   while (1)
   {  
      for(tempo=1;tempo;tempo++);
      Timerhs_set_compare_a(dt++);
      Timerhs_set_compare_b(dt++);
      Timerhs_set_compare_d(dt++);
   }
*/



//! @}

// PLL_DRV_H




/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file contains the function declarations for USB device task

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





//! @defgroup usb_device_task USB device task

//! @{


//_____ I N C L U D E S ____________________________________________________



//_____ M A C R O S ________________________________________________________



//_____ D E C L A R A T I O N S ____________________________________________



//!

//! Public : (bit) usb_suspended

//! usb_suspended is set to TRUE when USB is in suspend mode

//! usb_suspended is set to FALSE otherwise

//!/

extern  U8  usb_suspended;

//! Public : (bit) usb_connected

//! usb_connected is set to TRUE when VBUS has been detected

//! usb_connected is set to FALSE otherwise

//!/

extern  U8  usb_connected;


//!

//! @brief This function initializes the USB device controller.

//!

//! This function enables the USB controller and init the USB interrupts.

//! The aim is to allow the USB connection detection in order to send

//! the appropriate USB event to the operating mode manager.

//!

//!

//! @param none

//!

//! @return none

//!

void usb_device_task_init (void);

//!

//! @brief This function initializes the USB device controller

//!

//! This function enables the USB controller and init the USB interrupts.

//! The aim is to allow the USB connection detection in order to send

//! the appropriate USB event to the operating mode manager.

//! Start device function is executed once VBUS connection has been detected

//! either by the VBUS change interrupt either by the VBUS high level

//!

//! @param none

//!

//! @return none

//!

void usb_start_device (void);

//! @brief Entry point of the USB device mamagement

//!

//! This function is the entry point of the USB management. Each USB

//! event is checked here in order to launch the appropriate action.

//! If a Setup request occurs on the Default Control Endpoint,

//! the usb_process_request() function is call in the usb_standard_request.c file

//!

//! @param none

//!

//! @return none

void usb_device_task (void);



//! @}


/* _USB_DEVICE_TASK_H_ */





//_____ M A C R O S ________________________________________________________




//_____ D E F I N I T I O N S ______________________________________________


//!

//! Public : U16 g_usb_event

//! usb_connected is used to store USB events detected upon

//! USB general interrupt subroutine

//! Its value is managed by the following macros (See usb_task.h file)

//! Usb_send_event(x)

//! Usb_ack_event(x)

//! Usb_clear_all_event()

//! Is_usb_event(x)

//! Is_not_usb_event(x)

volatile U16 g_usb_event=0;



//!

//! Public : (bit) usb_connected

//! usb_connected is set to TRUE when VBUS has been detected

//! usb_connected is set to FALSE otherwise

//! Used with USB_DEVICE_FEATURE == ENABLED only

//!/

extern  U8  usb_connected;

//!

//! Public : (U8) usb_configuration_nb

//! Store the number of the USB configuration used by the USB device

//! when its value is different from zero, it means the device mode is enumerated

//! Used with USB_DEVICE_FEATURE == ENABLED only

//!/

extern U8 usb_configuration_nb;

//!

//! Public : (U8) remote_wakeup_feature

//! Store a host request for remote wake up (set feature received)

//!/

extern U8 remote_wakeup_feature;







//_____ D E C L A R A T I O N S ____________________________________________


/**
 * @brief This function initializes the USB process.
 *
 *  The function calls the coresponding usb mode initialization function
 *
 *  @param none
 *
 *  @return none
 */

void usb_task_init(void)
{
// Otherwise assume USB PADs regulator is not used

   usb_device_task_init();

   usb_remote_wup_feature =  0 ;

}

/**
 *  @brief Entry point of the USB mamnagement
 *
 *  The function calls the coresponding usb management function.
 *
 *  @param none
 *
 *  @return none
*/

void usb_task(void)
{
   usb_device_task();
}

//! @brief USB interrupt subroutine

//!

//! This function is called each time a USB interrupt occurs.

//! The following USB DEVICE events are taken in charge:

//! - Start Of Frame

//! - Suspend

//! - Wake-Up

//! - Resume

//! - Reset

//!

//! For each event, the user can launch an action by completing

//! the associate define (See conf_usb.h file to add action upon events)

//!

//! Note: Only interrupts events that are enabled are processed

//!

//! @param none

//!

//! @return none


 ISR(USB_GEN_vect)

{
//- VBUS state detection

   if ( ((USBINT & (1<<VBUSTI)) ? (1==1) : (0==1) )  &&  ((USBCON & (1<<VBUSTE)) ? (1==1) : (0==1) ) )
   {
      (USBINT = ~(1<<VBUSTI)) ;
      if ( ((USBSTA & (1<<VBUS)) ? (1==1) : (0==1) ) )
      {
         usb_connected =  (1==1) ;
         ;
         (g_usb_event |= (1<< 1 )) ;
         (UDIEN |= (1<<EORSTE)) ;
         usb_start_device();
         (UDCON &= ~(1<<DETACH)) ;
      }
      else
      {
         ;
         usb_connected =  (0==1) ;
         usb_configuration_nb = 0;
         (g_usb_event |= (1<< 2 )) ;
      }
   }
// - Device start of frame received

   if ( ((UDINT & (1<<SOFI)) ? (1==1) : (0==1) )  &&  ((UDIEN & (1<<SOFE)) ? (1==1) : (0==1) ) )
   {
      (UDINT = ~(1<<SOFI)) ;
      sof_action(); ;
   }
// - Device Suspend event (no more USB activity detected)

   if ( ((UDINT & (1<<SUSPI)) ? (1==1) : (0==1) )  &&  ((UDIEN & (1<<SUSPE)) ? (1==1) : (0==1) ) )
   {
      usb_suspended= (1==1) ;
// clear wake up to detect next event
      (UDINT = ~(1<<WAKEUPI)) ;
      (g_usb_event |= (1<< 5 )) ;
      (UDINT = ~(1<<SUSPI)) ;
      (UDIEN |= (1<<WAKEUPE)) ;
      (UDIEN &= ~(1<<EORSME)) ;
      (USBCON |= (1<<FRZCLK)) ;
      (PLLCSR &= (~(1<<PLLE)),PLLCSR=0 ) ;
      ;
   }
// - Wake up event (USB activity detected): Used to resume

   if ( ((UDINT & (1<<WAKEUPI)) ? (1==1) : (0==1) )  &&  ((UDIEN & (1<<WAKEUPE)) ? (1==1) : (0==1) ) )
   {
      if( (PLLCSR & (1<<PLOCK) ) == (0==1) )
      {



            (PLLFRQ &= ~ ( (1<<PDIV3)| (1<<PDIV2) | (1<<PDIV1)| (1<<PDIV0)) ,PLLFRQ|= ( (0<<PDIV3)| (1<<PDIV2) | (0<<PDIV1)| (0<<PDIV0)) , PLLCSR = ( ( 1<<PINDIV ) | (1<<PLLE))) ;

         while (!(PLLCSR & (1<<PLOCK))) ;
      }
      (USBCON &= ~(1<<FRZCLK)) ;
      (UDINT = ~(1<<WAKEUPI)) ;
      if(usb_suspended)
      {
         (UDIEN |= (1<<EORSME)) ;
         (UDIEN |= (1<<EORSTE)) ;
         (UDINT = ~(1<<WAKEUPI)) ;
         (UDIEN &= ~(1<<WAKEUPE)) ;
         ;
         (g_usb_event |= (1<< 6 )) ;
         (UDIEN |= (1<<SUSPE)) ;
         (UDIEN |= (1<<EORSME)) ;
         (UDIEN |= (1<<EORSTE)) ;

      }
   }
// - Resume state bus detection

   if ( ((UDINT & (1<<EORSMI)) ? (1==1) : (0==1) )  &&  ((UDIEN & (1<<EORSME)) ? (1==1) : (0==1) ) )
   {
      usb_suspended =  (0==1) ;
      (UDIEN &= ~(1<<WAKEUPE)) ;
      (UDINT = ~(1<<EORSMI)) ;
      (UDIEN &= ~(1<<EORSME)) ;
      ;
      (g_usb_event |= (1<< 7 )) ;
   }
// - USB bus reset detection

   if ( ((UDINT & (1<<EORSTI)) ? (1==1) : (0==1) ) &&  ((UDIEN & (1<<EORSTE)) ? (1==1) : (0==1) ) )
   {

      usb_remote_wup_feature =  0 ;

      (UDINT = ~(1<<EORSTI)) ;
      usb_init_device();
      ;
      (g_usb_event |= (1<< 8 )) ;
   }

}



