/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief This file provides a minimal VT100 terminal access through UART

//! and compatibility with Custom I/O support

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

//! \brief This file contains Uart lib header file.

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





/*_____ I N C L U D E - F I L E S ____________________________________________*/


/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief Provide Uart driver

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





/* UCSRnC register */






/* UCSRnA register */






/* UCSRnB register */



















/*#define Uart_set_baudrate(bdr)  ( UBRRH = (Uchar)((((Uint32)FOSC*1000L)/((Uint32)bdr*16)-1)>>8),                                  UBRRL = (Uchar)(((Uint32)FOSC*1000 )/((Uint32)bdr*16)-1)    )*/
















/*_____ D E F I N I T I O N S ______________________________________________*/











/*This file has been prepared for Doxygen automatic documentation generation.*/

//! \file *********************************************************************

//!

//! \brief Provide Baudrate configuration for MCU

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











/* _UART_BDR_H */




/*_____ D E C L A R A T I O N ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/


/*_____ P R O T O T Y P E S ____________________________________________________________*/


/**
 * @brief This function configures the UART configuration and timming following 
 * the constant definition of BAUDRATE and enables the UART controller.
 *
 * @pre before calling this function some declaration must be define in config.h:\n
 * - FOSC Frequency of crystal in kHz (mandatory)Standard crystals available
 * - BAUDRATE Baudrate in bit per second used for UART configuration (mandatory)
 * - UART_CONFIG : (by default 8 bits| 1 bit stop |without parity)
 *
 */

U8  uart_init (void);

/**
 * @brief This function allows to send a character on the UART
 *
 * @param uc_wr_byte character to print on UART.
 *
 * @return character sent.
 *
 * @par Note:
*  the type p_uart_ptchar and r_uart_ptchar can be define to macth with a printf
 * need.
 *
 */

int  uart_putchar ( int  uc_wr_byte);

/**
 * @brief This function allows to get a character from the UART
 *
 * @return character read.
 *
 */

char uart_getchar (void);

/**
 * @brief This function allows to inform if a character was received
 *
 * @return True if character received.
 *
 */

U8  uart_test_hit (void);



/* _UART_LIB_H_ */



/*_____ G L O B A L    D E F I N I T I O N _________________________________*/



/*_____ D E F I N I T I O N ________________________________________________*/


/*_____ M A C R O S ________________________________________________________*/



U8  uart_test_hit (void)
{
return  ( (UCSR1A) & 0x80 ) ;
}


U8  uart_init (void)
{




  ( (UBRR1) = (U16)(((U32) 16000 *1000L)/((U32) 57600 /2*16)-1)) ;
  ( (UCSR1A) |= (1<<U2X1)) ;
  ( (UCSR1C) = 0x06 ) ;


  ( (UCSR1B) |= 0x10 | 0x08 ) ;
  return  (1==1) ;
}


int  uart_putchar ( int  ch)
{
  while(! ( (UCSR1A) & 0x20 ) );
// Set Busy flag before sending (always)
  ;
  ( (UDR1) =ch) ;

  return ch;
}




char uart_getchar (void)
{
  register char c;

  while(! ( (UCSR1A) & 0x80 ) );
  c =  ( (UDR1) ) ;
  ;
  return c;
}


