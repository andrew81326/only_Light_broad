//-----------------------------------------------------------------------------
// SMBus_EEPROM.c
//-----------------------------------------------------------------------------
// Copyright 2014 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//
// Program Description:
//
// This example demonstrates how the EFM8UB2 SMBus interface can communicate
// with a 256 byte I2C Serial EEPROM (Microchip 24LC02B).
// - Interrupt-driven SMBus implementation
// - Only master states defined (no slave or arbitration)
// - Timer1 used as SMBus clock source
// - Timer2 used by SMBus for SCL low timeout detection
// - SCL frequency defined by <SMB_FREQUENCY> constant
// - Pinout:
//    P1.2 -> SDA (SMBus)
//    P1.3 -> SCL (SMBus)
//
//    P1.6 -> LED
//
//    all other port pins unused
//
// How To Test:
//
// 1) Place the switch in "AEM" mode.
// 2) Connect the device to a 24LC02B serial EEPROM (see the EEPROM datasheet
//    for the pinout information).
// 3) Connect the EFM8UB2 STK board to a PC using a mini USB cable.
// 4) Compile and download code to the EFM8UB2 STK board.
//    In Simplicity Studio IDE, select Run -> Debug from the menu bar,
//    click the Debug button in the quick menu, or press F11.
// 5) Run the code.
//    In Simplicity Studio IDE, select Run -> Resume from the menu bar,
//    click the Resume button in the quick menu, or press F8.
//         a) the test will indicate proper communication with the EEPROM by
//            turning on the LED0 at the end the end of the test
//         b) the test can also be verified by running to the if statements
//            in main and checking the sent and received values by adding
//            the variables to the Watch Window
//
//
// Target:         EFM8UB2
// Tool chain:     Simplicity Studio / Keil C51 9.51
// Command Line:   None
//
// Release 1.0 (BL)
//    - Initial Release
//    - 13 JAN 2015
//

//-----------------------------------------------------------------------------
// Includes and Device-Specific Parameters
//-----------------------------------------------------------------------------

#include <SI_EFM8UB2_Register_Enums.h>
#include <stdio.h>
#include "InitDevice.h"
#include "def.h"

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

// System clock frequency in Hz
#define  SYSCLK         12000000

#define  SMB_FREQUENCY  30000          // Target SCL clock rate
                                       // This example supports between 10kHz
                                       // and 100kHz

#define  WRITE          0x00           // SMBus WRITE command
#define  READ           0x01           // SMBus READ command

// Device addresses (7 bits, lsb is a don't care)
//#define  EEPROM_ADDR    0xA0           // Device address for slave target
                                       // Note: This address is specified
                                       // in the Microchip 24LC02B
                                       // datasheet.
#define  EEPROM_ADDR    0x14

// SMBus Buffer Size
#define  SMB_BUFF_SIZE  0x08           // Defines the maximum number of bytes
                                       // that can be sent or received in a
                                       // single transfer

// Status vector - top 4 bits only
#define  SMB_MTSTA      0xE0           // (MT) start transmitted
#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
#define  SMB_MRDB       0x80           // (MR) data byte received
// End status vector definition

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
uint8_t* pSMB_DATA_IN;           // Global pointer for SMBus data
                                       // All receive data is written here

uint8_t SMB_SINGLEBYTE_OUT;      // Global holder for single byte writes.

uint8_t* pSMB_DATA_OUT;          // Global pointer for SMBus data.
                                       // All transmit data is read from here

uint8_t SMB_DATA_LEN;            // Global holder for number of bytes
                                       // to send or receive in the current
                                       // SMBus transfer.

uint8_t WORD_ADDR;               // Global holder for the EEPROM word
                                       // address that will be accessed in
                                       // the next transfer

uint8_t TARGET;                  // Target SMBus slave address

bit SMB_BUSY = 0;                      // Software flag to indicate when the
                                       // EEPROM_ByteRead() or
                                       // EEPROM_ByteWrite()
                                       // functions have claimed the SMBus

bit SMB_RW;                            // Software flag to indicate the
                                       // direction of the current transfer

bit SMB_SENDWORDADDR;                  // When set, this flag causes the ISR
                                       // to send the 8-bit <WORD_ADDR>
                                       // after sending the slave address.

bit SMB_RANDOMREAD;                    // When set, this flag causes the ISR
                                       // to send a START signal after sending
                                       // the word address.
                                       // For the 24LC02B EEPROM, a random read
                                       // (a read from a particular address in
                                       // memory) starts as a write then
                                       // changes to a read after the repeated
                                       // start is sent. The ISR handles this
                                       // switchover if the <SMB_RANDOMREAD>
                                       // bit is set.

bit SMB_ACKPOLL;                       // When set, this flag causes the ISR
                                       // to send a repeated START until the
                                       // slave has acknowledged its address

// 16-bit SI_SFR declarations



//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
//void EEPROM_ByteWrite(uint8_t addr, uint8_t dat);
//void EEPROM_WriteArray(uint8_t dest_addr, uint8_t* src_addr,
//                       uint8_t len);
//uint8_t EEPROM_ByteRead(uint8_t addr);
//void EEPROM_ReadArray(uint8_t* dest_addr, uint8_t src_addr,
//                     uint8_t len);
void EEPROM_WordWrite(unsigned char SM_addr, unsigned char addr, unsigned char* src_addr,unsigned char len );
void EEPROM_ReadArray ( unsigned char SM_addr,unsigned char src_addr,unsigned char* dest_addr,unsigned char len);

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
//
// Main routine performs all configuration tasks, then loops forever sending
// and receiving SMBus data to the slave EEPROM.
void check_I2C(){
	int i;
	// If slave is holding SDA low because of an improper SMBus reset or error
	   while(!SDA)
	   {
	      // Provide clock pulses to allow the slave to advance out
	      // of its current state. This will allow it to release SDA.
	      XBR1 = 0x40;                     // Enable Crossbar
	      SCL = 0;                         // Drive the clock low
	      for(i = 0; i < 255; i++);        // Hold the clock low
	      SCL = 1;                         // Release the clock
	      while(!SCL);                     // Wait for open-drain
	                                       // clock output to rise
	      for(i = 0; i < 10; i++);         // Hold the clock high
	      XBR1 = 0x00;                     // Disable Crossbar
	   }
}
/*
void main (void)
{
   unsigned char in_buff[8] = {0};              // Incoming data buffer
   unsigned char out_buff[8] = "ABCDEFG";       // Outgoing data buffer

   unsigned long bat;

   uint8_t temp_char;            // Temporary variable
   bit error_flag = 0;                 // Flag for checking EEPROM contents
   uint8_t i;                    // Temporary counter variable

   PCA0MD &= ~0x40;                    // WDTE = 0 (disable watchdog timer)

   HFO0CN |= 0x03;

   check_I2C();


   enter_DefaultMode_from_RESET();
   System_Init ();
   Usb_Init ();


   in_buff[0]=0x00;
   in_buff[1]=0x10;

   EEPROM_WordWrite(0x14,0x01,in_buff,2);
   EEPROM_ReadArray ( 0x14,0x01,&out_buff,2);



   while(1);

}*/
//-----------------------------------------------------------------------------
// Support Functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// EEPROM_ByteWrite ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t addr - address to write in the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t dat - data to write to the address <addr> in the EEPROM
//                        range is full range of character: 0 to 255
//
// This function writes the value in <dat> to location <addr> in the EEPROM
// then polls the EEPROM until the write is complete.
//
void EEPROM_ByteWrite(uint8_t addr, uint8_t dat)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // Mark next transfer as a write
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 0;                 // Do not send a START signal after
                                       // the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling (The ISR
                                       // will automatically restart the
                                       // transfer if the slave does not
                                       // acknoledge its address.

   // Specify the Outgoing Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   SMB_SINGLEBYTE_OUT = dat;           // Store <dat> (local variable) in a
                                       // global variable so the ISR can read
                                       // it after this function exits

   // The outgoing data pointer points to the <dat> variable
   pSMB_DATA_OUT = &SMB_SINGLEBYTE_OUT;

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;

}


//------------------------------------------------------------------------------------
// EEPROM_ByteWrite ()
//------------------------------------------------------------------------------------
//
// This function writes the value in <dat> to location <addr> in the EEPROM then
// polls the EEPROM until the write is complete.
//
void EEPROM_WordWrite(unsigned char SM_addr, unsigned char addr, unsigned char* src_addr,unsigned char len )
{
   while (SMB_BUSY);                         // Wait for SMBus to be free.
   SMB_BUSY = 1;                             // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   //TARGET = SLAVE_ADDR;                     // Set target slave address
	TARGET = SM_addr;
   SMB_RW = WRITE;                           // Mark next transfer as a write
   SMB_SENDWORDADDR = 1;                     // Send Word Address after Slave Address
   SMB_RANDOMREAD = 0;                       // Do not send a START signal after
                                             // the word address
   SMB_ACKPOLL = 1;                          // Enable Acknowledge Polling (The ISR
                                             // will automatically restart the
                                             // transfer if the slave does not
                                             // acknoledge its address.

   // Specify the Outgoing Data
   WORD_ADDR = addr;                         // Set the target address in the
                                             // EEPROM's internal memory space


   pSMB_DATA_OUT = src_addr;      // The outgoing data pointer points to
                                             // the <dat> variable.

   SMB_DATA_LEN = len;                         // Specify to ISR that the next transfer
                                             // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;

}


//------------------------------------------------------------------------------------
// EEPROM_ReadArray ()
//------------------------------------------------------------------------------------
// Reads up to 256 data bytes from the EEPROM slave specified by the <EEPROM_ADDR>
// constant.
//
void EEPROM_ReadArray ( unsigned char SM_addr,unsigned char src_addr,unsigned char* dest_addr,
                       unsigned char len)
{
   while (SMB_BUSY);                         // Wait for SMBus to be free.
   SMB_BUSY = 1;                             // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   //TARGET = SLAVE_ADDR;                     // Set target slave address
	TARGET = SM_addr;
   SMB_RW = WRITE;                           // A random read starts as a write
                                             // then changes to a read after
                                             // the repeated start is sent. The
                                             // ISR handles this switchover if
                                             // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;                     // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                       // Send a START after the word address
   SMB_ACKPOLL = 1;                          // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = src_addr;                     // Set the target address in the
                                             // EEPROM's internal memory space

   pSMB_DATA_IN = (unsigned char*) dest_addr;// Set the the incoming data pointer


   SMB_DATA_LEN = len;                       // Specify to ISR that the next transfer
                                             // will contain <len> data bytes


   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                          // Wait until data is read

}


/*
//-----------------------------------------------------------------------------
// EEPROM_WriteArray ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t dest_addr - beginning address to write to in the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t* src_addr - pointer to the array of data to be written
//                        range is full range of character: 0 to 255
//
//   3) uint8_t len - length of the array to be written to the EEPROM
//                        range is full range of character: 0 to 255
//
// Writes <len> data bytes to the EEPROM slave specified by the <EEPROM_ADDR>
// constant.
//
void EEPROM_WriteArray(uint8_t dest_addr, uint8_t* src_addr,
                       uint8_t len)
{
   uint8_t i;
   uint8_t* pData = (uint8_t*) src_addr;

   for( i = 0; i < len; i++ ){
      EEPROM_ByteWrite(dest_addr++, *pData++);
   }

}

//-----------------------------------------------------------------------------
// EEPROM_ByteRead ()
//-----------------------------------------------------------------------------
//
// Return Value :
//   1) uint8_t data - data read from address <addr> in the EEPROM
//                        range is full range of character: 0 to 255
//
// Parameters   :
//   1) uint8_t addr - address to read data from the EEPROM
//                        range is full range of character: 0 to 255
//
// This function returns a single byte from location <addr> in the EEPROM then
// polls the <SMB_BUSY> flag until the read is complete.
//
uint8_t EEPROM_ByteRead(uint8_t addr)
{
   uint8_t retval;               // Holds the return value

   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // A random read starts as a write
                                       // then changes to a read after
                                       // the repeated start is sent. The
                                       // ISR handles this switchover if
                                       // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                 // Send a START after the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   pSMB_DATA_IN = &retval;             // The incoming data pointer points to
                                       // the <retval> variable.

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                    // Wait until data is read

   return retval;

}

//-----------------------------------------------------------------------------
// EEPROM_ReadArray ()
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t* dest_addr - pointer to the array that will be filled
//                                 with the data from the EEPROM
//                        range is full range of character: 0 to 255
//
//   2) uint8_t src_addr - beginning address to read data from the EEPROM
//                        range is full range of character: 0 to 255
//
//   3) uint8_t len - length of the array to be read from the EEPROM
//                        range is full range of character: 0 to 255
//
// Reads up to 256 data bytes from the EEPROM slave specified by the
// <EEPROM_ADDR> constant.
//
void EEPROM_ReadArray (uint8_t* dest_addr, uint8_t src_addr,
                       uint8_t len)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // A random read starts as a write
                                       // then changes to a read after
                                       // the repeated start is sent. The
                                       // ISR handles this switchover if
                                       // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                 // Send a START after the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling

   // Specify the Incoming Data
   WORD_ADDR = src_addr;               // Set the target address in the
                                       // EEPROM's internal memory space

   // Set the the incoming data pointer
   pSMB_DATA_IN = (uint8_t*) dest_addr;

   SMB_DATA_LEN = len;                 // Specify to ISR that the next transfer
                                       // will contain <len> data bytes

   // Initiate SMBus Transfer
   SMB0CN0_STA = 1;
   while(SMB_BUSY);                    // Wait until data is read

}
*/


//-----------------------------------------------------------------------------
// T0_Wait_ms
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) uint8_t ms - number of milliseconds to wait
//                        range is full range of character: 0 to 255
//
// Configure Timer0 to wait for <ms> milliseconds using SYSCLK as its time
// base.
//
void T0_Waitms (uint8_t ms)
{
   TCON &= ~0x30;                      // Stop Timer0; Clear TCON_TF0
   TMOD &= ~0x0f;                      // 16-bit free run mode
   TMOD |=  0x01;

   CKCON0 |= 0x04;                      // Timer0 counts SYSCLKs

   while (ms) {
      TCON_TR0 = 0;                         // Stop Timer0
      TH0 = ((-SYSCLK/1000) >> 8);     // Overflow in 1ms
      TL0 = ((-SYSCLK/1000) & 0xFF);
      TCON_TF0 = 0;                         // Clear overflow indicator
      TCON_TR0 = 1;                         // Start Timer0
      while (!TCON_TF0);                    // Wait for overflow
      ms--;                            // Update ms counter
   }

   TCON_TR0 = 0;                            // Stop Timer0
}


//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
/////////// Jason
extern unsigned char i2c_index;
extern unsigned char i2c_data[32];
char i2c_point=0;



//unsigned char get_one_i2c[32];
//unsigned char one_i2c_index=0;
unsigned char bat1=0;
unsigned char bat2=0;
unsigned char i2c_s=0;
unsigned char bat_flag=0;
unsigned char bat1_l=0;
unsigned int bat1_h=0;
unsigned char bat2_l=0;
unsigned int bat2_h=0;
unsigned int i2c_cnt=0;
////////Jason  end




//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void Delay (void);

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------

bit UART = 0;                          // '0 is UART0; '1' is UART1

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void send_i2c_data_out (unsigned char cc)
{
		if (cc=='P' || cc=='S' )
			   printf (" '%c' ",(uint8_t) cc);
		else
			   printf (" 0x%02x ",(unsigned)cc);

	    if (cc=='P' )
			   printf ("\n");

	    
}	

void send_bat_out (unsigned char cc)
{
       if (cc=='P' )
       {
		  i2c_s=0; 
		  return;
       }
       i2c_cnt++;
       if (i2c_cnt > MAX_I2C)
		i2c_cnt=MAX_I2C;

	   switch (i2c_s)
	   {
		   case 0:
		    if (cc==0x14)
		    {
		       i2c_s=1;
			  i2c_cnt=0; //821 331
		    }
            if (cc==0x16)
			  i2c_s=100; 		  
		   break;
		   case 1:
			   if (cc==0x01)
				  i2c_s=2;
			   else
				  i2c_s=200;   
		   break;
		   case 2:
			   if (cc==0x00)
				  i2c_s=3;
			   else
				  i2c_s=200;   
		   break;
		   case 3:
			   if (cc==0x20)
				  bat_flag=1;
			   else
				  bat_flag=0;  
			  
			   #ifdef DEBUG_MSG
			   if (cc==0x20)
				  printf ("Change to Bat2 0x%02x\n", (unsigned) cc);
			   else
				  printf ("Change to Bat1 0x%02x\n", (unsigned) cc);
			   #endif
               i2c_s=200;			  
		   break;
		   ///////////////////////////////// 0x14 end		   
		   case 100 : // Get 0x16 address
		      if (cc==0x0F)
				  i2c_s=101;
			   else
				  i2c_s=200;   
		   break;
		   case 101:
			   if (cc=='S')
				  i2c_s=102;
			   else
				  i2c_s=200;   
		   break;
		   case 102:
			   if (cc==0x17)
				  i2c_s=103;
			   else
				  i2c_s=200;    
		   break;
		   case 103:
			   if (bat_flag)   
				  bat2_l=cc;
			   else
				  bat1_l=cc;  
			   #ifdef DEBUG_MSG
			   if (bat_flag)
				  printf ("Bat2_l 0x%02x\n", (unsigned)cc);
			   else
				  printf ("Bat1_l 0x%02x\n", (unsigned)cc);
			   #endif
			   i2c_s=104;	  
		   break;
		   case 104:
			     if (bat_flag)   
			   {
				  bat2_h=((((unsigned int)cc << 8) | (unsigned int)bat2_l)*10)/325;
#ifdef  ALWAYS_ON
				  P1=0xF8;
				  if (bat2_h>0)  B2LED0=0;
				  if (bat2_h>20) B2LED1=0;
				  if (bat2_h>40) B2LED2=0;
				  if (bat2_h>60) B2LED3=0;
				  if (bat2_h>80) B2LED4=0;
#endif
			   }
			   else
			   {
				  bat1_h=((((unsigned int)cc << 8) | (unsigned int)bat1_l)*10)/325;
#ifdef  ALWAYS_ON
				  P2=0xF8;
				  if (bat1_h>0)  B1LED0=0;
				  if (bat1_h>20) B1LED1=0;
				  if (bat1_h>40) B1LED2=0;
				  if (bat1_h>60) B1LED3=0;
				  if (bat1_h>80) B1LED4=0;
#endif
			   }
			  #ifdef DEBUG_MSG
			   if (bat_flag)
				  printf ("Bat2_h=%d 0x%02x\n",(unsigned)bat2_h,(unsigned) cc);
			   else
				  printf ("Bat1_h=%d 0x%02x\n",(unsigned)bat1_h,(unsigned) cc);
			   #endif
			   i2c_s=200;	  
		   break;
		 
		   default:
		   break;
	   }
	    
}	 

void I2C_init(void)
{
	   SCON0_TI = 1;
	   SCON1 |= SCON1_TI__SET;

	   // transmit example UART0
	   UART = 0;   // select UART0
	   printf ("Hello, from UART0!\n");
	   UART = 0;
}
void I2C_main (void) {
   char input_char;

   enter_DefaultMode_from_RESET();

   // Indicate that UART0 and UART1 are ready to transmit
   SCON0_TI = 1;
   SCON1 |= SCON1_TI__SET;

   // transmit example UART0
   UART = 0;   // select UART0
   printf ("Hello, from UART0!\n");

   // transmit example UART1
  // UART = 1;                           // select UART1
 //  printf ("Hello, from UART1!\n");

   // receive example: a '1' turns LED on; a '0' turns LED off.
   // select which UART to receive on by changing the next line of code.

   UART = 0;                           // select UART: 0 = UART0, 1 = UART1

   while (1) {

	   if (i2c_index!=i2c_point)
	   {
			// putchar (i2c_data[i2c_point]);
			 #ifdef SEND_I2C_DATA
				send_i2c_data_out (i2c_data[i2c_point]);
			 #else	
				send_bat_out (i2c_data[i2c_point]);
			 #endif
			 i2c_point++;
			 if (i2c_point>I2C_QUEUE)
				 i2c_point=0;
			// LED^=1;
	   }


   }

}

//-----------------------------------------------------------------------------
// putchar
//-----------------------------------------------------------------------------
//
// Return Value : UART0/1 buffer value
// Parameters   : character to be transmitted across UART0/1
//
// This is an overloaded fuction found in the stdio library.  When the
// function putchar is called, either by user code or through calls to stdio
// routines such as printf, the following routine will be executed instead 
// of the function located in the stdio library.
//
// The function checks the UART global variable to determine which UART to 
// use to receive a character.
//
// The routine expands '\n' to include a carriage return as well as a 
// new line character by first checking to see whether the character  
// passed into the routine equals '\n'.  If it is, the routine waits for 
// SCON0_TI/TI1 to be set, indicating that UART 0/1 is ready to transmit another 
// byte.  The routine then clears SCON0_TI/TI1 bit, and sets the UART0/1 output 
// buffer to '0x0d', which is the ASCII character for carriage return.
//
// The routine the waits for SCON0_TI/TI1 to be set, clears SCON0_TI/TI1, and sets
// the UART output buffer to <c>.  
//
//-----------------------------------------------------------------------------

char putchar (char c)  {

   if (UART == 0) {

      if (c == '\n')  {                // check for newline character
         while (!SCON0_TI);                 // wait until UART0 is ready to transmit
         SCON0_TI = 0;                      // clear interrupt flag
         SBUF0 = 0x0d;                 // output carriage return command
      }
      while (!SCON0_TI);                    // wait until UART0 is ready to transmit
      SCON0_TI = 0;                         // clear interrupt flag
      return (SBUF0 = c);              // output <c> using UART 0
   }

   else if (UART == 1) {
      if (c == '\n')  {                // check for newline character
         while (!(SCON1 & 0x02));      // wait until UART1 is ready to transmit
         SCON1 &= ~0x02;               // clear TI1 interrupt flag
         SBUF1 = 0x0d;                 // output carriage return
      }
      while (!(SCON1 & 0x02));         // wait until UART1 is ready to transmit
      SCON1 &= ~0x02;                  // clear TI1 interrupt flag
      return (SBUF1 = c);              // output <c> using UART 1
   }
   else {
	   return 0xff;
   }
}

//-----------------------------------------------------------------------------
// _getkey
//-----------------------------------------------------------------------------
//
// Return Value : byte received from UART0/1
// Parameters   : none

// This is an overloaded fuction found in the stdio library.  When the
// function _getkey is called, either by user code or through calls to stdio
// routines such as scanf, the following routine will be executed instead 
// of the function located in the stdio library.
//
// The function checks the UART global variable to determine which UART to 
// use to receive a character.
//
// The routine waits for SCON0_RI/RI1 to be set, indicating that a byte has
// been received across the UART0/UART1 RX line.  The routine saves the 
// received character into a local variable, clears the SCON0_RI/RI1 interrupt
// flag, and returns the received character value.
//
//-----------------------------------------------------------------------------
char _getkey ()  {
  char c;

  if (UART == 0) {
    while (!SCON0_RI);                      // wait until UART0 receives a character
    c = SBUF0;                         // save character to local variable
    SCON0_RI = 0;                           // clear UART0 receive interrupt flag
    return (c);                        // return value received through UART0
  }

  else if (UART == 1) {
    while (!(SCON1 & 0x01));           // wait until UART1 receives a character
    c = SBUF1;                         // save character to local variable
    SCON1 &= ~0x01;                    // clear UART1 receive interrupt flag
    return (c);                        // return value received through UART1
  }
  else {
	  return 0xff;
  }
}

//-----------------------------------------------------------------------------
// Delay
//-----------------------------------------------------------------------------
//
// Return Value : none
// Parameters   : none
//
// Used for a small pause of approximately 40 us.
//
//-----------------------------------------------------------------------------

void Delay(void)
{
   int16_t x;
   for(x = 0;x < 500;x)
      x++;
}

// 1 is power off 0 is power on
unsigned char check_power(void)
{

	/*unsigned int i;
	unsigned char SDA_PINv;
	unsigned char SCL_PINv;

	i=0;
	SDA_PINv=SDA_PIN;
	SCL_PINv=SCL_PIN;
    while (!SDA_PINv && !SCL_PINv)
	{
    	SDA_PINv=SDA_PIN;
    	SCL_PINv=SCL_PIN;
		i++;
		if (i> 10000)
			return 1;
	}
	*/
	if(i2c_cnt>=MAX_I2C)
		return 1;

	 return 0;
}
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
