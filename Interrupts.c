//=========================================================
// src/Interrupts.c: generated by Hardware Configurator
//
// This file will be regenerated when saving a document.
// leave the sections inside the "$[...]" comment tags alone
// or they will be overwritten!
//=========================================================

// USER INCLUDES
#include <SI_EFM8UB2_Register_Enums.h>
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
#define  EEPROM_ADDR    0xA0           // Device address for slave target
                                       // Note: This address is specified
                                       // in the Microchip 24LC02B
                                       // datasheet.
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
extern uint8_t* pSMB_DATA_IN;          // Global pointer for SMBus data
                                       // All receive data is written here

//extern uint8_t SMB_SINGLEBYTE_OUT;     // Global holder for single byte writes.

extern uint8_t* pSMB_DATA_OUT;         // Global pointer for SMBus data.
                                       // All transmit data is read from here

extern uint8_t SMB_DATA_LEN;           // Global holder for number of bytes
                                       // to send or receive in the current
                                       // SMBus transfer.

extern uint8_t WORD_ADDR;              // Global holder for the EEPROM word
                                       // address that will be accessed in
                                       // the next transfer

extern uint8_t TARGET;                 // Target SMBus slave address

extern bit SMB_BUSY;                   // Software flag to indicate when the
                                       // EEPROM_ByteRead() or
                                       // EEPROM_ByteWrite()
                                       // functions have claimed the SMBus

extern bit SMB_RW;                     // Software flag to indicate the
                                       // direction of the current transfer

extern bit SMB_SENDWORDADDR;           // When set, this flag causes the ISR
                                       // to send the 8-bit <WORD_ADDR>
                                       // after sending the slave address.

extern bit SMB_RANDOMREAD;             // When set, this flag causes the ISR
                                       // to send a START signal after sending
                                       // the word address.
                                       // For the 24LC02B EEPROM, a random read
                                       // (a read from a particular address in
                                       // memory) starts as a write then
                                       // changes to a read after the repeated
                                       // start is sent. The ISR handles this
                                       // switchover if the <SMB_RANDOMREAD>
                                       // bit is set.

extern bit SMB_ACKPOLL;                // When set, this flag causes the ISR
                                       // to send a repeated START until the
                                       // slave has acknowledged its address

//-----------------------------------------------------------------------------
// SMBus Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// SMBus ISR state machine
// - Master only implementation - no slave or arbitration states defined
// - All incoming data is written starting at the global pointer <pSMB_DATA_IN>
// - All outgoing data is read from the global pointer <pSMB_DATA_OUT>
//
SI_INTERRUPT(SMBUS0_ISR, SMBUS0_IRQn)
{
   bit FAIL = 0;                       // Used by the ISR to flag failed
                                       // transfers

   static char i;                      // Used by the ISR to count the
                                       // number of data bytes sent or
                                       // received

   static bit SEND_START = 0;          // Send a start

   switch (SMB0CN0 & 0xF0)              // Status vector
   {
      // Master Transmitter/Receiver: START condition transmitted.
      case SMB_MTSTA:
         SMB0DAT = TARGET;             // Load address of the target slave
         SMB0DAT &= 0xFE;              // Clear the LSB of the address for the
                                       // R/W bit
         SMB0DAT |= SMB_RW;            // Load R/W bit
         SMB0CN0_STA = 0;                      // Manually clear START bit
         i = 0;                        // Reset data byte counter
         break;

      // Master Transmitter: Data byte (or Slave Address) transmitted
      case SMB_MTDB:
         if (SMB0CN0_ACK)                      // Slave Address or Data Byte
         {                             // Acknowledged?
            if (SEND_START)
            {
               SMB0CN0_STA = 1;
               SEND_START = 0;
               break;
            }
            if(SMB_SENDWORDADDR)       // Are we sending the word address?
            {
               SMB_SENDWORDADDR = 0;   // Clear flag
               SMB0DAT = WORD_ADDR;    // Send word address

               if (SMB_RANDOMREAD)
               {
                  SEND_START = 1;      // Send a START after the next SMB0CN0_ACK cycle
                  SMB_RW = READ;
               }

               break;
            }

            if (SMB_RW==WRITE)         // Is this transfer a WRITE?
            {

               if (i < SMB_DATA_LEN)   // Is there data to send?
               {
                  // send data byte
                  SMB0DAT = *pSMB_DATA_OUT;

                  // increment data out pointer
                  pSMB_DATA_OUT++;

                  // increment number of bytes sent
                  i++;
               }
               else
               {
                 SMB0CN0_STO = 1;              // Set SMB0CN0_STO to terminte transfer
                 SMB_BUSY = 0;         // Clear software busy flag
               }
            }
            else {}                    // If this transfer is a READ,
                                       // then take no action. Slave
                                       // address was transmitted. A
                                       // separate 'case' is defined
                                       // for data byte recieved.
         }
         else                          // If slave NACK,
         {
            if(SMB_ACKPOLL)
            {
               SMB0CN0_STA = 1;                // Restart transfer
            }
            else
            {
               FAIL = 1;               // Indicate failed transfer
            }                          // and handle at end of ISR
         }
         break;

      // Master Receiver: byte received
      case SMB_MRDB:
         if ( i < SMB_DATA_LEN )       // Is there any data remaining?
         {
            *pSMB_DATA_IN = SMB0DAT;   // Store received byte
            pSMB_DATA_IN++;            // Increment data in pointer
            i++;                       // Increment number of bytes received
            SMB0CN0_ACK = 1;                   // Set SMB0CN0_ACK bit (may be cleared later
                                       // in the code)

         }

         if (i == SMB_DATA_LEN)        // This is the last byte
         {
            SMB_BUSY = 0;              // Free SMBus interface
            SMB0CN0_ACK = 0;                   // Send NACK to indicate last byte
                                       // of this transfer
            SMB0CN0_STO = 1;                   // Send STOP to terminate transfer
         }

         break;

      default:
         FAIL = 1;                     // Indicate failed transfer
                                       // and handle at end of ISR
         break;
   }

   if (FAIL)                           // If the transfer failed,
   {
      SMB0CF &= ~0x80;                 // Reset communication
      SMB0CF |= 0x80;
      SMB0CN0_STA = 0;
      SMB0CN0_STO = 0;
      SMB0CN0_ACK = 0;

      SMB_BUSY = 0;                    // Free SMBus

      FAIL = 0;
   }

   SMB0CN0_SI = 0;                             // Clear interrupt flag
}


//-----------------------------------------------------------------------------
// Timer3 Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// A Timer3 interrupt indicates an SMBus SCL low timeout.
// The SMBus is disabled and re-enabled if a timeout occurs.
//
SI_INTERRUPT(TIMER3_ISR, TIMER3_IRQn)
{
	SMB0CF &= ~0x80;                    // Disable SMBus
	SMB0CF |= 0x80;// Re-enable SMBus
	TMR3CN0 &= ~0x80;// Clear Timer3 interrupt-pending flag
	SMB_BUSY = 0;// Free bus
}

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------


int bit_count=0;
int sda_s=0;
int sda_p=0;

unsigned char i2c_index;


unsigned char i2c_data[32];

int run_first;


unsigned char sda_bit=0x80;
unsigned char sda_index=0;
unsigned char i2c_bits=0;
unsigned char i=0;
//-----------------------------------------------------------------------------
// /INT0 ISR
//-----------------------------------------------------------------------------
//
// Whenever a negative edge appears on P0.0, LED0 is toggled.
// The interrupt pending flag is automatically cleared by vectoring to the ISR
//
//-----------------------------------------------------------------------------
SI_INTERRUPT(INT0_ISR, INT0_IRQn) // SDA_PIN
{
#ifdef	BUTTON_ONLY
#else
	if (SCL_PIN)
	{

		 i2c_data[i2c_index]='S';
		 i2c_index++;
		 if(i2c_index >I2C_QUEUE)
			i2c_index=0;
		 bit_count=0;
		  sda_s=1;
	}
  // LED0 = !LED0;
#endif
}

//-----------------------------------------------------------------------------
// /INT1 ISR
//-----------------------------------------------------------------------------
//
// Whenever a negative edge appears on P0.1, LED1 is toggled.
// The interrupt pending flag is automatically cleared by vectoring to the ISR
//
//-----------------------------------------------------------------------------
SI_INTERRUPT(INT1_ISR, INT1_IRQn)  // SCL_PIN
{

	  if(sda_s)
	  {
		  sda_p=SDA_PIN;

			  while(SCL_PIN)
			  {


					if (SDA_PIN !=sda_p)
					{
						 if( !sda_p )
						 {
							// LED1 ^=LED1;
     						 i2c_data[i2c_index]='P';
						     i2c_index++;
						     if(i2c_index >I2C_QUEUE)
						   		 i2c_index=0;
						 }

						 sda_s=0;
						return;
					}

			 }

		  bit_count ++;
		  if(bit_count> ACK_BIT )
		  {
			  bit_count=0;
			  i2c_data[i2c_index]=i2c_bits;
			  i2c_index++;
			  if(i2c_index >I2C_QUEUE)
			   i2c_index=0;


			  bit_count=0;
			  sda_bit=START_READ_BIT;

			  sda_index=0;
			 i2c_bits=0;

		  }
		  else
		  {

			  if (SDA_PIN  )
			    i2c_bits = i2c_bits | sda_bit;

			  sda_bit>>=1;
			  sda_index++;

		  }



	  }

 //  LED1 = !LED1;
}

