//-----------------------------------------------------------------------------
// F3xx_USB_Main.c
//-----------------------------------------------------------------------------
// Copyright 2005 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This application will communicate with a PC across the USB interface.
// The device will appear to be a mouse, and will manipulate the cursor
// on screen.
//
// How To Test:    See Readme.txt
//
//
// FID:            3XX000006
// Target:         C8051F32x/C8051F340
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
//                 Silicon Laboratories IDE version 2.6
// Command Line:   See Readme.txt
// Project Name:   F3xx_BlinkyExample
//
//
// Release 1.1
//    -Added feature reports for dimming controls
//    -Added PCA dimmer functionality
//    -16 NOV 2006
// Release 1.0
//    -Initial Revision (PD)
//    -07 DEC 2005
//
//-----------------------------------------------------------------------------
// Header Files
//-----------------------------------------------------------------------------

//#include "c8051f3xx.h"
#include <SI_EFM8UB2_Register_Enums.h>
#include <stdio.h>
#include <SI_EFM8UB2_Defs.h>
#include "def.h"

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------
extern void check_I2C(void);

extern void EEPROM_ReadArray (unsigned char SM_addr , unsigned char src_addr,unsigned char* dest_addr,unsigned char len);
extern void EEPROM_WordWrite(unsigned char SM_addr , unsigned char addr, unsigned char* src_addr,unsigned char len );
extern void enter_DefaultMode_from_RESET(void);

int  mode =0;
int  bat_ave=0,bat_num=0;

/////////////////////////////////////////
extern void I2C_init(void);
extern void send_bat_out (unsigned char cc);
extern void send_i2c_data_out (unsigned char cc);
// 1 is power off 0 is power on
extern unsigned char check_power(void);
extern unsigned char i2c_index;
extern unsigned char i2c_data[32];
extern char i2c_point;


extern unsigned int bat1_h;
extern unsigned int bat2_h;

extern bit UART;                          // '0 is UART0; '1' is UART1
/////////////////////////////////////////////
//-----------------------------------------------------------------------------
// Main Routine
//-----------------------------------------------------------------------------
void main(void)
{
	unsigned char bat_buff[8];

   unsigned long batv;

   unsigned int i,j;


   enter_DefaultMode_from_RESET();
  // I2C_init();
 //  PCA0MD &= ~0x40;
 //  HFO0CN |= 0x03;

 //  System_Init ();
//   Usb_Init ();

 //  check_I2C();
  // EA = 1;
//  IE |=IE_EA__ENABLED;
  // BLINK_SELECTOR=0;
  //    SCON0_TI = 1;
 //    SCON1 |= SCON1_TI__SET;

     // transmit example UART0
    // UART = 0;   // select UART0
    // printf ("Hello, from UART0!\n");

     // transmit example UART1
    // UART = 1;                           // select UART1
   //  printf ("Hello, from UART1!\n");

     // receive example: a '1' turns LED on; a '0' turns LED off.
     // select which UART to receive on by changing the next line of code.

     UART = 0;
     I2C_OE=0;
	 
     P1=0xF8;
     P2=0xF8;

    //  P1=0x00;
    // P2=0x00;
   //  while (1)
   // {
    //	 for(i=0;i<100;i++)
    //	    					   {
    //	    					     for (j=0;j<65530;j++);
    //	    					   }
      //  if(SDA_PIN && SCL_PIN)
    //    	EEPROM_ReadArray( 0x16,0x0D,(unsigned long*) &batv,1);
    //	 EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);
        //	EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
   //  }
     while (1)
     {
//#define NORMAL_MODE 1

#if 1 //def 	1 //NORMAL_MODE

       if (BUTTON)
    	  //	 for(i=0;i<100;i++)
    	  //	    					   {
#else
    	    	    					     for (j=0;j<65530;j++);
    	   // 	    					   }
#endif
       {
#ifdef 	NORMAL_MODE
    	   if (check_power())
#else
    	   if(1)
#endif
    	   {
#ifdef 	NORMAL_MODE
    		   I2C_OE=1;
#endif
    		   EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);		//Read Bat1 State
    		   i=bat_buff[0];
    		       if(i&0x01)
    		    	{
    		           		 bat_buff[1]=0x10;
    		               	 EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
    		               	 EEPROM_ReadArray( 0x16,0x0d,(unsigned long*) &batv,1);
    		               	 while(!batv)
    		               	 {
    		               		EEPROM_ReadArray( 0x16,0x0d,(unsigned long*) &batv,1);
    		               	 }
    		               	 		//Read Bat1 State
    		               	 batv>>=24;
    		               	 bat1_h=batv;
    		               	// BLINK_SELECTOR =batv;
    		    }
    		    EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);		//Read Bat1 State
    		    i=bat_buff[0];
    		     if(i&0x02)
    		    {
    		           		 bat_buff[1]=0x20;

    		           		 EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
    		           		 EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);		//Read Bat1 State
    		           		 EEPROM_ReadArray( 0x16,0x0d,(unsigned long*) &batv,1);		//Read Bat1 State
    		           		 batv>>=24;
    		           		 bat2_h=batv;
    		           		// BLINK_SELECTOR =batv|0x80;
    		     }//else


    	   }

    	   {
    					//  bat2_h=((((unsigned int)cc << 8) | (unsigned int)bat2_l)*10)/325;

    					  P1=0xF8;
    					  if (bat2_h>0)  B2LED0=0;
    					  if (bat2_h>20) B2LED1=0;
    					  if (bat2_h>40) B2LED2=0;
    					  if (bat2_h>60) B2LED3=0;
    					  if (bat2_h>80) B2LED4=0;
    					 // bat1_h=((((unsigned int)cc << 8) | (unsigned int)bat1_l)*10)/325;

    					  P2=0xF8;
    					  if (bat1_h>0)  B1LED0=0;
    					  if (bat1_h>20) B1LED1=0;
    					  if (bat1_h>40) B1LED2=0;
    					  if (bat1_h>60) B1LED3=0;
    					  if (bat1_h>80) B1LED4=0;

    					   for(i=0;i<100;i++)
    					   {
    					     for (j=0;j<65530;j++);
    					   }
    					    P1=0xF8;
    					      P2=0xF8;
    	   }
       }

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
	  PCON0|= PCON0_IDLE__IDLE;

   }

     /*
   while (1)
   {



   for(i=0;i<40;i++)
   {
     for (j=0;j<65530;j++);
   }

      if (1) //(BLINK_SELECTORUPDATE)
      {
        // BLINK_SELECTORUPDATE = 0;
         if(bat_num==0){
        	 bat_num=1;


        	 EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);		//Read Bat1 State
        	 i=bat_buff[0];
        	 if(i&0x01){
        		 bat_buff[1]=0x10;
            	 EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
            	 EEPROM_ReadArray( 0x16,0x0d,(unsigned long*) &batv,1);		//Read Bat1 State
            	 batv>>=24;
            	// BLINK_SELECTOR =batv;
        	 }//else
        		// BLINK_SELECTOR =0;
       }else{
        	 bat_num=0;
        	 EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);		//Read Bat1 State
        	 i=bat_buff[0];
        	 if(i&0x02){
        		 bat_buff[1]=0x20;

        		 EEPROM_WordWrite( 0x14,0x01, bat_buff,2);		//Switch to Bat1
        		 // EEPROM_ReadArray( 0x14,0x01,&bat_buff,2);		//Read Bat1 State
        		 EEPROM_ReadArray( 0x16,0x0d,(unsigned long*) &batv,1);		//Read Bat1 State
        		 batv>>=24;
        		// BLINK_SELECTOR =batv|0x80;
        	 }//else
        	//	 BLINK_SELECTOR =0x80;

         }
       //  SendPacket (IN_BLINK_SELECTORID);
      }
   }// While
     */
}

