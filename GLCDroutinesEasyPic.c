/******************************************************************************
 *
 * Author: Gabriel LoDolce
 * Author of last change: Josh White
 * Date of last change: 6/30/2017
 * Revision: 1.0
 *
 *******************************************************************************
 * 
 * FileName:        GLCDroutinesEasyPic.c
 * Dependencies:    Delays.h and p18cxxx.h
 * Processor:       PIC18F
 * Compiler:        C18
 *
 *******************************************************************************
 *
 * File Description: Implementation file for the GLCD routines library
 *
 ******************************************************************************/

#include <delays.h>
#include <string.h>
#include "GLCDroutinesEasyPic.h"

/*------------------------------------------------------------------------------
 * Static variables for internal use (not visible outside of this file)
 -----------------------------------------------------------------------------*/

/* The following instruction delays assume 16MHz clock. These can be changed for
** other oscillator frequencies but note the valid range is only 0-255
*/
static const unsigned char EnableDelayCount_ = 2; // 5 us delay
static const unsigned char DataDelayCount_ = 4; // 1 us delay

/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	WriteData
 *     Parameters:      unsigned char data
 *     Description:		This function sends a byte to the GLCD by setting LATD
 *                      and cycling the enable pin. To write data to the GLCD,
 *                      RS must be logic HIGH and RW must be logic LOW.
 *
 ******************************************************************************/
void WriteData(unsigned char data){
    GLCD_RS_LAT = 1;      // RS must be logic HIGH with RW logic LOW to tell the
    GLCD_RW_LAT = 0;      // GLCD that data is being written to it.
    
    GLCD_DATA_LAT = data; // Set the Data LAT line to the data input.
    
    Delay1TCY();Delay1TCY();Delay1TCY();Delay1TCY(); // Delay 1 us
    
    Enable();
}

/******************************************************************************
 *     Function Name:	Enable
 *     Parameters:      None
 *     Description:		This function switches the EN pin of the GLCD to enable 
 *                      data or an instruction
 *
 ******************************************************************************/
void Enable( void ){
    GLCD_E_LAT = 1;  // The GLCD E pin must first be driven logic HIGH...
    Delay10TCYx( EnableDelayCount_ ); // Delay 5 us
    GLCD_E_LAT = 0;  // and then driven logic LOW to submit the command.
    Delay10TCYx( EnableDelayCount_ );
    
}

/******************************************************************************
 *     Function Name:	InitGLCD
 *     Parameters:      None
 *     Description:		This function initializes the 123x64 WDG0151 graphic LCD
 *                      by sending the display on command to both controllers
 *
 ******************************************************************************/
void InitGLCD( void ){
    GLCD_CS0_LAT = 1;   // Turn both GLCD controllers off.
    GLCD_CS1_LAT = 1;
    GLCD_RST_LAT = 1;   // Enable GLCD RESET.
    GLCD_CS0_LAT = 0;
    GLCD_CS1_LAT = 0;   // Turn both controllers on again.
    GLCD_RS_LAT = 0;    // RS and RW instruction write configuration.
    GLCD_RW_LAT = 0;
    GLCD_DATA_LAT = 0x3F;   // This command turns the GLCD on.
    Enable();
}

/******************************************************************************
 *     Function Name:	SetColumn
 *     Parameters:      unsigned char col
 *     Description:		This function sets the cursor to the column on the GLCD
 *                      specified by col, which is a number from 0 to 127
 *
 ******************************************************************************/
void SetColumn(unsigned char col){
    GLCD_RS_LAT = 0;    // RS and RW low sets the GLCD to take instructions.
    GLCD_RW_LAT = 0;
    if(col < 64){       // If col is within the first 64 columns then it is
        GLCD_CS0_LAT = 0; // being controlled by the first controller.
        GLCD_CS1_LAT = 1;
    }else{              // Otherwise it is being controlled by the second one.
        GLCD_CS0_LAT = 1;
        GLCD_CS1_LAT = 0;
        col = col - 64; // No matter what, both controllers need a value 0 - 63.
    }                      
    col = (col | 0x40) & 0x7F; // To set Y address (column), DB7 must be LOW and
    GLCD_DATA_LAT = col;        // DB6 must be HIGH - hence the OR. Then the AND
    Enable();                   // sets the col values and retains the previous
}                               // settings.

/******************************************************************************
 *     Function Name:	SetPage
 *     Parameters:      unsigned char page
 *     Description:		This function sets the page of the GLCD. Can be from 0 
 *                      to 7.
 *
 ******************************************************************************/
void SetPage(unsigned char page){
    if(page > 7){       // Checks that the input is within the allowable range.
        return;
    }
    GLCD_RS_LAT = 0;    // Set RS and RW for an instruction.
    GLCD_RW_LAT = 0;
    page = (page | 0xB8) & 0xBF; // Similar to setting the column, setting the X
    GLCD_DATA_LAT = page;         // location (page) requires setting DB7 and
    Enable();                     // DB5:3 HIGH, DB6 LOW, and setting DB2:0 to
}                                 // the desired page value.

/******************************************************************************
 *     Function Name:	SetCursor
 *     Parameters:      unsigned char x, unsigned char y
 *     Description:		This function sets the page and column of the GLCD.
 *
 ******************************************************************************/
void SetCursor(unsigned char col, unsigned char page){
    SetColumn(col);     // Sets the desired column (X) and page (Y) together.
    SetPage(page);      // SetPage does not need to change CS since it is already
}                         // set using SetColumn.

/******************************************************************************
 *     Function Name:	ClearLine
 *     Parameters:      unsigned char page, unsigned char half
 *     Description:		This function completely clears the specified page (from
 *                      0 to 7) and half (0 being left and 1 being right) of the
 *                      GLCD.
 *
 ******************************************************************************/
void ClearLine(unsigned char page, unsigned char half){
    unsigned char i;
    if(half == 0){          // Half 0 is the left side of the GLCD.
        GLCD_CS0_LAT = 0;   
        GLCD_CS1_LAT = 1;
        for(i=0;i<65;i++){      // Loop through every column of the chosen page.
            SetCursor(i,page);  // Set page and column.
            WriteData(0x00);    // Writing 0x00 essentially "clears" that column.
        }
    }else if(half == 1){    // Half 1 is the right side of the GLCD.
        GLCD_CS0_LAT = 1;
        GLCD_CS1_LAT = 0;
        for(i=65;i<128;i++){
            SetCursor(i,page);
            WriteData(0x00);
        }
    }else{
        return;
    }
}

/******************************************************************************
 *     Function Name:	ClearGLCD
 *     Parameters:      None
 *     Description:		This function completely clears the entire GLCD screen.
 *
 ******************************************************************************/
void ClearGLCD( void ){
    unsigned char j;
    for(j=0;j<8;j++){       // Loops through every page on both halves of the
        ClearLine(j,0);     // GLCD to clear the entire thing.
        ClearLine(j,1);
    }
}

