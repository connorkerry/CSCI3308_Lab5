/******************************************************************************
 *
 * Author: Gabriel LoDolce
 * Author of last change: Josh White
 * Date of last change: 6/30/2017
 * Revision: 1.0
 *
 *******************************************************************************
 *
 * FileName:        GLCDroutinesEasyPic.h
 * Dependencies:    Delays.h and p18cxxx.h
 * Processor:       PIC18F
 * Compiler:        C18
 *
 *******************************************************************************
 * File Description: This library contains a set of functions for the 
 ******************************************************************************/

#include <p18cxxx.h>

#ifndef _GLCD_ROUTINES_B_
#define _GLCD_ROUTINES_B_

/*------------------------------------------------------------------------------
 * Definitions for this GLCD interface for EasyPic Pro v7
 -----------------------------------------------------------------------------*/

/* CS Pin Assignments */
#define GLCD_CS0_TRIS   TRISBbits.TRISB0
#define GLCD_CS0_LAT    LATBbits.LATB0

#define GLCD_CS1_TRIS   TRISBbits.TRISB1
#define GLCD_CS1_LAT    LATBbits.LATB1

/* RS Pin Assignments */
#define GLCD_RS_TRIS     TRISBbits.TRISB2
#define GLCD_RS_LAT      LATBbits.LATB2

/* RW Pin Assignments */
#define GLCD_RW_TRIS TRISBbits.TRISB3
#define GLCD_RW_LAT LATBbits.LATB3

/* E Pin Assignments */
#define GLCD_E_TRIS	TRISBbits.TRISB4
#define GLCD_E_LAT	LATBbits.LATB4

/* RST Pin Assignments */
#define GLCD_RST_TRIS   TRISBbits.TRISB5
#define GLCD_RST_LAT    LATBbits.LATB5

/* Data Pin Assignments. The GLCD uses the full byte so this is fairly straightforward
*/
#define GLCD_DATA_TRIS TRISD
#define GLCD_DATA_LAT LATD
#define GLCD_DATA_PORT PORTD

/*------------------------------------------------------------------------------
 * Public Library Functions
 -----------------------------------------------------------------------------*/

/******************************************************************************
 *     Function Name:	WriteData
 *     Parameters:      unsigned char data
 *     Description:		This function sends a byte to the GLCD by setting LATD
 *                      and cycling the enable pin
 *
 ******************************************************************************/
void WriteData(unsigned char data);

/******************************************************************************
 *     Function Name:	Enable
 *     Parameters:      None
 *     Description:		This function switches the EN pin of the GLCD to enable 
 *                      data or an instruction
 *
 ******************************************************************************/
void Enable( void );

/******************************************************************************
 *     Function Name:	InitGLCD
 *     Parameters:      None
 *     Description:		This function initializes the 123x64 WDG0151 graphic LCD
 *                      by sending the display on command to both controllers
 *
 ******************************************************************************/
void InitGLCD( void );

/******************************************************************************
 *     Function Name:	SetColumn
 *     Parameters:      unsigned char col
 *     Description:		This function sets the cursor to the column on the GLCD
 *                      specified by col, which is a number from 0 to 127
 *
 ******************************************************************************/
void SetColumn(unsigned char col);

/******************************************************************************
 *     Function Name:	SetPage
 *     Parameters:      unsigned char page
 *     Description:		This function sets the page of the GLCD. Can be from 0 
 *                      to 7.
 *
 ******************************************************************************/
void SetPage(unsigned char page);

/******************************************************************************
 *     Function Name:	SetCursor
 *     Parameters:      unsigned char x, unsigned char y
 *     Description:		This function sets the page (y) and column (x) of the 
 *                      GLCD.
 *
 ******************************************************************************/
void SetCursor(unsigned char x, unsigned char y);

/******************************************************************************
 *     Function Name:	ClearLine
 *     Parameters:      unsigned char page, unsigned char half
 *     Description:		This function completely clears the specified page (from
 *                      0 to 7) and half (0 being left and 1 being right) of the
 *                      GLCD.
 *
 ******************************************************************************/
void ClearLine(unsigned char page, unsigned char half);

/******************************************************************************
 *     Function Name:	ClearGLCD
 *     Parameters:      None
 *     Description:		This function completely clears the entire GLCD screen.
 *
 ******************************************************************************/
void ClearGLCD( void );


#endif


