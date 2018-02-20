 /****** SNAKE ******************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 * 
 * InitializeGLCD
 *
 * Draw Cell
 * 
 * Clear Cell
 *
 * UpdateGLCD
 * 
 * MoveSnake
 *
 * CheckKey
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/
#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include <stdlib.h>
#include "GLCDroutinesEasyPic.h"

#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
unsigned char DIRECTION = 'R';//'R' = right, 'L' = left, 'U' = up, 'D' = down
unsigned char RandX;
unsigned char RandY;
struct Snake {
    unsigned char Length;
    unsigned char X[100];
    unsigned char Y[100];
} Snake;
struct Food {
    unsigned char X;
    unsigned char Y;
} Food;
const unsigned char SNAKE_CELL[] = {0b11111111,
                                0b11111111,
                                0b11111111,
                                0b11111111,
                                0b11111111,
                                0b11111111,
                                0b11111111,
                                0b11111111};
const unsigned char FOOD_CELL[] = {0b11111111,
                                0b10000001,
                                0b10000001,
                                0b10000001,
                                0b10000001,
                                0b10000001,
                                0b10000001,
                                0b11111111};

const unsigned char BLANK_CELL[] = {0b00000000,
                                0b00000000,
                                0b00000000,
                                0b00000000,
                                0b00000000,
                                0b00000000,
                                0b00000000,
                                0b00000000};
/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void DrawCell(char x, char y, char type);
void EraseCell(char x, char y);
void MoveSnake(char DIRECTION);
void CheckKey(void);
void UpdateGLCD(void);
void HiPriISR(void);
void LoPriISR(void);
void TMR0handler(void);     // Interrupt handler for TMR0

#pragma code highVector=0x08
void atHighVector(void)
{
 _asm GOTO HiPriISR _endasm
}
#pragma code

#pragma code lowVector=0x18
void atLowVector(void)
{
 _asm GOTO LoPriISR _endasm
}
#pragma code

/******************************************************************************
 * main()
 *
 * This subroutine loops forever after calling Initial(). In the loop, the
 * GLCD is refreshed and if game over, display score and reset the board.
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
      while(1) {
          CheckKey();
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP0 for compare if desired, and enables LoPri 
 * interrupts for both.
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISC  = 0b00000100;
    LATC = 0x00;
    
    GLCD_CS0_TRIS = 0;
    GLCD_CS1_TRIS = 0;
    GLCD_RS_TRIS = 0;
    GLCD_RW_TRIS = 0;
    GLCD_E_TRIS = 0;
    GLCD_RST_TRIS = 0;
    GLCD_DATA_TRIS = 0;
    
    InitGLCD();
    ClearGLCD();

    //Initialize Snake and Food
    Snake.X[0] = 3;
    Snake.Y[0] = 4;
    Snake.X[1] = 4;
    Snake.Y[1] = 4;
    Snake.X[2] = 5;
    Snake.Y[2] = 4;
    Snake.Length = 4;
    srand ( TMR0L );
    RandX = (int) rand()%16;
    RandY = (int) rand()%8;
    Food.X = RandX;
    Food.Y = RandY;
    DrawCell(Snake.X[0],Snake.Y[0],1);
    DrawCell(Snake.X[1],Snake.Y[1],1);
    DrawCell(Snake.X[2],Snake.Y[2],1);
    DrawCell(Food.X,Food.Y,0);
    
    

    //Switch initialization
    TRISEbits.TRISE2 = 1;//DOWN button
    TRISEbits.TRISE3 = 1;//UP button
    TRISJbits.TRISJ7 = 1;//LEFT button
    TRISJbits.TRISJ3 = 1;//RIGHT button

    // Initializing TMR0
    T0CON = 0x03;                   // 00000100: 16 bit with 32 prescaler
    TMR0L = 0;                      // Clearing TMR0 registers
    TMR0H = 0;                      // Clear high register if used

    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}

/******************************************************************************
 * DrawCell()
 *
 * This subroutine draws either a snake cell or a food cell to the GLCD with an
 * input for the x and y position along with the indication of the type.
 * type = True (Snake Cell), type = False (Food Cell)
 * The x and y are used for the snake cell whereas the food cell is placed randomly
 ******************************************************************************/
void DrawCell(char x, char y, char type) {
    unsigned char i;
    if(type) {
        for(i=0;i<8;i++){
            SetCursor((x*8)+i, y);
            WriteData(SNAKE_CELL[i]);
        }
    }
    else {
        for(i=0;i<8;i++){
            SetCursor((x*8)+i, y);
            WriteData(FOOD_CELL[i]);
        }
    }   
}

/******************************************************************************
 * EraseCell()
 *
 * This subroutine clears cell on the GLCD given an input for the x and y position 
 ******************************************************************************/
void EraseCell(char x, char y) {
    unsigned char i;
    for(i=0;i<8;i++){
        SetCursor((x*8)+i, y);
        WriteData(BLANK_CELL[i]);
    } 
}


/******************************************************************************
 * UpdateGLCD()
 *
 * This subroutine clears the GLCD, turns it off, updates the matrix positions,
 * and turns on the GLCD.
 ******************************************************************************/
void UpdateGLCD() {
	MoveSnake(DIRECTION);
}


/******************************************************************************
 * MoveSnake()
 *
 * This subroutine performs the logic necessary to move the snake by adding to
 * the head and removing from the tail. 
 ******************************************************************************/
void MoveSnake(char DIRECTION) {
    unsigned char i;
    EraseCell(Snake.X[Snake.Length-1],Snake.Y[Snake.Length-1]);
    
    for(i=Snake.Length - 1; i>0; i--) {
        Snake.X[i] = Snake.X[i-1];
        Snake.Y[i] = Snake.Y[i-1];
        DrawCell(Snake.X[i],Snake.Y[i],1);
    }
    
    switch(DIRECTION) {
        case 'R':
            DrawCell(++Snake.X[0],Snake.Y[0],1);
            break;
        case 'L':
            DrawCell(--Snake.X[0],Snake.Y[0],1);
            break;
        case 'U':
            DrawCell(Snake.X[0],++Snake.Y[0],1);
            break;
        case 'D':
            DrawCell(Snake.X[0],--Snake.Y[0],1);
            break;
        default:
            DrawCell(++Snake.X[0],Snake.Y[0],1);
            break;
    }
    
    if(Snake.X[0] == Food.X && Snake.Y[0] == Food.Y) {
        RandX = (int) rand()%16;
        RandY = (int) rand()%8;
        Food.X = RandX;
        Food.Y = RandY;
        DrawCell(Food.X,Food.Y,0);
        Snake.Length++;
        for(i=Snake.Length - 1; i>0; i--) {
            Snake.X[i] = Snake.X[i-1];
            Snake.Y[i] = Snake.Y[i-1];
            DrawCell(Snake.X[i],Snake.Y[i],1);
        }
    
    }
    if(Snake.Length > 5) {
        
    }
    
    if(Snake.X[0] > 15) {
        Reset();
    }
    if(Snake.X[0] < 0) {
        Reset();
    }
    if(Snake.Y[0] > 7) {
        Reset();
    }
    if(Snake.Y[0] < 0) {
        Reset();
    }
}

/******************************************************************************
 * CheckKey()
 *
 * This subroutine checks which button was pressed and returns a value.
 ******************************************************************************/
void CheckKey() {//checks buttons and returns current direction
	
	if (PORTEbits.RE3 == 1 && DIRECTION != 'D' && DIRECTION != 'U') {//if down is pressed in, update dir to 'D'
		Delay10KTCYx(100); 
        DIRECTION = 'D';//update direction
	}
	else if (PORTEbits.RE2 == 1 && DIRECTION != 'U' && DIRECTION != 'D') {
		Delay10KTCYx(100); 
        DIRECTION = 'U';
	}
	else if (PORTJbits.RJ7 == 1 && DIRECTION != 'L' && DIRECTION != 'R') {
		Delay10KTCYx(100); 
        DIRECTION = 'L';
	}
	else if (PORTJbits.RJ3 == 1 && DIRECTION != 'R' && DIRECTION != 'L') {
		Delay10KTCYx(100); 
        DIRECTION = 'R';
	}

}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/
#pragma interrupt HiPriISR
void HiPriISR() {
    
}   // Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. A flag is set and cleared if a
 * button is pressed.
 ******************************************************************************/
#pragma interruptlow LoPriISR 
void LoPriISR() {
    while(1) {
        if( INTCONbits.TMR0IF ) { //Check if TMR0 interrupt flag is set
            UpdateGLCD();
            TMR0handler();
            continue;
        }
        break;
    }
}

/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles GLCD refresh
 ******************************************************************************/
void TMR0handler() {
    LATCbits.LATC4 = ~LATCbits.LATC4; //Toggle RD4
    
    TMR0H = 0x0B;               // Reset TMR0H
    TMR0L = 0xDC;               // Reset TMR0L     

    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}
