/*
 * 
 * name: 920Controller.c
 * @param
 * @return
 * @author Mick Bell
 * 
 * 920 Controller program for intereacting with an Ellioptt 920C control panel
 * It is built using a Pi4, the PI GPIO with bcm2835 library, ABElectronics IO pi Plus
 * I2C with ABElectronics C API
 * All IPC between 920Controller, GUI and Emulator is done via Unix Sockets 
 * for now (local host)
 * For threads (if used), Global shared data (with MUTEX if req) to 
 * do minimal Owner/Client comms
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


//++++++++++++++++++++++++++++++++++
//
// Includes
//
//++++++++++++++++++++++++++++++++++

#include <stdio.h>
#include <signal.h>

#include <bcm2835.h>
#include "ABE_IoPi.h"

//++++++++++++++++++++++++++++++++++
//
// Defines
//
//++++++++++++++++++++++++++++++++++

#define LED_JUMP 15
#define LED_STOP 18
#define LED_REDY 22 // doesn't do much - blinks on power up
#define LED_RSET 23


#define BUTTON_RSET 9
#define BUTTON_JUMP 17
#define BUTTON_STOP 27

#define I2C_Bus0 0x20
#define I2C_Bus1 0x21

#define I2C_BankA 0
#define I2C_BankB 1
#define NUM_SWITCHES 10
#define NUM_LED 18

#define STATE_IDLE 0
#define STATE_RSET 1
#define STATE_JUMP 2
#define STATE_STOP 3
#define STATE_QUIT -1

#define BOOL_TRUE 1
#define BOOL_FALSE 0

//++++++++++++++++++++++++++++++++++
//
// Static
//
//++++++++++++++++++++++++++++++++++

struct I2C 
{
    int pin;
    int channel;
    int bank;
    int offset;
};

struct I2C switches[] = 
{
//	pin	channel		bank		offset
	{1,	I2C_Bus0, 	I2C_BankA, 	0 },
	{2,	I2C_Bus0, 	I2C_BankA, 	1 },
	{3,	I2C_Bus0, 	I2C_BankA, 	2 },
     //	{4,	I2C_Bus0, 	I2C_BankA, 	3 },
	{5,	I2C_Bus0, 	I2C_BankA, 	4 },
     //	{6,	I2C_Bus0, 	I2C_BankA, 	5 },
	{7,	I2C_Bus0, 	I2C_BankA, 	6 },
     //	{8,	I2C_Bus0, 	I2C_BankA, 	7 },
	{9,	I2C_Bus0, 	I2C_BankB, 	8 },
     //	{10,	I2C_Bus0, 	I2C_BankB, 	9 },
	{11,	I2C_Bus0, 	I2C_BankB, 	0xA },
     // {12,	I2C_Bus0, 	I2C_BankB, 	0xB },
	{13,	I2C_Bus0, 	I2C_BankB, 	0xC },
     // {14,	I2C_Bus0, 	I2C_BankB, 	0xD },
	{15,	I2C_Bus0, 	I2C_BankB, 	0xE },
	{16,	I2C_Bus0, 	I2C_BankB, 	0xF }
};
    
struct I2C lamps[] =
{
//	pin	channel		bank		offset
	{1,	I2C_Bus1, 	I2C_BankA, 	0 },
	{2,	I2C_Bus1, 	I2C_BankA, 	1 },
	{3,	I2C_Bus1, 	I2C_BankA, 	2 },
    //	{4,	I2C_Bus1, 	I2C_BankA, 	3 },
	{5,	I2C_Bus1, 	I2C_BankA, 	4 },
    //  {6,	I2C_Bus1, 	I2C_BankA, 	5 },
	{7,	I2C_Bus1, 	I2C_BankA, 	6 },
    //	{8,	I2C_Bus1, 	I2C_BankA, 	7 },
	{9,	I2C_Bus1, 	I2C_BankB, 	8 },
    //	{10,	I2C_Bus1, 	I2C_BankB, 	9 },
	{11,	I2C_Bus1, 	I2C_BankB, 	0xA },
    //	{12,	I2C_Bus1, 	I2C_BankB, 	0xB },
	{13,	I2C_Bus1, 	I2C_BankB, 	0xC },
    //	{14,	I2C_Bus1, 	I2C_BankB, 	0xD },
	{15,	I2C_Bus1, 	I2C_BankB, 	0xE },
	{16,	I2C_Bus1, 	I2C_BankB, 	0xF }
};

 
//++++++++++++++++++++++++++++++++++
//
// Function Prototypes
//
//++++++++++++++++++++++++++++++++++  

void blinkenLights();
void stepLights();
void blinkGPIO();
void catchInt(); 
void clearUp();
void initialise();

 
//++++++++++++++++++++++++++++++++++
//
// MAIN
//
//++++++++++++++++++++++++++++++++++   

int main(int argc, char **argv)
{
    uint64_t systime;
    int FSMState = STATE_IDLE;
    int run = BOOL_TRUE;
    int buttonJump = HIGH;
    int buttonStop = HIGH;
    int buttonRset = HIGH;
    
    // INITIALISE 
    signal(SIGINT, catchInt); // catch control-C to end cleanly
    
    if (!bcm2835_init())
    {	
	printf("BMC2835 init failed\n");
	return -1 ;
    }

    // setup GPIO I/O settings and I2C settings etc
    initialise();
    
    // Ready
    blinkGPIO(LED_REDY, 60, 8);
    // clear
    bcm2835_gpio_write(LED_REDY, HIGH);
    
    // **** Init Server IP Scockets
    
    // **** Load Emul wrapper client store file
    
    // **** Enq/ACK Emul
    
    
    //++++++++++++++++++++++++++++++++++
    // Main loop
    //++++++++++++++++++++++++++++++++++
    
    FSMState = STATE_IDLE;
    
    while (run)
    {

	buttonJump = bcm2835_gpio_lev(BUTTON_JUMP);
	buttonStop = bcm2835_gpio_lev(BUTTON_STOP);
	buttonRset = bcm2835_gpio_lev(BUTTON_RSET);
	
	systime = bcm2835_st_read();	

	
	switch (FSMState)
	{
	    case STATE_IDLE :   //look for work
	    
		    if  ( !buttonJump )     //ie. 0 = grounded means button has been pushed
		    {
			printf("jump pressed\n");
			FSMState = STATE_JUMP;
			
		    } 
		    else if ( !buttonStop )  //ie. Red button grounded
		    {
			printf("stop pressed\n");
			FSMState = STATE_STOP;
			
		    } 
		    else if ( !buttonRset ) //Amber grounded
		    {
			printf("amber pressed \n");
			FSMState = STATE_RSET;
		    }
		    
		    //...... insert other action checks here - sockets etc
		    
		break;
		
	    case STATE_RSET :
		    
		    printf("doing Reset\n");
		    // Flash Amber led		    
		    blinkGPIO(LED_RSET,60,2);
		    
		    // clear
		    buttonRset = HIGH;
		      
		    printf("Flash LEDs in turn\n");
		    stepLights(160,5); // on for 80 ms, 5 times
		    
		    // hang about & LED Off
		    bcm2835_delay(5);
		    bcm2835_gpio_write(LED_RSET, LOW);


		    fflush(stdout);

		    FSMState = STATE_IDLE;
		    
		break;    
		
		
	    case STATE_JUMP :
		    
		    printf("doing jump\n");
		    
		    // clear button press(es) 
		    buttonJump = HIGH;
		    
		    //LED on
		    bcm2835_gpio_write(LED_JUMP, HIGH);
		    
		    blinkenLights(120, 40);
		    
		    // This is where to call Emulator
		    
		    //LED off
		    bcm2835_gpio_write(LED_JUMP, LOW);
		    
		    FSMState = STATE_IDLE;
		
		break;
		
	    case STATE_STOP :
		    
		    printf("doing stop\n");
		    // Flash RED led		    
		    blinkGPIO (LED_STOP, 20, 2);
		    run = BOOL_FALSE;
		    clearUp();
		    FSMState = STATE_QUIT;
		break;

	    default:
		    
		    FSMState = STATE_QUIT;
		break;
		
	}  //Swtich 
	
	systime = bcm2835_st_read() - systime;  // pause - delay 1kums less time taken on this loop
	bcm2835_delayMicroseconds(1000 - systime);
    }
    bcm2835_close();
    return 0;
}
//++++++++++++++++++++++++++++++++++
//
// Functions
//
//++++++++++++++++++++++++++++++++++

void blinkenLights (int delay, int times)
{
    union
    {
	uint32_t randInt32;  // to hold Elliot 18 bit word
	uint16_t randInt16[2];
	uint8_t  randByte[4];
    } randVals;
    
    randVals.randInt32 = 0;
    
    for (int i = 0; i < times; i++)
    {
	// clear I2C lamp registers 
	write_port(I2C_Bus0,I2C_BankA, 0x00);
	write_port(I2C_Bus0,I2C_BankB, 0x00);
	write_port(I2C_Bus1,I2C_BankA, 0x00);
	
	// randomise 
	randVals.randInt32 = rand() % 262145;  // 2^18 +1

	
	// write to I2C lamp registers
	write_port(I2C_Bus0,I2C_BankA, randVals.randByte[0]);
	write_port(I2C_Bus0,I2C_BankB, randVals.randByte[1]);
	write_port(I2C_Bus1,I2C_BankA, randVals.randByte[2]);
	
	bcm2835_delay(delay);
    }
    // clear I2C lamp registers 
    write_port(I2C_Bus0,I2C_BankA, 0x00);
    write_port(I2C_Bus0,I2C_BankB, 0x00);
    write_port(I2C_Bus1,I2C_BankA, 0x00);

}

void stepLights (int delay, int times)
{
    union
    {
	uint32_t stepInt32;  // to hold Elliot 18 bit word
	uint16_t stepInt16[2];
	uint8_t  stepByte[4];
    } stepVals;
    
    
    for (int i = 0; i < times; i++)
    {
	stepVals.stepInt32 = 1;
	
	for (int j = 0; j < 18 ; j++)
	{
	    // clear I2C lamp registers 
	    write_port(I2C_Bus0,I2C_BankA, 0x00);
	    write_port(I2C_Bus0,I2C_BankB, 0x00);
	    write_port(I2C_Bus1,I2C_BankA, 0x00);

	    // write to I2C lamp registers
	    write_port(I2C_Bus0,I2C_BankA, stepVals.stepByte[0]);
	    write_port(I2C_Bus0,I2C_BankB, stepVals.stepByte[1]);
	    write_port(I2C_Bus1,I2C_BankA, stepVals.stepByte[2]);
	    
	    // step lights 
	    stepVals.stepInt32 =  stepVals.stepInt32 << 1;

	    bcm2835_delay(delay);
	}
    }
    
    // clear I2C lamp registers 
    write_port(I2C_Bus0,I2C_BankA, 0x00);
    write_port(I2C_Bus0,I2C_BankB, 0x00);
    write_port(I2C_Bus1,I2C_BankA, 0x00);
}

void blinkGPIO (int pin, int delay, int times)
{
    for (int i = 0 ; i < times ; i++)
    {
	bcm2835_gpio_write(pin, HIGH);
	bcm2835_delay(delay);
	bcm2835_gpio_write(pin, LOW);
	bcm2835_delay(delay);
    }
}
	
	
void catchInt(int32_t sig, 
	    __attribute__((unused)) void (*handler)(int)) 
{
    printf("*** Execution terminated by interrupt 0x%X\n", sig);
    
    clearUp();    
    bcm2835_close(); 
    exit(0);
  
}
void clearUp( void ) 
{
    // clear GPIO LEDs
    bcm2835_gpio_write(LED_JUMP, LOW);
    bcm2835_gpio_write(LED_STOP, LOW);
    bcm2835_gpio_write(LED_RSET, LOW);
    bcm2835_gpio_write(LED_REDY, LOW);
  
    // clear I2C registers
    write_port(I2C_Bus0,I2C_BankA, 0x00);
    write_port(I2C_Bus0,I2C_BankB, 0x00);
    write_port(I2C_Bus1,I2C_BankA, 0x00);
    write_port(I2C_Bus1,I2C_BankB, 0x00);
   
    blinkGPIO(LED_STOP, 60,2);
}
void initialise (void)
{    
 
    // I2C init 
    
    IOPi_init(I2C_Bus0, I2C_BankA); // initialise 0  of the io pi buses on i2c address I2C_Bus0
    IOPi_init(I2C_Bus0, I2C_BankB); // initialise 1  of the io pi buses on i2c address I2C_Bus0
    IOPi_init(I2C_Bus1, I2C_BankA); // initialise 0  of the io pi buses on i2c address I2C_Bus1
    IOPi_init(I2C_Bus1, I2C_BankB); // initialise 1  of the io pi buses on i2c address I2C_Bus1
    
    // insert init 22 & 23 here
    
    //+++++++++++++++++++++++++++ Init I2C I/O 
    // Set I2C_Bus0 banks 0/1 to be input
    set_port_direction(I2C_Bus0, I2C_BankA, 0xFF); // set the direction for bank 0 to be input
    set_port_direction(I2C_Bus0, I2C_BankB, 0xFF); // set the direction for bank 1 to be input
    
    //set both I2C_Buses banks 0/1 to be output
    set_port_direction(I2C_Bus0, I2C_BankA, 0x00); // set the direction for bank 0 to be output
    set_port_direction(I2C_Bus0, I2C_BankB, 0x00); // set the direction for bank 1 to be output
    set_port_direction(I2C_Bus1, I2C_BankA, 0x00); // set the direction for bank 0 to be output
    set_port_direction(I2C_Bus1, I2C_BankB, 0x00); // set the direction for bank 1 to be output
    
    
    // Setup I2C pullups
    //set_port_pullups(I2C_Bus0, 0, 0xFF); // enable internal pullups for bank 0
    //invert_port(I2C_Bus0, I2C_BankA,0xFF); // invert output so bank will read as 0    
    //set_port_pullups(I2C_Bus0, 1, 0xFF); // enable internal pullups for bank 1
    //invert_port(I2C_Bus0, I2C_BankB,0xFF); // invert output so bank will read as 0

    //+++++++++++++++++++++++++++ Init GPIO
    
    // Set the GPIO LED pins to be an output
    bcm2835_gpio_fsel(LED_JUMP, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(LED_STOP, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(LED_RSET, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(LED_REDY, BCM2835_GPIO_FSEL_OUTP);


   
    // set the buttons Input
    bcm2835_gpio_fsel(BUTTON_JUMP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BUTTON_STOP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BUTTON_RSET, BCM2835_GPIO_FSEL_INPT);
  
    //  with a pullup
    bcm2835_gpio_set_pud(BUTTON_JUMP, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BUTTON_STOP, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BUTTON_RSET, BCM2835_GPIO_PUD_UP);

}
    
 
