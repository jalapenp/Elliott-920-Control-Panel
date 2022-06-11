
/*
 * 
 * name: 920CGUI.c
 * @param
 * @return
 * @author Mick Bell
 * 
 * 920 Controller program for intereacting with an Ellioptt 920C control panel
 * It is built using a Pi4, the PI GPIO with bcm2835 library, Gtk+3 library,
 * ABElectronics IO pi Plus I2C with ABElectronics C API
 * All IPC between 920Controller, GUI and Emulator is done via Unix pipe 
 * for now (local host)
 * For threads (if used), Global shared data (with MUTEX if req) to 
 * do minimal Owner/Client comms
 * 
 * Build using - gcc -Wall -o "%e" "%f" -pthread $(pkg-config gtk+-3.0 --cflags --libs) -export-dynamic -l bcm2835 -l png
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
 * 
 * TODO:
 *
 *  - Command line GPIO Y/N 
 *  - create GPIO blink function under idle/GTK
 *  - NOWAIT on pipes ? 
 *  - integrate AJH emulator (thread) 
 *  - gui blinken lights
 *  - Console Ctrl-C handler
 * 
 * 
 */
 
//#define PI400 1  // Use this to swap between PI400 with one IOPiPlus, and Pi4 with two.
//                    commented out for Pi4 + 2xIOPiPlus cards
 
/**********************************************/
//
// Includes
//
/**********************************************/


#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <string.h>
#include <ctype.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <signal.h>
#include <png.h>
#include <popt.h>


#include <bcm2835.h>
#include "ABE_IoPi.h" // AB Electronics IOPlus card 

#include "920gui.h"  // 920 and Emulate Defines are all in here

 
/**********************************************/
//
// Globals (to hand data across widget callbacks, and threads)
//
/**********************************************/

int      seconds = 0;
guint    timerId = 0;
gboolean GPIO = FALSE;
gboolean pressed = FALSE;
gboolean stopPressed = FALSE;

GtkWidget *jumpBtn;
GtkWidget *quitBtn;
GtkWidget *resetBtn;
GtkWidget *resumeBtn;
GtkWidget *stopBtn;
GtkWidget *timer;
guint	  idleMainRef;

GtkLabel  *status = NULL;
GtkSpinButton *spinAddress = NULL;

uint32_t randLoop = 0;
uint32_t blinkenLoop = 0; 
uint32_t jumpAddress = 0;

int FSMState = STATE_IDLE; 
int buttonJump = HIGH;
int buttonStop = HIGH;
int buttonRset = HIGH;
int buttonRsrt = HIGH;

// Emulator Thread variables - doesn't really matter if update mis-timed 
// so set by thread NOT using MUTEX as these are only for display
uint32_t dispQReg = 0;
uint32_t dispAReg = 0;
uint32_t dispMReg = 0;
uint32_t dispBfReg = 0;

union
{
    uint32_t regInt32;    // to hold Elliot 18 bit word
    uint16_t regInt16[2]; // 2x 16 bit - not sure if needed
    uint8_t  regByte[4];  // 4x Bytes for copying to I2C byte regs
} regVals;

/**********************************************/
//
// 900 Emulator globals 
//
/**********************************************/


typedef int32_t INT32;
typedef int64_t INT64;


/* Diagnostics related variables */
FILE *diag      = NULL;      // diagnostics output - set to either  stderr or .log

/* File handles for peripherals */
FILE *ptrFile   = NULL;       // paper tape reader
FILE *punFile   = NULL;       // paper tape punch
FILE *ttyiFile  = NULL;       // teleprinter input
FILE *ttyoFile  = NULL;       // teleprinter output

INT32 verbose   = 0;       // no diagnostics by default
INT32 diagCount = -1;      // turn diagnostics on at this instruction count
INT32 abandon   = -1;      // abandon on this instruction count 
INT32 diagFrom  = -1;      // turn on diagnostics when first reach this address
INT32 diagLimit = -1;      // stop after this number of instructions executed
INT32 monLoc    = -1;      // report if this location changes
INT32 monLast   = -1;

/* Input output streams */
char *ptrPath   = RDR_FILE;    // path for reader input file
char *punPath   = PUN_FILE;    // path for punch output file
char *ttyInPath = TTYIN_FILE;  // path for teletype input file
char *plotPath  = PLOT_FILE;   // path for plotter output
char *storePath = STORE_FILE;  // path for store image

INT32 lastttych   = -1; // last tty character punched
INT32 punchCount  = -1; // count of paper tape characters punched
INT32 ttyCount    = -1; // count of teletype character typed

/* Emulated store */
INT32 store [STORE_SIZE];
INT32 storeValid = FALSE; // set TRUE when a store image loaded

/* Machine state */
INT32 opKeys = 8181; // setting of keys on operator's control panel, overidden by
                     // -j option
INT32 aReg  = 0, qReg  = 0; // a and q registers
INT32 bReg  = BREGLEVEL1; // address in store of B register 
INT32 scReg = SCRLEVEL1;  // address in store of B register
INT32 lastSCR;       // used to detect dynamic loops
INT32 level = 1;     // priority level
INT64 iCount = 0L;   // count of instructions executed
INT32 instruction, f, a, m;
INT64 fCount[] =     // function code counts
                          {0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L};

/* Tracing */
INT32 traceOne      = FALSE; // TRUE => trace current instruction only

/* Plotter */
unsigned char *plotterPaper = NULL;    // != NULL => plotter has been used.
  
INT32 plotterPenX, plotterPenY, plotterPenDown, plotterUsed;
INT32 plotterPaperWidth  = PAPER_WIDTH;
INT32 plotterPaperHeight = PAPER_HEIGHT;
INT32 plotterPenSize     = PEN_SIZE;    

char ttyoChar;

/**********************************************************/
/*                         Emulator function prototype    */
/**********************************************************/


void  decodeArgs(INT32 argc, const char **argv); // decode command line
void  usage(poptContext optCon, INT32 exitcode, char *error, char *addl);
void  catchInt();              // interrupt handler
INT32 addtoi(char* arg);       // read numeric part of argument
void  emulate();               // run emulation
void  checkAddress(INT32 addr);// check address within store bounds
void  clearStore();            // clear main store
void  readStore();             // read in a store image
void  tidyExit();              // tidy up and exit
void  writeStore();            // dump out store image
void  printDiagnostics(INT32 i, INT32 f, INT32 a); // print diagnostic information for current instruction
void  printTime(INT64 us);     // print out time counted in microseconds
void  printAddr(FILE *f, INT32 addr); // print address in m^nnn format

void  movePlotter(INT32 bits); // Move the plotter pen
void  setupPlotter(void);      // Clear paper to white pixels
void  savePlotterPaper(void);  // Write paper image to PLOT_FILE
INT32 readTape();              // read from paper tape
void  punchTape(INT32 ch);     // punch to paper tape
INT32 readTTY();               // read from teletype
void  writeTTY(INT32 ch);      // write to teletype
void  flushTTY();              // force output of last tty output line
void  loadII();                // load initial orders
INT32 makeIns(INT32 m, INT32 f, INT32 a); // help for loadII
void  putTTYOchar(char ch); 


/**********************************************/
//
// non-GTK /Emulator Function Prototypes
//
/**********************************************/  

gboolean blinkenLights();
gboolean lightsOff();
gboolean stepLights();

static gboolean keyPressGui();

void blinkGPIO();
void catchInt(); 
void clearUp();
void I2CInitialise();


/**********************************************/
//
// GTK / Glade functions
//
/**********************************************/   


//++++++++++++++++++++++++++++ btnJumpClicked

void btnJumpClicked(GtkWidget *widget, 
		       __attribute__((unused)) gpointer   data)
{
    jumpAddress = gtk_spin_button_get_value_as_int (spinAddress);
    g_print("Jump addr = %d\n", jumpAddress);
    if (jumpAddress == 8)
    {
	regVals.regInt32 = 0;
	blinkenLoop = 0;
	

	if (timerId == 0)
	{	
	    // kick off randomised blinking    
	    gtk_label_set_label( status ,"Jump");
	    gtk_widget_set_opacity(jumpBtn, BUTTON_ON);
	    timerId = g_timeout_add(BLINKTIME,blinkenLights,NULL);
	}
	else
	{
	    // double press
	    gtk_label_set_label( status ,"Jump already running");
	}
    }
    else
    {
	// not a valid jump address
	gtk_label_set_label( status ,"Invalid Address");
    }
}

//++++++++++++++++++++++++++++ btnQuitClicked

void btnQuitClicked(GtkWidget *widget, 
		       __attribute__((unused)) gpointer   data)
{
    g_print("Quit Clicked\n");
    gtk_widget_set_opacity(quitBtn, BUTTON_ON);

        
    gtk_label_set_label( status  ,"Exit");
    gtk_widget_queue_draw(widget);
 
    gtk_main_quit();
}

//++++++++++++++++++++++++++++ btnResetClicked

void btnResetClicked( GtkWidget *widget, 
			__attribute__((unused)) gpointer   data)
{
    regVals.regInt32 = 1;
    blinkenLoop = 0;
    
    if (timerId == 0)
    {	
	// first time
	gtk_label_set_label( status ,"Reset");	
	gtk_widget_set_opacity(resetBtn, BUTTON_ON);
	//***MJB if (GPIO) set reset GPIO lamp on 

	timerId = g_timeout_add(BLINKTIME,stepLights,NULL);
    }
    else
    {
	gtk_label_set_label( status ,"Reset already running");
	gtk_widget_queue_draw(widget);
    }
}

//++++++++++++++++++++++++++++ btnResumeClicked
void btnRestartClicked(__attribute__((unused)) GtkWidget *widget, 
			__attribute__((unused)) gpointer   data)
{
    if (timerId == 0)
    {	
	// turn lights on for 5 seconds 
	stopPressed = FALSE;    
	gtk_label_set_label( status ,"Restart");
	gtk_widget_set_opacity(resumeBtn, BUTTON_ON);
	
	// set regs HIGH
	regVals.regInt32 =0xFFFFFFF;	
	//write I2C Regs
	write_port(I2C_Bus0,I2C_BankA, regVals.regByte[0]);
	write_port(I2C_Bus0,I2C_BankB, regVals.regByte[1]);
    #ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, regVals.regByte[2]);
    #endif
	
	//turn them off after 3 secs
	timerId = g_timeout_add_seconds(3, lightsOff, NULL);
    }
    else	
    {
	// double press
	gtk_label_set_label( status ,"Restart already running");
    }
}


//++++++++++++++++++++++++++++ btnStopClicked
void btnStopClicked(__attribute__((unused)) GtkWidget *widget, 
		       __attribute__((unused)) gpointer   data)
{
    if (timerId == 0)
    {	
	stopPressed = TRUE;
	gtk_label_set_label( status ,"Stop");
	gtk_widget_set_opacity(stopBtn, BUTTON_ON);
	//***MJB if (GPIO) set GPIO 11 on	
	// Copy switch I2C Reg to LED I2C Reg
	regVals.regInt32 = 0;

	regVals.regByte[0] = read_port (I2C_Bus1, I2C_BankA);
	regVals.regByte[1] = read_port (I2C_Bus1, I2C_BankB);	
#ifndef PI400
	regVals.regByte[0] = read_port (I2C_Bus2, I2C_BankA);
	regVals.regByte[1] = read_port (I2C_Bus2, I2C_BankB);	
	regVals.regByte[2] = read_port (I2C_Bus3, I2C_BankA);
#endif

	// g_print("Switches = 0x%X\n",regVals.regInt32);
	//write I2C Regs
	write_port(I2C_Bus0,I2C_BankA, regVals.regByte[0]);
	write_port(I2C_Bus0,I2C_BankB, regVals.regByte[1]);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, regVals.regByte[2]);
#endif
	
	//turn them off after 3 secs
	
	timerId = g_timeout_add_seconds(3, lightsOff, NULL);
	//***MJB if (GPIO) set GPIO 11 off
    }
    else	
    {
	// double press
	gtk_label_set_label( status ,"Stop already running");
    }
}

/**********************************************/
//
// 920 functions
//
/**********************************************/  


//++++++++++++++++++++++++++++ blinkenLights

gboolean blinkenLights(__attribute__((unused)) gpointer userData)
{
    if (blinkenLoop < 18)
    {
	regVals.regInt32 = 0;

	// clear I2C lamp registers 
	write_port(I2C_Bus0,I2C_BankA, 0x00);
	write_port(I2C_Bus0,I2C_BankB, 0x00);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, 0x00);
#endif	
	// randomise - replace with pipe for reg-val from Emu900
	regVals.regInt32 = rand() % 262145;  // 2^18 +1

	g_print("Blink Loop %d 0x%X\n", blinkenLoop,regVals.regInt32);	
	// write to I2C lamp registers
	write_port(I2C_Bus0,I2C_BankA, regVals.regByte[0]);
	write_port(I2C_Bus0,I2C_BankB, regVals.regByte[1]);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, regVals.regByte[2]);
#endif
	    
	gtk_label_set_label( status  ,"Blinken ");    
	blinkenLoop++;

    }
    else
    {		
	// clear I2C lamp registers 
	write_port(I2C_Bus0,I2C_BankA, 0x00);
	write_port(I2C_Bus0,I2C_BankB, 0x00);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, 0x00);
#endif

	// reset gloals
	blinkenLoop = 0;
	regVals.regInt32 = 0;
	g_source_remove(timerId);
	timerId = 0;
	
	// Dim the button
	gtk_widget_set_opacity(jumpBtn, BUTTON_DIM);
	
	// update status bar
	gtk_label_set_label( status  ,"Ready");
    }
	    
    return TRUE;
}


//++++++++++++++++++++++++++++ blinkGPIO

void blinkGPIO (int pin, int delay, int times)
{
    if (GPIO)
    {
	for (int i = 0 ; i < times ; i++)
	{
	    bcm2835_gpio_write(pin, HIGH);
	    // should be a GTK timeout
	    bcm2835_delay(delay);
	    
	    bcm2835_gpio_write(pin, LOW);
	    // should be a GTK timeout
	    bcm2835_delay(delay);
	}
    }
}

//++++++++++++++++++++++++++++ clearUp

void clearUp( void ) 
{    
    // clear GPIO LEDs
    bcm2835_gpio_write(LED_JUMP, LOW);
    bcm2835_gpio_write(LED_STOP, LOW);
    bcm2835_gpio_write(LED_RSET, LOW);
    bcm2835_gpio_write(LED_RSRT, LOW);
    bcm2835_gpio_write(LED_REDY, LOW);
  
    // clear I2C registers
    write_port(I2C_Bus0,I2C_BankA, 0x00);
    write_port(I2C_Bus0,I2C_BankB, 0x00);
    write_port(I2C_Bus1,I2C_BankA, 0x00);
    write_port(I2C_Bus1,I2C_BankB, 0x00);
#ifndef PI400
    write_port(I2C_Bus2,I2C_BankA, 0xFF);
    write_port(I2C_Bus2,I2C_BankB, 0xFF);
    write_port(I2C_Bus3,I2C_BankA, 0xFF);
    write_port(I2C_Bus3,I2C_BankB, 0xFF);
#endif
}


//++++++++++++++++++++++++++++++ I2CInitialise 

void I2CInitialise (void)
{    
 
    // ****************   I2C init 
                    
    IOPi_init(I2C_Bus0, I2C_BankA); // initialise 0  of the io pi buses on i2c address I2C_Bus0
    IOPi_init(I2C_Bus0, I2C_BankB); // initialise 1  of the io pi buses on i2c address I2C_Bus0
    IOPi_init(I2C_Bus1, I2C_BankA); // initialise 0  of the io pi buses on i2c address I2C_Bus1
    IOPi_init(I2C_Bus1, I2C_BankB); // initialise 1  of the io pi buses on i2c address I2C_Bus1
#ifndef PI400
    IOPi_init(I2C_Bus2, I2C_BankA); // initialise 0  of the io pi buses on i2c address I2C_Bus2
    IOPi_init(I2C_Bus2, I2C_BankB); // initialise 1  of the io pi buses on i2c address I2C_Bus2
    IOPi_init(I2C_Bus3, I2C_BankA); // initialise 0  of the io pi buses on i2c address I2C_Bus3
    IOPi_init(I2C_Bus3, I2C_BankB); // initialise 1  of the io pi buses on i2c address I2C_Bus3
#endif
    
    // Init I2C I/O 
    
    //set both I2C_Bus 0/1 banks A&B to be output (I2C 20/21)
    set_port_direction(I2C_Bus0, I2C_BankA, 0x00); // set the direction for bank 0 to be output
    set_port_direction(I2C_Bus0, I2C_BankB, 0x00); // set the direction for bank 1 to be output
#ifndef PI400
    set_port_direction(I2C_Bus1, I2C_BankA, 0x00); // set the direction for bank 0 to be output
    set_port_direction(I2C_Bus1, I2C_BankB, 0x00); // set the direction for bank 1 to be output
#else
    set_port_direction(I2C_Bus1, I2C_BankA, 0xFF); // set the direction for Bus 1 bank 0 to be input
    set_port_direction(I2C_Bus1, I2C_BankB, 0xFF); // set the direction for Bus 1 bank 1 to be input
#endif

#ifndef PI400
    // Set I2C_Bus0 banks 0/1 to be input (I2C bus 22/23)
    //
    set_port_direction(I2C_Bus2, I2C_BankA, 0xFF); // set the direction for Bus 2 bank 0 to be input
    set_port_direction(I2C_Bus2, I2C_BankB, 0xFF); // set the direction for Bus 2 bank 1 to be input
    set_port_direction(I2C_Bus3, I2C_BankA, 0xFF); // set the direction for Bus 3 bank 0 to be input
    set_port_direction(I2C_Bus3, I2C_BankB, 0xFF); // set the direction for Bus 3 bank 1 to be input
#endif

    // Setup I2C pullups & Inverts for switch reading
#ifdef PI400
    set_port_pullups(I2C_Bus1, I2C_BankA, 0xFF); // enable internal pullups for bank 0
    invert_port(I2C_Bus1, I2C_BankA, 0xFF); // invert output so bank will read as 0        
    set_port_pullups(I2C_Bus1, I2C_BankB, 0xFF); // enable internal pullups for bank 1
    invert_port(I2C_Bus1, I2C_BankB, 0xFF); // invert output so bank will read as 0
#else
    set_port_pullups(I2C_Bus2, I2C_BankA, 0xFF); // enable internal pullups for bank 0
    invert_port(I2C_Bus2, I2C_BankA, 0xFF); // invert output so bank will read as 0        
    set_port_pullups(I2C_Bus2, I2C_BankB, 0xFF); // enable internal pullups for bank 1
    invert_port(I2C_Bus2, I2C_BankB, 0xFF); // invert output so bank will read as 0
    set_port_pullups(I2C_Bus3, I2C_BankA, 0xFF); // enable internal pullups for bank 0
    invert_port(I2C_Bus3, I2C_BankA, 0xFF); // invert output so bank will read as 0        
    set_port_pullups(I2C_Bus3, I2C_BankB, 0xFF); // enable internal pullups for bank 1
    invert_port(I2C_Bus3, I2C_BankB, 0xFF); // invert output so bank will read as 0
#endif


    // ****************   Init GPIO
    
    // Set the GPIO LED pins to be an output
    //bcm2835_gpio_fsel(LED_JUMP, BCM2835_GPIO_FSEL_OUTP);
    //bcm2835_gpio_fsel(LED_STOP, BCM2835_GPIO_FSEL_OUTP);
    //bcm2835_gpio_fsel(LED_RSET, BCM2835_GPIO_FSEL_OUTP);
    //bcm2835_gpio_fsel(LED_RSRT, BCM2835_GPIO_FSEL_OUTP);
    
    bcm2835_gpio_fsel(LED_REDY, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(LED_ONON, BCM2835_GPIO_FSEL_OUTP);
  
    // set the buttons Input
    bcm2835_gpio_fsel(BUTTON_JUMP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BUTTON_STOP, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BUTTON_RSET, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(BUTTON_RSET, BCM2835_GPIO_FSEL_INPT);
  
    //  with a pullup
    bcm2835_gpio_set_pud(BUTTON_JUMP, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BUTTON_STOP, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BUTTON_RSET, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_set_pud(BUTTON_RSRT, BCM2835_GPIO_PUD_UP);

}
//++++++++++++++++++++++++++++++++++++++++ Idle Main
gint idleMain()
{
    buttonJump = bcm2835_gpio_lev(BUTTON_JUMP);
    buttonStop = bcm2835_gpio_lev(BUTTON_STOP);
    buttonRset = bcm2835_gpio_lev(BUTTON_RSET);
    buttonRsrt = bcm2835_gpio_lev(BUTTON_RSRT);
    
    //g_print("qaz\n");

    switch (FSMState)
    {
	case STATE_IDLE :   //look for work
	
		if  ( !buttonJump )     //ie. 0 = grounded means jump button has been flicked
		{
		    g_print("physical jump button pressed\n");
		    FSMState = STATE_JUMP;
		    
		} 
		else if ( !buttonStop )  //ie. Red button grounded
		{
		    g_print("physical stop pressed\n");
		    FSMState = STATE_STOP;
		    
		} 
		else if ( !buttonRset ) //Amber grounded
		{
		    g_print("physical Reset pressed \n");
		    FSMState = STATE_RSET;
		}
		else if ( !buttonRsrt ) // Green grounded
		{
		    g_print("physical Restart pressed");
		    FSMState = STATE_RSRT;
		}
		
		//...... insert other action checks here - pipes etc
		// update register displays
		
	    break;
	    
	case STATE_RSET :
		
		printf("doing reset\n");
		btnResetClicked ( resetBtn, NULL);
		FSMState = STATE_IDLE;
		
	    break;    
	    
	case STATE_RSRT :
		
		printf("doing restart\n");
		btnResetClicked ( resetBtn, NULL);
		FSMState = STATE_IDLE;
		
	    break;    
	    
	    
	case STATE_JUMP :
		
		printf("doing jump\n");
		
		btnJumpClicked ( jumpBtn, NULL);
				
		FSMState = STATE_IDLE;
	    
	    break;
	    
	case STATE_STOP :
		
		printf("doing stop\n");


		FSMState = STATE_IDLE;
	    break;

	default:
		// Shouldn't get here
		FSMState = STATE_QUIT;
		btnQuitClicked ( quitBtn, NULL);
	    break;
	    
    }  //Swtich 
  
    
    
    return TRUE;
}



//++++++++++++++++++++++++++++ keyPressConsole
void keyPressConsole (int sig, 
	    __attribute__((unused)) void (*handler)(int)) 
{
	g_print("Crtl C Console\n");
	gtk_label_set_label( status  ,"Stopping");
	gtk_main_quit();
}


//++++++++++++++++++++++++++++ keyPressGui

static gboolean keyPressGui(GtkWidget *widget,
			 GdkEventKey *event,
			 gpointer user_data)
{
    // Trap gui CtrlC and orderly shutdown
    if (event->keyval == 99 && event->type == 8 && event->state == 4)
    {
	//it's CRTL-C 
	g_print("Crtl C GUI\n");
	gtk_label_set_label( status  ,"Stopping");
 
	gtk_main_quit();
    }
    return FALSE;
}

//++++++++++++++++++++++++++++ lightsOff
gboolean lightsOff(__attribute__((unused)) gpointer userData)
{   
    
    // clear I2C lamp registers 
    write_port(I2C_Bus0,I2C_BankA, 0x00);
    write_port(I2C_Bus0,I2C_BankB, 0x00);
#ifndef PI400
    write_port(I2C_Bus1,I2C_BankA, 0x00);
#endif
    
    blinkenLoop = 0; 
    regVals.regInt32 = 0;
    g_source_remove(timerId);
    timerId = 0;
    
    // Set the calling button to dim again
    if (stopPressed)
    {
	gtk_widget_set_opacity(stopBtn, BUTTON_DIM);	
    }
    else
    {
	gtk_widget_set_opacity(resumeBtn, BUTTON_DIM);
    }
    gtk_label_set_label( status  ,"Ready");
	    
    return FALSE;
}


//++++++++++++++++++++++++++++ stepLights
gboolean stepLights(__attribute__((unused)) gpointer userData)
{
    
    if (blinkenLoop < 18 )
    {
	// clear I2C lamp registers 
	write_port(I2C_Bus0,I2C_BankA, 0x00);
	write_port(I2C_Bus0,I2C_BankB, 0x00);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, 0x00);
#endif

	// write to I2C lamp registers
	write_port(I2C_Bus0,I2C_BankA, regVals.regByte[0]);
	write_port(I2C_Bus0,I2C_BankB, regVals.regByte[1]);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, regVals.regByte[2]);
#endif

	regVals.regInt32 =  regVals.regInt32 << 1;
	blinkenLoop ++;
    }
    else
    {
	// clear I2C lamp registers 
	write_port(I2C_Bus0,I2C_BankA, 0x00);
	write_port(I2C_Bus0,I2C_BankB, 0x00);
#ifndef PI400
	write_port(I2C_Bus1,I2C_BankA, 0x00);
#endif
	
	blinkenLoop = 0; 
	regVals.regInt32 = 0;
	g_source_remove(timerId);
	timerId = 0;
	
	// Set the GTK button to dim again
	gtk_widget_set_opacity(resetBtn, BUTTON_DIM);
	
	//***MJB if (GPIO) set Lamp GPIO 11 off

	gtk_label_set_label( status  ,"Ready");

    }
	    
    return TRUE;
}


//++++++++++++++++++++++++++++ windowDelete
gboolean windowDelete(__attribute__((unused)) GtkWidget *widget, 
		      __attribute__((unused)) GdkEvent  *event,
		      __attribute__((unused)) gpointer   data)
{
    // Handle the user trying to close the window 
    g_print("%s called.\n",__FUNCTION__);
    blinkGPIO(LED_STOP, BLINKTIME, BLINKREPEAT);
    gtk_main_quit();

    return FALSE;    // Returning TRUE stops the window being deleted.
                    // Returning FALSE allows deletion.   
}

/**********************************************/
//
// All	Emulate functions
//
/**********************************************/  

INT32 addtoi (char *s) {
  INT32 value = 0;
  INT32 i;
  while  ( *s != '\0' )
    {
      INT32 ch = *s++;
      if  ( isdigit(ch) )
        value = value * 10 + ch - (int)'0'; 
      else if ( ch == '^' )
        {
	  if ( (i = atoi(s)) == 0 )
	    return -1; // atoi failed
          return value * 8192 + i;
	}
      else return -1;
    } // while
  return value;
}


/**********************************************************/
/*                         EMULATION                      */
/**********************************************************/


void emulate () {
  //***MJB close main Read Pipe
  //***MJB close emu Write pipe

//*** MJB loop around this until stopped

  INT32 exitCode = EXIT_SUCCESS; // reason for terminating
  INT32 tracing  = FALSE; // true if tracing enabled
  INT64 emTime   = 0L; // crude estimate of 900 elapsed time

  FILE *stop; // used to open stopFile

  // set up machine ready to execute
  clearStore();  // start with a cleared store
  readStore();   // read in store image if available
  loadII();      // load initial orders
  ttyoFile = stdout; // teletype output to stdout
  store[scReg] = opKeys; // set SCR from operator control panel keys
  
  if   ( verbose & 1 )
    {
      fprintf(diag,"Starting execution from location ");
      printAddr(diag, opKeys);
      fputc('\n', diag);
    }
  if   ( monLoc >= 0 ) monLast = store[monLoc]; // set up monitoring
  
  //***
  //*** Main execution loop ***
  //***
  
  // instruction fetch and decode loop
  while ( ++iCount )
    {
// *** MJB update global display regvals
/*

      if ( ! STOPPED )
      {
      
	  if (GLBL_STOP)
	  {
	      ??Save STORE
	      STOPPED = TRUE;
	  }
	  else
	  {
	      if (GLBL_RESUME)
	      {
		?? reload Store
		GLBL_STOP = FALSE;
		STOPPED = FALSE;
	      }
	  }
	  
	  ...exisitng code
      }
      else
      {
	// .... some sort of no-op code ......
      }


*/

      
      // increment SCR
      lastSCR = store[scReg];
      store[scReg]++;
      checkAddress(lastSCR);

      // fetch and decode instruction;
      instruction = store[lastSCR];
      f = (instruction >> FN_SHIFT) & FN_MASK;
      a = (instruction & ADDR_MASK) | (lastSCR & MOD_MASK);
      fCount[f]+=1; // track number of executions of each function code

      // perform B modification if needed
      if ( instruction >= BIT18 )
        {
  	  m = (a + store[bReg]) & MASK16;
	  emTime += 6;
	}
      else
	  m = a & MASK16;

      // perform function determined by function code f
      switch ( f )
        {

        case 0: // Load B
	    checkAddress(m);
	    qReg = store[m]; store[bReg] = qReg;
	    emTime += 30;
	    break;

          case 1: // Add
       	    aReg = (aReg + store[m]) & MASK18;
	    emTime += 23;
	    break;

          case 2: // Negate and add
	    checkAddress(m);
	    aReg = (store[m] - aReg) & MASK18;
	    emTime += 26;
	    break;

          case 3: // Store Q
	    checkAddress(m);
	    store[m] = qReg >> 1;
	    emTime += 25;
	    break;

          case 4: // Load A
	    checkAddress(m);
	    aReg = store[m];
	    emTime += 23;
	    break;

          case 5: // Store A
	    if   ( level == 1 && m >= 8180 && m <= 8191 )
	      {
		if ( verbose & 1 )
	            fprintf(diag,
		      "Write to initial instructions ignored in priority level 1");
	      }
	    else
	      {
		checkAddress(m);
	        store[m] = aReg;
	      }
	    emTime += 25;
	    break;

          case 6: // Collate
	    checkAddress(m);
	    aReg &= store[m];
	    emTime += 23;
	    break;

          case 7: // Jump if zero
  	    if   ( aReg == 0 )
	      {
	        traceOne = tracing && (verbose & 2);
	        store[scReg] = m;
		emTime += 28;
	      }
	    if  ( aReg > 0 )
	      emTime += 21;
	    else
	      emTime += 20;
	    break;

          case 8: // Jump unconditional
	    store[scReg] = m;
	    emTime += 23;
	    break;

          case 9: // Jump if negative
	    if   ( aReg >= BIT18 )
	      {
	        traceOne = tracing && (verbose & 2);
		store[scReg] = m;
		emTime += 25;
	      }
	    emTime += 20;
	    break;

          case 10: // increment in store
	    checkAddress(m);
 	    store[m] = (store[m] + 1) & MASK18;
	    emTime += 24;
	    break;

          case 11:  // Store S
	    {
	      qReg = store[scReg] & MOD_MASK;
	      store[m] = store[scReg] & ADDR_MASK;
	      emTime += 30;
	      break;
	    }

          case 12:  // Multiply
	    {
	      checkAddress(m);
	      {
	        // extend sign bits for a and store[m]
	        const INT64 al = (INT64) ( ( aReg >= BIT18 ) ? aReg - BIT19 : aReg );
	        const INT64 sl = (INT64) ( ( store[m] >= BIT18 ) ? store[m] - BIT19 : store[m] );
	        INT64  prod = al * sl;
	        qReg = (INT32) ((prod << 1) & MASK18 );
	        if   ( al < 0 ) qReg |= 1;
	        prod = prod >> 17; // arithmetic shift
 	        aReg = (int) (prod & MASK18);
	        emTime += 79;
	        break;
	      }
	    }

          case 13:  // Divide
	    {
	      checkAddress(m);
	      {
	        // extend sign bit for aq
	        const INT64 al   = (INT64) ( ( aReg >= BIT18 ) ? aReg - BIT19 : aReg ); // sign extend
	        const INT64 ql   = (INT64) qReg;
	        const INT64 aql  = (al << 18) | ql;
	        const INT64 ml   = (INT64) ( ( store[m] >= BIT18 ) ? store[m] - BIT19 : store[m] );
                const INT64 quot = (( aql / ml) >> 1) & MASK18;
	        const INT32 q     = (INT32) quot;
  	        aReg = q | 1;
	        qReg = q & 0777776;
	        emTime += 79;
	        break;
	      }
	    }

          case 14:  // Shift - assumes >> applied to a signed long or int is arithmetic
	    {
              INT32       places = m & ADDR_MASK;
	      const INT64 al  = (INT64) ( ( aReg >= BIT18 ) ? aReg - BIT19 : aReg ); // sign extend
	      const INT64 ql  = qReg;
	      INT64       aql = (al << 18) | ql;
	      
	      if   ( places <= 2047 )
	        {
		  emTime += (24 + 7 * places);
	          if   ( places >= 36 ) places = 36;
	          aql <<= places;
	        }
	      else if ( places >= 6144 )
	        { // right shift is arithmetic
	          places = 8192 - places;
		  emTime += (24 + 7 * places);
	          if ( places >= 36 ) places = 36;
		  aql >>= places;
	        }  
	      else
	        {
		  flushTTY();
	          fprintf(diag, "*** Unsupported i/o 14 i/o instruction\n");
	          printDiagnostics(instruction, f, a);
	          tidyExit(EXIT_FAILURE);
	          /* NOT REACHED */
	        }

	      qReg = (int) (aql & MASK18);
	      aReg = (int) ((aql >> 18) & MASK18);
	      break;
	    }

            case 15:  // Input/output etc
	      {
                const INT32 z = m & ADDR_MASK;
	        switch   ( z )
	    	  {

		    case 2048: // read from tape reader
		      { 
	                const INT32 ch = readTape(); 
	                aReg = ((aReg << 7) | ch) & MASK18;
			emTime += 4000; // assume 250 ch/s reader
	                break;
	               }

	            case 2052: // read from teletype
		      {
	                const INT32 ch = readTTY();
	                aReg = ((aReg << 7) | ch) & MASK18;
			emTime += 100000; // assume 10 ch/s teletype
	                break;
	              }

		  case 4864: // send to plotter
		    
		      movePlotter(aReg);
		      if   (aReg >= 16 )
		      {
			  emTime += 20000;  // 20ms per step
		      }
		      else
		      {
			  emTime += 3300;   // 3.3ms
		      }		  
		      break;

	            case 6144: // write to paper tape punch 
	              punchTape(aReg & 255);
		      emTime += 9091; // assume 110 ch/s punch
	              break;

	            case 6148: // write to teletype
	              writeTTY(aReg & 255);
		      emTime += 100000; // assume 10 ch/s teletype
	              break;	      
	  
	            case 7168:  // Level terminate
	              level = 4;
	              scReg = SCRLEVEL4;
		      bReg  = BREGLEVEL4;
		      emTime += 19;
	              break;

	            default:
		      flushTTY();
	              fprintf(diag, "*** Unsupported 15 i/o instruction\n");
	              printDiagnostics(instruction, f, a);
	              tidyExit(EXIT_FAILURE);
	              /* NOT REACHED */
		  } // end 15 switch
	      } // end case 15
	} // end function switch

        // check for change on monLoc
        if   ( monLoc >= 0 && store[monLoc] != monLast )
	  {
	    fprintf(diag, "Monitored location changed from %d to %d\n",
		monLast, store[monLoc]);
	    monLast = store[monLoc];
	    traceOne = TRUE;
          }

        // check to see if need to start diagnostic tracing
        if   ( (lastSCR == diagFrom) || ( (diagCount != -1) && (iCount >= diagCount)) )
	    tracing = TRUE;
        if   ( iCount == diagLimit )
	  {
	    tracing = TRUE;
	    abandon = iCount + 1000; // trace 1000 instructions
	  }

        // print diagnostics if required
        if   ( traceOne )
	  {
 	    flushTTY();
	    traceOne = FALSE; // dealt with single case
  	    printDiagnostics(instruction, f, a);
          }
	else if ( tracing && (verbose & 4) )
	  {
	    flushTTY();
	    printDiagnostics(instruction, f, a);
	  }
	  
	// check for limits
        if   ( (abandon != -1) && (iCount >= abandon) )
        {
	  flushTTY();
          if  ( verbose & 1 ) fprintf(diag, "Instruction limit reached\n");
          exitCode = EXIT_LIMITSTOP;
  	  break;
        }

        // check for dynamic stop
        if   ( store[scReg] == lastSCR ) 
	  {
	    flushTTY();
	    if   ( verbose & 1 )
	      {
	        fprintf(diag, "Dynamic stop at ");
	        printAddr(diag, lastSCR);
	         fputc('\n', diag);
	       }
	    if ( (stop = fopen(STOP_FILE, "w")) == NULL )
	      {
		fprintf(stderr, ERR_FOPEN_STOP_FILE);
		perror(STOP_FILE);
		exit(EXIT_FAILURE);
		/* NOT REACHED */
	      }

	    fprintf(stop, "%d", lastSCR);
	    fclose(stop);
	    exitCode = EXIT_DYNSTOP;
	    break;
	  }
// ***MJB put in 10 microsecond delay for "proper" emulation 

    } // end while fetching and decoding instructions

  // execution complete
  
  
  
  
  if   ( verbose & 1 ) // print statistics
    {
      fprintf(diag, "exit code %d\n", exitCode);
      fprintf(diag, "Function code count\n");
      for ( INT32 i = 0 ; i <= 15 ; i++ )
	{
	  fprintf(diag, "%4d: %8lld (%3lld%%)",
		  i, fCount[i], (fCount[i] * 100L) / iCount);
	  if  ( ( i % 4) == 3 ) fputc('\n', diag);
	}
       fprintf(diag, "%lld instructions executed in ", iCount);
       printTime(emTime);
       fprintf(diag, " of simulated time\n");
     }

  tidyExit(exitCode);
}

void checkAddress(INT32 addr)
{
  if   ( addr >= STORE_SIZE )
        {
	  flushTTY();
          fprintf(diag, "*** Address outside of available store (%d)\n", addr);
  	  tidyExit(EXIT_FAILURE);
        }
}


/**********************************************************/
/*             EMU STORE DUMP AND RECOVERY                */
/**********************************************************/

 
void clearStore() {
  for ( INT32 i = 0 ; i < STORE_SIZE ; i++ ) store[i] = 0;
  if  ( verbose & 1 )
    fprintf(diag, "Store (%d words) cleared\n", STORE_SIZE);
}

void readStore () {
  FILE *f  = fopen(storePath, "r");
  if   ( f != NULL )
    {
      // read store image from file
      INT32 i = 0, n, c;
      while ( (c = fscanf(f, "%d", &n)) == 1 )
	{
	  if  ( i >= STORE_SIZE )
	    {
	      fprintf(stderr, "*** %s exceeds store capacity (%d)\n", storePath, STORE_SIZE);
	      exit(EXIT_FAILURE);
	      /* NOT REACHED */
	    }
	  else store[i++] = n;
	} // while
      if ( c == 0 )
 	{
	  fprintf(stderr, "*** Format error in file %s\n", storePath);
	  exit(EXIT_FAILURE);
	  /* NOT REACHED */
        }
      else if ( ferror(f) )
	{
	  fprintf(stderr, "*** Error while reading %s", storePath);
	  perror(" - ");
	  exit(EXIT_FAILURE);
	  /* NOT REACHED */
        }
      fclose(f); // N.B. store file gets re-opened for writing at end of execution
      if   ( verbose & 1 )
	fprintf(diag, "%d words read in from %s\n", i, storePath);
    }
  else if  ( verbose & 1 ) 
    fprintf (diag, "No %s file found, store left empty\n", storePath);

  storeValid = TRUE;
}

void writeStore () {
   FILE *f = fopen(storePath, "w");
   if  ( f == NULL ) {
     fprintf(stderr, ERR_FOPEN_STORE_FILE);
     perror(storePath);
      exit(EXIT_FAILURE);
      /* NOT REACHED */ }
   for ( INT32 i = 0 ; i < STORE_SIZE ; ++i )
     {
       fprintf(f, "%7d", store[i]);
       if  ( ((i%10) == 0) && (i!=0) ) fputc('\n', f);
     }
   if  ( verbose & 1 )
	 fprintf(diag, "%d words written out to %s\n", STORE_SIZE, storePath);
   fclose(f);
}


/**********************************************************/
/*                      EMU DIAGNOSTICS                   */
/**********************************************************/


 void printDiagnostics(INT32 instruction, INT32 f, INT32 a) {
   // extend sign bit for A, Q and B register values
   INT32 an = ( aReg >= BIT18 ? aReg - BIT19 : aReg); 
   INT32 qn = ( qReg >= BIT18 ? qReg - BIT19 : qReg);
   INT32 bn = ( store[bReg] >= BIT18 ? store[bReg] - BIT19 : store[bReg]);
   fprintf(diag, "%10lld   ", iCount); // instruction count
   printAddr(diag, lastSCR);    // SCR and registers
   if   (instruction & BIT18 )
     {
       if   ( f > 9 )
	 fprintf(diag, " /");
      else
	fprintf(diag, "  /"); }
    else if  (f > 9 )
      fprintf(diag, "  ");
    else
      fprintf(diag, "   ");
    fprintf(diag, "%d %4d", f, a);
    fprintf(diag, " A=%+8d (&%06o) Q=%+8d (&%06o) B=%+7d (",
		 an, aReg, qn, qReg, bn);
    printAddr(diag, store[bReg]);
    fprintf(diag, ")\n");
}

void printTime (INT64 us) { // print out time in us
   INT32 hours, mins; float secs;
   hours = us / 360000000L;
   us -= (hours * 360000000L);
   mins = us / 60000000L;
   secs = ((float) (us - mins * 60000000L)) / 1000000L;
   fprintf(diag, "%d hours, %d minutes and %2.2f seconds", hours, mins, secs);
}

 void printAddr (FILE *f, INT32 addr) { // print out address in module form
   fprintf(f, "%d^%04d", (addr >> MOD_SHIFT) & 7, addr & ADDR_MASK);
}

/* Exit and tidy up */
 
void tidyExit (INT32 reason) {
  if ( storeValid )
    {
      flushTTY();
      writeStore(); // save store for next run
      if   ( verbose & 1 )
	fprintf(diag, "Copying over residual input to %s\n", RDR_FILE);
      if  ( ptrFile  != NULL )
	{
	  INT32 ch;
	  FILE *ptrFile2 = fopen(RDR_FILE, "wb");
	  if  ( ptrFile2 == NULL )
	    {
	      fprintf(stderr, "*** Unable to save paper tape to %s", RDR_FILE);
	      perror("");
	      putTTYOchar('\n');
	      exit(EXIT_FAILURE);
	      /* NOT REACHED */
	    }
	  while ( (ch = fgetc(ptrFile)) != EOF ) fputc(ch, ptrFile2);
	  fclose(ptrFile2);
	}
    }
  if ( ptrFile      != NULL ) fclose(ptrFile);
  if ( ttyiFile     != NULL ) fclose(ttyiFile);
  if ( punFile      != NULL ) fclose(punFile);
  if ( plotterPaper != NULL ) savePlotterPaper();
  if ( diag         != stderr ) fclose(diag);

  if ( verbose & 1 ) fprintf(diag, "Exiting %d\n", reason);
  exit(reason);
}


/**********************************************************/
/*                      GRAPH PLOTTER                     */
/**********************************************************/


void setupPlotter (void)
{
    INT32 paperSize = 3*plotterPaperWidth*plotterPaperHeight; 
    // Using 24bit R,G,B so 3 bytes per pixel.
    plotterPaper = malloc(paperSize);

    if  ( plotterPaper != NULL )
    {
	// Set to all 0xFF for white paper.
        memset(plotterPaper,0xFF,paperSize);
    }
    plotterPenX = 1500;
    plotterPenY = plotterPaperHeight-200;
    plotterPenDown = FALSE;
    if ( (plotterPenSize /= 3) == 0 ) plotterPenSize = 1;
    if  ( verbose & 1 ) fprintf(diag, "Starting plotting\n");
}

void savePlotterPaper (void)
{
    char *title = "Elliott 903 Plotter Output";
    INT32 y;
    FILE *fp;
    png_structp png_ptr;
    png_infop info_ptr;

    if  ( plotterPaper == NULL ) return;
    
	// Open file for writing (binary mode)
	fp = fopen(plotPath, "wb");
	if  ( fp == NULL ) {
		fprintf(stderr, ERR_FOPEN_PLOT_FILE);
		perror(plotPath);
		goto finalise;
	}

	// Initialize write structure
	png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if  (png_ptr == NULL ) {
		fprintf(stderr, "Could not allocate PNG write struct\n");
		goto finalise;
	}

	// Initialize info structure
	info_ptr = png_create_info_struct(png_ptr);
	if ( info_ptr == NULL ) {
		fprintf(stderr, "Could not allocate PNG info struct\n");
		goto finalise;
	}

	// Setup Exception handling
	if ( setjmp(png_jmpbuf(png_ptr)) ) {
		fprintf(stderr, "Error during png creation\n");
		goto finalise;
	}

	png_init_io(png_ptr, fp);

	// Write header (8 bit colour depth)
	png_set_IHDR(png_ptr, info_ptr, plotterPaperWidth, plotterPaperHeight,
			8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	// Set title
	if  ( title != NULL ) {
		png_text title_text;
		title_text.compression = PNG_TEXT_COMPRESSION_NONE;
		title_text.key = "Title";
		title_text.text = title;
		png_set_text(png_ptr, info_ptr, &title_text, 1);
	}

	png_write_info(png_ptr, info_ptr);

	// Write image data

	for ( y=0 ; y<plotterPaperHeight ; y++ ) {
		png_write_row(png_ptr, &plotterPaper[y * plotterPaperWidth * 3]);
	}

	// End write
	png_write_end(png_ptr, NULL);

	finalise:
	if  ( fp != NULL ) fclose(fp);
	if  ( info_ptr != NULL ) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	if  ( png_ptr != NULL ) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

}

void movePlotter(INT32 bits)
{
  static INT32 firstCall = TRUE;
  INT32 address;

  if  ( firstCall )  // Only try once !
    {
       setupPlotter();
       firstCall = FALSE;
    }

  if  ( plotterPaper == NULL ) return;   // Paper allocation failed.

  if  ( verbose & 8 ) fprintf(diag, "Plotter code %1o output\n", bits & 63);

  // hard stop at E and W margins
  if  ( (bits & 1 ) && (plotterPenX < plotterPaperWidth ) ) 
		     plotterPenX+=1; // East
  if  ( (bits & 2 ) && (plotterPenX > 0 ) ) 
		     plotterPenX-=1; // West
  if  ( bits &  4 )  plotterPenY-=1; // North
  if  ( bits &  8 )  plotterPenY+=1; // South
  if  ( bits & 16 )  plotterPenDown = FALSE;
  if  ( bits & 32 )  plotterPenDown = TRUE;
 
  if ( plotterPenDown )
    {
      for ( INT32 x = plotterPenX-plotterPenSize; x <= plotterPenX+plotterPenSize; x++ )
	  for ( INT32 y = plotterPenY-plotterPenSize; y <= plotterPenY+plotterPenSize; y++ )
	    if  ( (y >= 0) && ( y < plotterPaperHeight) ) // trim if outside N and S margins
	      {
		address = (y*plotterPaperWidth*3)+(x*3);
		// Three bytes are for R,G,B.  Set all to zero for black pen.
		plotterPaper[address++] = 0x0;
		plotterPaper[address++] = 0x0;
		plotterPaper[address  ] = 0x0;
	      }
    }
}


/**********************************************************/
/*                    PAPER TAPE SYSTEM                   */
/**********************************************************/


/* Paper tape reader */
INT32 readTape() {
  INT32 ch;
  if   ( ptrFile == NULL )
    {
      if  ( (ptrFile = fopen(ptrPath, "rb")) == NULL )
	{
	  flushTTY();
          fprintf(stderr,"*** %s ", ERR_FOPEN_RDR_FILE);
          perror(ptrPath);
          putTTYOchar('\n');
          tidyExit(EXIT_FAILURE);
	  /* NOT REACHED */
        }
      else if  ( verbose & 1 )
	{
	  flushTTY();
	  fprintf(diag, "Paper tape reader file %s opened\n", ptrPath);
	}
    }
  if  ( (ch = fgetc(ptrFile)) != EOF )
      {
	if  ( verbose & 8 )
	  {
	    flushTTY();
	    traceOne = TRUE;
	    fprintf(diag, "Paper tape character %3d read\n", ch);
	  }
        return ch;
      }
    else
      {
	flushTTY();
        if  ( verbose & 1 ) fprintf(diag, "Run off end of input tape\n");
        tidyExit(EXIT_RDRSTOP);
	/* NOT REACHED */
      }
  return 0;   // Too keep gcc happy
}

/* paper tape punch */
void punchTape(INT32 ch) {
  if ( punchCount++ >= REEL )
    {
      flushTTY();
      fprintf(diag,"Excessive output to punch\n");
      exit(EXIT_PUNSTOP);
      /* NOT REACHED */
    }
  if  ( punFile == NULL )
    {
      if  ( (punFile = fopen(punPath, "wb")) == NULL )
	{
	  flushTTY();
	  printf("*** %s ", ERR_FOPEN_PUN_FILE);
	  perror("punPath");
	  putTTYOchar('\n');
	  tidyExit(EXIT_FAILURE);
	  /* NOT REACHED */
	}
      else if  ( verbose & 1 )
	{
	  flushTTY();
	 fprintf(diag, "Paper tape punch file %s opened\n", punPath);
	}
    }
  if  ( fputc(ch, punFile) != ch )
    {
      flushTTY();
      printf("*** Problem writing to ");
      perror(punPath);
      putTTYOchar('\n');
      tidyExit(EXIT_FAILURE);
      /* NOT REACHED */
    }
  if  ( verbose & 8 )
    {
      flushTTY();
      traceOne = TRUE;
      fprintf(diag, "Paper tape character %d punched\n", ch);
    }
}

/* Teletype */
INT32 readTTY() {
  INT32 ch;
  if   ( ttyCount++ >= REEL )
    {
      flushTTY();
      fprintf(stderr,"Excessive output to teletype\n");
      exit(EXIT_PUNSTOP);
      /* NOT REACHED */
    }
  if   ( ttyiFile == NULL )
    {
      if  ( (ttyiFile = fopen(ttyInPath, "rb")) == NULL )
	{
	  flushTTY();
          printf("*** %s ", ERR_FOPEN_TTYIN_FILE);
          perror(ttyInPath);
          putTTYOchar('\n');
          tidyExit(EXIT_FAILURE);
	  /* NOT REACHED */
        }
      else if ( verbose & 1 )
	{
	  flushTTY();
	  fprintf(diag,"Teletype input file %s opened\n", TTYIN_FILE);
	}
    }
    if  ( (ch = fgetc(ttyiFile)) != EOF )
      {
	if ( verbose & 8 )
	  {
	    flushTTY();
	    traceOne = TRUE;
	    fprintf(diag, "Read character %d from teletype\n", ch);
	  }
	putTTYOchar(ch); // local echoing assumed
        return ch;
      }
    else
      {
        if  ( verbose & 1 )
	  {
	    flushTTY();
	    fprintf(diag, "Run off end of teleprinter input\n");
	  }
        tidyExit(EXIT_TTYSTOP);
      }
    return 0;   // Too keep gcc happy
}

void writeTTY(INT32 ch) {
  INT32 ch2 = ( ((ch &= 127) == 10 ) || ((ch >= 32) && (ch <= 122)) ? ch : -1 );
  if  ( verbose & 8 )
    {
      flushTTY();
      traceOne = TRUE;
      fprintf(diag, "Character %d output to teletype", ch);
      if  ( ch2 == -1 )
	fprintf(diag, " - ignored\n");
      else
	fprintf(diag, "(%c)\n", ch2);
    }
  if  ( ch2 != -1 )
      putTTYOchar((lastttych = ch2));
}

void flushTTY() {
  if  ( (lastttych != -1) && (lastttych != '\n') )
    {
      putTTYOchar('\n');
      lastttych = -1;
    }
}

/**********************************************************/
/*               INITIAL INSTRUCTIONS                     */
/**********************************************************/


void loadII() {
  store[8180] = (-3 & MASK18);
  store[8181] = makeIns(0,  0, 8180);
  store[8182] = makeIns(0,  4, 8189);
  store[8183] = makeIns(0, 15, 2048);
  store[8184] = makeIns(0,  9, 8186);
  store[8185] = makeIns(0,  8, 8183);
  store[8186] = makeIns(0, 15, 2048);
  store[8187] = makeIns(1,  5, 8180);
  store[8188] = makeIns(0, 10,    1);
  store[8189] = makeIns(0,  4,    1);
  store[8190] = makeIns(0,  9, 8182);
  store[8191] = makeIns(0,  8, 8177);
  if  ( verbose & 1 )
    fprintf(diag, "Initial orders loaded\n");
}

INT32 makeIns(INT32 m, INT32 f, INT32 a) {
  return ((m << 17) | (f << 13) | a);
}

void putTTYOchar (char ch)
{
  putchar(ch);
//***MJB redirect to pipe for screen display  
}


/**********************************************/
//
//                     MAIN
//
/**********************************************/   

int main ( int argc, char **argv) {
    
    

    GtkWidget *window;
    GtkBuilder *bld = NULL;
    GError *error = NULL;
#ifdef PI400
    printf("PI400 test\n");
#endif
    printf("Version %s %s\n", __DATE__, __TIME__);
    
    // Init GTK windowing 
    gtk_init (&argc , &argv); 

    // if -g / -G option input, no GPIO buttons/lights
    // insert code here !!!!!
    GPIO = FALSE;
    
    // create trap for console Ctrl-C
    // insert code here !!!!!

    if (!bcm2835_init())
    {	
	printf("BMC2835 init failed\n");
	return -1 ;
    }
    
    // Init all the GPIO & I2C registers and pins for 920 lamps & switches
    if (GPIO) I2CInitialise();
    
    // Load GTK Glade gui
    bld = gtk_builder_new();
    if( gtk_builder_add_from_file (bld,"920gui.glade" , &error) == 0)
    {
        g_print("gtk_builder_add_from_file FAILED\n");
	g_print("%s\n",error->message);
	return EXIT_FAILURE;
    }
    
    window  = GTK_WIDGET (gtk_builder_get_object (bld,"mainWin"));

    // Save global widget pointers
    status = GTK_LABEL( gtk_builder_get_object (bld,"statusLbl"));
    spinAddress = GTK_SPIN_BUTTON( gtk_builder_get_object ( bld, "spinJumpAddress"));
    jumpBtn = GTK_WIDGET (gtk_builder_get_object (bld,"btnJump"));
    quitBtn = GTK_WIDGET (gtk_builder_get_object (bld,"btnQuit"));
    resetBtn = GTK_WIDGET (gtk_builder_get_object (bld,"btnReset"));
    resumeBtn = GTK_WIDGET (gtk_builder_get_object (bld,"btnResume"));
    stopBtn = GTK_WIDGET (gtk_builder_get_object (bld,"btnStop"));
    
//***MJB create global variables to hold pointers to file dialog gadgets
    
    // Set button colours from CSS
    GdkDisplay *display = gdk_display_get_default();
    GdkScreen *screen = gdk_display_get_default_screen(display);
    GtkCssProvider *provider = gtk_css_provider_new();
    gtk_style_context_add_provider_for_screen(screen, 
						GTK_STYLE_PROVIDER (provider), 
						GTK_STYLE_PROVIDER_PRIORITY_USER);

    // load GTK CSS
    error = NULL;
    gtk_css_provider_load_from_data(provider, css,-1, &error);
    if (error)
    {
	g_print("CSS override failed\n");
	g_print("%s\n",error->message);
    } 
    

    // individual Widget callbacks are set in Glade "Signals" panel
    // add additional non-Glade signal callbacks
    g_signal_connect(window, "delete_event", G_CALLBACK(windowDelete), NULL);
    g_signal_connect(window, "key_press_event", G_CALLBACK(keyPressGui), NULL);
    gtk_builder_connect_signals(bld,NULL);
 
    // Add GPIO loop for GTK Idle
    if (GPIO) idleMainRef = g_idle_add( idleMain, NULL ); 
 
//***MJB create Pipes
//***MJB create Thread

   

    gtk_widget_show_all (window);
    gtk_main ();
    // Return to here means Window has been closed
    // All lamps off
    // BCM & I2C close
    if (GPIO) clearUp();
        
    return EXIT_SUCCESS;
}
