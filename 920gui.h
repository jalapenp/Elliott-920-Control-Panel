
/********************/
//
// 920 Defines
//
/********************/

#define BLINKTIME 160
#define BLINKREPEAT 32

#define LAMP_ON 1
#define LAMP_OFF 0

#define STEPTIME 160
#define STEPREPEAT 4

// GPIO button pins
#define BUTTON_RSET 4

#define BUTTON_JUMP 17
#define BUTTON_STOP 27
#define BUTTON_RSRT 25


// GPIO LED pins


#define LED_JUMP 15
#define LED_STOP 18
#define LED_RSET 23
#define LED_RSRT 24

#define LED_REDY 22 // power indicator- needs set on/off by rc0.d job on startup & shutdown
#define LED_ONON 26 // doesn't do much - blinks on program start then stays on, blinks again & off on exit


//GTK button brightness
#define BUTTON_DIM 0.4
#define BUTTON_ON 1


// I2C Busses & banks
#define I2C_BankA 0
#define I2C_BankB 1
#define I2C_Bus0 0x20
#define I2C_Bus1 0x21
#define I2C_Bus2 0x22
#define I2C_Bus3 0x23


// Idle Loop States
#define STATE_IDLE 0
#define STATE_RSET 1
#define STATE_JUMP 2
#define STATE_STOP 3
#define STATE_RSRT 4
#define STATE_QUIT -1



//++++++++++++++++++++++++++++++++++
//
// Static
//
//++++++++++++++++++++++++++++++++++

/*
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
*/

//CSS to override default GTK colours (normally a very drab grey)
const char *css = 
"\
button#btnJump {\
    background-color: cornflowerblue;\
    color: black;\
    font-size: 20px;\
    font-weight: bolder;\
    border: 2px solid black;\
}\
button#btnReset {\
    background-color: gold;\
    color: black;\
    font-size: 20px;\
    font-weight: bolder;\
    border: 2px solid black;\
}\
button#btnResume {\
    background-color: yellowgreen;\
    color: black;\
    font-size: 20px;\
    font-weight: bolder;\
    border: 2px solid black;\
}\
button#btnStop {\
    background-color: tomato;\
    color: black;\
    font-size: 20px;\
    font-weight: bolder;\
    border: 2px solid black;\
}\
textview {\
    background-color: gray;\
    font-size : 10pt;\
}\
";


/**********************************************************/
/*                         EMU 900 DEFINES                */
/**********************************************************/


// Default file names
#define LOG_FILE   "log.txt"   // diagnostic output file path
#define RDR_FILE   ".reader"   // paper tape reader input file path
#define PUN_FILE   ".punch"    // paper tape punch output file path
#define TTYIN_FILE ".ttyin"    // teletype input file path
#define STORE_FILE ".store"    // store image - n.b., ERR_FOPEN_STORE_FILE
#define PLOT_FILE  ".plot.png" // plotter output as png file
#define STOP_FILE  ".stop"     // dynamic stop address
#define TTYOUT_FILE ".ttyout"  // save ttyo text

#define USAGE "Usage: emu900[-adjmrstv] <reader file> <punch file> <teletype file>\n"
#define ERR_FOPEN_DIAG_LOGFILE  "Cannot open log file"
#define ERR_FOPEN_RDR_FILE      "Cannot open paper tape input file - "
#define ERR_FOPEN_PUN_FILE      "Cannot open paper tape punch file - "
#define ERR_FOPEN_TTYIN_FILE    "Cannot open teletype input file  - "
#define ERR_FOPEN_PLOT_FILE     "Could not open plotter output file for writing - "
#define ERR_FOPEN_STORE_FILE    "Could not open store dump file for writing - "
#define ERR_FOPEN_STOP_FILE     "Could not open stop file for writing - "

/* defined in glib header
// Booleans
#define TRUE  1
#define FALSE 0
*/

// Exit codes from emulation
#define EXIT_DYNSTOP       0 // = EXIT_SUCCESS
//      EXIT_FAILURE       1
#define EXIT_RDRSTOP       2
#define EXIT_TTYSTOP       4
#define EXIT_LIMITSTOP     8
#define EXIT_PUNSTOP      16

/* Useful constants */
#define BIT19       01000000
#define MASK18       0777777
#define BIT18       00400000
#define MASK16      00177777
#define ADDR_MASK       8191
#define MOD_MASK    00160000
#define MOD_SHIFT         13
#define FN_MASK           15
#define FN_SHIFT          13

// Locations of B register and SCR for priority levels 1 and 4
#define SCRLEVEL1  0
#define SCRLEVEL4  6
#define BREGLEVEL1 1
#define BREGLEVEL4 7

#define STORE_SIZE 16384 // 16K

#define REEL 10*12*1000  // reel of paper tape in characters (1,000 feet, 10 ch/in)

#define PAPER_WIDTH  3600  // 0.1 mm steps - 34cm max on B-L plotter
#define PAPER_HEIGHT 3600  // 0.1 mm stemps
#define PEN_SIZE        4  // pen nib size in steps
