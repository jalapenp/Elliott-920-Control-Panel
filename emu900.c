// Elliott 903 emulator - Andrew Herbert - 05/04/2021

// Emulator for Elliott 903 / 920B.
// Does not implement 'undefined' effects.
// Has simplified handling of priority levels and initial orders.
// No support (as yet) for: interactive use of teletype, line printer, 
// card reader or magnetic tape.

// Plotter support added by Peter Onion 21/03/2021

// Pre-requisites:
//    LIBOPT for command line decoding
//    LIBPNG for plotter output

// Usage: emu900 [-d?] [-reader=file] [-punch=file] [-ttyin=file] [-plot=file]
//        [-store=file] [-d|-dfile] [-a|-abandon=integer] [-h|-height=integer]
//        [-j|-jump=integer] [-m|-monitor=address] [-p|-Pen=integer]
//        [-r|-rtrace=integer] [-s|-start=address] [-t|-trace=integer]
//        [-w|-width=integer] [-v|-verbose=integer] [-?|--help] [--usage]

// Verbosity is controlled by the -v argument.  The level of reporting can be selected
// by ORing the following values:
//
//    1      -- general diagnostic reports, e.g., dynamic stop, etc
//    2      -- report jumps taken in traces
//    4      -- report every instruction executed in traces 
//    8      -- report input/output characters in traces
//
// By default diagnostic reports are written to stderr, unless the dfile argument is
// present in which case the reports are written to the file log.txt.

// At beginning reads in contents of store from the file .store by default unless
// overridden by the -store argument. If the file cannot be found the store is set to
// all zeros. At the end of run the emulator dumps out contents of store to the store
// file, unless there have been catastrophic errors. This is to simulate
// retention of data in core store between entry points.

// Paper tape input from the file .reader unless overridden by the -reader argument on
// the command line. At the end it copies any unconsumed  input back to the file
// overwriting previous content, unless there have been catastrophic errors. This is to
// emulate leaving a tape in the reader between successive runs.

// The input file should be a raw byte stream representing eight bit paper tape
// codes, either binary of one of the Elliott telecodes.  There is a companion
// program "to900text" which converts a UTF-8 character file to its equivalent
// in Elliott 900 telecode. 

// Teletype input is taken from the file .ttyin unless overridden by the -ttyin
// argument on the command line. Teletype output is sent to stdout.  

// Paper tape output is send to the file .punch, unless overridden by a -punch argument
// on the command line.  The output is a byte stream of binary characters as would
// have been output to the physical punch.  There is a companion program "from900text"
// which converts a file containing 900 telecode output to it's UTF-8 equivalent.

// There is a limit of output characters on paper tape or teletype roughly equal to a
// reel of paper tape (120,000 characters).

// Plotter output is sent to the file .plot.png unless overridden by a -plot argument.
// The output is in a PHG format.

// The size of the plotting area can be set using the -width and -height command line
// arguments.  These set the size in plotter steps.  The size of the pen nib can be
// set using the -pen command line argument.  The default is 3 steps (0.3mm).

// By default the simulator jumps to 8181 to start execution, unless overriden by
// -jump argument on the command line.  The jump address can be in the range 0-8191.

// The emulation terminates either when a dynamic stop is detected or about 1.5 days of
// emulated real time have elapsed.  On a dynamic stop the emulator writes the stop
// address to the file .stop. 

// A smaller limit on maximum number of instructions to be executed can be set using
// the -abandon command line argument.

// A specific store location can be monitored for be written to using the -monitor
// command line argument.  The address must not exceed the available store size.

// The -start argument starts tracing from the specified location. -trace turns on
// tracing after the specified number of instructions have been executed. -rtrace is
// like -start but stops tracing after a further 1000 instructions have been executed.
// -trace overrides -trace.  -trace/-rtrace and -start can both be specified and tracing
// will start from whichever condition occurs first.  The address part of -start must not
// exceed the available store size.

// Addresses for the -start and -monitor arguments can be written in the form m^a where
// m represents an 8K store module number and a an address within the selected store
// module.

// The program exits with an exit code indicating the reason for completion,
// e.g., 0 = dynamic stop, 1 = run out of paper tape input, etc.2 = run out of
// teletype input, 3 = reached execution limit, 255 = catastropic error.  The
// contents of the store,reader, punch and plotter files are undefined after a
// catastrophic error.

/**********************************************************/
/*                     HEADER FILES                       */
/**********************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <png.h>
#include <popt.h>


/**********************************************************/
/*                         EMU 900 DEFINES                        */
/**********************************************************/


// Default file names
#define LOG_FILE   "log.txt"   // diagnostic output file path
#define RDR_FILE   ".reader"   // paper tape reader input file path
#define PUN_FILE   ".punch"    // paper tape punch output file path
#define TTYIN_FILE ".ttyin"    // teletype input file path
#define STORE_FILE ".store"    // store image - n.b., ERR_FOPEN_STORE_FILE
#define PLOT_FILE  ".plot.png" // plotter output as png file
#define STOP_FILE  ".stop"     // dynamic stop address

#define USAGE "Usage: emu900[-adjmrstv] <reader file> <punch file> <teletype file>\n"
#define ERR_FOPEN_DIAG_LOGFILE  "Cannot open log file"
#define ERR_FOPEN_RDR_FILE      "Cannot open paper tape input file - "
#define ERR_FOPEN_PUN_FILE      "Cannot open paper tape punch file - "
#define ERR_FOPEN_TTYIN_FILE    "Cannot open teletype input file  - "
#define ERR_FOPEN_PLOT_FILE     "Could not open plotter output file for writing - "
#define ERR_FOPEN_STORE_FILE    "Could not open store dump file for writing - "
#define ERR_FOPEN_STOP_FILE     "Could not open stop file for writing - "

// Booleans
#define TRUE  1
#define FALSE 0

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


/**********************************************************/
/*                         GLOBALS                        */
/**********************************************************/

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
/*                         FUNCTIONS                      */
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


/**********************************************************/
/*                           MAIN                         */
/**********************************************************/


INT32 main (INT32 argc, const char **argv) {
   signal(SIGINT, catchInt); // allow control-C to end cleanly
   diag = stderr;            // set up diagnostic output for reports
   decodeArgs(argc, argv);   // decode command line and set options etc

   emulate();                // run emulation
   //***MJB tell main  finished 
}

void catchInt(INT32 sig, void (*handler)(int)) {
  flushTTY();
  fprintf(stderr, "*** Execution terminated by interrupt\n");
  tidyExit(EXIT_FAILURE);
}


/**********************************************************/
/*                  DECODE ARGUMENTS                      */
/**********************************************************/


void decodeArgs (INT32 argc, const char *argv[])
{
  INT32 c;
  char *buffer = NULL;
  char number[12];
  poptContext optCon;
  struct poptOption optionsTable[] = {
      {"reader",  '\0', POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &ptrPath, 0, "paper tape reader input", "file"},
      {"punch",   '\0', POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &punPath, 0, "paper tape punch output", "file"},
      {"ttyin",   '\0', POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &ttyInPath, 0, "teletype input", "file"},
      {"plot",    '\0', POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &plotPath, 0, "plotter output", "file"},
      {"store",   '\0', POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &storePath, 0, "store image", "file"},
      {"dfile",   'd',  POPT_ARG_NONE | POPT_ARGFLAG_ONEDASH,
       0, 1, "diagnostics to file", ""},    
      {"abandon", 'a',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &abandon, 0, "abandon after n instructions", "integer"},
      {"height",  'h',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &plotterPaperHeight, 0, "plotter paper height in steps", "integer"},
      {"jump",    'j',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &opKeys, 2, "jump to address", "integer"},
      {"monitor", 'm',  POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &buffer, 3, "monitor location", "address"},
      {"Pen", 'p',      POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &plotterPenSize, 4, "plotter pen size in steps", "integer"},
      {"rtrace",  'r',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &diagLimit, 0, "trace 1000 instructions after "
        "first n", "integer"},
      {"start",   's',  POPT_ARG_STRING | POPT_ARGFLAG_ONEDASH,
       &buffer, 5, "start tracing at location n", "address"},
      {"trace",   't',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &diagCount, 0, "turn on tracing after n instructions", "integer"},
      {"width",   'w',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &plotterPaperWidth, 0, "plotter paper width in steps", "integer"},
      {"verbose", 'v',  POPT_ARG_INT | POPT_ARGFLAG_ONEDASH,
       &verbose, 0, "verbosity", "integer"},
      POPT_AUTOHELP
      POPT_TABLEEND
    };

  optCon = poptGetContext(NULL, argc, argv, optionsTable, 0);

  while ( (c = poptGetNextOpt(optCon)) > 0 ) {

    switch (c) {

    case 1: // -d
      if ( ! (diag = fopen(LOG_FILE, "w"))  )
	{
	  fprintf(stderr, "Cannot open log file %s:", LOG_FILE);
	  perror(ERR_FOPEN_DIAG_LOGFILE);
	  exit(EXIT_FAILURE);
	  /* NOT REACHED */
	}
      fprintf(stderr, "Diagnostics are being sent to file %s\n", LOG_FILE);
      break;

    case 2: // j address
      if ( opKeys >= 8192 )
	usage(optCon, EXIT_FAILURE, "can only jump to addresses less than 8192", NULL);
      break;

    case 3: // m address (monitor address)
      monLoc = addtoi(buffer);
      if ( monLoc == -1 )
	usage(optCon, EXIT_FAILURE, "malformed address", buffer);
      if ( monLoc >= STORE_SIZE )
	usage(optCon, EXIT_FAILURE, "monitor address outside store bounds", buffer);
      break;

    case 4: // p plotter pen size
      if ( plotterPenSize > 12 )
	usage(optCon, EXIT_FAILURE, "maximum pen size is 12", NULL);
      break;

    case 5: // s address (trace from location)
      diagFrom = addtoi(buffer);
      if ( diagFrom == -1 )
	usage(optCon, EXIT_FAILURE, "malformed address", buffer);
      if ( diagFrom >= STORE_SIZE )
	usage(optCon, EXIT_FAILURE, "tracing start address outside store bounds", buffer);
      break;
      
    default:
      fprintf(stderr, "internal error in decodeArgs (%d)\n", c);
      exit(EXIT_FAILURE);
      /* NOT REACHED */
    }					 
  }

  if ( c < -1 ) // bombed out on an error
    {
      fprintf(stderr, "%s: %s\n", poptBadOption(optCon, 0), poptStrerror(c));
      exit(EXIT_FAILURE);
      /* NOT REACHED */
    }
  
  if ( (buffer = (char *) poptGetArg(optCon)) != NULL ) // check for extra arguments
       usage(optCon, EXIT_FAILURE, "unexpected argument", buffer);

  poptFreeContext(optCon); // release context
       
  // tidy up and report options
  if ( verbose >= 16 )
    {
      sprintf(number, "%d", verbose);
      usage(optCon, EXIT_FAILURE, "verbosity setting larger than 15", number);
    }
  if (diagLimit >= 0 )
    {
      diagCount = diagFrom = -1; // -r overides -s, -t
    }
  if  ( verbose & 1 )
     {
	if ( diag != stderr )
	  fprintf(diag, "Diagnostic logging directed to %s\n", LOG_FILE);
        fprintf(diag, "Paper tape will be read from %s\n", ptrPath);
        fprintf(diag, "Paper tape will be punched to %s\n", punPath);
        fprintf(diag, "Teletype input will be read from %s\n", ttyInPath);
        fprintf(diag, "Plotter output will go to %s\n", plotPath);
	fprintf(diag, "Plotter paper width %d, height %d\n", plotterPaperWidth, plotterPaperHeight);
	fprintf(diag, "Plotter pen size %d steps\n", plotterPenSize);
        fprintf(diag, "Store image will be read from %s\n", storePath);
	fprintf(diag, "Execution will commence at address ");
	printAddr(diag, opKeys);
	fprintf(diag," (%d)\n", opKeys);
        if ( abandon >= 0 )
	  fprintf(diag, "Execution will be abandoned after %d instructions executed\n",
		    abandon);
	if ( diagCount >= 0 )
	  fprintf(diag, "Tracing will start after %d instructions executed\n", diagCount);
	if ( diagFrom >= 0 )
	  fprintf(diag, "Tracing will start from location %d onwards\n", diagFrom);
	if ( diagLimit >= 0 )
	  fprintf(diag, "Limited tracing will start after %d instructions executed\n",
		  diagLimit);
	if ( monLoc >= 0 )
	  {
	    fprintf(diag, "Location ");
	    printAddr(diag, monLoc);
	    fprintf(diag, " (%d) will be monitored\n", monLoc);
	  }
       }
}

void usage (poptContext optCon, INT32 exitcode, char *error, char *addl)
{
  poptPrintUsage(optCon, stderr, 0);
  if (error) fprintf(stderr, "%s: %s\n", error, addl);
  exit(exitcode);
}

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


//*** Main execution loop ***

  // instruction fetch and decode loop
  while ( ++iCount )
    {
// *** MJB update global regvals
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
/*              STORE DUMP AND RECOVERY                   */
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
/*                      DIAGNOSTICS                       */
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
