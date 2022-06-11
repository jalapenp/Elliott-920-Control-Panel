/*
 * 
 * name: GPIOLamp.c
 * @param
 * @return
 * @author Mick Bell
 * 
 * Simple GPIO Lamp On/Off command line program 
 *
 *  Parameters <led> the GPIO pin, <value> (HIGH (1) / LOW (0)as defined by bcm2835.h)
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

//++++++++++++++++++++++++++++++++++
//
// Defines
//
//++++++++++++++++++++++++++++++++++

// none

 
//++++++++++++++++++++++++++++++++++
//
// MAIN
//
//++++++++++++++++++++++++++++++++++   

int main(int argc, char **argv)
{

    uint8_t led;
    uint8_t onOff;
 
   
    // Parameters

    if (argc != 3) 
    {
	printf("Usage: GPIOLamp <LED> <On/Off> (True/False)\n");
	return (-1);
    }
 
    // First argument is executable name  
    printf("exe name=%s\n", argv[0]);
    printf("Version %s %s\n", __DATE__, __TIME__);
    
    //Simple positional parameters
    led = atoi(argv[1]);
    onOff = atoi(argv[2]);

    printf("Setting LED = 0x%X \n", led);
    printf("With On/Off = 0x%X\n", onOff);
    
    
    if (!bcm2835_init())
    {	
	printf("BMC2835 init failed\n");
	return -1 ;
    }

    bcm2835_gpio_fsel(led, BCM2835_GPIO_FSEL_OUTP);
    
    // set according to input
    bcm2835_gpio_write(led, onOff);
    
    // (and leave it at that)
    
    bcm2835_close();
    return 0;
}
	

 
