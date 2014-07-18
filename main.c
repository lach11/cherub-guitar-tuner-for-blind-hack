
/*
 * @author:		Quinton Pratt
 * @date:		5 May 2014
 * @script:		AVR Guitar Tuner Hack for Cherub WST-550G
 * @mcu:		ATMEGA328P with 4MHz external crystal & 22pF crystal caps
 * @compiler:	Eclipse with AVR Plugin - MacOSX
 * @version:	1.2
 * @email:		quinton.pratt@gmail.com
 * @url:		http://www.beyondthebreadboard.com
 *
 * ------------------------------------------------------------------------------------------------------------------------
 *
 * @about:		This is for an AVR based guitar tuner hack using a modified Cherub WST-550G guitar tuner for a
 * 				blind person. The project schematic and code was taken from http://lushprojects.com/guitartuner/.
 *
 * 				The project was initially written for the ATMEGA168 but given pin/register compatibility and that I only
 * 				had an ATMEGA328P on hand, I opted for the ATMEGA328P instead.
 *
 * 				In trying to implement the code provided by the original author, we had significant issues in trying to
 * 				get the code to work properly. The original code allowed only the startup tones to work as well as the
 * 				string select momentary switch. Nothing was actioned with LED read states i.e no tuning feedback tones
 * 				were generated.As a result, I have modified the code to get this working. Some functions have been retained from the
 * 				original code.
 *
 * 				Some of the functionality may differ from that purported by the original project. Basically
 * 				I have re-written it to play a higher pitch sound if too high, a lower pitch sound if too low, and a
 * 				mid pitch tone if tuned correctly. The tones repeat a set number of times depending on how close the
 * 				tuning is to being in tune. If the target note matches the current note read from the Cherub tuner,
 * 				then the tones repeat from 5 to 1 (+50% to +10% or -50% to -10%). The in tune tone has a distinct sound.
 * 				If the target note doesnt match the Cherub note LED read, then a tone sequence is played (7 repeats) to
 * 				indicate which direction to tune. If no LEDs are reading high, the response is nothing.
 *
 * 				The original code was not commented so I have provided comments throughout this version. In addition,
 * 				the power supply was changed to 2 x AAA batteries to allow smoother operation of the AVR rather than the
 * 				original 3V CR2032 button battery which was used to power both the Cherub and AVR circuits.
 *
 * 	------------------------------------------------------------------------------------------------------------------------
 *
 * @copyright:	No copyright details provided in the original code. Given it was made publicly available, the assumption
 * 				is that the code is released under the GNU General Public License, version 3 (GPL-3.0) or later. So this
 * 				copyright coverage is stipulated here.
 *
 * @disclaimer: THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE
 * 				STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM ÒAS ISÓ WITHOUT WARRANTY
 * 				OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * 				MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE
 * 				OF THE PROGRAM IS WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY
 * 				SERVICING, REPAIR OR CORRECTION.
 *
 * @liability:	IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL ANY COPYRIGHT HOLDER, OR ANY
 * 				OTHER PARTY WHO MODIFIES AND/OR CONVEYS THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES,
 * 				INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR INABILITY
 * 				TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES
 * 				SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER PROGRAMS), EVEN
 * 				IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *
 * @circuit description:
 * ------------------------------------------------------------------------------------------------------------------------`dzxxxxxxxxxxxxxxxxxxxxxxxxx
 *
 * ATMEGA328P (28DIP)
 * ---------------------
 * PC6 (1)		-> RESET
 * PD0 (2)		-> -50% LED
 * PD1 (3)		-> -40% LED
 * PD2 (4)		-> -30% LED
 * PD3 (5)		-> +50% LED
 * PD4 (6)		-> 5A LED
 * VCC (7)		-> +3V
 * GND (8)		-> GND
 * PB6 (9)		-> 4MHZ
 * PB7(10)		-> 4MHZ
 * PD5 (11)		-> 1E LED
 * PD6 (12)		-> 2B LED
 * PD7 (13)		-> 3G LED
 * PB0 (14) 	-> 4D LED
 * PB1 (15)		-> AUDIO OUT
 * PB2 (16)		-> 6E LED
 * PB3 (17)		-> STRING SELECT MOMENTARY SWITCH (grounded with internal pullup)
 * PB4 (18)		-> GREEN 0% LED
 * PB5 (19)		-> RED 0% LED
 * AVCC (20)	-> +3V
 * AREF (21)	-> NC
 * GND (22)		-> GND
 * PC0 (23)		-> -10% LED
 * PC1 (24)		-> -20% LED
 * PC2 (25)		-> +40% LED
 * PC3 (26)		-> +30% LED
 * PC4 (27)		-> +20% LED
 * PC5 (28)		-> +10% LED
 *
 * @changelog:
 * v 1.0		had note and scale led reads looking for a logic high and hardware soldered around cathode
 * 				accordingly. However, logic levels weren't changing as expected with the two circuits connected.
 * 				Resoldered to different legs of the LED leaving default read as high and activated led read
 * 				a proper logic low.
 *
 * v 1.1		to address the issues highlighted under v 1.0, all the logic read states have been changed in
 * 				this version to initiate a response when a led read goes from logic high (LED light off) to logic
 * 				low (LED light on).
 * v 1.2		reordered pin connections to cherub to better align with pcb layout.
 *
 */

/****************************************************************************************************************/
/* INCLUDES */
/****************************************************************************************************************/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

/****************************************************************************************************************/
/* DEFINES */
/****************************************************************************************************************/

//define from original code
#ifndef ABS
#define ABS(a) ((a) >= 0 ? (a) : -(a))
#endif

//define note led pins
#define E6			PB0
#define A5			PB2
#define D4			PD7
#define G3			PD6
#define B2			PD5
#define E1			PD4

//define scale LED pins
#define P50			PB5 //+50%
#define P40			PB4 //+40%
#define P30			PC5 //+30%
#define P20			PC4 //+20%
#define P10			PD0 //+10%
#define GREEN_ZERO	PC3 //GREEN 0%
#define RED_ZERO	PD1 //RED 0%
#define N10			PC2 //-10%
#define	N20			PD2 //-20%
#define N30			PC1 //-30%
#define N40			PD3 //-40%
#define N50			PC0 //-50%

//button and audio out
#define BTN_SEL		PB3 //NOTE SELECT MOMENTARY SWITCH
#define EARPHONE	PB1 //STEREO OUT

//led read definitions
#define E1_READ		(PIND & (1<<E1))==0 //READ 1E NOTE LED
#define B2_READ		(PIND & (1<<B2))==0 //READ 2B NOTE LED
#define G3_READ		(PIND & (1<<G3))==0 //READ 3G NOTE LED
#define D4_READ		(PIND & (1<<D4))==0 //READ 4D NOTE LED
#define A5_READ		(PINB & (1<<A5))==0 //READ 5A NOTE LED
#define E6_READ		(PINB & (1<<E6))==0 //READ 6E NOTE LED

#define P50_READ	(PINB & (1<<P50))==0 //READ +50% SCALE LED
#define P40_READ	(PINB & (1<<P40))==0 //READ +40% SCALE LED
#define P30_READ	(PINC & (1<<P30))==0 //READ +30% SCALE LED
#define P20_READ	(PINC & (1<<P20))==0 //READ +20% SCALE LED
#define P10_READ	(PIND & (1<<P10))==0 //READ +10% SCALE LED

#define G0_READ		(PINC & (1<<GREEN_ZERO))==0 //READ GREEN 0% SCALE LED
#define R0_READ		(PIND & (1<<RED_ZERO))==0 //READ RED 0% SCALE LED

#define N10_READ	(PINC & (1<<N10))==0 //READ -10% SCALE LED
#define N20_READ	(PIND & (1<<N20))==0 //READ -20% SCALE LED
#define N30_READ	(PINC & (1<<N30))==0 //READ -30% SCALE LED
#define N40_READ	(PIND & (1<<N40))==0 //READ -40% SCALE LED
#define N50_READ	(PINC & (1<<N50))==0 //READ -50% SCALE LED

//button state reads
#define BTN_READ_L	(PINB & (1<<BTN_SEL))==0 //READ NOTE SELECT LOGIC LOW STATE
#define BTN_READ_H	(PINB & (1<<BTN_SEL)) //READ NOTE SELECT LOGIC HIGH STATE

//other
#define MY_DELAY	100 //program delay

/****************************************************************************************************************/
/* VARIABLE DECLARATIONS */
/****************************************************************************************************************/

double notes[9] = {440.0 , 329.6, 261.6, 196.0, 146.8, 110.0, 587.3, 82.4, 220.0 }; // Note frequenices Hz - original

int onperiod[5] = {6,10,14,18,22}; // Beep on times (cs) - original array variable
int offperiod[5] = {3, 5, 7, 9, 11}; // Beep off times (cs) - original array variable

int8_t lastnote=0; //original array variable
int8_t targetnote=1; //original array variable - stores user selected note to be tuned
uint16_t scale,note; //original variables but scope changed from local in main in the original to global here.


/****************************************************************************************************************/
/* PIN INITIALISATION */
/****************************************************************************************************************/

/*****************************************************************************************************************
 * Function:		ioinit()
 * About:			Part original function. Initialises pins for LED reads as well as timer counter register for
 * 					pwm tones. Essentially, this amended function is the same as the original with the exception
 * 					that low bits are explicitly set for clarity and is based on defines for each pin-Cherub
 * 					connection as set out in the define section.
 * Parameters:		NIL
 * Returns:			NIL
 *****************************************************************************************************************/

void ioinit(void) {

	//initialise PORTD - D4, G3, B2, E1, P10, RED_ZERO, N40 as inputs
	//------------------------------------------------------------------------------------------------------------
	DDRD &= ~((1<<D4) | (1<<G3) | (1<<B2) | (1<<E1) | (1<<P10) | (1<<RED_ZERO) | (1<<N40));

	//initialise PORTC - P30, P20, GREEN_ZERO, N10, N30, N50 as inputs
	//------------------------------------------------------------------------------------------------------------
	DDRC &= ~((1<<P30) | (1<<P20) | (1<<GREEN_ZERO) | (1<<N10) | (1<<N30) | (1<<N50));

	//initialise PORTB - E6, A5, P50, P40, note select as inputs; audio as output
	//------------------------------------------------------------------------------------------------------------
	DDRB &= ~((1<<E6) | (1<<A5) | (1<<P50) | (1<<P40) | (1<<BTN_SEL));
	DDRB |= (1<<EARPHONE); //send earphone line as output
	PORTB |= (1<<BTN_SEL); //pullup on button

	//initialise timer/counter
	//------------------------------------------------------------------------------------------------------------
	TCCR1A=_BV(WGM11);
	TCCR1B=_BV(WGM13)|_BV(WGM12)|_BV(CS11);

} //end ioinit()


/****************************************************************************************************************/
/* TONE FORMING FUNCTIONS */
/****************************************************************************************************************/

/*****************************************************************************************************************
 * Function:		setftone()
 * About:			Original function. Activates PWM tone sequence. Taken from original code.
 * Parameters:		double
 * Returns:			NIL
 *****************************************************************************************************************/

void setftone(double f,uint8_t d) {
	int16_t	t;
	if (f==0)
		TCCR1A=_BV(WGM11);
	else {
		t=F_CPU/(8.0*f);
		ICR1H=t>>8;
		ICR1L=t&0xff;
		t=t>>d;
		OCR1AH=t>>8;
		OCR1AL=t&0xff;
		TCCR1A=_BV(COM1A1)|_BV(WGM11);
	} //end if
} //end setftone()


/*****************************************************************************************************************
 * Function:		setf()
 * About:			Original function. Just passes data to setftone() after type conversion of f and addition
 * 					of an extra parameter for the call.
 * Parameters:		double
 * Returns:			NIL
 *****************************************************************************************************************/

void setf(double f) {
	setftone(f,1);
} //end setf()


/****************************************************************************************************************/
/* TIMER FUNCTIONS */
/****************************************************************************************************************/

/*****************************************************************************************************************
 * Function:		delaycentiseconds()
 * About:			Defines larger time periods. Taken from original code.
 * Parameters:		uint8_t, specifies number of for loop repeats
 * Returns:			NIL
 *****************************************************************************************************************/

void delaycentiseconds(int n){
	uint16_t i;
	for(i=0;i<n;i++)
		_delay_ms(10);
} //end delaycentiseconds()


/*****************************************************************************************************************
 * Function:		delaydecseconds()
 * About:			Defines larger time periods. Taken from original code.
 * Parameters:		uint8_t, specifies number of for loop repeats
 * Returns:			NIL
 *****************************************************************************************************************/

void delaydecseconds(int n){
	uint16_t i;
	for(i=0;i<10*n;i++)
		_delay_ms(10);
} //end delaydecseconds()


/****************************************************************************************************************/
/* TONE PATTERN FUNCTIONS */
/****************************************************************************************************************/

/*****************************************************************************************************************
 * Function:		playscale()
 * About:			Original function. Used here to just play the startup sequence when the AVR is powered.
 * Parameters:		n, int8_t, determines number of loops to perform.
 * Returns:			NIL
 *****************************************************************************************************************/

void playscale(int8_t n) {

	int8_t i;
	setf(0);
	delaycentiseconds(10);

	for(i=0;i<n;i++) {
		setf(notes[i]);
		delaycentiseconds(20);
		setf(0);
		delaycentiseconds(1);
	} //end for

	delaycentiseconds(10);

} //end playscale()


/*****************************************************************************************************************
 * Function:		play_too_high()
 * About:			Added function. Plays a sequence to signify the note being tuned is too high.
 * Parameters:		n, int8_t, determines number of loops to perform.
 * Returns:			NIL
 *****************************************************************************************************************/

void play_too_high(int8_t n) {

	int8_t i;
	setf(0);
	delaycentiseconds(10);

	for(i=0;i<n;i++) {
		setf(notes[0]);
		delaycentiseconds(20);
		setf(0);
		delaycentiseconds(1);
	} //end for

	delaycentiseconds(10);

} //end playhigh()


/*****************************************************************************************************************
 * Function:		play_too_low()
 * About:			Added function. Plays a sequence to signify the note being tuned is too low.
 * Parameters:		n, int8_t, determines number of loops to perform.
 * Returns:			NIL
 *****************************************************************************************************************/

void play_too_low(int8_t n) {

	int8_t i;
	setf(0);
	delaycentiseconds(10);

	for(i=0;i<n;i++) {
		setf(notes[5]);
		delaycentiseconds(20);
		setf(0);
		delaycentiseconds(1);
	} //end for

	delaycentiseconds(10);

} //end playhigh()


/*****************************************************************************************************************
 * Function:		play_in_tune()
 * About:			Added function. Plays a sequence to signify the note being tuned is in tune.
 * Parameters:		n, int8_t, determines number of loops to perform.
 * Returns:			NIL
 *****************************************************************************************************************/

void play_in_tune(int8_t n) {

	int8_t i;
	setf(0);
	delaycentiseconds(10);

	for(i=0;i<n;i++) {
		setf(notes[3]);
		delaycentiseconds(20);
		setf(0);
		delaycentiseconds(0);
	} //end for

	delaycentiseconds(10);

} //end playhigh()


/****************************************************************************************************************/
/* TUNING PROCESSING FUNCTIONS */
/****************************************************************************************************************/

/*****************************************************************************************************************
 * Function:		process_guitar_playing()
 * About:			This is the core function that determines what audible feedback to provide to the user to let
 * 					them know whether they are in tune, or to tune up or down. It supercedes the original function
 * 					- processnoteplaying() which didn't register any input state reads from LEDs and/or
 * 					didnt trigger the outcome. Either way, this function was written to perform that function
 * 					instead and is an addition to the program.
 * Parameters:		NIL
 * Returns:			NIL
 *****************************************************************************************************************/

void process_guitar_tuning(void) {

	uint8_t n, which_note;
	n=0; //zero variable at start so that default return is zero.

	if (E1_READ) n=2; //1E LED HIGH
	if (B2_READ) n=3; //2B LED HIGH
	if (G3_READ) n=4; //3G LED HIGH
	if (D4_READ) n=5; //4D LED HIGH
	if (A5_READ) n=6; //5A LED HIGH
	if (E6_READ) n=7; //6E LED HIGH

	_delay_ms(MY_DELAY);

	if(n>0) { //if getting a note LED read from the cherub tuner

		which_note = targetnote;
		if(n==which_note) { //if the user selected target note matches the cherub detected note

			if (P50_READ) play_too_high(5); //+50% LED HIGH
			if (P40_READ) play_too_high(4); //+40% LED HIGH
			if (P30_READ) play_too_high(3); //+30% LED HIGH
			if (P20_READ) play_too_high(2); //+20% LED HIGH
			if (P10_READ) play_too_high(1); //+10% LED HIGH

			/* ZEROS */
			//if (R0_READ) s=7; //red 0% LED HIGH -> not in tune
			if (G0_READ) play_in_tune(5); //green 0% LED HIGH -> in tune

			/* TOO LOW */
			if (N10_READ) play_too_low(1); //-10% LED HIGH
			if (N20_READ) play_too_low(2); //-20% LED HIGH
			if (N30_READ) play_too_low(3); //-30% LED HIGH
			if (N40_READ) play_too_low(4); //-40% LED HIGH
			if (N50_READ) play_too_low(5); //-50% LED HIGH

		} else { //user selected note and cherub detected note don't match
			//if the note LED is lower than the target note -> too low -> play too high tune
			if(n<which_note) play_too_high(7);

			//if the note LED is higher than the target note -> too high -> play too low tune
			if(n>which_note) play_too_low(7);
		} //end if

	} //end if
} //end process_guitar_tuning()


/****************************************************************************************************************/
/* MAIN FUNCTION */
/****************************************************************************************************************/

/*****************************************************************************************************************
 * Function:		main()
 * About:			Added function. Reads state of Cherub note LEDs and returns 8 bit integer corresponding to
 * 					note LED reading high, or 0 otherwise.
 * Parameters:		NIL
 * Returns:			int, standard main return
 *****************************************************************************************************************/

int main (void) {

	/* main variables */
	//-------------------------------------------------------------------------------------------------------------
	ioinit(); //initialise pins and pwm
	int8_t state=0; //original variable. Enables active start after first string select action.
	int8_t waitingforkey=1; //original variable. store state for waiting for string select input
	int8_t i; //original variable. simple counter variable.

	/* play startup tones - plays a scale up and down. Retained from original code. */
	//-------------------------------------------------------------------------------------------------------------
	for(i=0;i<6;i++) {
		setf(notes[i]);
		delaycentiseconds(20);
		setf(0);
		delaycentiseconds(1);
	} //end for

	for(i=1;i<6;i++) {
		setf(notes[5-i]);
		delaycentiseconds(20);
		setf(0);
		delaycentiseconds(1);
	} //end for

	delaycentiseconds(15);

	/* set target note at startup */
	//-------------------------------------------------------------------------------------------------------------
	targetnote++; //target note 1 causing noise and the increment seemed to solve the issue for whatever reason.

	/* main loop */
	//-------------------------------------------------------------------------------------------------------------
	while(1) {
		if (waitingforkey) { //set state for waiting for user input at string select
			if (BTN_READ_L) { //if string select button pressed
				targetnote=targetnote+1; //increment target note
				_delay_ms(MY_DELAY);
				if (targetnote>7) //reset target note when exceeds note range
					targetnote=2;
				playscale(targetnote-1); //play tone sequence of target note - basically moderates tone repeats
				waitingforkey=0;
				state=0;
			} //end if
		} else {
			if (BTN_READ_H) //string select not pressed. High as internal pullup and switch grounds the pin.
				waitingforkey=1;
		} //end if

		//if active state set then process tuning LED states for both note and scale and output tone sequence.
		switch (state) {
			case 0:
				state=0;
				process_guitar_tuning();
				break;
			default:break;
		} //end switch

	} //end while

    return (0); //standard main return

} //end main
