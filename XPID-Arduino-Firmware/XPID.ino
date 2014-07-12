/*
	X-Sim PID - Moto Monster Edition Firmware
	Copyright (c) 2014 Martin Wiedenbauer, particial use is only allowed with a reference link to the x-sim.de project

	This firmware comes with a X-SIM GUI (Graphically User Interface = X-Sim Plugin) interface and easy setup interface to play around 
	with PID values. It is designed for	users with less knowlege about programming and that want to simple upload and use the firmware. 
	After loading this firmware to the arduino, the firmware is detected in the interface setup automatically on any USB port.
	Do not forget to clean the X-Sim Converter USO setup. If you have loaded another 3rd party firmware before, you might block the comport.
	Same for the X-Sim Extractor OBD2 setup, remove the Moto Monster arduino comport from the list.
	It comes with many additionally	features like brake, 360°, GUI for pot scaling, power disable options etc. to get all the X-Sim features.
	This program will control the Moto Monster Shield of Sparksfun, with a analogue pot feedback, compared to a serial target input value from X-Sim
	You can get this H-Bridge easy and cheap from eBay.
	Target is a Arduino UNO R3 but will work on all Arduino with Atmel 328.
	This means, Arduino nano, micro, Duemilanove and UNO are tested. The moto monster shield does mechanically only be plug'n'play with UNO and Duemilanove, else you must wire it.
	Arduinos with an FTDI serial chip need a change to lower baudrates of 9600 bit/s.
	Read the below code and comment or uncomment the needed baudrate option with a double slash (//).

	Help for wiring the Moto Monster Shield

	Motor Pins (on Moto Monster Shield)

	JP3 VCC - 12V from power supply
	JP3 GND - GND from power supply
	JP1 OUTA1 - connect to Motor A
	JP1 OUTB1 - connect to Motor A
	JP2 OUTA2 - connect to Motor B
	JP2 OUTB2 - connect to Motor B

	You may switch the two cables to the motor if you see that the feedback is not working (motor move in wrong direction).
	Instead of switch the motor cable you can of course switch the +5V and GND cable of the pot which do the same.
	If the motor control is working but you must change the direction, you have to simple switch pot and motor cables.

	Analog Pins, must be connected not to the Moto Monster Shield but to a feedback pot connected to the motor axis

	Pin A4 - input of feedback positioning from motor A, wire here the feedback pot middle pin
	Pin A5 - input of feedback positioning from motor B, wire here the feedback pot middle pin
	         Other pot pins go to +5V and GND of arduino.

	Important:
	Do not wire a motor monster GND power supply pin with GND of arduino or the pot!
	Use a heatsink on the VNH2SP30 chips on the motor monster shield. Maybe a fan is additionally attached.
	Start with a low PWM frequency setup in the X-Sim GUI. The PWM setup is only for reducing hearable switching noise.


	Pin out setup of arduino for Moto Monster Shield of SparksFun (needed only for programming and understanding code)

	Pin  5 - PWMA - Speed for Motor 1. 
	Pin  6 - PWMB - Speed for Motor 2.
	Pin  4 - INA1 - motor 2 turn 
	Pin  7 - INA2 - motor 1 turn
	Pin  8 - INB1 - motor 1 turn
	Pin  9 - INB2 - motor 2 turn
	Pin A0 - ENA  - current status input (not used)
	Pin A1 - ENB  - current status input (not used)
	Pin A2 - CSA  - power sense analogue input for over current detection (not used)
	Pin A3 - CSB  -	power sense analogue input for over current detection (not used)

	As well 5v and GND pins tapped in to feed feedback pots too.



	Command input protocol	(always 5 bytes, beginning with 'X' character and ends with a XOR checksum)
	'X' 1 H L C				Set motor 1 position to High and Low value 0 to 1023
	'X' 2 H L C				Set motor 2 position to High and Low value 0 to 1023
	'X' 3 H L C				Set motor 1 P Proportional value to High and Low value
	'X' 4 H L C				Set motor 2 P Proportional value to High and Low value
	'X' 5 H L C				Set motor 1 I Integral value to High and Low value
	'X' 6 H L C				Set motor 2 I Integral value to High and Low value
	'X' 7 H L C				Set motor 1 D Derivative value to High and Low value
	'X' 8 H L C				Set motor 2 D Derivative value to High and Low value

	'X' 200 0 0 C			Send back over serial port both analogue feedback raw values
	'X' 201 0 0 C			Send back over serial port the current pid count
	'X' 202 0 0 C			Send back over serial port the firmware version (used for x-sim autodetection)
	'X' 203 M V C			Write EEPROM on address M (only 0 to 255 of 1024 Bytes of the EEPROM) with new value V
	'X' 204 M 0 C			Read EEPROM on memory address M (only 0 to 255 of 1024 Bytes of the EEPROM), send back over serial the value
	'X' 205 0 0 C			Clear EEPROM
	'X' 206 0 0 C			Reread the whole EEPRom and store settings into fitting variables
	'X' 207 0 0	C			Disable power on motor 1
	'X' 208 0 0	C			Disable power on motor 2
	'X' 209 0 0	C			Enable power on motor 1
	'X' 210 0 0	C			Enable power on motor 2
	'X' 211 0 0	C			Send all debug values
	
	EEPROM memory map
	00		empty eeprom detection, 111 if set, all other are indicator to set default
	01-02	minimum 1
	03-04	maximum 1
	05		dead zone 1
	06-07	minimum 2
	08-09	maximum 2
	10		dead zone 2
	11-12	P component of motor 1
	13-14	I component of motor 1
	15-16	D component of motor 1
	17-18	P component of motor 2
	19-20	I component of motor 2
	21-22	D component of motor 2
	23		pwm1 offset
	24		pwm2 offset
	25		pwm1 maximum
	26		pwm2 maximum
	27		pwm frequency divider (1,8,64)

*/

#include <EEPROM.h>

//Some speed test switches for testers ;)
#define FASTADC  1 //Hack to speed up the arduino analogue read function, comment out with // to disable this hack

// defines for setting and clearing register bits
#ifndef cbi
	#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
	#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define LOWBYTE(v)   ((unsigned char) (v))								//Read
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))
#define BYTELOW(v)   (*(((unsigned char *) (&v) + 1)))					//Write
#define BYTEHIGH(v)  (*((unsigned char *) (&v)))

#define   GUARD_MOTOR_1_GAIN   100.0     
#define   GUARD_MOTOR_2_GAIN   100.0

//Firmware version info
int firmaware_version_mayor=2;
int firmware_version_minor =0;

//360° option for flight simulators
bool turn360motor1 = false;
bool turn360motor2 = false;

int virtualtarget1;
int virtualtarget2;
int currentanalogue1 = 0;
int currentanalogue2 = 0;
int target1=512;
int target2=512;
int low=0;
int high=0;
unsigned long hhigh=0;
unsigned long hlow=0;
unsigned long lhigh=0;
unsigned long llow=0;
int buffer=0;
int buffercount=-1;
int commandbuffer[5]={0};
unsigned long pidcount	= 0;		// unsigned 32bit, 0 to 4,294,967,295
byte errorcount	= 0;		// serial receive error detected by checksum

//Atmel chip port number for direct port manipulation
//http://arduino.cc/en/Hacking/PinMapping
#define ATMELA 0
#define ATMELB 1
#define ATMELC 2
#define ATMELD 3
#define ATMELE 4
#define ATMELF 5
//Structure for direct port manipulation data
struct mp
{
	int port;			//Atmel port, not Arduino
	int *portstatus;	//Pointer to current register value
	int pinnumber;		//Pinnumber of port at Atmel chip, not the arduino digital port!
	int arduino_pin;	//For I/O setup with arduino framework
};

// fixed DATA for direct port manipulation, exchange here each value if your h-Bridge is connected to another port pin
// This pinning overview is to avoid the slow pin switching of the arduino libraries
// 
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
// 
int portdstatus				=PORTD; 	// read the current port D bit mask
int portbstatus				=PORTB; 	// read the current port B bit mask
//Fill the pin swaping for direct port manipulation
//http://arduino.cc/en/Hacking/PinMapping
//Warning: using arduinos digitawrite() instead of using port manipulation will decrease the PID update rate and will heat up the H-BRIDGE because directions are both enabled
//Following structure: Atmel chip port name, pointer to portstatus variable for store old values, Atmel portpin, Arduino digital portpin for I/O setup
mp ControlPinM1Inp1	={ATMELD,&portdstatus,7,7}; // motor A INP1 output, this is the atmel chip pin description, PortD7, InA1 Motor1 clockwise, Arduino Pin 7
mp ControlPinM1Inp2	={ATMELB,&portbstatus,0,8}; // motor A INP2 output, this is the atmel chip pin description, PortB0, InB1 Motor1 counterclockwise, Arduino Pin 8
mp ControlPinM2Inp1	={ATMELD,&portdstatus,4,4}; // motor B INP1 output, this is the atmel chip pin description, PortD4, InA2 Motor2 clockwise, Arduino Pin 4
mp ControlPinM2Inp2 ={ATMELB,&portbstatus,1,9}; // motor B INP2 output, this is the atmel chip pin description, PortB1, InB2 Motor2 counterclockwise, Arduino Pin 9
int PWMPinM1				=5;			// motor A PWM output
int PWMPinM2				=6;			// motor B PWM output

// Pot feedback inputs
int FeedbackPin1			= A4;		// select the input pin for the potentiometer 1, port PC4 on atmel chip
int FeedbackPin2			= A5;		// select the input pin for the potentiometer 2, port PC5 on atmel chip
int FeedbackMax1			= 1021;		// Maximum position of pot 1 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin1			= 2;		// Minimum position of pot 1 to scale, do not use 0 because it cannot control outside the pot range
int FeedbackMax2			= 1021;		// Maximum position of pot 2 to scale, do not use 1023 because it cannot control outside the pot range
int FeedbackMin2			= 2;		// Minimum position of pot 2 to scale, do not use 0 because it cannot control outside the pot range
int FeedbackPotDeadZone1	= 0;		// +/- of this value will not move the motor		
int FeedbackPotDeadZone2	= 0;		// +/- of this value will not move the motor
// 360° Case detection border values
float quarter1				= 254.75;
float quarter2				= 254.75;
float threequarter1			= 764.25;
float threequarter2			= 764.25;

//PID variables
int motordirection1		= 0;			// motor A move direction 0=brake, 1=forward, 2=reverse
int motordirection2		= 0;			// motor B move direction 0=brake, 1=forward, 2=reverse
int oldmotordirection1	= 0;
int oldmotordirection2	= 0;
double K_motor_1		= 1;
double proportional1	= 4.200;		//initial value
double integral1		= 0.400;
double derivative1		= 0.400;
double K_motor_2		= 1;
double proportional2	= 4.200;
double integral2		= 0.400;
double derivative2		= 0.400;
int OutputM1					= 0;
int OutputM2					= 0;
double integrated_motor_1_error = 0;
double integrated_motor_2_error = 0;
float last_motor_1_error		= 0;
float last_motor_2_error		= 0; 
int disable						= 1; //Motor stop flag
int pwm1offset					= 50;
int pwm2offset					= 50;
int pwm1maximum					= 255;
int pwm2maximum					= 255;
float pwm1divider				= 0.8039;
float pwm2divider				= 0.8039;
float pwmfloat					= 0;
int pwmfrequencydivider			= 1; //31kHz

byte debugbyte =0;				//This values are for debug purpose and can be send via
int debuginteger =0;			//the SendDebug serial 211 command to the X-Sim plugin
double debugdouble =0;	

void setPwmFrequency(int pin, int divisor) 
{
	byte mode;
	if(pin == 5 || pin == 6 || pin == 9 || pin == 10) 
	{
		switch(divisor) 
		{
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 64: mode = 0x03; break;
			case 256: mode = 0x04; break;
			case 1024: mode = 0x05; break;
			default: return;
		}
		if(pin == 5 || pin == 6) 
		{
			TCCR0B = TCCR0B & 0b11111000 | mode;
		} 
		else 
		{
			TCCR1B = TCCR1B & 0b11111000 | mode;
		}
	} 
	else 
	{
		if(pin == 3 || pin == 11) 
		{
			switch(divisor) 
			{
				case 1: mode = 0x01; break;
				case 8: mode = 0x02; break;
				case 32: mode = 0x03; break;
				case 64: mode = 0x04; break;
				case 128: mode = 0x05; break;
				case 256: mode = 0x06; break;
				case 1024: mode = 0x7; break;
				default: return;
			}
			TCCR2B = TCCR2B & 0b11111000 | mode;
		}
	}
}

void setup()
{
	//Serial.begin(115200);   //Uncomment this for arduino UNO without ftdi serial chip
	Serial.begin(9600);  //Uncomment this for arduino nano, arduino with ftdi chip or arduino duemilanove
	portdstatus=PORTD;
	portbstatus=PORTB;
	pinMode(ControlPinM1Inp1.arduino_pin, OUTPUT);
	pinMode(ControlPinM1Inp2.arduino_pin, OUTPUT);
	pinMode(ControlPinM2Inp1.arduino_pin, OUTPUT);
	pinMode(ControlPinM2Inp2.arduino_pin, OUTPUT);
	pinMode(PWMPinM1,		  OUTPUT);
	pinMode(PWMPinM2,		  OUTPUT);
	analogWrite(PWMPinM1,	  0);
	analogWrite(PWMPinM2,	  0);
	UnsetMotor1Inp1();
	UnsetMotor1Inp2();
	UnsetMotor2Inp1();
	UnsetMotor2Inp2();
	disable=1;
	//TCCR1B = TCCR1B & 0b11111100; //This is a hack for changing the PWM frequency to a higher value, if removed it is 490Hz
	setPwmFrequency(PWMPinM1, 1);
	setPwmFrequency(PWMPinM2, 1);
#if FASTADC
	// set analogue prescale to 16
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
#endif
}

void WriteEEPRomWord(int address, int intvalue)
{
	int low,high;
	high=intvalue/256;
	low=intvalue-(256*high);
	EEPROM.write(address,high);
	EEPROM.write(address+1,low);
}

int ReadEEPRomWord(int address)
{
	int low,high, returnvalue;
	high=EEPROM.read(address);
	low=EEPROM.read(address+1);
	returnvalue=(high*256)+low;
	return returnvalue;
}

void WriteEEProm()
{
	EEPROM.write(0,111);
	WriteEEPRomWord(1,FeedbackMin1);
	WriteEEPRomWord(3,FeedbackMax1);
	EEPROM.write(5,FeedbackPotDeadZone1);
	WriteEEPRomWord(6,FeedbackMin2);
	WriteEEPRomWord(8,FeedbackMax2);
	EEPROM.write(10,FeedbackPotDeadZone2);
	WriteEEPRomWord(11,int(proportional1*10.000));
	WriteEEPRomWord(13,int(integral1*10.000));
	WriteEEPRomWord(15,int(derivative1*10.000));
	WriteEEPRomWord(17,int(proportional2*10.000));
	WriteEEPRomWord(19,int(integral2*10.000));
	WriteEEPRomWord(21,int(derivative2*10.000));
	if(pwm1offset > 180 || pwm2offset > 180 || pwm1maximum < 200 || pwm2maximum < 200)
	{
		pwm1offset=50;
		pwm2offset=50;
		pwm1maximum=255;
		pwm2maximum=255;
		pwm1divider=0.8039;
		pwm2divider=0.8039;
	}
	EEPROM.write(23,pwm1offset);
	EEPROM.write(24,pwm2offset);
	EEPROM.write(25,pwm1maximum);
	EEPROM.write(26,pwm2maximum);
	if(pwmfrequencydivider != 1 && pwmfrequencydivider != 8)
	{
		pwmfrequencydivider=1;
	}
	EEPROM.write(27,pwmfrequencydivider);
}

void ReadEEProm()
{
	int evalue = EEPROM.read(0);
	if(evalue != 111) //EEProm was not set before, set default values
	{
		WriteEEProm();
		return;
	}
	FeedbackMin1=ReadEEPRomWord(1);
	FeedbackMax1=ReadEEPRomWord(3);
	FeedbackPotDeadZone1=EEPROM.read(5);
	FeedbackMin2=ReadEEPRomWord(6);
	FeedbackMax2=ReadEEPRomWord(8);
	FeedbackPotDeadZone2=EEPROM.read(10);
	proportional1=double(ReadEEPRomWord(11))/10.000;
	integral1=double(ReadEEPRomWord(13))/10.000;
	derivative1=double(ReadEEPRomWord(15))/10.000;
	proportional2=double(ReadEEPRomWord(17))/10.000;
	integral2=double(ReadEEPRomWord(19))/10.000;
	derivative2=double(ReadEEPRomWord(21))/10.000;
	pwm1offset=EEPROM.read(23);
	pwm2offset=EEPROM.read(24);
	pwm1maximum=EEPROM.read(25);
	pwm2maximum=EEPROM.read(26);
	if(pwm1offset > 180 || pwm2offset > 180 || pwm1maximum < 200 || pwm2maximum < 200)
	{
		pwm1offset=50;
		pwm2offset=50;
		pwm1maximum=255;
		pwm2maximum=255;
		pwm1divider=0.8039;
		pwm2divider=0.8039;
		EEPROM.write(23,pwm1offset);
		EEPROM.write(24,pwm2offset);
		EEPROM.write(25,pwm1maximum);
		EEPROM.write(26,pwm2maximum);
	}
	else
	{
		pwmfloat=float(pwm1maximum-pwm1offset);
		pwm1divider=pwmfloat/255.000;
		pwmfloat=float(pwm2maximum-pwm2offset);
		pwm2divider=pwmfloat/255.000;
	}
	pwmfrequencydivider=EEPROM.read(27);
	if(pwmfrequencydivider != 1 && pwmfrequencydivider != 8)
	{
		pwmfrequencydivider=1;
		EEPROM.write(27,pwmfrequencydivider);
	}
	quarter1=float(FeedbackMax1-FeedbackMin1)/4.000;
	quarter2=float(FeedbackMax2-FeedbackMin2)/4.000;
	threequarter1=quarter1*3.000;
	threequarter2=quarter1*3.000;
	setPwmFrequency(5, pwmfrequencydivider);
	setPwmFrequency(6, pwmfrequencydivider);
}

void SendAnalogueFeedback(int analogue1, int analogue2)
{
	high=analogue1/256;
	low=analogue1-(high*256);
	Serial.write('X');
	Serial.write(200);
	Serial.write(high);
	Serial.write(low);
	high=analogue2/256;
	low=analogue2-(high*256);
	Serial.write(high);
	Serial.write(low);
}

void SendPidCount()
{
	unsigned long value=pidcount;
	hhigh=value/16777216;
	value=value-(hhigh*16777216);
	hlow=value/65536;
	value=value-(hlow*65536);
	lhigh=value/256;
	llow=value-(lhigh*256);
	Serial.write('X');
	Serial.write(201);
	Serial.write(int(hhigh));
	Serial.write(int(hlow));
	Serial.write(int(lhigh));
	Serial.write(int(llow));
	Serial.write(errorcount);
}

void SendDebugValues()
{
	//The double is transformed into a integer * 10 !!!
	int doubletransfere=int(double(debugdouble*10.000));
	Serial.write('X');
	Serial.write(211);
	Serial.write(debugbyte);
	Serial.write(HIGHBYTE(debuginteger));
	Serial.write(LOWBYTE(debuginteger));
	Serial.write(HIGHBYTE(doubletransfere));
	Serial.write(LOWBYTE(doubletransfere));
}

void SendFirmwareVersion()
{
	Serial.write('X');
	Serial.write('-');
	Serial.write('P');
	Serial.write('I');
	Serial.write('D');
	Serial.write(' ');
	Serial.write(48+firmaware_version_mayor);
	Serial.write('.');
	Serial.write(48+firmware_version_minor);
}

void EEPromToSerial(int eeprom_address)
{
	int retvalue=EEPROM.read(eeprom_address);
	Serial.write('X');
	Serial.write(204);
	Serial.write(retvalue);
}

void ClearEEProm()
{
	for(int z=0; z < 1024; z++)
	{
		EEPROM.write(z,255);
	}
}

void ParseCommand()
{
	if(commandbuffer[0]==1)			//Set motor 1 position to High and Low value 0 to 1023
	{
		target1=(commandbuffer[1]*256)+commandbuffer[2];
		disable=0;
		return;
	}
	if(commandbuffer[0]==2)			//Set motor 2 position to High and Low value 0 to 1023
	{
		target2=(commandbuffer[1]*256)+commandbuffer[2];
		disable=0;
		return;
	}

	if(commandbuffer[0]==200)		//Send both analogue feedback raw values
	{
		SendAnalogueFeedback(currentanalogue1, currentanalogue2);
		return;
	}
	if(commandbuffer[0]==201)		//Send PID count
	{
		SendPidCount();
		return;
	}
	if(commandbuffer[0]==202)		//Send Firmware Version
	{
		SendFirmwareVersion();
		return;
	}
	if(commandbuffer[0]==203)		//Write EEPROM
	{
		EEPROM.write(commandbuffer[1],uint8_t(commandbuffer[2]));
		return;
	}
	if(commandbuffer[0]==204)		//Read EEPROM
	{
		EEPromToSerial(commandbuffer[1]);
		return;
	}
	if(commandbuffer[0]==205)		//Clear EEPROM
	{
		ClearEEProm();
		return;
	}
	if(commandbuffer[0]==206)		//Reread the whole EEPRom and store settings into fitting variables
	{
		ReadEEProm();
		return;
	}
	if(commandbuffer[0]==207 || commandbuffer[0]==208)		//Disable power on both motor
	{
		analogWrite(PWMPinM1,	  0);
		UnsetMotor1Inp1();
		UnsetMotor1Inp2();
		analogWrite(PWMPinM2,	  0);
		UnsetMotor2Inp1();
		UnsetMotor2Inp2();
		disable=1;
		return;
	}
	if(commandbuffer[0]==209 || commandbuffer[0]==210)		//Enable power on both motor
	{
		analogWrite(PWMPinM1,	  128);
		UnsetMotor1Inp1();
		UnsetMotor1Inp2();
		analogWrite(PWMPinM2,	  128);
		UnsetMotor2Inp1();
		UnsetMotor2Inp2();
		disable=0;
		return;
	}
	if(commandbuffer[0]==211)		//Send all debug values
	{
		SendDebugValues();
		return;
	}
}

void FeedbackPotWorker()
{
	currentanalogue1 = analogRead(FeedbackPin1);
	currentanalogue2 = analogRead(FeedbackPin2);
	//Notice: Minimum and maximum scaling calculation is done in the PC plugin with faster float support
}

bool CheckChecksum() //Atmel chips have a comport error rate of 2%, so we need here a checksum
{
	byte checksum=0;
	for(int z=0; z < 3; z++)
	{
		byte val=commandbuffer[z];
		checksum ^= val;
	}
	if(checksum==commandbuffer[3]){return true;}
	return false;
}

void SerialWorker()
{
	while(Serial.available()) 
	{
		if(buffercount==-1)
		{
			buffer = Serial.read();
			if(buffer != 'X'){buffercount=-1;}else{buffercount=0;}
		}
		else
		{
			buffer = Serial.read();
			commandbuffer[buffercount]=buffer;
			buffercount++;
			if(buffercount > 3)
			{
				if(CheckChecksum()==true){ParseCommand();}else{errorcount++;}
				buffercount=-1;
			}
		}
	}
}

void CalculateVirtualTarget()
{
	if(turn360motor1==true)
	{
		virtualtarget1=target1;
		if(currentanalogue1 > int(threequarter1) && target1 < int(quarter1)){virtualtarget1+=FeedbackMax1;}
		else{if(currentanalogue1 < int(quarter1) && target1 > int(threequarter1)){virtualtarget1=0-FeedbackMax1-target1;}}
	}
	else
	{
		virtualtarget1=target1;
	}
	if(turn360motor2==true)
	{
		virtualtarget2=target2;
		if(currentanalogue2 > int(threequarter2) && target2 < int(quarter2)){virtualtarget2+=FeedbackMax2;}
		else{if(currentanalogue2 < int(quarter2) && target2 > int(threequarter2)){virtualtarget2=0-FeedbackMax2-target2;}}
	}
	else
	{
		virtualtarget2=target2;
	}
}

void CalculateMotorDirection()
{
	if(virtualtarget1 > (currentanalogue1 + FeedbackPotDeadZone1) || virtualtarget1 < (currentanalogue1 - FeedbackPotDeadZone1))
	{
		if (OutputM1 >= 0)  
		{                                    
			motordirection1=1;				// drive motor 1 forward
		}  
		else 
		{                                              
			motordirection1=2;				// drive motor 1 backward
			OutputM1 = abs(OutputM1);
		}
	}
	else
	{
		motordirection1=0;
	}

	if(virtualtarget2 > (currentanalogue2 + FeedbackPotDeadZone2) || virtualtarget2 < (currentanalogue2 - FeedbackPotDeadZone2))
	{
		if (OutputM2 >= 0)  
		{                                    
			motordirection2=1;				// drive motor 2 forward
		}  
		else 
		{                                              
			motordirection2=2;				// drive motor 2 backward
			OutputM2 = abs(OutputM2);
		}
	}
	else
	{
		motordirection2=0;
	}

	OutputM1 = constrain(OutputM1, -255, 255);
	OutputM2 = constrain(OutputM2, -255, 255);
}

int updateMotor1Pid(int targetPosition, int currentPosition)   
{
	float error = (float)targetPosition - (float)currentPosition; 
	float pTerm_motor_R = proportional1 * error;
	integrated_motor_1_error += error;                                       
	float iTerm_motor_R = integral1 * constrain(integrated_motor_1_error, -GUARD_MOTOR_1_GAIN, GUARD_MOTOR_1_GAIN);
	float dTerm_motor_R = derivative1 * (error - last_motor_1_error);                            
	last_motor_1_error = error;
	return constrain(K_motor_1*(pTerm_motor_R + iTerm_motor_R + dTerm_motor_R), -255, 255);
}

int updateMotor2Pid(int targetPosition, int currentPosition)   
{
	float error = (float)targetPosition - (float)currentPosition; 
	float pTerm_motor_L = proportional2 * error;
	integrated_motor_2_error += error;                                       
	float iTerm_motor_L = integral2 * constrain(integrated_motor_2_error, -GUARD_MOTOR_2_GAIN, GUARD_MOTOR_2_GAIN);
	float dTerm_motor_L = derivative2 * (error - last_motor_2_error);                            
	last_motor_2_error = error;

	return constrain(K_motor_2*(pTerm_motor_L + iTerm_motor_L + dTerm_motor_L), -255, 255);
}

void CalculatePID()
{
	OutputM1=updateMotor1Pid(virtualtarget1,currentanalogue1);
	OutputM2=updateMotor2Pid(virtualtarget2,currentanalogue2);
}

void SetPWM()
{
	//Calculate pwm offset and maximum
	pwmfloat=OutputM1;
	pwmfloat*=pwm1divider;
	pwmfloat+=float(pwm1offset);
	OutputM1=pwmfloat;
	if(OutputM1 > pwm1maximum){OutputM1=pwm1maximum;}
	pwmfloat=OutputM2;
	pwmfloat*=pwm2divider;
	pwmfloat+=float(pwm2offset);
	OutputM2=pwmfloat;
	if(OutputM2 > pwm2maximum){OutputM2=pwm2maximum;}

	//Set hardware pwm
	if(motordirection1 != 0)
	{
		analogWrite(PWMPinM1, int(OutputM1));
	}
	else
	{
		analogWrite(PWMPinM1, 0);
	}
	if(motordirection2 != 0)
	{
		analogWrite(PWMPinM2, int(OutputM2));
	}
	else
	{
		analogWrite(PWMPinM2, 0);
	}
}

//Direct port manipulation, change here your port code

void SetMotor1Inp1()
{
	*ControlPinM1Inp1.portstatus |= 1 << ControlPinM1Inp1.pinnumber;
	if(ControlPinM1Inp1.port==ATMELB){PORTB = *ControlPinM1Inp1.portstatus;}
	if(ControlPinM1Inp1.port==ATMELD){PORTD = *ControlPinM1Inp1.portstatus;}
}

void UnsetMotor1Inp1()
{
	*ControlPinM1Inp1.portstatus &= ~(1 << ControlPinM1Inp1.pinnumber);
	if(ControlPinM1Inp1.port==ATMELB){PORTB = *ControlPinM1Inp1.portstatus;}
	if(ControlPinM1Inp1.port==ATMELD){PORTD = *ControlPinM1Inp1.portstatus;}
}

void SetMotor1Inp2()
{
	*ControlPinM1Inp2.portstatus |= 1 << ControlPinM1Inp2.pinnumber;
	if(ControlPinM1Inp2.port==ATMELB){PORTB = *ControlPinM1Inp2.portstatus;}
	if(ControlPinM1Inp2.port==ATMELD){PORTD = *ControlPinM1Inp2.portstatus;}
}

void UnsetMotor1Inp2()
{
	*ControlPinM1Inp2.portstatus &= ~(1 << ControlPinM1Inp2.pinnumber);
	if(ControlPinM1Inp2.port==ATMELB){PORTB = *ControlPinM1Inp2.portstatus;}
	if(ControlPinM1Inp2.port==ATMELD){PORTD = *ControlPinM1Inp2.portstatus;}
}

void SetMotor2Inp1()
{
	*ControlPinM2Inp1.portstatus |= 1 << ControlPinM2Inp1.pinnumber;
	if(ControlPinM2Inp1.port==ATMELB){PORTB = *ControlPinM2Inp1.portstatus;}
	if(ControlPinM2Inp1.port==ATMELD){PORTD = *ControlPinM2Inp1.portstatus;}
}

void UnsetMotor2Inp1()
{
	*ControlPinM2Inp1.portstatus &= ~(1 << ControlPinM2Inp1.pinnumber);
	if(ControlPinM2Inp1.port==ATMELB){PORTB = *ControlPinM2Inp1.portstatus;}
	if(ControlPinM2Inp1.port==ATMELD){PORTD = *ControlPinM2Inp1.portstatus;}
}

void SetMotor2Inp2()
{
	*ControlPinM2Inp2.portstatus |= 1 << ControlPinM2Inp2.pinnumber;
	if(ControlPinM2Inp2.port==ATMELB){PORTB = *ControlPinM2Inp2.portstatus;}
	if(ControlPinM2Inp2.port==ATMELD){PORTD = *ControlPinM2Inp2.portstatus;}
}

void UnsetMotor2Inp2()
{
	*ControlPinM2Inp2.portstatus &= ~(1 << ControlPinM2Inp2.pinnumber);
	if(ControlPinM2Inp2.port==ATMELB){PORTB = *ControlPinM2Inp2.portstatus;}
	if(ControlPinM2Inp2.port==ATMELD){PORTD = *ControlPinM2Inp2.portstatus;}
}

void SetHBridgeControl() //With direct port manipulation for speedup the arduino framework!
{
	//Motor 1
	if(motordirection1 != oldmotordirection1)
	{
		if(motordirection1 != 0)
		{
			if(motordirection1 == 1)
			{
				SetMotor1Inp1();
				UnsetMotor1Inp2();
			}
			else
			{
				UnsetMotor1Inp1();
				SetMotor1Inp2();
			}
		}
		else
		{
			UnsetMotor1Inp1();
			UnsetMotor1Inp2();
		}
		oldmotordirection1=motordirection1;
	}

	//Motor 2
	if(motordirection2 != oldmotordirection2)
	{
		if(motordirection2 != 0)
		{
			if(motordirection2 == 1)
			{
				SetMotor2Inp1();
				UnsetMotor2Inp2();
			}
			else
			{
				UnsetMotor2Inp1();
				SetMotor2Inp2();
			}
		}
		else
		{
			UnsetMotor2Inp1();
			UnsetMotor2Inp2();
		}
		oldmotordirection2=motordirection2;
	}
}

void loop()
{
	//Read all stored PID and Feedback settings
	ReadEEProm();
	//Program loop
	while (1==1) //Important hack: Use this own real time loop code without arduino framework delays
	{
		FeedbackPotWorker();
		SerialWorker();
		CalculateVirtualTarget();
		CalculatePID();
		CalculateMotorDirection();
		if(disable==0)
		{
			SetPWM();
			SetHBridgeControl();
		}
		pidcount++;
	}
}
