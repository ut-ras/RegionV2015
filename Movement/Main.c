#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/motor.h>
#include <RasLib/inc/linesensor.h>
#include <RasLib/inc/servo.h>
#include <RasLib/inc/encoder.h>
#include <stdbool.h>
#include <stdlib.h>
#include "Main.h"
#include <stdint.h>
#define LEFT 1
#define RIGHT -1
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4
int locCurrent;
int locStart;
int locEnd;
bool rightWall;
bool frontWall;
int orientation;
static tLineSensor *gls;
bool endFound;
int* ptr;
int pastCells[50];
int nextcell;

tADC *adc[2];
//tMotor *servomotor[2];
tMotor *rightMotor;
tMotor *leftMotor;
tEncoder *rightEncoder;
tEncoder	*leftEncoder;

float frontSensor;
float rightSensor;
float line[8];

//Maze Optimization Seeing Easter-egg Solver II (MOSES II)
//TODO:
//Tell Vision Processing the cell we are in everytime we change cells
//Utilize Encoders

// Definition of struct Motor
typedef struct TLEMotor {
    // Motor 'abstract' function
    void (*SetMotor)(struct TLEMotor *mtr, float input);

    // PWM signals used by motors
    tPWM *pwm0;
    tPWM *pwm1;

    // True if braking is applied
    tBoolean brake;

    // Set to switch motor direction
    tBoolean invert;
} tTLEMotor;

// Buffer of motor structs to use
// There can only be the total count of pins/2 since each
// motor needs 2 pins
static tTLEMotor tleMotorBuffer[PIN_COUNT / 2];

static int tleMotorCount = 0;



// This function sets a motor speed
static void SetDRVMotor(tTLEMotor *mtr, float input) { 
    // Check the input range
    if (input > 1 || input < -1)
        return;
    
    // invert if set
    if (mtr->invert)
        input *= -1;

    // Operate the motor controller
    // Motor controller operation is specific 
    // to the TLE5205-2
    if (mtr->brake) {
        if (input < 0) {
            // CCW (P, ~P)			
            SetPWM(mtr->pwm0, 0.0f, 0.0f);
            SetPWM(mtr->pwm1, 0.0f - (input), 0.0f);					
        } else if (input > 0) {
            // CW (P, 0)
            SetPWM(mtr->pwm0, 0.0f + (input), 0.0f);
            SetPWM(mtr->pwm1, 0.0f, 0.0f);
        } else {
            // S (1, 0)
            SetPWM(mtr->pwm0, 1.0f, 0.0f);
            SetPWM(mtr->pwm1, 1.0f, 0.0f);
        }
    } else {
        if (input < 0) {
            // CCW (P, 1)
            SetPWM(mtr->pwm0, 0.0f, 0.0f);
            SetPWM(mtr->pwm1, 0.0f - (input), 0.0f);
        } else if (input > 0) {
            // CW (P, P)
            SetPWM(mtr->pwm0, 0.0f + (input), 0.0f);
            SetPWM(mtr->pwm1, 0.0f, 0.0f);
        } else {
            // S (1, 1)
            SetPWM(mtr->pwm0, 1.0f, 0.0f);
            SetPWM(mtr->pwm1, 1.0f, 0.0f);
        }
    }
}


tTLEMotor *_InitializeDRVMotor(tPin a, tPin b, tBoolean brake, tBoolean invert) {
    // Grab the next motor
    tTLEMotor *mtr = &tleMotorBuffer[tleMotorCount++];
    
    // Setup the initial data
    mtr->brake = brake;
    mtr->invert = invert;
    
    // Initialize pwm on both pins
    mtr->pwm0 = InitializePWM(a, 1600.0f);
    mtr->pwm1 = InitializePWM(b, 1600.0f);

    // Set parent methods
    mtr->SetMotor = SetDRVMotor;
    
    // Return the new motor
    return mtr;
}

tMotor *InitializeDRVMotor(tPin a, tPin b, tBoolean brake, tBoolean invert) {
    return (tMotor *)_InitializeDRVMotor(a, b, brake, invert);
}

void initGPIOLineSensor(void) {
    // use 8 I/O pins to initialize a GPIO line sensor
    gls = InitializeGPIOLineSensor(PIN_C7, PIN_C6, PIN_C5, PIN_C4, PIN_B3, PIN_B2, PIN_B1, PIN_B0);    
}

void initIRSensor() {
    adc[1] = InitializeADC(PIN_E1); 																//right
    adc[0] = InitializeADC(PIN_E5);																	//front
}

void InitEncoders(void){
rightEncoder = InitializeEncoder(PIN_B4, PIN_B5, false);
leftEncoder = InitializeEncoder(PIN_B6, PIN_B7, false);
};

void initMotor() {
		rightMotor = InitializeDRVMotor(PIN_D3,PIN_D2,true,false);				
		leftMotor = InitializeDRVMotor(PIN_D0,PIN_D1,true,false);				
}

struct linkedList{
	int value;
	struct linkedList *next;
};

struct linkedList *criticalPath;

void init(void);
void explore(void);
void sprint(void);
void setOrientation(int direction);

int main () {
	init();		
	
	while (GetPin(PIN_F0)){	};
	double speed = .15;
	
	ResetEncoder(rightEncoder);
	ResetEncoder(leftEncoder);
	
	Wait(1);
	SetMotor(leftMotor, 1);
	SetMotor(rightMotor, 1);	
	Wait(1);
	SetMotor(leftMotor, 0);
	SetMotor(rightMotor, 0);	
	
	//forward(speed);
	
	GetEncoder(rightEncoder);
	GetEncoder(leftEncoder);
	
	
	Printf("Encoder values:  %10d  %10d  \r \n", GetEncoder(leftEncoder),GetEncoder(rightEncoder));
	
	
	//explore();
	while(1) {};
	sprint();
}

void init(){
	int x = 0;
	initIRSensor();
	initMotor();	
	InitializeGPIO();
	initGPIOLineSensor();
	InitializeSystemTime();
	InitializeUART(115200);
	InitEncoders();
	orientation = NORTH;
	
	//set start and end locations
	endFound = false;
	locCurrent = 48;
	locEnd = 9;
	
	criticalPath->value = locCurrent;												//set start of crit path
	
	for(x = 0; x < 49; x++)
		pastCells[x] = 0;
}


void releaseRest(struct linkedList *list){
	if(list->next != NULL){
		releaseRest(list->next);
	}
	free(list);
	
}

void explore(){
	double speed = .2;
	bool stop = true;
//	int startTime;
//	int time;
//	bool limit = true;
//	startTime = GetTime();
		Wait(2);
	while(1){

//		if (GetPin(PIN_F0)){stop = false;};
//		time = GetTime(); 
//		if (time-startTime >= 180){limit = false;}
		
		rightSensor = ADCRead(adc[1])*1000;
		frontSensor = ADCRead(adc[0])*1000;
		rightWall = (rightSensor > 550) ? true : false;
		frontWall = (frontSensor > 550) ? true : false;
	
		if(locCurrent == locEnd){
			endFound = true;
		}						//GREEN LED ON
		if			(rightWall&&frontWall){
			turn(LEFT);
			forward(speed);		
		}
		else if	(rightWall&&!frontWall){
			forward(speed);	
			/*
																						//Forward one cell
			
			
				if (!endFound) {	
					if(pastCells[locCurrent] != 1){									//Check for repeat cell in crit path
						struct linkedList *link;											//
						struct linkedList *list = criticalPath;				//Point to path list
						link->value = locCurrent;											//Add	value to a new struct
						for(;list->next != NULL; list = list->next);	//Get to end of list
						list->next = link;														//Add new struct to the list
					}else{
						struct linkedList *list = criticalPath;
						for(;list->value == locCurrent || list->next != NULL; list = list->next);
						releaseRest(list->next);
						list->next = NULL;
					}						
					pastCells[locCurrent] = 1;											//Set cell to visited
				}	*/														
			}		
		else if (!rightWall){
			turn(RIGHT);
			forward(speed);
		}
																												//Tell Beaglebone locCurrent (put in brackets so beaglebone reads as file)
		//Sending to beaglebone
		char locChar[4];
		if (locCurrent < 10) {
			locChar[0] = '0';
			locChar[1] = (locCurrent) + 0x30;
			locChar[2] = '\n';
			locChar[3] = 0;
		}
		else {
			locChar[0] = (locCurrent / 10 % 10) + 0x30;
			locChar[1] = (locCurrent % 10) + 0x30;
			locChar[2] = '\n';
			locChar[3] = 0;
		}
		Puts(locChar, 4);
	}
}

void forward(double speed){ 
																					
	int i;
	int curve[8];	
	float line[8];
	int curvetotal;
	int numLineSensor = 0;
	int lost = 1;
	int lostDir = 0;
	
	//update cell number
	if 			(orientation == NORTH)	{locCurrent += -7;}
	else if (orientation == EAST)		{locCurrent += 1;}
	else if (orientation == SOUTH)	{locCurrent += 7;}
	else if (orientation == WEST)		{locCurrent += -1;}
		
	
	SetPin(PIN_F2,1);
	SetMotor(leftMotor, .2);
	SetMotor(rightMotor, .2);	
	Wait(.4);
	SetPin(PIN_F2,0);
	while( numLineSensor <= 4 ){	
				curvetotal = 10;
				numLineSensor = 0;
				lost = 1;
				LineSensorReadArray(gls, line); 
       // Printf("Line Sensor: [");
        for (i = 0; i < 8; i++) {
					if(line[i]>.8f){
						curve[i] =1;
						numLineSensor++;
						lost = 0;
						if ((abs(i-3)) <= curvetotal) {curvetotal = (i-3);}
					}
					else{
						curve[i] =0;
					}
        }
			//	Printf("%d ", curvetotal);
      // Printf("\b]        \r");
		
				//line[0] is far left
				
				if (lost == 1) {
					curvetotal = lostDir;
				}	
				 
				/*
				
				if (line[3] > .8f || line[4] > .8f){
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed);
				}
				else if (line[0] > .8f){
					lostDir = -3;
					SetMotor(leftMotor, speed*.3);
					SetMotor(rightMotor, speed*.9);
				}
				else if (line[7] > .8f){
					lostDir = 4;
					SetMotor(leftMotor, speed*.9);
					SetMotor(rightMotor, speed*.3);
				}
				else if (line[1] > .8f){
					lostDir = -3;
					SetMotor(leftMotor, speed*.7);
					SetMotor(rightMotor, speed);
				}
				else if (line[6] > .8f){
					lostDir = 4;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed*.7);
				}
				else if (line[2] > .8f){
					lostDir = -3;
					SetMotor(leftMotor, speed*.9);
					SetMotor(rightMotor, speed);
				}
				else if (line[5] > .8f){
					lostDir = 4;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed*.9);
				}
				*/
				
				if 			(curvetotal==-3){
					lostDir = 4;
					SetMotor(leftMotor, speed*.5);
					SetMotor(rightMotor, speed);
				}		
				else if (curvetotal==-2){
					lostDir = 4;
					SetMotor(leftMotor, speed*.7);
					SetMotor(rightMotor, speed);
				}		
				else if (curvetotal==-1){
					lostDir = 4;
					SetMotor(leftMotor, speed*.9);
					SetMotor(rightMotor, speed);
				}		
				else if	(curvetotal==0){
					lostDir = 4;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed);
				}
				else if (curvetotal==1){
					lostDir = -3;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed);
				}
				else if (curvetotal==2){
					lostDir = -3;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed*.9);
				}		
				else if (curvetotal==3){
					lostDir = -3;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed*.7);
				}		
				else if (curvetotal==4){
					lostDir = -3;
					SetMotor(leftMotor, speed);
					SetMotor(rightMotor, speed*.5);
				}
				else {
					SetMotor(leftMotor, 0);
					SetMotor(rightMotor, 0);
				};
	}		
	SetMotor(leftMotor, 0);
	SetMotor(rightMotor, 0);
	Wait(1);
}

void turn(int direction){

	int i;
	int curve[8];	
	float line[8];
	int curvetotal;
	int numLineSensor = 0;


	//turn 90 degrees in place
	if (direction == RIGHT) {
		
		SetMotor(leftMotor,  .2);
		SetMotor(rightMotor,-.2);
		Wait(.9);
		
		/*
		while(numLineSensor <= 6){
			curvetotal = 0;
			numLineSensor = 0;
			SetMotor(leftMotor,   .2);
			SetMotor(rightMotor, -.2);
			LineSensorReadArray(gls, line); 
      Printf("Line Sensor: [");
        for (i = 0; i < 8; i++) {
					if(line[i]>.8f){
						curve[i] =1;
						numLineSensor++;
					}
					else{
						curve[i] = 0;
					}
					if (((curve[i])==1)&&(abs(i-3)>curvetotal)){
						curvetotal = (i-3);
					}
        }
		}
		*/
		
		//update orientation
		if(orientation==WEST){ orientation = NORTH;}
		else{	orientation += 1;}	
	}
	if (direction == LEFT) {
		SetMotor(leftMotor, -.2);
		SetMotor(rightMotor,.2);
		Wait(.9);
		
		/*
		while(numLineSensor <= 6){
			
			
			curvetotal = 0;
			numLineSensor = 0;
			LineSensorReadArray(gls, line); 
      Printf("Line Sensor: [");
        for (i = 0; i < 8; i++) {
					if(line[i]>.8f){
						curve[i] =1;
						numLineSensor++;
					}
					else{
						curve[i] =0;
					}
					if (((curve[i])==1)&&(abs(i-3)>curvetotal)){
						curvetotal = (i-3);
					}
        }
		} */
		
		
		
		//update orientation
		if(orientation==NORTH){orientation = WEST;}
		else 									{orientation += -1;}		
		
	}
		SetMotor(leftMotor,  0);
		SetMotor(rightMotor, 0);
		Wait(1);
}

void setOrientation(int direction){
	while (orientation != direction) {
	if ((orientation + 1 == direction)||(orientation - 3 == direction)) {turn(RIGHT);}
	else if ((orientation - 1 == direction)||(orientation + 3 == direction)) {turn(LEFT);}
	else {turn(RIGHT); turn(RIGHT);}
	}
}

void sprint(void) {
	double speed = 1;
	for(;criticalPath->next != NULL || criticalPath->value != locEnd; criticalPath = criticalPath->next){
		if (criticalPath->value == locCurrent + 1 ) {setOrientation(EAST); forward(speed);}
		if (criticalPath->value == locCurrent - 1 ) {setOrientation(WEST); forward(speed);}
		if (criticalPath->value == locCurrent + 7 ) {setOrientation(SOUTH); forward(speed);}
		if (criticalPath->value == locCurrent - 7 ) {setOrientation(NORTH); forward(speed);}
	}
																													//FINISH RED LED ON
}

