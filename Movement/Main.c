#include <RASLib/inc/common.h>
#include <RASLib/inc/gpio.h>
#include <RASLib/inc/time.h>
#include <RASLib/inc/adc.h>
#include <RASLib/inc/motor.h>
#include <RasLib/inc/linesensor.h>
#include <RasLib/inc/servo.h>
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
            SetPWM(mtr->pwm1, 1.0f-input, 0.0f);					
     //       SetPWM(mtr->pwm0, 1.0f+input, 0.0f);
     //       SetPWM(mtr->pwm1, -input, 1.0f+input);
        } else if (input > 0) {
            // CW (P, 0)
            SetPWM(mtr->pwm0, 1.0f+input, 0.0f);
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
            SetPWM(mtr->pwm1, 1.0f-input, 0.0f);
 //           SetPWM(mtr->pwm0, 1.0f+input, 0.0f);
  //          SetPWM(mtr->pwm1, 1.0f, 0.0f);
        } else if (input > 0) {
            // CW (P, P)
            SetPWM(mtr->pwm0, 1.0f+input, 0.0f);
            SetPWM(mtr->pwm1, 0.0f, 0.0f);
        } else {
            // S (1, 1)
            SetPWM(mtr->pwm0, 1.0f, 0.0f);
            SetPWM(mtr->pwm1, 1.0f, 0.0f);
        }
    }
}

// Function to initialize a motor on a pair of pins
// The returned pointer can be used by the SetMotor function
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
    //gls = InitializeGPIOLineSensor();    
}

void initIRSensor() {
    adc[0] = InitializeADC(PIN_E1); 																//right
    adc[1] = InitializeADC(PIN_E0);																	//front
}
void initMotor() {
		rightMotor = InitializeDRVMotor(PIN_D2,PIN_D3,true,false);				
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

int main () {
	init();		
	SetMotor(rightMotor, 1);
	SetMotor(leftMotor, -1);
	while(1){};
	//READY GREEN LED ON
	explore();
	sprint();
}

void init(){
	int x = 0;
	initIRSensor();
	initMotor();	
	initGPIOLineSensor();
	InitializeSystemTime();
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
	
	int startTime;
	int time;
	bool limit = true;
	startTime = GetTime();
	
	while(limit){
		
		time = GetTime(); 
		if (time-startTime >= 180){limit = false;}
		
		rightSensor = ADCRead(adc[0])*1000;
		frontSensor = ADCRead(adc[1])*1000;
		rightWall = (rightSensor > 300) ? true : false;
		frontWall = (frontSensor > 300) ? true : false;
	
		if(locCurrent == locEnd){endFound = true;}						//GREEN LED ON
		if(rightWall){
			if(frontWall){turn(LEFT);}
			else{
				forward();																				//Forward one cell
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
				}															
			}
		}
		else{turn(RIGHT);}
																													//Tell Beaglebone locCurrent (put in brackets so beaglebone reads as file)
	}
}

void forward(){ 																					//Moves forward to middle of next cell
																													//update cell number
	if 			(orientation == NORTH)	{locCurrent += -7;}
	else if (orientation == EAST)		{locCurrent += 1;}
	else if (orientation == SOUTH)	{locCurrent += 7;}
	else if (orientation == WEST)		{locCurrent += -1;}
		SetPin(PIN_F3,true);			
		SetMotor(leftMotor, 1);
		SetMotor(rightMotor, 1);
		WaitUS(200);
		SetMotor(leftMotor,  0);
		SetMotor(rightMotor, 0);
		SetPin(PIN_F3,false);
		/* Line Following Start point
		
		LineSensorReadArray(gls, line);
		if((line[3]>0.5)&&(line[4]>0.5)){
			SetMotor(leftMotor, 1);
			SetMotor(rightMotor, 1);
		}
		else if (line[5]>0.5&&line[4]>0.5){
			SetMotor(leftMotor, .8);
			SetMotor(rightMotor, .4);
		}
		else if (line[2]>0.5&&line[3]>0.5){
			SetMotor(leftMotor, .4);
			SetMotor(rightMotor, .8);
		}
		else if (line[7]>0.5&&line[6]>0.5){
			SetMotor(leftMotor, .3);
			SetMotor(rightMotor, -.25);
		}
		else if (line[0]>0.5&&line[1]>0.5){
			SetMotor(leftMotor, -.25);
			SetMotor(rightMotor, .3);
		}
		else if (line[6]>0.5&&line[5]>0.5){
			SetMotor(leftMotor, .6);
			SetMotor(rightMotor, .1);
		}
		else if (line[1]>0.5&&line[2]>0.5){
			SetMotor(leftMotor, .6);
			SetMotor(rightMotor, .1);
		}
					*/
	
}

void turn(int direction){																	//turn 90 degrees in place
	if (direction == RIGHT) {
																													//update orientation
			if(orientation==WEST){orientation = NORTH;}
			else 							{orientation += 1;}
			SetPin(PIN_F1,true);																											//perform turn
			SetMotor(leftMotor, -1);
			SetMotor(rightMotor, 1);
			WaitUS(200);
			SetMotor(leftMotor,  0);
			SetMotor(rightMotor, 0);
			SetPin(PIN_F1,false);	
	}
	if (direction == LEFT) {
																													//update orientation
			if(orientation==NORTH){orientation = WEST;}
			else 									{orientation += -1;}		
			SetPin(PIN_F2,true);																											//perform turn
			SetMotor(leftMotor,  1);
			SetMotor(rightMotor,-1);
			WaitUS(200);
			SetMotor(leftMotor,  0);
			SetMotor(rightMotor, 0);
			SetPin(PIN_F2,false);	
	}
}

void setOrientation(int direction){
	while (orientation != direction) {
	if ((orientation + 1 == direction)||(orientation - 3 == direction)) {turn(RIGHT);}
	if ((orientation - 1 == direction)||(orientation + 3 == direction)) {turn(LEFT);}
	else {turn(RIGHT); turn(RIGHT);}
	}
}

void sprint(void) {
	for(;criticalPath->next != NULL || criticalPath->value != locEnd; criticalPath = criticalPath->next){
		if (criticalPath->value == locCurrent + 1 ) {setOrientation(EAST); forward();}
		if (criticalPath->value == locCurrent - 1 ) {setOrientation(WEST); forward();}
		if (criticalPath->value == locCurrent + 7 ) {setOrientation(SOUTH); forward();}
		if (criticalPath->value == locCurrent - 7 ) {setOrientation(NORTH); forward();}
	}
																													//FINISH RED LED ON
}

