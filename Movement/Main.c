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
tBoolean ledOn = true;
tBoolean ledOff = false;

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

//struct linkedList *criticalPath;
int criticalPath[50];
int critical_num = 0;

int find_index(int value)
{
   int i;
   for (i=0; i < critical_num; i++)
   {
	 if (criticalPath[i] == value)
	 {
	    return(i);  /* it was found */
	 }
   }
   return(-1);  /* if it was not found */
}

void print_array()
{
   int i;
   for(i=0; i < critical_num; i++)
   {
	 Printf("%d ", criticalPath[i]);
   }
   Printf("\n");
}

void deleteAfter(int value)
{
	int val = find_index(value);
	for (int i = critical_num - 1; i >= val; i--) {
		criticalPath[i] = 0;
		critical_num--;
	}
}

void delete(int value) {
	int val = find_index(value);
	criticalPath[val] = 0;
	critical_num--;
}

void deleteIndex(int index) {
	criticalPath[index] = 0;
	critical_num--;
}

void add(int value) {
	criticalPath[critical_num] = value;
	critical_num++;
}

void init(void);
void explore(void);
void sprint(void);
void setOrientation(int direction);

int main () {
	init();		
	double speed = .3;	
	//LINKED LIST TESTING
	//TESTING END
	while (GetPin(PIN_F0)){	};
	SetPin(PIN_F3, ledOff);
	explore();
	//turn(RIGHT);
	//forward(speed);

	//explore();
	
	while (GetPin(PIN_F0)){	};
	sprint();
	while(1){};
}

void init(){
	int x = 0;
	SetPin(PIN_F3, ledOn);
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
	locCurrent = 48;									//7x7 Starts in 49, rest is 48
	locEnd = 1;												//6x6 and 7x7 ends in 1, rest is 9
	
	//criticalPath->value = locCurrent;												//set start of crit path
	
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
	double speed = .3;
	while(locCurrent!=locEnd){
		
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
			if (!endFound) {	
					if(pastCells[locCurrent] != 1){									//Check for repeat cell in crit path
						/*
						struct linkedList *link;											//
						struct linkedList *list = criticalPath;				//Point to path list
						link->value = locCurrent;											//Add	value to a new struct
						*/
						add(locCurrent);
						/*
						for(;list->next != NULL; list = list->next);	//Get to end of list
						list->next = link;														//Add new struct to the list
						*/
					}else{
						/*
						struct linkedList *list = criticalPath;
						for(;list->value == locCurrent || list->next != NULL; list = list->next);
						releaseRest(list->next);
						list->next = NULL;
						*/
						deleteAfter(locCurrent);
						
					}						
					pastCells[locCurrent] = 1;											//Set cell to visited
				}															
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
	SetPin(PIN_F1, ledOn);
}

void forward(double speed){ 
																					
	int i;
	float adjustedLine[8];	
	float line[8];
	float lineError[8] = {.22,.20,.19,.19,.20,.18,.17,.25};
	float lineWeightRight[8] = {.8,.7,.65,.6,.55,.5,.45,.4};
	float lineWeightLeft[8] = {.4,.45,.5,.55,.6,.65,.7,.8};
	int numLineSensor = 0;
	int lost = 1;
	int lostDir = 0;
	float rightSpeed;
	float leftSpeed;
	
	//update cell number
	if 			(orientation == NORTH)	{locCurrent += -7;}
	else if (orientation == EAST)		{locCurrent += 1;}
	else if (orientation == SOUTH)	{locCurrent += 7;}
	else if (orientation == WEST)		{locCurrent += -1;}
		
	SetMotor(leftMotor,  .2);
	SetMotor(rightMotor, .2);		
	Wait(.3);
	while( numLineSensor <=5 ){	
			numLineSensor = 0;
			rightSpeed = 0;
			leftSpeed = 0;
			LineSensorReadArray(gls, line); 
       for (i = 0; i < 8; i++) {
					if(line[i]>.7){numLineSensor++;}
					if(line[i]>1){line[i] = 1.0 ;}
					adjustedLine[i] = line[i] - lineError[i];
					rightSpeed += (adjustedLine[i] * lineWeightRight[i]);
					leftSpeed += (adjustedLine[i] * lineWeightLeft[i]);						
			}
			
			rightSpeed = rightSpeed / 2;
			leftSpeed = leftSpeed / 2;
			Printf("%.2f ", leftSpeed);
			Printf("%.2f ", rightSpeed);
			Printf("\b]        \r");			
				
			if (numLineSensor>=4){
				leftSpeed=0;
				rightSpeed=0;
			}
			if (numLineSensor == 0){
				if(lost == 0){
				leftSpeed =.2;
				rightSpeed =.5;
				}
				else if(lost == 1){
				leftSpeed =.5;
				rightSpeed =.2;				
				}
			}
			if ((leftSpeed*speed) >= .3){leftSpeed = .3/speed;}
			if ((rightSpeed*speed) >= .3){rightSpeed = .3/speed;}
			
			SetMotor(leftMotor,  (leftSpeed*speed*.75));
			SetMotor(rightMotor, (rightSpeed*speed));	
			
			if (rightSpeed > leftSpeed){
			lost = 1;
			}
			else {
			lost = 0;
			}			
	}
	SetMotor(leftMotor,  0);
	SetMotor(rightMotor, 0);
	Wait(.5);
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
		Wait(.5);
		
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
		Wait(.5);
		
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
	/*
	for(;criticalPath->next != NULL || criticalPath->value != locEnd; criticalPath = criticalPath->next){
		if (criticalPath->value == locCurrent + 1 ) {setOrientation(EAST); forward(speed);}
		if (criticalPath->value == locCurrent - 1 ) {setOrientation(WEST); forward(speed);}
		if (criticalPath->value == locCurrent + 7 ) {setOrientation(SOUTH); forward(speed);}
		if (criticalPath->value == locCurrent - 7 ) {setOrientation(NORTH); forward(speed);}
	}
	*/
	int i = 0;
	while (criticalPath[i] != locEnd) {
		if (criticalPath[i] == locCurrent + 1 ) {setOrientation(EAST); forward(speed);}
		if (criticalPath[i] == locCurrent - 1 ) {setOrientation(WEST); forward(speed);}
		if (criticalPath[i] == locCurrent + 7 ) {setOrientation(SOUTH); forward(speed);}
		if (criticalPath[i] == locCurrent - 7 ) {setOrientation(NORTH); forward(speed);}
		deleteIndex(i);
		i++;
	}
	SetPin(PIN_F1, ledOn);
	//FINISH RED LED ON*/
																													//FINISH RED LED ON*/
}

