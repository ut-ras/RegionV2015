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

void initGPIOLineSensor(void) {
    // use 8 I/O pins to initialize a GPIO line sensor
    //gls = InitializeGPIOLineSensor();    
}

void initIRSensor() {
    adc[0] = InitializeADC(PIN_E1); 																//right
    adc[1] = InitializeADC(PIN_E0);																	//front
}
void initMotor() {
		leftMotor = InitializeTLEMotor(PIN_D2,PIN_D3,true,false);				//left 
		rightMotor = InitializeTLEMotor(PIN_D0,PIN_D1,true,true);				//right
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
					
	while (true) {
		SetMotor(leftMotor, 1);
		SetMotor(rightMotor, 1);
		WaitUS(50);
		SetMotor(leftMotor,  0);
		SetMotor(rightMotor, 0);
		
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
}

void turn(int direction){																	//turn 90 degrees in place
	if (direction == RIGHT) {
																													//update orientation
			if(orientation==WEST){orientation = NORTH;}
			else 							{orientation += 1;}
																													//perform turn
			SetMotor(leftMotor, -1);
			SetMotor(rightMotor, 1);
			WaitUS(50);
			SetMotor(leftMotor,  0);
			SetMotor(rightMotor, 0);
	}
	if (direction == LEFT) {
																													//update orientation
			if(orientation==NORTH){orientation = WEST;}
			else 									{orientation += -1;}		
																													//perform turn
			SetMotor(leftMotor,  1);
			SetMotor(rightMotor,-1);
			WaitUS(50);
			SetMotor(leftMotor,  0);
			SetMotor(rightMotor, 0);
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

