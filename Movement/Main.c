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

tADC *adc[2];
//tMotor *servomotor[2];
tMotor *rightMotor;
tMotor *leftMotor;
float frontSensor;
float rightSensor;
float line[8];

struct linkedList{
	int value;
	struct linkedList *next;
};

struct linkedList *criticalPath;

//Maze Optimization Seeing Easter-egg Solver II (MOSES II)

//Tell Vision Processing the cell we are in everytime we change cells
//Give Maze Navigation a list of Cells from Start to End (Not After)


void initGPIOLineSensor(void) {
    // use 8 I/O pins to initialize a GPIO line sensor
    gls = InitializeGPIOLineSensor(PIN_C7, PIN_C6, PIN_E0, PIN_D3, PIN_D2, PIN_D1, PIN_D0, PIN_B5);    
}

void initIRSensor() {
    adc[0] = InitializeADC(PIN_E5);
    adc[1] = InitializeADC(PIN_E4);
}
void initMotor() {
		leftMotor = InitializeServoMotor(PIN_C4,false);			//left motor
		rightMotor = InitializeServoMotor(PIN_C5,true);			//right motor
}

void init();

int main () {
	init();
	
}

void init(){
	int x = 0;
	initIRSensor();
	initMotor();	
	orientation = NORTH;
	
	//set start and end locations
	endFound = false;
	locCurrent = 48;
	locEnd = 9;
	
	criticalPath->value = locCurrent;				//set start of crit path
	
	for(x = 0; x < 49; x++)
		pastCells[x] = 0;
	
	explore();
	
}

void follow();

void explore(){
	while(true){
		rightSensor = ADCRead(adc[0])*1000;
		frontSensor = ADCRead(adc[1])*1000;
		rightWall = rightSensor > 300 ? true : false;
		frontWall = frontSensor > 300 ? true : false;
	
		if(locCurrent == locEnd){
			//LED ON
			endFound = true;
		}
		
		if(rightWall){
			if(frontWall){
				//turn left
				turn(LEFT);
					if(orientation==1){
						orientation = 4;
					}
					else {
					orientation += -1;
					}
			}
			else{
					if (orientation == NORTH){
						locCurrent += -7;
					}
					else if (orientation == EAST){
						locCurrent += 1;
					}
					else if (orientation == SOUTH){
						locCurrent += 7;			
					}
					else if (orientation == WEST){
						locCurrent += -1;			
					}
				forward();	//go forward one cell
				if (!endFound) {	
					if(pastCells[locCurrent] != 1){				//Check for repeat cell in crit path
						struct linkedList *link;
						link->value = locCurrent;						
						
						struct linkedList *list = criticalPath;
						for(;list->next != NULL; list = list->next);
						list->next = link;
					}else{
						struct linkedList *list = criticalPath;
						for(;list->value == locCurrent || list->next != NULL; list = list->next);
						list->next = NULL;
					}
						
					pastCells[locCurrent] = 1;	//Set Visited
				}															
			}
		}
		else{
			//turn right
			turn(RIGHT);
				if(orientation==4){
					orientation = 1;
				}
				else {
					orientation += 1;
				}
		}
												//Tell Beaglebone locCurrent
	}
}

void forward(){ 				//Moves forward to middle of next cell
	while (true) {
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
	}				
}

void turn(int direction){						//turns 90 degrees in place
	//change orientation
	
}

