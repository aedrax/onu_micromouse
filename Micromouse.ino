/* Define pins for HC-SR04 ultrasonic sensor */
#define echoPin 6 // Echo Pin = Analog Pin 0
#define trigPin 7 // Trigger Pin = Analog Pin 1
#define LEDPin 13 // Onboard LED
long duration; // Duration used to calculate distance
long HR_dist = 0; // Calculated Distance
byte minimumRange = 5; //Minimum Sonar range
int maximumRange = 400; //Maximum Sonar Range


#include <Wire.h>//needed for the MotorShield
#include "Adafruit_MotorShield.h"
/* Using the motor shield */
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); //the AdaFruit motorshield object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);    //pointer to the right motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);     //pointer to the left motor




/* get the distances for the for the IR sensors */
#include "DistanceGP2Y0A41SK.h"

DistanceGP2Y0A41SK leftIrSensor;  //the analog IR sensor on the left side
DistanceGP2Y0A41SK rightIrSensor; //the analog IR sensor on the right side
#define LEFT_IR_PIN A0
#define RIGHT_IR_PIN A1
byte leftIrSensorValue;    //placeholder for the distance of the left IR sensor
byte rightIrSensorValue;         //placeholder for the distance of the right IR sensor
int forwardDistance;       //placeholder for the distance ahead of the robot, potentially used to find distance traveled








/* encoders stuff */
#include "PololuWheelEncoders.h" //Does not work with Arduino Mega
//#include "Encoder.h" //used when trying the Arduino Mega
#define ENCODER_LEFT_A A3
#define ENCODER_LEFT_B A2
#define ENCODER_RIGHT_A 4
#define ENCODER_RIGHT_B 5
//The Encoders are implimented when using an Arduino Mega
//Encoder LeftWheelEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B);
//Encoder RightWheelEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

#define FORWARD_SPEED 40 //speed of the motors when driving straight out of 255

//wall_threshold is the distances to check when driving to make sure the robot doesn't drive into the wall
#define WALL_THRESHOLD_FORWARD 7 //cm from the front sensor the robot can get
#define WALL_THRESHOLD_LEFT 7    //cm from the left IR sensor the robot can get
#define WALL_THRESHOLD_RIGHT 7   //cm from the right IR sensor the robot can get
#define TURN_NUMBER_LEFT 14      //When turning Left the encoder of the right wheel is about this when reaching 90 degrees
#define TURN_NUMBER_RIGHT -17    //When turning Right the encoder of the right wheel is about this when reaching 90 degrees
#define TURN_SPEED  30           //speed for turning the robot
#define TURN_CORRECTION 30       //speed to turn one wheel when correcting the robot
#define TURN_CORRECTION_DELAY 40 //length in milliseconds to correct motors when driving toward walls, higer value is a sharper turn
#define LEFT 0                   //for things like turning LEFT or what cell is LEFT of the robot
#define RIGHT 1                  //for things like turning RIGHT or what cell is RIGHT of the robot
#define OPPOSITE 2               //for things like turning OPPOSITE or what cell is OPPOSITE of the direction the robot is facing
int LEFT_ENCODER_TICKS_FORWARD = 40;//number of ticks the left encoder must go to traverse one cell
int RIGHT_ENCODER_TICKS_FORWARD = 40;//number of ticks the right encoder must go to travers one cell
int leftEncoderValue;            //placeholder of the value of the left encoder
int rightEncoderValue;           //placeholder of the value of the right encoder

//check distance is to know if the wall is in the cell or not
//i.e. if there is something whithin this range, mark it as there being a wall there
#define CHECK_DISTANCE_FORWARD 10 //centimeters forward
#define CHECK_DISTANCE_LEFT 10    //centimeters from the left IR sensor
#define CHECK_DISTANCE_RIGHT 10   //centimeters from the right IR sensor

#include "Cell.h"      //Describes a 'Cell' of the maze
#include "CellStack.h" //Stack of Cells which is the final path of the mouse

//to save memory, bitwise techniques are used so individual bits of the data byte in a Cell
//can be used like booleans, simply use '&' to see if it is true
//i.e. if a cell has a north wall, saying cell.data & IS_NORTH would return true
#define IS_NORTH 1	    //0b00000001 if a cell has a wall on the north end
#define IS_EAST 2	    //0b00000010 if a cell has a wall on the east end
#define IS_SOUTH 4	    //0b00000100 if a cell has a wall on the south end
#define IS_WEST 8      	    //0b00001000 if a cell has a wall on the west end
#define IS_EXPLORED 16	    //0b00010000 if a cell has been explored already
//the position byte of a cell contains both the X and the Y position
//the first four bits are for X and the other four are for Y i.e. 0bXXXXYYYY
#define Y_ONLY 15	    //0b00001111 when bitwise & is used, it returns only the Y value of the position
#define X_SHIFT 4           //X_SHIFT 4 bits for X location

#define MAZE_LENGTH 16      //the number of cells the maze is in each direction
//the memory hog, an array of cells to hold all the information about the maze
Cell maze[MAZE_LENGTH][MAZE_LENGTH];
Cell* currentCell;          //pointer to the current cell the robot is in
CellStack stack;            //the stack of cells which in the end will be the path the robot takes start to finish
byte currentDirection;      //the current direction the robot is facing, the direction will be one of: IS_NORTH, IS_EAST, IS_SOUTH, IS_WEST
#define USELESS 255         //a really high value to put as the distance in a cell to keep the robot from going in there, like with dead ends

void setup()
{	
        //start serial
	Serial.begin (9600);
	
        //start motor shield
	AFMS.begin();
	
        //initialize the encoders
	PololuWheelEncoders::init(ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_RIGHT_B, ENCODER_RIGHT_A);

        //connect the IR sensors
	leftIrSensor.begin(LEFT_IR_PIN);
	rightIrSensor.begin(RIGHT_IR_PIN);
	
	//Setup the trigger and Echo pins of the HC-SR04 sensor
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)

	//initialize the maze with the position info and no known walls
	for (byte y = 0; y < MAZE_LENGTH; y++)
	{
		for(byte x = 0; x < MAZE_LENGTH; x++)
		{
		
			maze[x][y].position = (x << X_SHIFT); //x position
			maze[x][y].position |= (y);//y position
			maze[x][y].data = 0x00;
			
		}
	}
	//fill the distances of each cell
	//this will result in generating cell distances for each cell
	//for example an 8x8 maze would look like this:
	/*
	+---+---+---+---+---+---+---+---+
	| 6 | 5 | 4 | 3 | 3 | 4 | 5 | 6 |
	+---+---+---+---+---+---+---+---+
	| 5 | 4 | 3 | 2 | 2 | 3 | 4 | 5 |
	+---+---+---+---+---+---+---+---+
	| 4 | 3 | 2 | 1 | 1 | 2 | 3 | 4 |
	+---+---+---+---+---+---+---+---+
	| 3 | 2 | 1 | 0 | 0 | 1 | 2 | 3 |
	+---+---+---+---+---+---+---+---+
	| 3 | 2 | 1 | 0 | 0 | 1 | 2 | 3 |
	+---+---+---+---+---+---+---+---+
	| 4 | 3 | 2 | 1 | 1 | 2 | 3 | 4 |
	+---+---+---+---+---+---+---+---+
	| 5 | 4 | 3 | 2 | 2 | 3 | 4 | 5 |
	+---+---+---+---+---+---+---+---+
	| 6 | 5 | 4 | 3 | 3 | 4 | 5 | 6 |
	+---+---+---+---+---+---+---+---+
	*/
	byte half = MAZE_LENGTH / 2;
	for (byte y = 0; y < half; y++)
	{
		for (byte x = 0; x < half; x++)
		{
			byte m = MAZE_LENGTH - 2 - x - y;
			maze[x][y].distance = m;
			maze[MAZE_LENGTH - 1 - x][y].distance = m;
			maze[MAZE_LENGTH - 1 - x][MAZE_LENGTH - 1 - y].distance = m;
			maze[x][MAZE_LENGTH - 1 - y].distance = m;
		}
	}
	
        //this is just a test to make sure the distance map is generated correctly
	printMap();

        //setup the information of the first cell when the robot is in cell 0,0
	currentDirection = IS_NORTH;        //this direction is now considered north, all the cool kids say so
	currentCell = &maze[0][0];          //the currentCell pointer now points to the address of the 0,0 cell
	currentCell->data |= IS_EAST;       //assume the cell has an east wall
	currentCell->data |= IS_WEST;       //assume the cell has a west wall
	currentCell->data |= IS_SOUTH;      //assume the cell has a south wall
	currentCell->data |= IS_EXPLORED;   //the robot is in it so mark the cell as explored
	stack.push(currentCell);            //push the first cell on to the stack of cell locations
}



void loop()
{
	while (currentCell->distance == 0);//makes it stop in the center
	updateWalls();//check to see if there are wall left, right, and ahead
	byte moveDirection = getBestDirection();//determine which way the robot should drive
	move(moveDirection);//drive that determined way
	
}

//should kinda explain its self
void stop()
{
	leftMotor->run(RELEASE);
	rightMotor->run(RELEASE);
}




//--------------------getDistance() FUNCTION ---------------//
int getSonarDistance()
{

	// The following trigPin/echoPin cycle is used to determine the
	//distance of the nearest object by bouncing soundwaves off of it. 
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);

	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);

//Calculate the distance (in cm) based on the speed of sound.
	HR_dist = duration / 58.2;

	//Send the reading from the ultrasonic sensor to the computer 
	if (HR_dist >= maximumRange || HR_dist <= minimumRange)
	{
		// Send a 0 to computer and Turn LED ON to indicate "out of range" 
		digitalWrite(LEDPin, HIGH);
		return 0;
	}
	else
	{
		// Send the distance to the computer using Serial protocol, and
		//turn LED OFF to indicate successful reading. 
		digitalWrite(LEDPin, LOW);		
		return HR_dist;
	}
}

//my best attempt at driving straight in a maze using a master and slave motor as well as correcting with the sensors
void driveStraight()
{
        //set the motors to drive forward
	leftMotor->run(FORWARD);
	rightMotor->run(FORWARD);
	//The powers we give to both motors. masterPower will remain constant while slavePower will change so that
	//the right wheel keeps the same speed as the left wheel.
	int masterPower = FORWARD_SPEED;
	int slavePower = FORWARD_SPEED;

	//Essentially the difference between the master encoder and the slave encoder. Negative if slave has
	//to slow down, positive if it has to speed up. If the motors moved at exactly the same speed, this
	//value would be 0.
	int error = 0;

	//'Constant of proportionality' which the error is divided by. Usually this is a number between 1 and 0 the
	//error is multiplied by, but we cannot use floating point numbers. Basically, it lets us choose how much
	//the difference in encoder values effects the final power change to the motor.
	int kp = 3;

	int currentLeft;  //the value retreived from the left encoder before it is reset
	int currentRight; //the value retreived from the right encoder before it is reset
	bool isCloseToWall = false;//if the loop of driving straight has to break, this determines if it was because it was to close to a wall
	

	//while the robot has not moved forward one cell
	while(true)
	{
		forwardDistance = getSonarDistance();//update the distance of the front sensor
		leftIrSensorValue = leftIrSensor.getDistanceCentimeter();//update the left IR sensor value
		rightIrSensorValue = rightIrSensor.getDistanceCentimeter();//update the right IR sensor value


		if(forwardDistance < WALL_THRESHOLD_FORWARD     //if close to the front wall
                  || leftIrSensorValue < WALL_THRESHOLD_LEFT    //or close to the left wall
                  || rightIrSensorValue < WALL_THRESHOLD_RIGHT) //or close to the right wall
		{
			isCloseToWall = true;
			break;
		}
                if(leftEncoderValue > LEFT_ENCODER_TICKS_FORWARD   //if the left wheel has turned enough times
                && rightEncoderValue > RIGHT_ENCODER_TICKS_FORWARD)//if the right wheel has turned enough times
                {
                   break;
                }


		//Set the motor powers to their respective variables.
		leftMotor->setSpeed(masterPower);
		rightMotor->setSpeed(slavePower);

		//This is where the magic happens. The error value is set as a scaled value representing the amount the slave
		//motor power needs to change. For example, if the left motor is moving faster than the right, then this will come
		//out as a positive number, meaning the right motor has to speed up.
		//Reset the encoders every loop so we have a fresh value to use to calculate the error.
		currentLeft = PololuWheelEncoders::getCountsAndResetM1();
		currentRight = PololuWheelEncoders::getCountsAndResetM2();

		leftEncoderValue += currentLeft;    //update the global value of the left encoder
		rightEncoderValue += currentRight;  //update the global value of the right encoder
		error = currentLeft - currentRight; //calculate the error

		//This adds the error to slavePower, divided by kp. The '+=' operator literally means that this expression really says
		//"slavePower = slavepower + error / kp", effectively adding on the value after the operator.
		//Dividing by kp means that the error is scaled accordingly so that the motor value does not change too much or too
		//little. You should 'tune' kp to get the best value. For us, this turned out to be around 5.
		slavePower += error / kp;

		//Makes the loop repeat ten times a second. If it repeats too much we lose accuracy due to the fact that we don't have
		//access to floating point math, however if it repeats to little the proportional algorithm will not be as effective.
		//Keep in mind that if this value is changed, kp must change accordingly.
		delay(100);		
	}
	if(isCloseToWall)
	{
		//the sensor detects it's approaching the left wall then turn right slightly
		if(leftIrSensorValue < WALL_THRESHOLD_LEFT)
		{
			leftMotor->setSpeed(TURN_CORRECTION);
			rightMotor->run(RELEASE);//turn off the right motor
		}
		//the sensor detects it's approaching the right wall then turns left slightly
		else if(rightIrSensorValue < WALL_THRESHOLD_RIGHT)
		{
			leftMotor->run(RELEASE);//turn off the left motor
			rightMotor->setSpeed(TURN_CORRECTION);
		}
		//if there is a wall in front of the mouse, don't move forward pretty much
		if(forwardDistance < WALL_THRESHOLD_FORWARD)
		{
			//turn both motors off
			leftMotor->run(RELEASE);
			rightMotor->run(RELEASE);
		}
		//this delay allows for mouse to adjust it's forward course
		//the delay is how long to actually turn for
		delay(TURN_CORRECTION_DELAY);
	}
}


void turn(int direction)
{
	//set both motors to the correct turn speed
	leftMotor->setSpeed(TURN_SPEED);
	rightMotor->setSpeed(TURN_SPEED);
	
	//if turning left, left motor is backward and right motor is forward
	//if turning right, left motor is forward and right motor is backward
	leftMotor->run((direction == LEFT) ? BACKWARD : FORWARD);
	rightMotor->run((direction == LEFT) ? FORWARD : BACKWARD);

	//reset the encoder values
	PololuWheelEncoders::getCountsAndResetM1();
	PololuWheelEncoders::getCountsAndResetM2();
	//LeftWheelEncoder.write(0);
	//RightWheelEncoder.write(0);

	//we are basing our turns based on how far the right encoder has changed
	//so set that to 0
	rightEncoderValue = 0;
	if(direction == LEFT)
	{
		//right wheel is moving forward
		while(rightEncoderValue < TURN_NUMBER_LEFT)
		{
			//update the rightEncoderValue to whatever it is
			rightEncoderValue = PololuWheelEncoders::getCountsM2();
		}
	}
	else
	{
		//right wheel is moving backward
		while(rightEncoderValue > TURN_NUMBER_RIGHT)
		{
			//update the rightEncoderValue to whatever it is
			rightEncoderValue = PololuWheelEncoders::getCountsM2();
		}
	}
	
	stop();
	//delay half a second
	delay(500);
	//update the current direction to whatever was left or right of the old direction
	currentDirection = getDirection(direction);
}

//this was just for testing to pring a map of the distances of the cells
void printMap()
{
	for (int i = 0; i < MAZE_LENGTH; i++)
	{
		for (int j = 0; j < MAZE_LENGTH; j++)
		{
			Serial.print(maze[i][j].distance);
			Serial.print(" ");
		}
		Serial.println("");
	}
}





//updateds the cells to say if there are walls on the various sides of the cell
void updateWalls()
{
	//check the front sensor and update the wall if there is one there
	if (getSonarDistance() <= CHECK_DISTANCE_FORWARD)
	{
		updateWall(currentDirection);
	}
	//check the left sensor and update the wall if there is one there
	if (leftIrSensor.getDistanceCentimeter() <= CHECK_DISTANCE_LEFT)
	{
		updateWall(getDirection(LEFT));
	}
	//check the right sensor and update the wall if there is one there
	if (rightIrSensor.getDistanceCentimeter() <= CHECK_DISTANCE_RIGHT)
	{
		updateWall(getDirection(RIGHT));
	}
}

void updateWall(byte direction)
{
	//enables the wall exists bit in the current cell for the current direction
	currentCell->data |= direction;
}

//returns the direction which is left, right, or opposite of the current direction
byte getDirection(byte direction)
{
	if (direction == LEFT)
	{
		switch (currentDirection)
		{
			case IS_NORTH:
				return IS_WEST;
			case IS_EAST:
				return IS_NORTH;
			case IS_SOUTH:
				return IS_EAST;
			case IS_WEST:
				return IS_SOUTH;
		}
	}
	else if (direction == RIGHT)
	{
		switch (currentDirection)
		{
			case IS_NORTH:
				return IS_EAST;
			case IS_EAST:
				return IS_SOUTH;
			case IS_SOUTH:
				return IS_WEST;
			case IS_WEST:
				return IS_NORTH;
		}
	}	
	else//turn around
	{
		switch (currentDirection)
		{
		case IS_NORTH:
			return IS_SOUTH;
		case IS_EAST:
			return IS_WEST;
		case IS_SOUTH:
			return IS_NORTH;
		case IS_WEST:
			return IS_EAST;
		}
	}
}

byte getBestDirection()
{
	//if the cell ahead of the robot is open and the distance is shorter than
	//the current distance, and hasn't been explored go there
	if (!(currentCell->data & currentDirection) && currentCell->distance > getCellDistance(currentDirection) && !getIsCellExplored(currentDirection))
	{
		return currentDirection;
	}

	//if the cell to the right is open and the distance is shorter than the current distance, and hasn't been explored go there
	if (!(currentCell->data & getDirection(RIGHT)) && currentCell->distance > getCellDistance(getDirection(RIGHT)) && !getIsCellExplored(currentDirection))
	{
		return getDirection(RIGHT);
	}

	//if the cell to the left is open and the distance is shorter than the current distance, and hasn't been explored go there
	if (!(currentCell->data & getDirection(LEFT)) && currentCell->distance > getCellDistance(getDirection(LEFT)) && !getIsCellExplored(currentDirection))
	{
		return getDirection(LEFT);
	}

	//if the cell ahead of the robot is open and the distance is shorter than
	//the current distance, go there
	if (!(currentCell->data & currentDirection) && currentCell->distance > getCellDistance(currentDirection))
	{
		return currentDirection;
	}

	//if the cell to the right is open and the distance is shorter than the current distance, go there
	if (!(currentCell->data & getDirection(RIGHT)) && currentCell->distance > getCellDistance(getDirection(RIGHT)))
	{
		return getDirection(RIGHT);
	}

	//if the cell to the left is open and the distance is shorter than the current distance, go there
	if (!(currentCell->data & getDirection(LEFT)) && currentCell->distance > getCellDistance(getDirection(LEFT)))
	{
		return getDirection(LEFT);
	}

	//if the cell ahead of the robot is open go there
	if (!(currentCell->data & currentDirection) && currentCell->distance > getCellDistance(currentDirection))
	{
		return currentDirection;
	}

	//if the cell to the right is open go there
	if (!(currentCell->data & getDirection(RIGHT)) && currentCell->distance > getCellDistance(getDirection(RIGHT)))
	{
		return getDirection(RIGHT);
	}

	//if the cell to the left is open go there
	if (!(currentCell->data & getDirection(LEFT)) && currentCell->distance > getCellDistance(getDirection(LEFT)))
	{
		return getDirection(LEFT);
	}
	
	return getDirection(OPPOSITE);
}
bool getIsCellExplored(byte direction)
{
	byte x = (currentCell->position >> X_SHIFT);
	byte y = (currentCell->position & Y_ONLY);
	switch (direction)
	{
	case IS_NORTH:
		y++;
		break;
	case IS_EAST:
		x++;
		break;
	case IS_SOUTH:
		y--;
		break;
	case IS_WEST:
		x--;
		break;
	}
	return maze[x][y].data & IS_EXPLORED;
}
byte getCellDistance(byte direction)
{
	byte x = (currentCell->position >> X_SHIFT);
	byte y = (currentCell->position & Y_ONLY);
	switch (direction)
	{
		case IS_NORTH:
			y++;
			if (y == MAZE_LENGTH)
			{
				return USELESS;
			}
			break;
		case IS_EAST:
			x++;
			if (x == MAZE_LENGTH)
			{
				return USELESS;
			}
			break;
		case IS_SOUTH:
			if (y == 0)
			{
				return USELESS;
			}
			y--;
			break;
		case IS_WEST:
			if (x == 0)
			{
				return USELESS;
			}
			x--;
			break;
	}
	return maze[x][y].distance;
}

void move(byte direction)
{
	if (currentDirection != direction)
	{
		if (getDirection(LEFT) == direction)
		{
			turn(LEFT);
		}
		else if (getDirection(RIGHT) == direction)
		{
			turn(RIGHT);
		}
		else
		{
			//turn around
			turn(LEFT);
			turn(LEFT);
		}
	}
	forwardDistance = getSonarDistance();
	int goalDistance = forwardDistance - CHECK_DISTANCE_FORWARD;
        leftEncoderValue = 0;
        rightEncoderValue = 0;
	//while (!(forwardDistance < goalDistance && leftEncoderValue >= LEFT_ENCODER_TICKS_FORWARD && rightEncoderValue >= RIGHT_ENCODER_TICKS_FORWARD))
	while ((leftEncoderValue < LEFT_ENCODER_TICKS_FORWARD && rightEncoderValue < RIGHT_ENCODER_TICKS_FORWARD))
	{
                Serial.println(leftEncoderValue);
                Serial.println(rightEncoderValue);
		driveStraight();
	}		
	stop();

	byte x = (currentCell->position >> X_SHIFT);
	byte y = (currentCell->position & Y_ONLY);
	switch (currentDirection)
	{
		case IS_NORTH:
			y++;
			break;
		case IS_EAST:
			x++;			
			break;
		case IS_SOUTH:
			y--;
			break;
		case IS_WEST:
			x--;
			break;
	}
	currentCell = &maze[x][y];
	if (stack.contains(currentCell))
	{
		Cell* temp;
		//keep popping cell off stack untill it finds the cell and put it back on
		//this gets rid of the unneccessary cells like in a dead end
		//I am pretty sure I spelled ^ wrong
		while (stack.pop() != currentCell);
	}
	currentCell->data |= IS_EXPLORED;
	stack.push(currentCell);
}
