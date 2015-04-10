#include "Cell.h"
#include "CellStack.h"
//#include "QueueList.h"
//#include "QueueArray.h"
#include <Wire.h>









/* Define pins for HC-SR04 ultrasonic sensor */
#define echoPin 6 // Echo Pin = Analog Pin 0
#define trigPin 7 // Trigger Pin = Analog Pin 1
#define LEDPin 13 // Onboard LED
long duration; // Duration used to calculate distance
long HR_dist = 0; // Calculated Distance
byte minimumRange = 5; //Minimum Sonar range
int maximumRange = 400; //Maximum Sonar Range







#include "Adafruit_MotorShield.h"
/* Using the motor shield */
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
#define FORWARD_SPEED 40




/* get the distances for the for the IR sensors */
#include "DistanceGP2Y0A41SK.h"

DistanceGP2Y0A41SK irLeft;
DistanceGP2Y0A41SK irRight;
#define LEFT_IR_PIN A0
#define RIGHT_IR_PIN A1
byte leftIR;
byte rightIR;
int forwardDistance;








/* encoders stuff */
//#include "PololuWheelEncoders.h"
#include "Encoder.h"
#define ENCODER_LEFT_A 18
#define ENCODER_LEFT_B 19
#define ENCODER_RIGHT_A 2
#define ENCODER_RIGHT_B 3
Encoder LeftWheelEncoder(ENCODER_LEFT_A, ENCODER_LEFT_B);
Encoder RightWheelEncoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

//wall_threshold is for driving and not hitting the wall
#define WALL_THRESHOLD_FORWARD 5 //cm from the the front sensor to up to
#define WALL_THRESHOLD_LEFT 7
#define WALL_THRESHOLD_RIGHT 7
#define TURN_NUMBER_LEFT 14 //number the encoder should check when turning left
#define TURN_NUMBER_RIGHT -17 //number the encoder should check when turning right
#define TURN_SPEED  30 //speed for turning
#define TURN_CORRECTION 30 //speed to turn one wheel on correction
#define TURN_CORRECTION_DELAY 50 //delay to turn for error in milliseconds
#define LEFT 0
#define RIGHT 1
#define BEHIND 2
int LEFT_ENCODER_TICKS_FORWARD;
int RIGHT_ENCODER_TICKS_FORWARD;
int leftEncoder;
int rightEncoder;
//check distance is to know if the wall is in the cell or not
#define CHECK_DISTANCE_FORWARD 12 //centimeters forward
#define CHECK_DISTANCE_LEFT 15
#define CHECK_DISTANCE_RIGHT 15


#define ISNORTH 1		//0b00000001
#define ISEAST 2		//0b00000010
#define ISSOUTH 4		//0b00000100
#define ISWEST 8		//0b00001000
#define ISEXPLORED 16	//0b00010000
#define NOTEXPLORED 239 //0b11101111
#define Y_ONLY 15		//0b00001111
#define SHIFT 4 //shift 4 bits

#include "Cell.h"

//#include "QueueArray.h"

#define MAZE_LENGTH 12
Cell maze[MAZE_LENGTH][MAZE_LENGTH];
Cell* currentCell;
CellStack stack;
byte currentDirection;
#define USELESS 255

void setup()
{
	
	Serial.begin (9600);
	
	AFMS.begin();
	
	leftMotor->setSpeed(30);
	rightMotor->setSpeed(30);
	
	//PololuWheelEncoders::init(ENCODER_LEFT_B, ENCODER_LEFT_A, ENCODER_RIGHT_B, ENCODER_RIGHT_A);


	irLeft.begin(LEFT_IR_PIN);
	irRight.begin(RIGHT_IR_PIN);


	//delay(5000);
	
	LEFT_ENCODER_TICKS_FORWARD = LeftWheelEncoder.read();//PololuWheelEncoders::getCountsAndResetM1();	
	RIGHT_ENCODER_TICKS_FORWARD = RightWheelEncoder.read();//PololuWheelEncoders::getCountsAndResetM2();
	LeftWheelEncoder.write(0);
	RightWheelEncoder.write(0);

	Serial.println(F("TICKS"));
	Serial.println(LEFT_ENCODER_TICKS_FORWARD);
	Serial.println(RIGHT_ENCODER_TICKS_FORWARD);
	//Setup the trigger and Echo pins of the HC-SR04 sensor
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)

	
	for (byte y = 0; y < MAZE_LENGTH; y++)
	{
		for(byte x = 0; x < MAZE_LENGTH; x++)
		{
		
			maze[x][y].position = (x << SHIFT); //x position
			maze[x][y].position |= (y);//y position
			maze[x][y].data = 0x00;
			
		}
	}
	//fill the distances
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
	printMap();
	currentDirection = ISNORTH;
	currentCell = &maze[0][0];
	currentCell->data |= ISEAST;
	currentCell->data |= ISWEST;
	currentCell->data |= ISSOUTH;
	currentCell->data |= ISEXPLORED;
	stack.push(currentCell);
	bool canMove = !(currentCell->data & currentDirection); 
	bool better = currentCell->distance > getCellDistance(currentDirection);
	bool isExplored = !getIsCellExplored(currentDirection);
	
}



void loop()
{
	while (currentCell->distance == 0);
	updateWalls();
	byte moveDirection = getBestDirection();
	move(moveDirection);
	
}

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
//Serial.println("0");
		digitalWrite(LEDPin, HIGH);
		return 0;
	}
	else
	{
		// Send the distance to the computer using Serial protocol, and
		//turn LED OFF to indicate successful reading. 
//Serial.println(HR_dist);
		digitalWrite(LEDPin, LOW);		
		return HR_dist;
	}
}


void driveStraight()
{
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
	
	leftEncoder = 0;
	rightEncoder = 0;

	int currentLeft;
	int currentRight;
	bool isCloseToWall = false;
	
	




	

	//while the robot has not moved forward one cell
	while(true)
	{
		forwardDistance = getSonarDistance();
		leftIR = irLeft.getDistanceCentimeter();
		rightIR = irRight.getDistanceCentimeter();


		if(forwardDistance < WALL_THRESHOLD_FORWARD || leftIR < WALL_THRESHOLD_LEFT || rightIR < WALL_THRESHOLD_RIGHT)
		{
			isCloseToWall = true;
			break;
		}


		//Set the motor powers to their respective variables.
		leftMotor->setSpeed(masterPower);
		rightMotor->setSpeed(slavePower);

		//This is where the magic happens. The error value is set as a scaled value representing the amount the slave
		//motor power needs to change. For example, if the left motor is moving faster than the right, then this will come
		//out as a positive number, meaning the right motor has to speed up.
		//Reset the encoders every loop so we have a fresh value to use to calculate the error.
		currentLeft = LeftWheelEncoder.read();//PololuWheelEncoders::getCountsAndResetM1();
		currentRight = RightWheelEncoder.read();//PololuWheelEncoders::getCountsAndResetM2();
		LeftWheelEncoder.write(0);
		RightWheelEncoder.write(0);

		leftEncoder += currentLeft;
		rightEncoder += currentRight;
		error = currentLeft - currentRight;

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
		if(leftIR < WALL_THRESHOLD_LEFT)
		{
			leftMotor->setSpeed(TURN_CORRECTION);
			rightMotor->run(RELEASE);//turn off the right motor
		}
		//the sensor detects it's approaching the right wall then turns left slightly
		else if(rightIR < WALL_THRESHOLD_RIGHT)
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
	leftMotor->setSpeed(TURN_SPEED);
	rightMotor->setSpeed(TURN_SPEED);
	leftMotor->run((direction == LEFT) ? BACKWARD : FORWARD);
	rightMotor->run((direction == LEFT) ? FORWARD : BACKWARD);

	//PololuWheelEncoders::getCountsAndResetM1();
	//PololuWheelEncoders::getCountsAndResetM2();
	LeftWheelEncoder.write(0);
	RightWheelEncoder.write(0);

	rightEncoder = 0;
	if(direction == LEFT)
	{
		//reset the encoder to 0

		//right wheel is moving forward
		while(rightEncoder < TURN_NUMBER_LEFT)
		{
			rightEncoder = RightWheelEncoder.read();//PololuWheelEncoders::getCountsM2();
			//Serial.println(rightEncoder);

		}
	}
	else
	{
		//left wheel is moving forward

		while(rightEncoder > TURN_NUMBER_RIGHT)
		{
			rightEncoder = RightWheelEncoder.read();//PololuWheelEncoders::getCountsM2();

		}
	}

	leftMotor->run(RELEASE);
	rightMotor->run(RELEASE);
	//delay half a second
	delay(500);
	currentDirection = getDirection(direction);
}


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






void updateWalls()
{
	if (getSonarDistance() <= WALL_THRESHOLD_FORWARD)
	{
		updateWall(currentDirection);
	}
	if (irLeft.getDistanceCentimeter() <= WALL_THRESHOLD_LEFT)
	{
		updateWall(getDirection(LEFT));
	}
	if (irRight.getDistanceCentimeter() <= WALL_THRESHOLD_RIGHT)
	{
		updateWall(getDirection(RIGHT));
	}
}

void updateWall(byte direction)
{
	currentCell->data |= direction;
}

byte getDirection(byte direction)
{
	if (direction == LEFT)
	{
		switch (currentDirection)
		{
			case ISNORTH:
				return ISWEST;
			case ISEAST:
				return ISNORTH;
			case ISSOUTH:
				return ISEAST;
			case ISWEST:
				return ISSOUTH;
		}
	}
	else if (direction == RIGHT)
	{
		switch (currentDirection)
		{
			case ISNORTH:
				return ISEAST;
			case ISEAST:
				return ISSOUTH;
			case ISSOUTH:
				return ISWEST;
			case ISWEST:
				return ISNORTH;
		}
	}	
	else//turn around
	{
		switch (currentDirection)
		{
		case ISNORTH:
			return ISSOUTH;
		case ISEAST:
			return ISWEST;
		case ISSOUTH:
			return ISNORTH;
		case ISWEST:
			return ISEAST;
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
	
	return getDirection(BEHIND);
}
bool getIsCellExplored(byte direction)
{
	byte x = (currentCell->position >> SHIFT);
	byte y = (currentCell->position & Y_ONLY);
	switch (direction)
	{
	case ISNORTH:
		y++;
		break;
	case ISEAST:
		x++;
		break;
	case ISSOUTH:
		y--;
		break;
	case ISWEST:
		x--;
		break;
	}
	return maze[x][y].data & ISEXPLORED;
}
byte getCellDistance(byte direction)
{
	byte x = (currentCell->position >> SHIFT);
	byte y = (currentCell->position & Y_ONLY);
	switch (direction)
	{
		case ISNORTH:
			y++;
			if (y == MAZE_LENGTH)
			{
				return USELESS;
			}
			break;
		case ISEAST:
			x++;
			if (x == MAZE_LENGTH)
			{
				return USELESS;
			}
			break;
		case ISSOUTH:
			if (y == 0)
			{
				return USELESS;
			}
			y--;
			break;
		case ISWEST:
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
	int goalDistance = forwardDistance - CHECK_DISTANCE_FORWARD;
	while (forwardDistance < goalDistance && leftEncoder >= LEFT_ENCODER_TICKS_FORWARD && rightEncoder >= RIGHT_ENCODER_TICKS_FORWARD)
	{
		driveStraight();
	}		
	stop();

	byte x = (currentCell->position >> SHIFT);
	byte y = (currentCell->position & Y_ONLY);
	switch (currentDirection)
	{
		case ISNORTH:
			y++;
			break;
		case ISEAST:
			x++;			
			break;
		case ISSOUTH:
			y--;
			break;
		case ISWEST:
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
	currentCell->data |= ISEXPLORED;
	stack.push(currentCell);
}