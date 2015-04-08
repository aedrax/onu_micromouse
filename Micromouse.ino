#include "QueueList.h"
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
int LEFT_ENCODER_TICKS_FORWARD;
int RIGHT_ENCODER_TICKS_FORWARD;
int leftEncoder;
int rightEncoder;
#define CHECK_DISTANCE_FORWARD 12 //centimeters forward
#define CHECK_DISTANCE_LEFT 10
#define CHECK_DISTANCE_RIGHT 10


#define ISNORTH 1		//0b00000001
#define ISEAST 2		//0b00000010
#define ISSOUTH 4		//0b00000100
#define ISWEST 8		//0b00001000
#define ISEXPLORED 16	//0b00010000
#define Y_ONLY 15		//0b00001111
#define SHIFT 4 //shift 4 bits

struct Cell
{
	//byte x;
	//byte y;
	byte position; //positions are stored as0bXXXXYYYY
	byte distance;
	byte data;
};

//#include "QueueArray.h"

#define MAZE_LENGTH 16
Cell maze[MAZE_LENGTH][MAZE_LENGTH];
bool turbo = false;
QueueList<byte> moveQueue;
byte useless = 255;
bool TO_CENTER = true;
bool TO_START = false;
bool Goal;
byte Xpos, Ypos;
#define North 1
#define East 2
#define West 3
#define South 4
#define Forward 1
#define RotateLeft 2
#define RotateRight 3
#define Spin 4
byte currentDir;
bool speedRun, speedRunCapable;





void setup()
{
	
	Serial.begin (9600);
	
	AFMS.begin();
	
	leftMotor->setSpeed(30);
	rightMotor->setSpeed(30);
	
	//PololuWheelEncoders::init(ENCODER_LEFT_B, ENCODER_LEFT_A, ENCODER_RIGHT_B, ENCODER_RIGHT_A);


	irLeft.begin(LEFT_IR_PIN);
	irRight.begin(RIGHT_IR_PIN);


	delay(5000);
	
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

	
	for(byte i = 0; i < MAZE_LENGTH; i++)
	{
		for(byte j = 0; j < MAZE_LENGTH; j++)
		{
		
			maze[i][j].position = ((j + 1) << SHIFT); //x position
			maze[i][j].position |= (i + 1);//y position
			maze[i][j].distance = useless;
			maze[i][j].data = 0x00;
			
		}
	}
	
	Goal = TO_CENTER;
	Xpos = 0;
	Ypos = 0;
	speedRun = false;
	speedRunCapable = false;
	currentDir = North;
}



void loop()
{
	Serial.println(F("qqqqq"));
	floodfill();
	byte nextMove = nextStep();
	Serial.println(F("RAAAA"));
	
	switch(nextMove)
	{
		
		case Forward:
			driveStraight();
			break;
		case RotateRight:
			turn(RIGHT);
			break;
		case RotateLeft:
			turn(LEFT);
			break;
		case Spin:
			turn(LEFT);
			turn(LEFT);
			break;
		default:
			stop();
	}
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
	forwardDistance = getSonarDistance();
	int goalDistance = forwardDistance - CHECK_DISTANCE_FORWARD;

	//while the robot has not moved forward one cell
	while(true)
	{
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
		forwardDistance = getSonarDistance();
		if(forwardDistance < goalDistance && leftEncoder >= LEFT_ENCODER_TICKS_FORWARD && rightEncoder >= RIGHT_ENCODER_TICKS_FORWARD)
			break;
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
			Serial.println(rightEncoder);

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
}














































































/////////floodfill

void floodfill()
{
	int currentDistance;
	
	QueueList<Cell*> queue;
	Cell* cell;
	boolean speedy;
	
	for(int i = 0; i < MAZE_LENGTH; i++)
	{
		for(int j = 0; j < MAZE_LENGTH; j++)
		{
			maze[i][j].distance = useless;
		}
	}
	
	if (Goal == TO_START)
	{
		maze[0][0].distance = 0;
		queue.push(&maze[0][0]);
		speedy = false;
	}
	else
	{
		byte center = MAZE_LENGTH / 2;
		maze[center][center].distance = 0;
		queue.push(&maze[center][center]);
		maze[center + 1][center].distance = 0;
		queue.push(&maze[center + 1][center]);
		maze[center][center + 1].distance = 0;
		queue.push(&maze[center][center + 1]);
		maze[center + 1][center + 1].distance = 0;
		queue.push(&maze[center + 1][center + 1]);
		if (speedRun && speedRunCapable)
		{
			speedy = true;
		}
		else
		{
			speedy = false;
		}
	}
	while (!queue.isEmpty())
	{
		cell = queue.pop();
		currentDistance = cell->distance;
		byte x = cell->position >> SHIFT;
		byte y = cell->position & Y_ONLY;
		Serial.println(queue.count());
		if (!(cell->data & ISNORTH))
		{
			if (((currentDistance + 1) < getNeighborDistance(x, y, North)) && ( !speedy || getNeighborExplored(x, y, North)))
			{
				maze[y - 1][(x)].distance = currentDistance + 1;
				queue.push(&maze[y - 1][x]);
			}
		}

		if (!(cell->data & ISSOUTH))
		{
			if (((currentDistance + 1) < getNeighborDistance(x, y, South)) && ( !speedy || getNeighborExplored(x, y, South)))
			{
				maze[(y) + 1][x].distance = currentDistance + 1;
				queue.push(&maze[(y) + 1][x]);
			}
		}
		if (!(cell->data & ISWEST))
		{
			if (((currentDistance + 1) < getNeighborDistance(x, y, West)) && ( !speedy || getNeighborExplored(x, y, West)))
			{
				maze[y][x - 1].distance = currentDistance + 1;
				queue.push(&maze[y][x - 1]);
			}
		}
		if (!(cell->data & ISEAST))
		{
			if (((currentDistance + 1) < getNeighborDistance(x, y, East)) && ( !speedy || getNeighborExplored(x, y, East)))
			{
				maze[y][x + 1].distance = currentDistance + 1;
				queue.push(&maze[y][x + 1]);
			}
		}
	}
	/*
	  MazeCell here = robotLocation.getCurrentLocation();
      if (getDistance(here) == USELESS)
      {
         //System.out.println("Purging Knowledge");
         maze.clearMaze();
         speedRunCapable = false;
         for (int i = 0; i < size.width; i++)
         {
            for (int j = 0; j < size.height; j++)
            {
               explored[i][j] = false;
            }
         }
         explored[here.getX() - 1][here.getY() - 1] = true;
         checkWalls();
         floodfill();
      }
      */
}


byte nextStep()
{
	byte next;
	byte nextDir;
	if (moveQueue.isEmpty())
	{
		if (!(maze[Ypos][Xpos].data & ISEXPLORED))
		{
			checkWalls();
			maze[Ypos][Xpos].data | ISEXPLORED;
		}
		if (atGoal())
		{
			if((Goal == TO_CENTER) && speedRunCapable == false)
			{
				speedRunCapable = true;
				//blockOutCenter();
			}
			Goal = !Goal;
			floodfill();
		}
		nextDir = GetBestDirection();
		turbo = getNeighborExplored(nextDir);
		if (nextDir == currentDir)
		{
			next = Forward;
		}
		else if (nextDir == getLeft())
		{
			next = RotateLeft;
			currentDir = getLeft();
			moveQueue.push(Forward);
		}
		else if (nextDir == getRight())
		{
			next = RotateRight;
			currentDir = getRight();
			moveQueue.push(Forward);
		}
		else
		{
			next = Spin;
			moveQueue.push(Forward);
		}
	}
	else
	{
		next = moveQueue.pop();
	}
	return next;
}



byte GetBestDirection()
{
	byte bestDis = maze[Ypos][Xpos].distance;
	byte bestDir = 0;
	if ((bestDis > getNeighborDistance(currentDir)) && !isWallAhead())
	{
		bestDir = currentDir;
	}

	if ((bestDis > getNeighborDistance(North)) && !getWall(North))
	{
		bestDir = North;
	}

	if ((bestDis > getNeighborDistance(East)) && !getWall(East))
	{
		bestDir = East;
	}

	if ((bestDis > getNeighborDistance(West)) && !getWall(West))
	{
		bestDir = West;
	}

	if ((bestDis > getNeighborDistance(South)) && !getWall(South))
	{
		bestDir = South;
		
	}
	bestDis = getNeighborDistance(bestDir);
	if (bestDir = 0)
	{
		floodfill();
		return GetBestDirection();
	}
	else
	{
		return bestDir;
	}
}

boolean getNeighborExplored(byte dir)
{
	return getNeighborExplored(Xpos, Ypos, dir);
}

boolean getNeighborExplored(byte X, byte Y, byte dir)
{
	boolean neighbor;
	if ((dir == North) && (Y != 1))
	{
		neighbor = (maze[Y - 1][X].data & ISEXPLORED);
	}
	else if ((dir == South) && (Y != MAZE_LENGTH))
	{
		neighbor = (maze[Y + 1][X].data & ISEXPLORED);
	}
	else if ((dir == East) && (X != MAZE_LENGTH))
	{
		neighbor = (maze[Y][X + 1].data & ISEXPLORED);
	}
	else if ((dir == West) && (X != 1))
	{
		neighbor = (maze[Y][X - 1].data & ISEXPLORED);
	}
	else
	{
		neighbor = false;
	}
	return neighbor;
}

 //
   //  Returns the distance of the MazeCell adjacent to the passed MazeCell and
    // is adjacent in the specified direction.
//
byte getNeighborDistance(byte dir)
{
	return getNeighborDistance(Xpos, Ypos, dir);
}

byte getNeighborDistance(byte X, byte Y, byte dir)
{
	byte neighborDis;
	if ((dir == North) && (Ypos != 1))
	{
		neighborDis = maze[Ypos - 1][Xpos].distance;
	}
	else if ((dir == South) && (Ypos != MAZE_LENGTH))
	{
		neighborDis = maze[Ypos + 1][Xpos].distance;
	}
	else if ((dir == East) && (Xpos != MAZE_LENGTH))
	{
		neighborDis = maze[Ypos][Xpos + 1].distance;
	}
	else if ((dir == West) && (Xpos != 1))
	{
		neighborDis = maze[Ypos][Xpos - 1].distance;
	}
	else
	{
		neighborDis = useless;
	}
	return neighborDis;
}

boolean getWall(byte dir)
{
	switch(currentDir)
	{
		case North:
			break;
		case East:
			switch(dir)
			{
				case North:
					dir = West;
					break;
				case East:
					dir = North;
					break;
				case South:
					dir = East;
					break;
				case West:
					dir = South;
					break;
			}
			break;
		case South:
			switch(dir)
			{
				case North:
					dir = South;
					break;
				case East:
					dir = West;
					break;
				case South:
					dir = North;
					break;
				case West:
					dir = East;
					break;
			}
			break;
		case West:
			switch(dir)
			{
				case North:
					dir = East;
					break;
				case East:
					dir = South;
					break;
				case South:
					dir = West;
					break;
				case West:
					dir = North;
					break;
			}
			break;
	}
	switch(dir)
	{
		case North:
			return (getSonarDistance() < CHECK_DISTANCE_FORWARD);
		case East:
			return (irRight.getDistanceCentimeter() < CHECK_DISTANCE_RIGHT);
		case West:
			return (irLeft.getDistanceCentimeter() < CHECK_DISTANCE_LEFT);
	}
	return false;
}

void checkWalls()
{
	if (getWall(North))
	{
		maze[Ypos][Xpos].data | ISNORTH;
	}
	if (getWall(South))
	{
		maze[Ypos][Xpos].data | ISSOUTH;
	}
	if (getWall(East))
	{
		maze[Ypos][Xpos].data | ISEAST;
	}
	if (getWall(West))
	{
		maze[Ypos][Xpos].data | ISWEST;
	}
}

boolean isWallAhead()
{
	return getWall(currentDir);
}

int getLeft()
{
	switch (currentDir)
	{
		case North :
			return West;
		case East :
			return North;
		case South :
			return East;
		case West :
			return South;
	}
}

int getRight()
{
	switch (currentDir)
    {
    	case North :
            return East;
        case East :
            return South;
        case South :
            return West;
        case West :
            return North;
  	}
}

boolean atGoal()
{
	byte center = MAZE_LENGTH / 2;
	if ((Goal == TO_START) && (Xpos == 1) && (Ypos == MAZE_LENGTH))
	{
		return true;
	}
	if ((Goal == TO_CENTER) && ((Xpos == center) || (Xpos == center + 1)) && ((Ypos == center) || (Ypos == center + 1)))
	{
		return true;
	}
	return false;
}
