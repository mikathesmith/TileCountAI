#pragma config(Sensor, S1,     leftTouch,      sensorEV3_Touch)
#pragma config(Sensor, S2,     rightTouch,     sensorEV3_Touch)
#pragma config(Sensor, S3,     colour,         sensorEV3_Color)
#pragma config(Sensor, S4,     distance,       sensorEV3_Ultrasonic)
#pragma config(Motor,  motorB,          leftWheel,     tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorC,          rightWheel,    tmotorEV3_Large, PIDControl, encoder)
// Generated code - assign sensors and motors their name and port

/* COSC343 Artificial Intelligence
* Assignment 1 - Robot on a chessboard
*
* Matthew Boyes   - 8378715 - boyma492
* Alex McKirdy    - 5533438 - mckal165
* Matt Offen      - 6681486 - offma951
* Mika Smith      - 5761675 - smimi478
*/

////////////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////////////
int speed = 30;

// We expect the tower to be at least this close when sensing.
int const towerDistance = 156;

// Measured colours:
// Note that white is a higher value than black - setting it here means that
//   comparisons can be made when sensing the initial value.
int white = 0;
int black = 100;

// How much of a tolerance is allowed when sensing for the black colour (used in senseBlack).
int senseThreshold = 15;

// Sensor values - constantly updated
int currentCol;
float currentDis;


////////////////////////////////////////////////////////////////////////////////
// Scanning Methods
////////////////////////////////////////////////////////////////////////////////

// Constantly updates current colour.
task scanColour(){
	while(true){
		currentCol = SensorValue[colour];
		if(currentCol > white)
			white = currentCol; // Store white as the maximum colour value observed.
		if(currentCol < black)
			black = currentCol; // Store black as the minium colour value observed.
	}
}

// Constantly updates distance from tower.
task scanSonar(){
	while(true){
		currentDis = SensorValue[distance];
	}
}

// For debugging purposes, display all the stats.
task displayStats() {
	while (true) {
		displayCenteredBigTextLine(1, "Wht: %d Blk: %d", white, black);
		displayCenteredBigTextLine(3, "currentCol: %d", currentCol);
		displayCenteredBigTextLine(7, "L: %d R: %d", nMotorEncoder[leftWheel], nMotorEncoder[rightWheel]);
	}
}

// Returns whether the robot is currently on a black tile.
bool senseBlack(){
	return currentCol < black + senseThreshold;
}


// Stop movement by setting both motors to 0.
void stopMovement(){
	motor[leftWheel]  = 0;
	motor[rightWheel] = 0;
}

// Move forward by setting both wheels to the same speed for the same rotation.
void moveBoth(int spd, int rotations){
	nMotorEncoder[leftWheel] = 0;
	nMotorEncoder[rightWheel] = 0;

	while(nMotorEncoder[leftWheel]< rotations){
		motor[leftWheel] = spd;
		motor[rightWheel] = spd;
	}

	stopMovement();
}

// A 90 degree turn to the right.
void turn90(){
	int hold = 360; // Turn needed for the robot to move 90 degrees.
	nMotorEncoder[leftWheel] = 0;
	motor[leftWheel] = speed;
	while(nMotorEncoder[leftWheel] < hold) { } // Do nothing while Motor B is still running.
	motor[leftWheel] = 0;
	sleep(500);
	stopMovement();
}

// Method that drives forward off the S tile and turns at the first black.
void started(){
	motor[leftWheel] = speed;
	motor[rightWheel] = speed;

	while(senseBlack()) {} // Move forward until we can no longer sense black.

	sleep(500); // Give the robot time to move off the start block onto the grey tile.
	while(!senseBlack()) {} // Keep moving while the robot is not on black.

	stopMovement();
	sleep(500);

	playSound(soundBlip);
	turn90();
}

/* First part of the correct method:
* Turns until it no longer senses black, then returns to where it started
* Turned in to a function as it was being peformed on both motors
* Returns the encoder value (which will be however long it takes to leave black)
* CorrectPartOne scans the distance the tMotor parameter has to pivot until its off course: */
int correctPartOne(tMotor m) {
	stopMovement();
	int initial = nMotorEncoder[m]; // Remember the motor's current position.
	int encoderVal = 0;

	motor[m] = speed;
	while (senseBlack()) { // Update encoderVal with the last value before it's off course (no longer on black).
		encoderVal = nMotorEncoder[m] - initial;
	}
	stopMovement();
	motor[m] = -speed; // Reverse pivot the robot back around to where it started.
	while (nMotorEncoder[m] > initial) { }
	stopMovement();
	return encoderVal; // Return the encoderVal representing how far the robot is from going off the tile.
}

// CorrectPartTwo straightens the robot by pivoting the specified tMotor by the turnAmount.
void correctPartTwo(tMotor m, int turnAmount) {
	int initial = nMotorEncoder[m]; // Store the current motor's encoder.
	motor[m] = speed; // Start pivoting the robot.
	while(nMotorEncoder[m] < (initial + turnAmount)) { } // Keep pivoting until the specified tMotor has passed the turn amount.
	stopMovement();
}

// Correct centers the robot every black tile using CorrectPartOne and CorrectPartTwo.
void correct(){
	stopMovement();
	int encoderValueLeft = correctPartOne(leftWheel); // Get the distance the left wheel has to turn before it's off course.
	int encoderValueRight = correctPartOne(rightWheel); // Get the distance the right wheel has to turn before it's off course.
	int turnAmount = abs(encoderValueLeft - encoderValueRight) / 6; // Correct the robot by turning it one sixth of the difference between motor values.

	if(encoderValueLeft < encoderValueRight)
		correctPartTwo(rightWheel, turnAmount); // Pivot from the right wheel if the robot is too close to the left.
	else
		correctPartTwo(leftWheel, turnAmount); // Pivot from the left wheel if the robot is too close to the right.

	stopMovement();
}

// Counts the 15 black tiles.
void tileCount(){
	int count = 1; // The robot starts on a tile to start counting from 1
	bool passedTile = false;
	int currentPos;

	while(count < 15){ // Keep counting black tiles while the count is less than 15
		displayCenteredBigTextLine(5, "Tile Count: %d", count);

		motor[leftWheel] = speed;
		motor[rightWheel] = speed;

		currentPos = nMotorEncoder[leftWheel];
		if(!passedTile && senseBlack()){
			playSound(soundBlip);
			count++;
			if(count < 15)
				correct(); // Correct the robots angle on all tiles except the last
			passedTile = true;
		}
		else if(!senseBlack()){
			passedTile = false;
		}
	}
	stopMovement();
	displayCenteredBigTextLine(5, "DONE! %d", count);
}

//This method is to sense the tower.
//The loop will be broken once the current distance is less than our specified distance
//indicating the robot is facing the direction of the tower.
void senseTower(){
	while(currentDis > towerDistance){
		motor[leftWheel] = speed;
	}
	sleep(90); // Rotate for a little longer to get closer to the center of the tower.
	motor[leftWheel] = 0;
}

// Main method where functions and tasks are called in sequence.
task main(){
	startTask(scanColour);
	startTask(displayStats);

	started(); // Returns facing forward after the first black tile has been counted.
	sleep(500);

	tileCount(); // Returns after all 15 tiles have been counted.
	sleep(500);

	turn90(); // Returns after the robot has turned 90 degrees and is roughly facing the tower.
	motor[rightWheel] = 20;
	sleep(420); // Let the robot face slightly too far left to avoid the handle on the bottle.
	stopMovement();

	// Stop sensing colour as we've done the 15 black tiles.
	stopTask(scanColour);

	// Move closer to the tower
	moveBoth(speed, 3600);
	stopMovement();
	sleep(500);

	startTask(scanSonar); // Start scanning the sonar sensor.

	// While not touching the tower, call to sense tower
	while(SensorValue[leftTouch]!=1 && SensorValue[rightTouch]!=1){
		senseTower();
		moveBoth(speed, 360);
		stopMovement();
	}
	moveBoth(speed, 420); // Tower may already be knocked off at this stage, but this makes sure.
	stopMovement();
	playSound(soundFastUpwardTones);
	displayCenteredBigTextLine(2, "Finished Program!");
	sleep(5000);
}
