#include <Adafruit_NeoPixel.h>

//boolean to check which game is active, true is obstacles, false is blocks.
boolean game1 = false;

//boolean to check if the game is finished
boolean finished = true;

//boolean to check if pre game move has been done
boolean preGame = false;

//boolean to check if the post game move has been done
boolean postGame = false;

//boolean for start of game delay
boolean startDelay = true;

//If the longMovement is in progress, and if the pulse counters for the longMovement function needs to be counted
boolean longMovementComplete = true;

//If the longMovement is paused, and if the pulse counters for the longMovement function needs to be counted
boolean longMovementPause = true;

//these booleans check if target destination has been reached
boolean targetAComplete = false;
boolean targetBComplete = false;
boolean targetCComplete = false;

//check if the robot is retrieving blocks (bringing blocks back to dropoffs)
boolean retrieval = true;

//this boolean checks if the robot should return to last know position
boolean returnToLastKnowPosition = false;

//assigns pins to motor control
const int leftMotor = 6;
const int leftMotorReverse = 5;
const int rightMotor = 10;
const int rightMotorReverse = 9;

//assigns pin to green test led
const int testLed = 13;

//assings interrupt pins to rotationoutput
const int leftWheelInt = 3;
const int rightWheelInt = 2;

//the pins for the neopixel leds
const int neoPin = 7;
const int neoNumber = 4;

//assigns pins to Ultra Sonic Sensor
const int sonicTrig = 11;
const int sonicEcho = 12;

//assign pin to servo motor
const int servoPin = 8;

// line sensor pins
const int analogIn0 = A0;
const int analogIn1 = A1;
const int analogIn2 = A2;
const int analogIn3 = A3;
const int analogIn4 = A4;
const int analogIn5 = A5;
const int analogIn6 = A6;
const int analogIn7 = A7;

//default motor power
const int LEFT_MOTOR_POWER = 180;
const int RIGHT_MOTOR_POWER = 180;

//adjusts how much the longMovement turns should be limited, is counted in rotation sensor pulses
const int TURN_ADJUSTMENT = 2;

//adjustable motor power
int leftMotorPower = 0;
int rightMotorPower = 0;

//rotationsensor counters
int leftWheelCounter = 0;
int rightWheelCounter = 0;

//rotationsensor counters that are never resetted
int posLeftWheelCounter = 0;
int posRightWheelCounter = 0;

//these variables are the pulse counters for the longMovement Function
int longMovementDistanceLeft = 0;
int longMovementDistanceRight = 0;

//this variable is the desired pulse count the longMovement function wants to drive
int longMovementDistanceTarget = 0;

//set duration and distance for USSensor to 0
long duration = 0;
int distance = 0;

//millis for USSensor
unsigned long previousMillis1 = 0;

//millis for tracking location
unsigned long previousMillis2 = 0;

//interval between USSensor pulses
const long interval1 = 500;

//interval between tracking location function
const long interval2 = 500;

//variables for keeping track of the position of the robot
int counterDifference = 0;

int prevCounter = 0;

//the int hold the start position of the robot
const int startPositionX = 7;
const int startPositionY = 7;

//these variables keep track of the location of the robot.
//Position 0,0 is the bottom left cornor of the playing field and is meassured in centimeters
double positionX = 0;
double positionY = 0;

//the compass heading of the robot
double compass = 0;

//these varaibles are used to return to the last know position on the search pattern
double lastKnowX = 0;
double lastKnowY = 0;
double lastKnowCompas = 0;

//positions where the bot has to dropOff at the end of the game
const int dropOffX = 0;
const int dropOffY = 230;

//these variables hold the location of the position of the current target
double targetPositionX = 0;
double targetPositionY = 0;

//the position of the 3 targets
const double A1X = 50;
const double A1Y = 170;
const double B1X = 170;
const double B1Y = 100;
const double C1X = 100;
const double C1Y = 30;

//the starting position
const double DX = 40;
const double DY = 15;

//the position of the 3 dropoff points.
const double A2X = 80;
const double A2Y = 5;
const double B2X = 120;
const double B2Y = 5;
const double C2X = 160;
const double C2Y = 5;

//variables that hold the positions of the targets of the current game
double AX = 0;
double AY = 0;
double BX = 0;
double BY = 0;
double CX = 0;
double CY = 0;

Adafruit_NeoPixel neopixels(neoNumber, neoPin, NEO_GRB + NEO_KHZ800);

byte BL = 0;
byte BR = 1;
byte FR = 2;
byte FL = 3;

void setup() {
  //the pinmode assignment of the pins
  pinMode(leftMotor, OUTPUT);
  pinMode(leftMotorReverse, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(rightMotorReverse, OUTPUT);
  pinMode(sonicTrig, OUTPUT);
  pinMode(sonicEcho, INPUT);
  pinMode(testLed, OUTPUT);

  digitalWrite(testLed, HIGH);

  neopixels.begin();

  attachInterrupt(digitalPinToInterrupt(leftWheelInt), int0_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheelInt), int1_ISR, RISING);

  Serial.begin(9600);

  checkGame();
//  finished = false;
//  preGame = true;
}



// main program
void loop() {
  digitalWrite(testLed, HIGH);
  //checks if it should run a game or a pre or post game move
  if (!finished) {
    //a small delay at the start of a game so you don't touch the robot when it starts
    if (startDelay) {
      delay(500);
      resetCounters();
      startDelay = false;
    }
    //Millis function that times 500 ms between ultra sonic sensor pulses, and dodges when an object is close
    if (!retrieval && !game1) {
      trackPosition();
      searchPattern();
    } else {

    }
    //if the obstacle game is running or the get blocks game is running and the retrieval routine isn't active
    if (game1 || (!game1 && !retrieval)) {
      unsigned long currentMillis = millis();
      if ((currentMillis - previousMillis1) >= interval1) {
        previousMillis1 = millis();
        USSensor();
        if (distance < 20) {
          if (game1) {
            obstacleDodge();
          } else if (!retrieval) {
            getBlock(distance);
          }
        }
      }
    }
    ledColor();
    longMovement(0);
    delay(5);
  } else if (!preGame) {
    //set the args to the horizontal en vertical distance from the drop-off point to the inside of the left bottom of the playing field
        preGameMove(10, 10);
  } else if (!postGame) {
    //alter the constants of the drop-off point after the game, to the correct position
        postGameMove();
  }
}

//a function that determines if game obstacles is finished
void obstaclesFinished() {
  if (targetCComplete && targetBComplete && targetAComplete && positionX == AX && positionY == AY) {
    finished = true;
  }
}

//a move from the relay wand drop-off point to our start position
void preGameMove(int toStartX, int toStartY) {
  gripperClose();
  int moveX = 0;
  int moveY = 0;
  moveX = startPositionX + toStartX;
  moveY = startPositionY + toStartY;
  resetCounters();
  movement(0, moveY, moveY, 0);
  preMovement(500);
  movement(3, 8, 7, -50);
  movement(0, moveX, moveX, 0);
  //in the blocks game the search pattern is from left to right, in the obstacle game it should start straight
  if (game1) {
    preMovement(500);
    movement(2, 7, 8, -50);
  } else {
    compass = 90;
  }
  basicNavigation(4);
  finished = false;
  preGame = true;
}

//when the robot is done with a game, movements should be made to present the relay wand to the next group
void postGameMove() {
  //postGame = false;
  do {
    longMovement(0);
  } while (!longMovementComplete && !postGame);
  gripperOpen();
  resetCounters();
  preMovement(200);
  movement(1, 15, 15, 0);
  basicNavigation(4);
}

//a simple motor power adjustment
void motorAdjust() {
  int counterDifference = 0;
  counterDifference = leftWheelCounter - rightWheelCounter;
  if (counterDifference > 4) {
    leftMotorPower *= 0.98;
  } else if (counterDifference < -4) {
    rightMotorPower *= 0.98;
  }
  if (leftMotorPower < (LEFT_MOTOR_POWER * 0.90) || rightMotorPower < (RIGHT_MOTOR_POWER * 0.90)) {
    resetMotorPower(0);
  }
}

//checks which game is active and sets variables accordingly
void checkGame() {
  if (!game1) {
    AX = A2X;
    AY = A2Y;
    BX = B2X;
    BY = B2Y;
    CX = C2X;
    CY = C2Y;
    retrieval = false;
  } else {
    positionX = startPositionX;
    positionY = startPositionY;
    AX = A1X;
    AY = A1Y;
    BX = B1X;
    BY = B1Y;
    CX = C1X;
    CY = C1Y;
    retrieval = true;
  }
}

// variables for target, sets next target if previous target has been completed
void targetPosition() {
  if (returnToLastKnowPosition) {
    targetPositionX = lastKnowX;
    targetPositionY = lastKnowY;
  } else if (!targetAComplete) {
    targetPositionX = AX;
    targetPositionY = AY;
  } else if (!targetBComplete) {
    targetPositionX = BX;
    targetPositionY = BY;
  } else if (!targetCComplete) {
    targetPositionX = CX;
    targetPositionY = CY;
  } else if (!finished && game1) {
    targetPositionX = AX;
    targetPositionY = AY;
  } else {
    targetPositionX = dropOffX;
    targetPositionY = dropOffY;
  }
}

// variables and adjustments for targets
void targetAndPositionAdjustment() {
  if (returnToLastKnowPosition) {
    positionX = lastKnowX;
    positionY = lastKnowY;
    //returnToLastKnowPosition = false;
  } else if (!targetAComplete) {
    positionX = AX;
    positionY = AY;
    targetAComplete = true;
  } else if (!targetBComplete) {
    positionX = BX;
    positionY = BY;
    targetBComplete = true;
  } else if (!targetCComplete) {
    positionX = CX;
    positionY = CY;
    targetCComplete = true;
  } else {
    positionX = AX;
    positionY = AY;
  }
}

// A function that let's the robot either, go straight, go back, turn left, turn right or stop altogether.
void basicNavigation(int basicNav) {
  switch (basicNav) {
    case 0: // forwards
      analogWrite(leftMotor, leftMotorPower);
      analogWrite(rightMotor, rightMotorPower);
      analogWrite(leftMotorReverse, 0);
      analogWrite(rightMotorReverse, 0);
      break;
    case 1: // backwards
      analogWrite(leftMotorReverse, leftMotorPower);
      analogWrite(rightMotorReverse, rightMotorPower);
      analogWrite(leftMotor, 0);
      analogWrite(rightMotor, 0);
      break;
    case 2: // turn left
      analogWrite(leftMotorReverse, leftMotorPower);
      analogWrite(rightMotor, rightMotorPower);
      analogWrite(leftMotor, 0);
      analogWrite(rightMotorReverse, 0);
      break;
    case 3: // turn right
      analogWrite(leftMotor, leftMotorPower);
      analogWrite(rightMotorReverse, rightMotorPower);
      analogWrite(leftMotorReverse, 0);
      analogWrite(rightMotor, 0);
      break;
    case 4: // stop
      analogWrite(leftMotor, 0);
      analogWrite(rightMotor, 0);
      analogWrite(leftMotorReverse, 0);
      analogWrite(rightMotorReverse, 0);
      break;
  }
}

//returns the motorPower to the predeterment amount
void resetMotorPower(int limiter) {
  leftMotorPower = LEFT_MOTOR_POWER;
  rightMotorPower = RIGHT_MOTOR_POWER;
  leftMotorPower -= limiter;
  rightMotorPower -= limiter;
}

//resets the rotationsensor counters
void resetCounters() {
  leftWheelCounter = 0;
  rightWheelCounter = 0;
}

//a function that lets the robot make a basic move for a certain distances
//distanceLeftWheel and distanceRightWheel are the amount of pulse from the rotation sensors that need to be achieved.
void movement(int basicNavInputMovement, int distanceLeftWheel, int distanceRightWheel, int movementLimiter) {
  resetMotorPower(movementLimiter);
  while (leftWheelCounter < distanceLeftWheel || rightWheelCounter < distanceRightWheel) {
    basicNavigation(basicNavInputMovement);
  }
  resetCounters();
}

//a function that stops the robot for a short period of time
void preMovement(int delayLength) {
  resetMotorPower(0);
  basicNavigation(4);
  delay(delayLength);
}

//a set of commands that makes the robot drive around an obstacle
void obstacleDodge() {
  longMovementPause = true;
  resetCounters();
  preMovement(500);
  movement(2, 7, 8, -50);
  movement(0, 20, 20, 0);
  preMovement(500);
  movement(3, 8, 7, -50);
  movement(0, 50, 50, 0);
  preMovement(500);
  movement(3, 8, 7, -50);
  movement(0, 20, 20, 0);
  preMovement(500);
  movement(2, 7, 8, -50);
  basicNavigation(4);
  longMovementDistanceLeft += 50;
  longMovementDistanceRight += 50;
  delay(200);
  Serial.print(longMovementPause); //LEAVE THIS SERIAL PRINT HERE, OTHERSWISE THE FUNCTION BREAKS!!!!!!
  longMovementPause = false;
}

//this function let's the robot drive for a determined distance without the use of a loop, so the program can continiue
void longMovement(int basicNavInputLongMovement) {
  if (!retrieval) {
    resetMotorPower(0);
    searchPattern();
    return;
  }
  if (longMovementPause) {
    resetMotorPower(0);
    longMovementPause = false;
  }
  setLongMovementTarget();
  basicNavigation(basicNavInputLongMovement);
  if ((longMovementDistanceLeft > longMovementDistanceTarget && longMovementDistanceRight > longMovementDistanceTarget) && !longMovementComplete) {
    basicNavigation(4);

    targetAndPositionAdjustment();
    longMovementComplete = true;
    longMovementPause = true;
    if (finished) {
      postGame = true;
      return;
    }
    obstaclesFinished();
    if (!game1 && !returnToLastKnowPosition) {
      dropBlock();
    }
  }
}

//sets the target distance the robot needs to drive for the longMovement function
void setLongMovementTarget() {
  if (longMovementComplete) {
    double targetDifferenceX = 0;
    double targetDifferenceY = 0;
    targetPosition();
    targetDifferenceX = targetPositionX - positionX;
    targetDifferenceY = targetPositionY - positionY;
    setLongMovementTargetHeading(targetDifferenceX, targetDifferenceY);
    longMovementDistanceTarget = pythagoras(targetDifferenceX, targetDifferenceY);
    longMovementComplete = false;
  }
}

//determines the in what angle the robot should drive to achieve it's target, and moves accordingly
void setLongMovementTargetHeading(double distanceToX, double distanceToY) {
  double rotationAngle = 0;
  double turnAngle = 0;
  rotationAngle = rotationHeading(distanceToX, distanceToY);
  if (rotationAngle < 0) {
    rotationAngle = 360 + rotationAngle;
  }
  turnAngle = rotationAngle - compass;
  setCompassHeading(rotationAngle);
  double turnDistance;
  turnDistance = calcTurn(turnAngle);
  preMovement(500);
  int turnDistanceInt = 0;
  turnDistanceInt = turnDistance + 0.5;
  resetCounters();
  if (turnDistanceInt > 1) {
    turnDistanceInt -= TURN_ADJUSTMENT;
  }
  if (turnAngle > 0) {
    movement(3, turnDistanceInt, turnDistanceInt, -50);
  } else if (turnAngle < 0) {
    turnDistanceInt *= -1;
    movement(2, turnDistanceInt, turnDistanceInt, -50);
  }
  longMovementDistanceLeft = 0;
  longMovementDistanceRight = 0;
}

//a function that keeps track of the position based on rotation sensors
void trackPosition() {
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis2) >= interval2) {
    previousMillis2 = millis();
    if (posLeftWheelCounter < posRightWheelCounter) {
      counterDifference = (posLeftWheelCounter - prevCounter);
    } else {
      counterDifference = (posRightWheelCounter - prevCounter);
    }
    if (compass == 90) {
      positionX += counterDifference;
    } else if (compass == 270) {
      positionX -= counterDifference;
    }
    if (posLeftWheelCounter < posRightWheelCounter) {
      prevCounter = posLeftWheelCounter;
    } else {
      prevCounter = posRightWheelCounter;
    }
  }
}

// Search pattern
void searchPattern() {

  // variables for sensors
  int s1 = analogRead(analogIn0);
  int s8 = analogRead(analogIn7);

  //if sensor value is above 800, while driving to the left, make a right U turn
  if (s1 > 800 && s8 > 800 && compass == 270) {
    basicNavigation(4);
    movement(1, 2, 2, 0);
    preMovement(500);
    movement(3, 7, 7, -50);
    movement(0, 7, 7, 0);
    preMovement(500);
    movement(3, 7, 7, -50);
    compass = 90;
    positionX = 10;
    positionY += 12;

  }
  //if sensor value is above 800, while driving to the right, make a left U turn
  else if (s1 > 800 && s8 > 800 && compass == 90) {
    basicNavigation(4);
    movement(1, 2, 2, 0);
    preMovement(500);
    movement(2, 7, 7, -50);
    movement(0, 7, 7, 0);
    preMovement(500);
    movement(2, 7, 7, -50);
    compass = 270;
    positionX = 290;
    positionY += 12;
  }
  // move forward if nothing is detected
  else {
    basicNavigation(0);
    motorAdjust();
  }
}

//a function that activates the ultra sonic sensor
void USSensor() {
  digitalWrite(sonicTrig, LOW);
  delayMicroseconds(2);

  digitalWrite(sonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonicTrig, LOW);

  duration = pulseIn(sonicEcho, HIGH);
  distance = duration * 34 / 2000;
}

//function to use the servo to close the gripper
void gripperClose() {
  for (int pulseCounter = 0; pulseCounter <= 10; pulseCounter++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(1200);
    digitalWrite(servoPin, LOW);
    delay(20); // between pulses
  }
}

//function to use the servo to open the gripper
void gripperOpen() {
  for (int pulseCounter = 0; pulseCounter <= 10; pulseCounter++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(servoPin, LOW);
    delay(20); // between pulses
  }
}

// function to get the block
void getBlock(int distance) {
  resetCounters();
  distance -= 3;
  basicNavigation(4);
  gripperOpen();
  preMovement(50);
  movement(0, distance, distance, 0);
  basicNavigation(4);
  gripperClose();
  lastKnowX = positionX;
  lastKnowY = positionY;
  lastKnowCompas = compass;
  retrieval = true;
}

// function to drop block
void dropBlock() {
  gripperOpen();
  returnToLastKnowPosition = true;
  do {
    longMovement(0);
  } while (!longMovementComplete);
  returnToLastKnowPosition = false;
  double turnAngle = 0;
  turnAngle = lastKnowCompas - compass;
  setCompassHeading(lastKnowCompas);
  double turnDistance;
  turnDistance = calcTurn(turnAngle);
  preMovement(500);
  int turnDistanceInt = 0;
  turnDistanceInt = turnDistance + 0.5;
  resetCounters();
  if (turnDistanceInt > 1) {
    turnDistanceInt -= TURN_ADJUSTMENT;
  }
  if (turnAngle > 0) {
    movement(3, turnDistanceInt, turnDistanceInt, -50);
  } else if (turnAngle < 0) {
    turnDistanceInt *= -1;
    movement(2, turnDistanceInt, turnDistanceInt, -50);
  }
  longMovementDistanceLeft = 0;
  longMovementDistanceRight = 0;
  retrieval = false;
}

void ledColor() {
  if (!game1) {
    neopixels.setPixelColor(FR, neopixels.Color(0, 255, 0));
    neopixels.setPixelColor(FL, neopixels.Color(0, 255, 0));
    neopixels.setPixelColor(BR, neopixels.Color(0, 255, 0));
    neopixels.setPixelColor(BL, neopixels.Color(0, 255, 0));
    neopixels.show();
  } else {
    neopixels.setPixelColor(FR, neopixels.Color(0, 0, 255));
    neopixels.setPixelColor(FL, neopixels.Color(0, 0, 255));
    neopixels.setPixelColor(BR, neopixels.Color(0, 0, 255));
    neopixels.setPixelColor(BL, neopixels.Color(0, 0, 255));
    neopixels.show();
  }
}

// math functions to determine heading and calculate turn radius
double rotationHeading(double targetX, double targetY) {
  double result = 0;
  result = (atan2(targetX, targetY) * 4068) / 71;
  return result;
}

double calcTurn(double targetTurn) {
  double result = 0;
  result = 0.09444 * targetTurn;
  return result;
}

double pythagoras(double distanceToX, double distanceToY) {
  double result = 0;
  result = sqrt((distanceToX * distanceToX) + (distanceToY * distanceToY));
  return result;
}

// compass heading 0-360 degrees
void setCompassHeading(int adjustment) {
  compass = adjustment;
  if (compass == 360) {
    compass = 0;
  } else if (compass > 360) {
    compass -= 360;
  }
}

//interrupt service routine that counts wheel rotations
void int0_ISR() {
  leftWheelCounter++;
  posLeftWheelCounter++;
  if ((!longMovementComplete) && (!longMovementPause)) {
    longMovementDistanceLeft++;
  }
}

//interrupt service routine that counts wheel rotations
void int1_ISR() {
  rightWheelCounter++;
  posRightWheelCounter++;
  if ((!longMovementComplete) && (!longMovementPause)) {
    longMovementDistanceRight++;
  }
}
