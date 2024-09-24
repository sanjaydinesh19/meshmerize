#include <QTRSensors.h>
#include <math.h>  

#define LEFT_MOTOR_PIN1 2
#define LEFT_MOTOR_PIN2 3
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 5
#define LEFT_ENCODER_PIN 18
#define RIGHT_ENCODER_PIN 19

#define KP 0.1
#define KI 0.001
#define KD 0.5


QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

int leftSpeed = 0;
int rightSpeed = 0;
const int baseSpeed = 150;


int lastError = 0;
int integral = 0;

char path[100];
int pathLength = 0;


#define MAX_VISITS 100
struct Intersection {
  int x, y;  
  char direction;  
};
Intersection visited[MAX_VISITS];
int visitCount = 0;
int posX = 0, posY = 0;  e
char currentDirection = 'N';  
void setup() {
  
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){30, 31, 32, 33, 34, 35, 36, 37}, SensorCount);

  calibrateSensors();
  Serial.begin(9600);
}

void loop() {
  uint16_t position = qtr.readLineWhite(sensorValues);

  int error = position - 3500;
  integral += error;
  int derivative = error - lastError;
  int motorSpeed = KP * error + KI * integral + KD * derivative;

  leftSpeed = baseSpeed + motorSpeed;
  rightSpeed = baseSpeed - motorSpeed;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

   
  int intersectionType = checkIntersection();
  handleIntersection(intersectionType);

  lastError = error;
}

void calibrateSensors() {
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20);
  }
}

void driveMotors(int left, int right) {
  analogWrite(LEFT_MOTOR_PIN1, left);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, right);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}

int checkIntersection() {
  bool left = sensorValues[0] > 900;
  bool right = sensorValues[7] > 900;
  bool straight = (sensorValues[3] > 900 || sensorValues[4] > 900);

  if (left && right && straight) return 7;  
  if (left && straight) return 6;
  if (right && straight) return 5;
  if (left && right) return 4;  
  if (left) return 3;
  if (right) return 2;
  if (straight) return 1;
  if (!left && !right && !straight) return 0;  
}

void handleIntersection(int intersectionType) {
  driveMotors(0, 0);
  delay(100);

  if (checkVisitedIntersection()) {
    // If the intersection was visited before, take an alternate path or backtrack
    backtrackOrAlternate();
  } else {
    switch (intersectionType) {
      case 7:
      case 6:
      case 4:
      case 3:
        turnLeft();
        path[pathLength++] = 'L';
        updatePosition('L');
        break;
      case 2:
        turnRight();
        path[pathLength++] = 'R';
        updatePosition('R');
        break;
      case 5:
      case 1:  // Straight path only
        // Continue straight
        path[pathLength++] = 'S';
        updatePosition('S');
        break;
      case 0:  // Dead end
        turnAround();
        path[pathLength++] = 'U';
        updatePosition('U');
        break;
    }
  }
}

bool checkVisitedIntersection() {
  // Check if this intersection was visited before
  for (int i = 0; i < visitCount; i++) {
    if (visited[i].x == posX && visited[i].y == posY && visited[i].direction == currentDirection) {
      return true;
    }
  }
  return false;
}

void backtrackOrAlternate() {
  // Decide to either backtrack or take a different path
  if (currentDirection == 'L') {
    turnRight();
    path[pathLength++] = 'R';
  } else if (currentDirection == 'R') {
    turnLeft();
    path[pathLength++] = 'L';
  } else {
    turnAround();
    path[pathLength++] = 'U';
  }
  updatePosition(path[pathLength - 1]);
}

void turnLeft() {
  // Turn left with PID control (as implemented previously)
}

void turnRight() {
  // Turn right with PID control (as implemented previously)
}

void turnAround() {
  // Perform a U-turn with PID control (as implemented previously)
}

void updatePosition(char move) {
  if (move == 'L') {
    if (currentDirection == 'N') posX--;
    else if (currentDirection == 'E') posY++;
    else if (currentDirection == 'S') posX++;
    else posY--;
    currentDirection = 'W';
  } else if (move == 'R') {
    if (currentDirection == 'N') posX++;
    else if (currentDirection == 'E') posY--;
    else if (currentDirection == 'S') posX--;
    else posY++;
    currentDirection = 'E';
  } else if (move == 'S') {
    if (currentDirection == 'N') posY++;
    else if (currentDirection == 'E') posX++;
    else if (currentDirection == 'S') posY--;
    else posX--;
  } else if (move == 'U') {
    // U-turn changes direction but keeps position unchanged
    if (currentDirection == 'N') currentDirection = 'S';
    else if (currentDirection == 'S') currentDirection = 'N';
    else if (currentDirection == 'E') currentDirection = 'W';
    else currentDirection = 'E';
  }

  // Record this intersection as visited
  visited[visitCount].x = posX;
  visited[visitCount].y = posY;
  visited[visitCount].direction = currentDirection;
  visitCount++;
}
