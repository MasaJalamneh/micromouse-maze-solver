/**********************************************************************************************************************************************************************************
*
*                                                                                    [Group Project]
*
*                                                                                1211403 Rawand Bawatneh
*                                                                                1210279 Klarein Wassaya
*                                                                                1212145 Masa Jalamneh
*                                                                                1201397 Mohammad Alabed
*
********************************************************************************************************************************************************************************/
#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <queue>
VL53L0X lidar;
MPU6050 mpu;

float yaw = 0;
unsigned long lastTime = 0;

const int encoderPinC1 = 16;
const int encoderPinC2 = 23;
const int enablePin = 14;
const int input1Pin = 26;
const int input2Pin = 27;

const int encoder2PinC1 = 17;
const int encoder2PinC2 = 4;
const int enable2Pin = 25;
const int input3Pin = 33;
const int input4Pin = 32;

const int irLeftPin = 34;
const int irRightPin = 35;

const int irThreshold = 220;
const int lidarThreshold = 100;
const int motorSpeed = 85;
const int motor2Speed = 78;
const int TICKS_PER_20CM = 382;  // Adjust based on testing
char direction = 'N';  // Initial direction (Facing North)
const int goalX1 = 3, goalY1 = 3;
const int goalX2 = 3, goalY2 = 4;
const int goalX3 = 4, goalY3 = 3;
const int goalX4 = 4, goalY4 = 4;

volatile long Ticks = 0;
std::queue<char> path;  // Stores the movement path
bool firstRun = true;   // Flag to check if we are exploring or following the path
const int GRID_SIZE = 8;
int cost[GRID_SIZE][GRID_SIZE] = {
    {6, 5, 4, 3, 3, 4, 5, 6},
    {5, 4, 3, 2, 2, 3, 4, 5},
    {4, 3, 2, 1, 1, 2, 3, 4},
    {3, 2, 1, 0, 0, 1, 2, 3},
    {3, 2, 1, 0, 0, 1, 2, 3},
    {4, 3, 2, 1, 1, 2, 3, 4},
    {5, 4, 3, 2, 2, 3, 4, 5},
    {6, 5, 4, 3, 3, 4, 5, 6}
};
int posX = 0, posY = 7;

bool useLeftHand = true;  // Start with left-hand rule, can be toggled
void IRAM_ATTR EncoderISR() {
    Ticks++;
}
// Flags
bool goalReached = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!lidar.init()) {
        Serial.println("Failed to detect LiDAR sensor!");
        while (1);
    }
    lidar.setTimeout(500);
    lidar.startContinuous();

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    calibrateMPU();

    pinMode(encoderPinC1, INPUT);
    pinMode(encoderPinC2, INPUT);
    pinMode(encoder2PinC1, INPUT);
    pinMode(encoder2PinC2, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinC1), EncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2PinC1), EncoderISR, RISING);

    pinMode(enablePin, OUTPUT);
    pinMode(input1Pin, OUTPUT);
    pinMode(input2Pin, OUTPUT);
    pinMode(enable2Pin, OUTPUT);
    pinMode(input3Pin, OUTPUT);
    pinMode(input4Pin, OUTPUT);

    pinMode(irLeftPin, INPUT);
    pinMode(irRightPin, INPUT);

    analogWrite(enablePin,motorSpeed);
    analogWrite(enable2Pin, motor2Speed);
    stopMotors();
    Serial.println("Motors Stopped.");
    delay(5000);
    moveRobot();
}

float gyroZOffset = 0;

void calibrateMPU() {
    yaw = 0;
    lastTime = millis();
    long sum = 0;
    int n = 500;
    for (int i = 0; i < n; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(2);
    }
    gyroZOffset = sum / (float)n;
    Serial.print("Gyro Z Offset: "); Serial.println(gyroZOffset);
}

float getYawChange() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    float adjustedGz = (gz - gyroZOffset) / 131.0;
    return adjustedGz * dt;
}

void rotate90(bool left) {
    Serial.println(left ? "Turning LEFT..." : "Turning RIGHT...");
    calibrateMPU();
    float targetAngle = (left ? -90 : 90) * 0.95;
    yaw = 0;

    digitalWrite(input1Pin, left ? LOW : HIGH);
    digitalWrite(input2Pin, left ? HIGH : LOW);
    digitalWrite(input3Pin, left ? LOW : HIGH);
    digitalWrite(input4Pin, left ? HIGH : LOW);

    while (abs(yaw) < abs(targetAngle)) {
        yaw += getYawChange();
        delay(1);
    }
    stopMotors();
    Serial.println("Motors Stopped.");

    // Update direction based on rotation
    if (left) {  // Left Turn
        if (direction == 'N') direction = 'W';
        else if (direction == 'W') direction = 'S';
        else if (direction == 'S') direction = 'E';
        else if (direction == 'E') direction = 'N';
    } else {  // Right Turn
        if (direction == 'N') direction = 'E';
        else if (direction == 'E') direction = 'S';
        else if (direction == 'S') direction = 'W';
        else if (direction == 'W') direction = 'N';
    }

    if (firstRun) path.push(left ? 'L' : 'R');  // Save turn direction
    Serial.print("New Direction: ");
    Serial.println(direction);
}


void stopMotors() {
    digitalWrite(input1Pin, LOW);
    digitalWrite(input2Pin, LOW);
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, LOW);
}

void moveForward20cm() {
    Ticks = 0;
    digitalWrite(input1Pin, HIGH);
    digitalWrite(input2Pin, LOW);
    digitalWrite(input3Pin, LOW);
    digitalWrite(input4Pin, HIGH);
    
    Serial.println("Moving Forward...");
    
    while (Ticks < TICKS_PER_20CM) {
        if (lidar.readRangeContinuousMillimeters() < lidarThreshold) {
            stopMotors();
            Serial.println("Obstacle detected. Stopping.");
            return;
        }
    }

    stopMotors();
    Serial.println("Motors Stopped.");

    // Update Position Based on Direction
    if (direction == 'N') posY++;
    else if (direction == 'S') posY--;
    else if (direction == 'E') posX++;
    else if (direction == 'W') posX--;

    Serial.print("New Position: (");
    Serial.print(posX);
    Serial.print(", ");
    Serial.print(posY);
    Serial.println(")");

    // Check if Goal is Reached
    if ((posX == goalX1 && posY == goalY1) ||
        (posX == goalX2 && posY == goalY2) ||
        (posX == goalX3 && posY == goalY3) ||
        (posX == goalX4 && posY == goalY4)) {
        Serial.println("🎯 Goal Reached!");
        goalReached = true;
        return;
    }

    if (firstRun) path.push('F');  // Save forward movement
}


// Main Movement Logic
void moveRobot() {
    stopMotors();
    // Read LiDAR distance
    int lidarDistance = lidar.readRangeContinuousMillimeters();

    Serial.print("LiDAR Distance: "); 
    Serial.println(lidarDistance);

    // Read IR sensor values
    int irLeftValue = analogRead(irLeftPin);
    Serial.print("Left IR Distance: "); 
    Serial.println(irLeftValue);

    int irRightValue = analogRead(irRightPin);
    Serial.print("Right IR Distance: "); 
    Serial.println(irRightValue);

    // If an obstacle is detected by LiDAR
    if (lidarDistance < lidarThreshold) {  
        stopMotors();
        delay(100);  // Small delay before rechecking sensor values

        Serial.print("Front Wall Detected !! "); 

        // Re-read IR values
        irLeftValue = analogRead(irLeftPin);  
        Serial.print("Left IR Distance: "); 
        Serial.println(irLeftValue);

        irRightValue = analogRead(irRightPin);
        Serial.print("Right IR Distance: "); 
        Serial.println(irRightValue);

        if (useLeftHand) {
            Serial.print("Left Hand Rule ..."); 
            // Left-Hand Rule Logic
            if (irLeftValue > irThreshold) {  
                rotate90(true);  // Turn Left (no wall on left)
                Serial.print("Turning Left ..."); 
            } else if (lidar.readRangeContinuousMillimeters() >= lidarThreshold) {  
                moveForward20cm();  // Move Forward (no front obstacle)
                Serial.print("Moving Forward ..."); 
            } else if (irLeftValue <= irThreshold && irRightValue > irThreshold) {
                rotate90(false);  // Turn Right (wall on left, no wall on right)
                Serial.print("Turning Right ...");   
            } else if (irLeftValue > irThreshold && irRightValue > irThreshold) {  
                rotate90(true);  // Turn Left (both sides open)
                Serial.print("Turning Left ...");   
            } else {  
                rotate90(true);  // Turn Left (no IR signals)
                rotate90(true);  // Turn 180° (Dead end)
                Serial.print("Turning Back 180 ...");   
                useLeftHand = false;  // Switch to Right-Hand Rule
            }
        } else {
            Serial.print("Right Hand Rule ..."); 
            // Right-Hand Rule Logic
            if (irRightValue > irThreshold) {  
                rotate90(false);  // Turn Right (no wall on right)
                Serial.print("Turning Right ...");   
            } else if (lidar.readRangeContinuousMillimeters() >= lidarThreshold) {  
                moveForward20cm();  // Move Forward (no front obstacle)
                Serial.print("Moving Forward ...");   
            } else if (irRightValue <= irThreshold && irLeftValue > irThreshold) {  
                rotate90(true);  // Turn Left (wall on right, no wall on left)
                Serial.print("Turning Left ...");   
            } else if (irLeftValue > irThreshold && irRightValue > irThreshold) {  
                rotate90(false);  // Turn Right (both sides open)
                Serial.print("Turning Right ...");   
            } else {
                rotate90(true);  // Turn Right (no IR signals)
                rotate90(true);  // Turn 180° (Dead end)
                Serial.print("Turning Back 180 ...");   
                useLeftHand = true;  // Switch back to Left-Hand Rule
            }
        }
    } else {
        // Move forward if no obstacles detected
        moveForward20cm();
    }

    delay(100);  // Small delay before the next cycle
}

void followSavedPath() {
    if (path.empty()) {
        Serial.println("Path completed!");
        return;
    }
    
    char nextMove = path.front();
    path.pop();
    
    if (nextMove == 'F') moveForward20cm();
    else if (nextMove == 'L') rotate90(true);
    else if (nextMove == 'R') rotate90(false);
}
void printPath() {
    Serial.print("Path taken: ");
    while (!path.empty()) {
        Serial.print(path.front());
        Serial.print(" -> ");
        path.pop();
    }
    Serial.println("Goal!");
}
void loop() {
    if (!goalReached) {
        moveRobot();  
    } else if (firstRun) {
        Serial.println("Goal reached! Storing path...");
        firstRun = false;  // Now follow the path instead of exploring
        printPath();
    } else {
        followSavedPath();  // Move based on stored path
    }
}