//--Colour Sensor and two ToF sensors--//
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_VL53L0X.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//Varibles for Menu handaling
#define BUTTON_PIN 34
bool button_pushed = true;
int8_t task = 1;

//Display Object
Adafruit_SSD1306 display(128, 64, &Wire, -1);
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C


Servo servo1; // Colour Sensing Servo
Servo servo2; // Ball Picking Servo

//VL53L0X tof1;   // ToF on channel 5
//VL53L0X tof2;   // ToF on channel 4
// ---------- ToF SENSORS ----------
Adafruit_VL53L0X tof1;  // CH4
Adafruit_VL53L0X tof2;  // CH5

Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// ---------- COLOR SENSORS ----------
Adafruit_TCS34725 color1 = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  // CH3

Adafruit_TCS34725 color2 = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  // CH7

uint16_t dist1 = 0;
uint16_t dist2 = 0;


//------BFS parameters-----//

struct Node{
  int8_t x;
  int8_t y;
};

struct direction{
  int8_t delta_x;
  int8_t delta_y;
};

enum node_state{
  Free,
  NotKnow,
  Blocked,
  Box
};

node_state grid[9][9] = {
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow, NotKnow},
  {Blocked, Blocked, Blocked, Blocked, Blocked, Blocked, Blocked, Blocked, Blocked}
};

const direction directions[4] = { {0,1}, {0,-1}, {1,0}, {-1,0}}; // North, South, East, West

const Node Search_path[81] = {
  {0,0}, {1,0}, {2,0}, {3,0}, {4,0}, {5,0}, {6,0}, {7,0}, {8,0},
  {8,1}, {7,1}, {6,1}, {5,1}, {4,1}, {3,1}, {2,1}, {1,1}, {0,1},
  {0,2}, {1,2}, {2,2}, {3,2}, {4,2}, {5,2}, {6,2}, {7,2}, {8,2},
  {8,3}, {7,3}, {6,3}, {5,3}, {4,3}, {3,3}, {2,3}, {1,3}, {0,3},
  {0,4}, {1,4}, {2,4}, {3,4}, {4,4}, {5,4}, {6,4}, {7,4}, {8,4},
  {8,5}, {7,5}, {6,5}, {5,5}, {4,5}, {3,5}, {2,5}, {1,5}, {0,5},
  {0,6}, {1,6}, {2,6}, {3,6}, {4,6}, {5,6}, {6,6}, {7,6}, {8,6},
  {8,7}, {7,7}, {6,7}, {5,7}, {4,7}, {3,7}, {2,7}, {1,7}, {0,7},
  {0,8}, {1,8}, {2,8}, {3,8}, {4,8}, {5,8}, {6,8}, {7,8}, {8,8}
};

int currentpoint_index = 0;
int nextpoint_index = 1;
direction current_direction = {0,1};
Node current_Node;
Node next_Node;
Node next_NodeTemp;
int8_t dx = 0;
int8_t dy = 0;
int8_t new_des_index;
Node new_destination_node;
bool going_recalculated_path = false;

//---Constructing que for path node que---//
const int MAX_Q = 81;  // max queue size
Node path_node_que[MAX_Q] = {{0,0}, {1,0}};

int front = 0;
int rear = 2;
int count = 2;

Node path_out[81];

const int8_t dirs[4][2] = {
  { 1, 0 },
  { -1, 0 },
  { 0, 1 },
  { 0, -1 }
};



#define TCA_ADDR 0x70

//Select multiplexer channel 
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}


//Ramp
MPU6050 mpu;

// ---- DMP variables ----
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

float roll = 0; 
float pitch = 0;
float yaw = 0;

unsigned long startTime = 0;


/* --- IR SENSOR ARRAY PINS --- */
const int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int Weight[8] = {0, 2000, 3000, 4000, 5000, 6000, 7000, 9000};
int sensor[8];
long total = 0;
long weightedSum = 0;

/* ---Control 28BYJ-48 Stepper by Desired Angle--- */

// ULN2003 Pins
const int IN1 = 33;
const int IN2 = 35;
const int IN3 = 37;
const int IN4 = 39;
float stepDelay = 2;  // speed (ms) smaller = faster
int stepIndex = 0;

// 4-step sequence for 28BYJ-48
int stepSequence[4][4] = {
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};


/* ---Colour Sensor--- */
String colour = "None";
String neededColour = "None";

//---Ultrasonic sensor parameters---//
long ultra_duration =0;
long ultra_distance = 0;
#define trigPin  51
#define echoPin  49



// SIDE ULTRASONIC PINS
// -------------------------
#define TRIG_PIN 12   
#define ECHO_PIN 11   


#define AIN1 47
#define AIN2 45
#define PWMA 10   // Left motor PWM

#define BIN1 43
#define BIN2 41
#define PWMB 13   // Right motor PWM

#define STBY 26   // Standby pin

#define CIN1 28
#define CIN2 32
#define CPWM_PIN 6  

int juncDetected = 0;
int limit = 0;

/* --- MOTOR PINS (YOUR EXACT PINS) --- */

// Motor 1 (LEFT)
const int M1_PWM = 4;
const int M1_IN1 = 25;
const int M1_IN2 = 31;

// Motor 2 (RIGHT)
const int M2_PWM = 5;
const int M2_IN1 = 27;
const int M2_IN2 = 29;

// ===== ENCODER PINS =====
const int ENC_LEFT = 2; 
const int ENC_RIGHT = 19;

const int TRIG_PIN1 = 51;
const int ECHO_PIN1 = 49;

volatile long leftCount = 0;
volatile long rightCount = 0;
volatile long mediancount = 0;

const int PPR = 220 ;
const float WHEEL_DIAMETER = 0.065;
const float WHEEL_BASE = 0.165;

float robotCircumference = 3.14159 * WHEEL_BASE;
float turnDistance = robotCircumference / 4.0;
float wheelCircumference = 3.14159 * WHEEL_DIAMETER;
long ticksPer90Turn = (long)((turnDistance / wheelCircumference) * PPR);
long ticksPer180Turn = (long)((turnDistance / wheelCircumference) * PPR * 2 );


// =================== BARCODE CONFIG ======================
float BIT_1_WIDTH_CM = 6.0;   // 6 cm white = "1"
float BIT_0_WIDTH_CM = 3.0;   // 3 cm white = "0"

float WIDTH_TOLERANCE = 0.40;   // ±40%

float cmPerTick = 0.09282;   

// ================== INTERNAL BARCODE STATE ===============

String barcode = "";


/* --- SPEED & PID CONSTANTS --- */
int baseSpeed = 0;
float Kp = 0.1;       
float Kd = 0.05;        

float lastError = 0;


/* Wave-scan state */
int waveState = 0;     // 0 = Right, 1 = Left
unsigned long waveTimer = 0;
int waveTurn = 0;     // turning offset
unsigned long sweepDuration = 300; // ms for each half-wave

/* Dashed Line Following PID */
float Kdp = 0.1, Kdi = 0.0001, Kdd = 0.3;
float d_error, d_prevError, d_integral, d_lastError;

int whiteFound = 0;



// ============================================
// ENHANCED CIRCULAR WALL FOLLOWING WITH PID
// ============================================

// Configuration Constants
const float WALL_RADIUS = 45.0;           // cm - radius of circular wall
const float DESIRED_DISTANCE = 20.0;      // cm - desired gap from wall
const float ROBOT_TRACK_WIDTH = 15.0;     // cm - distance between wheels (ADJUST TO YOUR ROBOT)


// Speed Settings
const int BASE_SPEED = 140;               // Base speed for wall following
const int MIN_SPEED = 80;                 // Minimum motor speed
const int MAX_SPEED = 255;                // Maximum motor speed
const int BLIND_MODE_SPEED = 110;         // Speed when ultrasonic lost

// Ultrasonic Filtering
const int FILTER_WINDOW = 5;              // Moving average window
const float MAX_VALID_DISTANCE = 30.0;    // cm - max valid reading
const float MIN_VALID_DISTANCE = 5.0;     // cm - min valid reading

// Circular Path Calculation
//const float TURN_RATE_CIRCULAR = calculateCircularTurnRate();

// State Variables
float distanceBuffer[FILTER_WINDOW] = {0};
int bufferIndex = 0;

unsigned long lastValidReading = 0;
const unsigned long BLIND_MODE_TIMEOUT = 2000; // ms - how long to run blind
bool isInBlindMode = false;
float lastValidDistance = DESIRED_DISTANCE;

// Derivative filter (reduce noise)
float previousDerivative = 0;
const float DERIVATIVE_FILTER = 0.3;  // Low-pass filter coefficient

int Red = 0;
int Blue = 0;
int Green = 0;

// Create one VL53L0X object and reuse it for each channel
Adafruit_VL53L0X tof = Adafruit_VL53L0X();


void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(CPWM_PIN, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);

  // Wake up the driver
  digitalWrite(STBY, LOW); 

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor pins
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // Encoder pins
  pinMode(ENC_LEFT, INPUT_PULLUP);
  pinMode(ENC_RIGHT, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT), rightEncoderISR, RISING);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //LED pins
  pinMode(42,OUTPUT);
  pinMode(46,OUTPUT);
  pinMode(50,OUTPUT);

  Wire.begin();
  Wire.setClock(100000); // 100kHz for stability

  // ---------------- Color 1 (CH 3) ----------------
  tcaSelect(3);
  if (!color1.begin()) {
    Serial.println("❌ Color Sensor 1 (CH3) NOT FOUND!");
  } else {
    Serial.println("✅ Color Sensor 1 (CH3) detected!");
  }
  delay(100);

  //---OLED Display----//
  tcaSelect(4);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  



  // ------ Colour sensor on CH7 ------
  tcaSelect(7);
  if (!colorSensor.begin()) {
    Serial.println("ERROR: Colour sensor (CH7) NOT FOUND!");
  } 
  else {
    Serial.println("Colour Sensor Ready on CH7");
  }
  delay(100);
  Serial.println("All sensors initialized!");



  // ---------------- ToF A (CH2) ----------------
  tcaSelect(2);
  if (!tof.begin()) Serial.println("❌ ToF_A (CH2) NOT FOUND!");
  else Serial.println("✅ ToF_A (CH2) detected!");


  /*servo1.attach(8);     // First servo on pin 8 - Colour Sensing
  servo2.attach(7);     // Second servo on pin 7 - Ball Picking
  servo1.write(60);     // Start at center
  servo2.write(0);     // Start at center
  delay(500);

  servo1.detach();
  servo2.detach();*/

  //initialize disply
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  delay(1000);
}

void loop() {

  if (digitalRead(BUTTON_PIN)==LOW && !button_pushed){
    button_pushed = true;
    delay(200);
  }
  if (button_pushed){
    leftCount=1;
    while (true){
      if (leftCount>41){leftCount=1;}
      if (digitalRead(BUTTON_PIN) == LOW){
        button_pushed=false;
        delay(3000);
        break;
      }

      task = (leftCount/6) +1;
      showMenu(task);
      delay(80);

    }
  }
  else{showMenu(task);}

  if (task==1){
    /*-----Task 1-----*/

  // Grid Search

    /*gostraight(0,100);
    centerTheRobot(100);
    turn90Left();
    juncDetected = 0;
    while (juncDetected == 0) {
      linefollow(0,100);
    }
    centerTheRobot(100);
    turn90Right();
    gostraightTask1(0,-100);
    centerTheRobot(100);*/

    rotateAngle(400);
    current_Node = {0,0};

    for (int x = 0; x < 81; x++) {
      
      next_Node = Search_path[x];
      if (current_Node.x == next_Node.x && current_Node.y == next_Node.y) {
        continue;
        
      } else {

        while (grid[next_Node.x][next_Node.y] == NotKnow) {
          shortestPath9x9(current_Node, next_Node, grid, path_out);
          for (int i = 0; i < count; i++) {
            int index = (front + i) % MAX_Q;  // handle circular queue
            Serial.print("Node ");
            Serial.print(i);
            Serial.print(": x=");
            Serial.print(path_node_que[index].x);
            Serial.print(", y=");
            Serial.println(path_node_que[index].y);
          } 

          bool exitWhile = false;
          while ((current_Node.x != next_Node.x || current_Node.y != next_Node.y) && !exitWhile) {
            int length = count;
            for (int z = 0; z < count; z++) {
              next_NodeTemp = dequeue();
              

              if (grid[next_NodeTemp.x][next_NodeTemp.y] == NotKnow) {
                rotate(current_direction, current_Node, next_NodeTemp);
                ultra_distance = getDistance1();

                if (ultra_distance < 20) {
                  digitalWrite(42,HIGH);
                  delay(500);
                  digitalWrite(42,LOW);

                  int16_t d = readToF_mm(2);
                  if (d <= 250) {
                    grid[next_NodeTemp.x][next_NodeTemp.y] = Box;
                    digitalWrite(42,HIGH);
                    digitalWrite(46,HIGH);
                    digitalWrite(50,HIGH);
                    delay(500);
                    digitalWrite(42,LOW);
                    digitalWrite(46,LOW);
                    digitalWrite(50,LOW);
                  } else {
                    grid[next_NodeTemp.x][next_NodeTemp.y] = Blocked;
                    digitalWrite(42,HIGH);
                    delay(500);
                    digitalWrite(42,LOW);
                  }
                  gostraightTask1(0,-100);
                  centerTheRobot(100);
                  exitWhile = true;
                  
                  break;
                  
                } else {
                  while(juncDetected==0){linefollow(0,100);}
                  centerTheRobot(100);
                  grid[next_NodeTemp.x][next_NodeTemp.y] = Free;
                  current_Node = next_NodeTemp;
                  digitalWrite(46,HIGH);
                  delay(250);
                  digitalWrite(46,LOW);
                  Serial.print(current_Node.x);
                  Serial.print(",");
                  Serial.print(current_Node.y);
                  Serial.println();
                }
              }
              else { // If next_NodeTemp is FREE
                rotate(current_direction, current_Node, next_NodeTemp);
                while(juncDetected==0){linefollow(0,100);}
                current_Node = next_NodeTemp;
                digitalWrite(46,HIGH);
                delay(250);
                digitalWrite(46,LOW);
                centerTheRobot(100);
  
                Serial.print(current_Node.x);
                Serial.print(",");
                Serial.print(current_Node.y);
                Serial.println();
              }
              juncDetected=0;
            }
          }
        }
      }    
    }

    digitalWrite(50, HIGH);
    digitalWrite(42, HIGH);
    digitalWrite(46, HIGH);
    delay(1000);
    digitalWrite(50, LOW);
    digitalWrite(42, LOW);
    digitalWrite(46, LOW);

    // Box Placing

    for (int x = 0; x<9; x++) {
      for (int y = 0; y<9; y++) {
        if (grid[x][y] == Box) {
          shortestPath9x9(current_Node, {x,y}, grid, path_out);
          
          int length = count;
          for (int z = 0; z < (count - 1); z++) {
            next_NodeTemp = dequeue();
            rotate(current_direction, current_Node, next_NodeTemp);
            while(juncDetected==0){
              linefollow(0,100);
            }
            current_Node = next_NodeTemp;           
            centerTheRobot(100);
            juncDetected = 0;
          }
          next_NodeTemp = dequeue(); 
          rotate(current_direction, current_Node, next_NodeTemp);
          //current_Node = next_NodeTemp; 
          linefollow(0.07,100);
          colourSense1();

          if (colour == "Red") {
            Red = 1;
            shortestPath9x9(current_Node, {6,8}, grid, path_out);
            rotateAngle(-400);
          } else if (colour == "Blue") {
            Blue = 1;
            shortestPath9x9(current_Node, {2,8}, grid, path_out);
            rotateAngle(-400);
          } else if (colour == "Green") {
            Green = 1;
            shortestPath9x9(current_Node, {4,8}, grid, path_out);
            rotateAngle(-400);
          } else {
            continue;
          }


          gostraightTask1(0,-100);

          length = count;
          for (int z = 0; z < (count - 1); z++) {
            next_NodeTemp = dequeue();
            rotate(current_direction, current_Node, next_NodeTemp);
            while(juncDetected==0){
              linefollow(0,100);
            }
            current_Node = next_NodeTemp;           
            centerTheRobot(100);
            juncDetected = 0;
          }
          next_NodeTemp = dequeue();  
          rotate(current_direction, current_Node, next_NodeTemp);
          //current_Node = next_NodeTemp; 
          gostraight(0.1,100);
          rotateAngle(400);
          gostraight(0.1,-100);
          gostraightTask1(0,-100);
          centerTheRobot(100);

          
        }
      }
    }

    delay(500);
    
    if (Red == 0) {
      digitalWrite(42,HIGH);
      delay(2000);
      digitalWrite(42,LOW);
    } else if (Blue == 0) {
      digitalWrite(50,HIGH);
      delay(2000);
      digitalWrite(50,LOW);
    } else {
      digitalWrite(46,HIGH);
      delay(2000);
      digitalWrite(46,LOW);
    }
    //Ending the Task 1

    shortestPath9x9(current_Node, {8,8}, grid, path_out);
          
    int length = count;
    for (int z = 0; z < count; z++) {
      next_NodeTemp = dequeue();
      rotate(current_direction, current_Node, next_NodeTemp);
      while(juncDetected==0){
        linefollow(0,100);
      }
      current_Node = next_NodeTemp;           
      centerTheRobot(100);
      juncDetected = 0;
    }

    
    turn90Left();
    gostraight(0.05,100);
    turn90Right();
    gostraightTask1(0,-100);
    gostraight(0.04,100);
    task=2;

  }

  else if(task==2){
      /*-----Task 2-----*/

    // Dashed Line Following and Ramp Setting

    gostraight(0,100);
    centerTheRobot(100);

    juncDetected = 0;
    while (juncDetected == 0) {
      linefollowTask2(100,"Left");
    }
    gostraight(0.05,-120);
    turn180();
    gostraightTask1(0,-150);
    gostraight(0.2,-150);
    gostraight(0,120);
    centerTheRobot(100);

    juncDetected = 0;
    while (juncDetected == 0) {
      linefollowTask2(100,"Right");
    }

    gostraight(0.2, -150);
    turn90Left();
    
    
    //Ramp Climbing

    gostraight(0.35,-150);
    gostraight(0.1,-220);
    task=3;

  }
  else if(task==3){

      /*---- -Task 3------*/  

    //Align the robot with horizontal white stripes 
    gostraight(0,100);
    gostraight(0.3,100);

    juncDetected = 0;
    while (juncDetected == 0) {
      linefollowTask3(0,100);
    }
    centerTheRobot(100);
    turn90Right();

    // i. Entering The Barcode
    juncDetected = 0;
    while (juncDetected == 0) {
      linefollow(0, 100);
    }
    Serial.print("Linefollowing ended");
    Serial.println();
    delay(500);
    gostraight(0.1,-100);

    // ii. Barcode Reading
    if (juncDetected) {

      Serial.println("Straight white line ended!");
      Serial.println("Starting barcode scanning...");

      barcode = "";
      
      barcodeRead(0.6, 95);
      stopMotors();
      delay(500);
      Serial.print("FINAL BARCODE: ");
      Serial.println(barcode);

      int decodedNumber = 0;
      int barcodeLength = barcode.length();
      for (int i=0; i<barcodeLength; i++) {
        int digit = barcode[i] - '0';
        int x = 1 << (barcode.length() - (i+1)); // Multiplied by Powers of 2
        decodedNumber += digit * x ;
      }
      
      if (decodedNumber % 3 == 0) {
        //digitalWrite(42, HIGH);
        //delay(1000);
        //digitalWrite(42, LOW);
      } else if (decodedNumber % 3 == 1) {
        //digitalWrite(46, HIGH);
        //delay(1000);
        //digitalWrite(46, LOW);
      } else if (decodedNumber % 3 == 2) {
        //digitalWrite(50, HIGH);
        //delay(1000);
        //digitalWrite(50, LOW);
      } else {
        /*digitalWrite(42, HIGH); 
        digitalWrite(46, HIGH); 
        digitalWrite(50, HIGH); */
        //delay(2000);
        /*digitalWrite(42, LOW);
        digitalWrite(46, LOW); 
        digitalWrite(50, LOW);*/ 
      }
    }

    showNumberTwo();


    // iii. Exiting Task 3
    juncDetected = 0;
    while(juncDetected == 0) {
      linefollow(0,100);
    } 
    centerTheRobot(100);
    turn90Right();
    task=4;
  }
  
  else if(task==4){
    /*------Task 4------*/

    // Beginning - Before Circular Wall Following
    juncDetected = 0;
    linefollow(0.2,100);
    turn90Left();

    // 2. Outer Wall Following
    outerWallFollow(100,10,10);

    gostraight(0.2,100);
    turnAngleRight(30);
    delay(1000);
    gostraight(0.2,100);
    turn90Right();
    gostraight(0.3,100);
    turn90Right();
    


    // 3. Inner Wall Following 
    turnAngleRight(60);
    for (int x = 0; x < 7; x++) {
      if (x == 0) {
        turnAngleLeft(20);
      } else {
        gostraight(0.25,100);
        turnAngleLeft(50);
      }
    }


    // 4. End of Task 4
    outerWallFollow(100,10,10);
    centerTheRobot(100);
    turn90Left();
    juncDetected = 0;
    while (juncDetected == 0) {
      linefollow(0,120);
    }
    centerTheRobot(100);
    turn90Right();
    gostraightTask1(0,-120);
    centerTheRobot(100);
    task=5;
  }

  else if(task==5){
    /*------Task 5------*/

    //Dashed Line
    while (juncDetected == 0) {
      linefollowTask5(100); 
    }
    centerTheRobot(100);


    //Wall Following - 1st Half
    outerWallFollowTask5(100,4,4);
    gostraight(0.18, 100);
    turn90Right();

    //Wall Following - 2nd Half
    gostraight(0.1,100);
    outerWallFollowTask5(100,4,4);
    task=6;

  }
  else{
    /*------Task 6------*/

  gostraight(0,100);
  gostraight(0.25,100);
  delay(1000);

  //Shooting
  // ---- LEFT MOTOR (A output) Forward ----
  digitalWrite(STBY, HIGH); 

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 80);  // speed 0–255

  // ---- RIGHT MOTOR (B output) Forward ----
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 80);  // speed 0–255

  digitalWrite(CIN1, HIGH);
  digitalWrite(CIN2, HIGH);
  analogWrite(CPWM_PIN, 250);
  delay(5000);

  }

}




  /*if (digitalRead(BUTTON_PIN) == LOW){button_pushed=true;}
  

 
  else if(task==2){

  // 5. Ball Picking
  else if(task ==5){
    servo1.attach(8);
    servoMotor1(60,175,10);
    String neededColour = "Yes" ;
    gostraight3(100);
    delay(500);
    servoMotor1(175,60,10);
    delay(500);
    servo2.attach(7);
    servoMotor2(0,175,5);
    delay(500);
    servoMotor2(175,130,15);
    servo1.detach();
    servo2.detach();
  }*/




void ramps(){
  tcaSelect(5);
  float leftPWM=0;
  float rightPWM=0;
  float bases=90;
  float startp=0;
  float currentp=0;
  float starty=0;
  float currenty=0;
  float correction=0;
  float kp12=1;
  float kd12=2;
  int speed_factor=1.4;
  float current_speed=0;
  getTiltAngles(roll,pitch,yaw);
  delay(500);
  startp=pitch;
  starty=yaw;
  while (true){
    getTiltAngles(roll,pitch,yaw);
    Serial.print("R=");
    Serial.print(roll);
    Serial.print(",");
    Serial.print("P=");
    Serial.print(roll);
    Serial.print(",");
    Serial.print("Y=");
    Serial.print(roll);
    Serial.print(",");
    Serial.println();
    currentp=pitch;
    currenty=yaw;
    current_speed=bases+(currentp-startp);
    leftPWM=current_speed*1.2678;
    rightPWM=current_speed;

    setMotor(leftPWM, rightPWM);
  }
  return;
}




void showMenu(int selected) {

  tcaSelect(4);

  display.clearDisplay();
  display.setTextSize(1);

  const char* items[] = {
    "1) Grid",
    "2) Ramp",
    "3) Barcode",
    "4) Round Wall",
    "5) Arrow/Wall Following",
    "6) Shooting"
  };

  const int itemCount = 6;
  const int rowHeight = 10;        // 6 rows × 10px = 60px (fits in 64px)
  const int yOffset = 2;           // small top padding

  for (int i = 0; i < itemCount; i++) {
    int y = i * rowHeight + yOffset;

    if ((i + 1) == selected) {
      // highlight full row
      display.fillRect(0, y - 1, 128, rowHeight, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }

    // vertical centering inside the row
    display.setCursor(2, y + 2);
    display.println(items[i]);
  }

  display.display();
}


void showNumberTwo() {
  tcaSelect(4);     // Select same multiplexer channel as menu

  display.clearDisplay();
  display.setTextSize(3);      // Big size for visibility
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(50, 20);   // Center-ish position

  display.println("2");

  display.display();
}


//----- BFS Parameters-----//

void rotate(direction &current_direction, const Node &currentNode, const Node &nextNode) {

  // compute next movement
  dx = nextNode.x - currentNode.x;
  dy = nextNode.y - currentNode.y;

  // CASE 1: Move North (0,1)
  if (dx == 0 && dy == 1) {

      if (current_direction.delta_x == 0 && current_direction.delta_y == 1) {
          // already facing north
      }
      else if (current_direction.delta_x == 0 && current_direction.delta_y == -1) {
          turn180();
          
      }
      else if (current_direction.delta_x == 1 && current_direction.delta_y == 0) {
          turn90Left();
        
      }
      else {
          turn90Right();
          
      }
  }

  // CASE 2: Move South (0,-1)
  else if (dx == 0 && dy == -1) {

      if (current_direction.delta_x == 0 && current_direction.delta_y == -1) {
          // already facing south
      }
      else if (current_direction.delta_x == 0 && current_direction.delta_y == 1) {
          turn180();
          

      }
      else if (current_direction.delta_x == 1 && current_direction.delta_y == 0) {
          turn90Right();
          
      }
      else {
          turn90Left();
          
      }
  }

  // CASE 3: Move East (1,0)
  else if (dx == 1 && dy == 0) {

      if (current_direction.delta_x == 1 && current_direction.delta_y == 0) {
          // already east
      }
      else if (current_direction.delta_x == 0 && current_direction.delta_y == 1) {
          turn90Right();
          
      }
      else if (current_direction.delta_x == -1 && current_direction.delta_y == 0) {
          turn180();
      }
      else {
          turn90Left();
          
      }
  }

  // CASE 4: Move West (-1,0)
  else if (dx == -1 && dy == 0) {

      if (current_direction.delta_x == -1 && current_direction.delta_y == 0) {
          // already west
      }
      else if (current_direction.delta_x == 0 && current_direction.delta_y == 1) {
          turn90Left();
          
      }
      else if (current_direction.delta_x == 1 && current_direction.delta_y == 0) {
          turn180();
          
      }
      else {
          turn90Right();
        
      }
  }


  // Update direction
  current_direction.delta_x = dx;
  current_direction.delta_y = dy;
}

// ---- Function to find index of a given element in search_path ----
int findNodeIndex(Node target) {
  for (int i = 0; i < 81; i++) {
    if (Search_path[i].x == target.x &&
        Search_path[i].y == target.y) {
      return i;      // found
    }
  }
  return -1; // NOT FOUND
}

//---Enque---//
bool enqueue(Node item) {
  if (count == MAX_Q) return false; // queue full

  path_node_que[rear] = item;
  rear = (rear + 1) % MAX_Q;
  count++;
  return true;
}

//---Deque---//
Node dequeue() {
  Node item = {0,0};         // default if queue empty
  if (count == 0) return item;

  item = path_node_que[front];
  front = (front + 1) % MAX_Q;
  count--;
  return item;
}

int shortestPath9x9(Node start, Node end, node_state grid[9][9], Node path_out[81]) {

  bool visited[9][9] = {0};
  Node parent[9][9];

  // initialize parents + mark blocked as visited
  for (int x = 0; x < 9; x++) {
    for (int y = 0; y < 9; y++) {
      parent[x][y] = { -1, -1 };
      if (grid[x][y] == Blocked) visited[x][y] = true;
    }
  }

  // BFS queue (max 81 nodes)
  Node q[81];
  int front = 0, back = 0;

  // push start node
  q[back++] = start;
  visited[start.x][start.y] = true;

  bool found = false;

  while (front < back) {
    Node cur = q[front++];

    if (cur.x == end.x && cur.y == end.y) {
      found = true;
      break;
    }

    // explore 4 directions
    for (int i = 0; i < 4; i++) {
      int nx = cur.x + dirs[i][0];
      int ny = cur.y + dirs[i][1];

      if (nx < 0 || nx >= 9 || ny < 0 || ny >= 9) continue;
      if (visited[nx][ny]) continue;

      visited[nx][ny] = true;
      parent[nx][ny] = cur;
      q[back++] = { (int8_t)nx, (int8_t)ny };
    }
  }

  if (!found)
    return 0;

  // reconstruct path backwards
  Node rev[81];
  int len = 0;
  Node cur = end;

  while (!(cur.x == start.x && cur.y == start.y)) {
    rev[len++] = cur;
    cur = parent[cur.x][cur.y];
  }


  // reverse into output array
  for (int i = 0; i < len; i++)
    path_out[i] = rev[len - 1 - i];

  ::front = 0;
  ::rear = 0;
  ::count = 0;

  for (int i = 0; i < len; i++) {
      enqueue(path_out[i]) ;
    }
  for (int i = 0; i < 81; i++) {
    path_out[i] = {0,0};
  }
  return len; 

}




// ====== ENCODER ISR ======
void leftEncoderISR() {
  leftCount++;
}

void rightEncoderISR() {
  rightCount++;
}

// ====== MOTOR CONTROL ======
void setMotor(int leftSpeed, int rightSpeed) {

  if (leftSpeed >= 0) {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
  } else {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  Serial.print("Left : ");
  Serial.print(leftSpeed);
  Serial.print("    Right : ");
  Serial.print(rightSpeed);
  analogWrite(M1_PWM, constrain(leftSpeed, 90, 255));

  if (rightSpeed >= 0) {
    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
  } else {
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(M2_PWM, constrain(rightSpeed, 90, 255));
}

void setMotor_low(int leftSpeed, int rightSpeed) {

  if (leftSpeed >= 0) {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
  } else {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(M1_PWM, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) {
    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
  } else {
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(M2_PWM, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
  
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, HIGH);
  delay(30);
  return;
}

// ====== 90° TURN ======
void turn90Left() {
  leftCount = 0;
  rightCount = 0;
  mediancount=0;

  float Kp_turn = 0.8;
  float Kd_turn = 0.3;

  long target = ticksPer90Turn;
  long diff, prevDiff = 0, derivative;
  float correction;

  while (abs(mediancount) < target) {
    mediancount = (leftCount+rightCount)/2;

    diff = target - mediancount; 
    derivative = diff - prevDiff;
    correction = Kp_turn * diff + Kd_turn * derivative;
    prevDiff = diff;

    int leftPWM = 90 + correction;  
    int rightPWM = -90 - correction;

    leftPWM  = constrain(leftPWM,  90, 255);
    rightPWM = constrain(rightPWM,  -255, -90);

    setMotor(leftPWM, rightPWM);
  }

  stopMotors();
}


// ====== 90° TURN ======
void turn90Right() {
  leftCount = 0;
  rightCount = 0;
  mediancount=0;

  float Kp_turn = 0.8;
  float Kd_turn = 0.3;

  long target = ticksPer90Turn;
  long diff, prevDiff = 0, derivative;
  float correction;

  while (abs(mediancount) < target) {
    mediancount = (leftCount+rightCount)/2;

    diff = target - mediancount; 
    derivative = diff - prevDiff;
    correction = Kp_turn * diff + Kd_turn * derivative;
    prevDiff = diff;

    int leftPWM = -90 - correction;  
    int rightPWM = 90 + correction;

    leftPWM  = constrain(leftPWM, -255, -90);
    rightPWM = constrain(rightPWM,  90, 255);

    setMotor(leftPWM, rightPWM);
  }

  stopMotors();
}

// ====== ANGULAR TURN ======
void turnAngleLeft(float angle) {
  leftCount = 0;
  rightCount = 0;
  mediancount=0;

  float Kp_turn = 0.8;
  float Kd_turn = 0.3;

  long target = ticksPer180Turn * (angle/180);
  long diff, prevDiff = 0, derivative;
  float correction;

  while (abs(mediancount) < target) {
    mediancount = (leftCount+rightCount)/2;

    diff = target - mediancount; 
    derivative = diff - prevDiff;
    correction = Kp_turn * diff + Kd_turn * derivative;
    prevDiff = diff;

    int leftPWM = 90 + correction;  
    int rightPWM = -90 - correction;

    leftPWM  = constrain(leftPWM,  90, 255);
    rightPWM = constrain(rightPWM,  -255, -90);

    setMotor(leftPWM, rightPWM);
  }

  stopMotors();
}


// ====== ANGULAR TURN ======
void turnAngleRight(float angle) {
  leftCount = 0;
  rightCount = 0;
  mediancount=0;

  float Kp_turn = 0.8;
  float Kd_turn = 0.3;

  long target = ticksPer180Turn * angle/180;
  long diff, prevDiff = 0, derivative;
  float correction;

  while (abs(mediancount) < target) {
    mediancount = (leftCount+rightCount)/2;

    diff = target - mediancount; 
    derivative = diff - prevDiff;
    correction = Kp_turn * diff + Kd_turn * derivative;
    prevDiff = diff;

    int leftPWM = -90 - correction;  
    int rightPWM = 90 + correction;

    leftPWM  = constrain(leftPWM, -255, -90);
    rightPWM = constrain(rightPWM,  90, 255);

    setMotor(leftPWM, rightPWM);
  }

  stopMotors();
}

void turn180() {
  centerTheRobot(100);
  leftCount = 0;
  rightCount = 0;
  mediancount=0;

  float Kp_turn = 0.4;
  float Kd_turn = 0.3;

  long target = 2.15 * ticksPer90Turn;
  long diff, prevDiff = 0, derivative;
  float correction;

  while (abs(mediancount) < target) {
    mediancount = (leftCount+rightCount)/2;

    diff = target - mediancount; 
    derivative = diff - prevDiff;
    correction = Kp_turn * diff + Kd_turn * derivative;
    prevDiff = diff;

    int leftPWM = -150 - correction;  
    int rightPWM = 100 + correction;

    leftPWM  = constrain(leftPWM, -255, -90);
    rightPWM = constrain(rightPWM,  90, 255);

    setMotor(leftPWM, rightPWM);
  }

  stopMotors();
}

// Line following(without a loop) until a junction is detected(when the given distance is 0) or line following until a given distance is covered
void linefollow(float distance, int baseSpeed) {

  leftCount = 0;
  rightCount = 0;

  if (distance == 0) {
    
    IRsense();
    
    if (total == 0) {
      juncDetected = 1;
      stopMotors();
      return;   

    } else if (total == 8) {
      gostraight(0, 120);

    } else {
      float position;
      if (total > 0 && total < 8) position = weightedSum / total;
      else position = 4500;

      /* ---- ERROR ---- */      
      float error = position - 4500;

      /* ---- PD CONTROL ---- */      
      float P = error;
      float D = error - lastError;
      float correction = (Kp * P) + (Kd * D);

      delay(5);
      lastError = error;

      correction = constrain(correction, -100, 100);

      int leftSpeed = 0;
      int rightSpeed = 0;

      leftSpeed  = baseSpeed*1.27681 - correction;
      rightSpeed = baseSpeed + correction;

      leftSpeed  = constrain(leftSpeed,  90, 255);
      rightSpeed = constrain(rightSpeed, 90, 255);

      setMotor(leftSpeed, rightSpeed);
    } 

  } else {

    long tickCount = (PPR * distance) / wheelCircumference;

    total = 0;

    while (leftCount < tickCount || rightCount < tickCount) {

      IRsense();
    
      if (total == 0) {
        stopMotors();
        return;

      } else {

        float position;
        if (total > 0 && total < 8) position = weightedSum / total;
        else position = 4500;

        /* ---- ERROR ---- */      
        float error = position - 4500;

        /* ---- PD CONTROL ---- */      
        float P = error;
        float D = error - lastError;
        float correction = (Kp * P) + (Kd * D);

        delay(5);
        lastError = error;

        correction = constrain(correction, -100, 100);

        int leftSpeed  = baseSpeed*1.27681 - correction;
        int rightSpeed = baseSpeed + correction;

        leftSpeed  = constrain(leftSpeed,  90, 255);
        rightSpeed = constrain(rightSpeed, 90, 255);

        setMotor(leftSpeed, rightSpeed);
      } 
    }
    stopMotors();
  }
}



void linefollowTask2(int baseSpeed, String side) {

  leftCount = 0;
  rightCount = 0;

  IRsense();
  
  if (total == 0) {
    juncDetected = 1;
    stopMotors();
    return;   

  } else if (total == 8) {
    gostraight2(0.12, baseSpeed);
    if (whiteFound) {
      gostraight(0.01,baseSpeed);
      whiteFound=0;
      IRsense();
      int leftTotal = 0;
      int rightTotal = 0;
      for (int i=0; i<3; i++) {
        if (sensor[i] < 500) {
          rightTotal++;
        }
      }
      for (int i=5; i<8 ; i++) {
        if (sensor[i] < 500) {
          leftTotal++;
        }
      }

      if (leftTotal >= rightTotal) {
        while (sensor[3] > 500 && sensor[4] > 500) {
          IRsense();
          setMotor(150,-120);     
        }
      } else {
        float start = millis();
        float now = millis();
        while (sensor[3] > 500 && sensor[4] > 500  && (now - start < 2500)) {
          now = millis();
      
          IRsense();
          setMotor(-150,120);     
        }
        if (now - start >= 2500) {
          gostraight(0.03,100);
        }
      }
      
      stopMotors();
    } else {
      while(whiteFound==0){
        gostraight(0.08,-100);
        waveScan(side);

        //gostraight(0.08, baseSpeed);
      }
      whiteFound=0;
    }


    

  } else {
    float position;
    if (total > 0 && total < 8) position = weightedSum / total;
    else position = 4500;

    /* ---- ERROR ---- */      
    float error = position - 4500;

    /* ---- PD CONTROL ---- */      
    float P = error;
    float D = error - lastError;
    float correction = (Kp * P) + (Kd * D);

    delay(5);
    lastError = error;

    correction = constrain(correction, -100, 100);

    int leftSpeed  = baseSpeed*1.27681 - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed  = constrain(leftSpeed,  90, 255);
    rightSpeed = constrain(rightSpeed, 90, 255);

    setMotor(leftSpeed, rightSpeed);

  } 
  delay(5);


}

void linefollowTask3(float distance, int baseSpeed) {

  leftCount = 0;
  rightCount = 0;

  if (distance == 0) {
    
    IRsense();
    int Total1 = 0;
    int Total2 = 0;
    for (int x=0; x<4; x++) {
      if (sensor[x] != 0) {
        Total1 ++;
      }
    }

    for (int x=4; x<8; x++) {
      if (sensor[x] != 0) {
        Total2 ++;
      }
    }
    
    if (total = 0 || Total1 == 0 || Total2 == 0) {
      juncDetected = 1;
      stopMotors();
      return;   

    } else if (total == 8) {
      gostraight(0, 120);

    } else {
      float position;
      if (total > 0 && total < 8) position = weightedSum / total;
      else position = 4500;

      /* ---- ERROR ---- */      
      float error = position - 4500;

      /* ---- PD CONTROL ---- */      
      float P = error;
      float D = error - lastError;
      float correction = (Kp * P) + (Kd * D);

      delay(5);
      lastError = error;

      correction = constrain(correction, -100, 100);

      int leftSpeed = 0;
      int rightSpeed = 0;

      leftSpeed  = baseSpeed*1.27681 - correction;
      rightSpeed = baseSpeed + correction;

      leftSpeed  = constrain(leftSpeed,  90, 255);
      rightSpeed = constrain(rightSpeed, 90, 255);

      setMotor(leftSpeed, rightSpeed);
    } 

  } else {

    long tickCount = (PPR * distance) / wheelCircumference;

    total = 0;

    while (leftCount < tickCount || rightCount < tickCount) {

      IRsense();
    
      if (total == 0) {
        stopMotors();
        return;

      } else {

        float position;
        if (total > 0 && total < 8) position = weightedSum / total;
        else position = 4500;

        /* ---- ERROR ---- */      
        float error = position - 4500;

        /* ---- PD CONTROL ---- */      
        float P = error;
        float D = error - lastError;
        float correction = (Kp * P) + (Kd * D);

        delay(5);
        lastError = error;

        correction = constrain(correction, -100, 100);

        int leftSpeed  = baseSpeed*1.27681 - correction;
        int rightSpeed = baseSpeed + correction;

        leftSpeed  = constrain(leftSpeed,  90, 255);
        rightSpeed = constrain(rightSpeed, 90, 255);

        setMotor(leftSpeed, rightSpeed);
      } 
    }
    stopMotors();
  }
}


void linefollowTask5(int baseSpeed) {

  leftCount = 0;
  rightCount = 0;

  IRsense();
  
  if (total == 0) {
    juncDetected = 1;
    stopMotors();
    return;   

  } else if (total == 8) {

    gostraight2(0, baseSpeed);
    centerTheRobot(baseSpeed);

    IRsense();
    int leftTotal = 0;
    int rightTotal = 0;
    for (int i=0; i<3; i++) {
      if (sensor[i] < 500) {
        rightTotal++;
      }
    }
    for (int i=5; i<8 ; i++) {
      if (sensor[i] < 500) {
        leftTotal++;
      }
    }

    if (leftTotal >= rightTotal) {
      while (sensor[3] > 500 && sensor[4] > 500) {
        IRsense();
        setMotor(150,-120);     
      }
    } else {
      while (sensor[3] > 500 && sensor[4] > 500) {
        IRsense();
        setMotor(-150,120);     
      }
    }

    stopMotors();

  } else {
    float position;
    if (total > 0 && total < 8) position = weightedSum / total;
    else position = 4500;

    /* ---- ERROR ---- */      
    float error = position - 4500;

    /* ---- PD CONTROL ---- */      
    float P = error;
    float D = error - lastError;
    float correction = (Kp * P) + (Kd * D);

    delay(5);
    lastError = error;

    correction = constrain(correction, -100, 100);

    int leftSpeed  = baseSpeed*1.27681 - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed  = constrain(leftSpeed,  90, 255);
    rightSpeed = constrain(rightSpeed, 90, 255);

    setMotor(leftSpeed, rightSpeed);

  } 
  delay(5);


}


void centerTheRobot(int Speed) {
  leftCount = 0;
  rightCount = 0;
  gostraight(0.07,Speed);
  return;
}

//Go straight for any specific distance on any surface(when d is given) or go until white line is found
void gostraight(float d, int Speed) {

  long diff = 0;
  long prevDiff = 0;
  long s_derivative = 0;
  long s_correction = 0;

  long tickCount = (PPR * d) / wheelCircumference;

  // === PID VARIABLES ===
  float Ksp = 3;
  float Ksd = 1;

  float error = 0, prevError = 0, derivative = 0;
  float correction = 0;

  leftCount = 0;
  rightCount = 0;

  if (d == 0) {


    total = 8;

    while (total == 8) {
      IRsense();
      // get encoder counts for the last 100ms
      long L_count = leftCount;
      long R_count = rightCount;

      // calculate error (difference)
      diff = L_count - R_count;

      // PID calculations
      s_derivative = diff - prevDiff;

      s_correction = Ksp * diff + Ksd * s_derivative;
      prevDiff = diff;

      int leftPWM = 0;
      int rightPWM = 0;

      if (Speed < 0) {
        leftPWM  = Speed*1.27681 + s_correction;
        rightPWM = Speed - s_correction;
        leftPWM  = constrain(leftPWM,  -255, -90);
        rightPWM = constrain(rightPWM, -255, -90);

      } else {
        leftPWM  = Speed*1.27681 - s_correction;
        rightPWM = Speed + s_correction;
        leftPWM  = constrain(leftPWM,  90, 255);
        rightPWM = constrain(rightPWM, 90, 255);
      }
      
      // Apply motor commands
      setMotor(leftPWM, rightPWM);
    } 
    
    stopMotors();
    return;
    
    
  } else {

    while (leftCount < tickCount || rightCount < tickCount) {
    

      // get encoder counts for the last 100ms
      long L_count = leftCount;
      long R_count = rightCount;

      // calculate error (difference)
      diff = L_count - R_count;

      // PID calculations
      s_derivative = diff - prevDiff;

      s_correction = Ksp * diff + Ksd * s_derivative;
      prevDiff = diff;

      int leftPWM = 0;
      int rightPWM = 0;

      if (Speed < 0) {
        leftPWM  = Speed*1.27681 + s_correction;
        rightPWM = Speed - s_correction;
        leftPWM  = constrain(leftPWM,  -255, -90);
        rightPWM = constrain(rightPWM, -255, -90);

      } else {
        leftPWM  = Speed*1.27681 - s_correction;
        rightPWM = Speed + s_correction;
        leftPWM  = constrain(leftPWM,  90, 255);
        rightPWM = constrain(rightPWM, 90, 255);
      }

      // Apply motor commands
      setMotor(leftPWM, rightPWM);
      Serial.println();
    }
  } 
  stopMotors();
  return;
  
}

// Go straight a specific distance on any surface until white finds and suddenly stops
void gostraight2(float d, int Speed) {

  long diff = 0;
  long prevDiff = 0;
  long s_derivative = 0;
  long s_correction = 0;

  long tickCount = (PPR * d) / wheelCircumference;
  Serial.print(PPR);
  Serial.println();
  Serial.print(d);
  Serial.println();
  
  Serial.println();
  Serial.print(tickCount);

  // === PID VARIABLES ===
  float Ksp = 1.5;
  float Ksd = 0.3;

  float error = 0, prevError = 0, derivative = 0;
  float correction = 0;

  leftCount = 0;
  rightCount = 0;

  while (leftCount < tickCount || rightCount < tickCount) {

    IRsense();
    if (total == 8) {
      
      // get encoder counts for the last 100ms
      long L_count = leftCount;
      long R_count = rightCount;

      // calculate error (difference)
      diff = L_count - R_count;

      // PID calculations
      s_derivative = diff - prevDiff;

      s_correction = Ksp * diff + Ksd * s_derivative;
      prevDiff = diff;

      int leftPWM = 0;
      int rightPWM = 0;

      if (Speed < 0) {
        leftPWM  = Speed*1.27681 + s_correction;
        rightPWM = Speed - s_correction;
        leftPWM  = constrain(leftPWM,  -255, -90);
        rightPWM = constrain(rightPWM, -255, -90);

      } else {
        leftPWM  = Speed*1.27681 - s_correction;
        rightPWM = Speed + s_correction;
        leftPWM  = constrain(leftPWM,  90, 255);
        rightPWM = constrain(rightPWM, 90, 255);
      }
      
      // Apply motor commands
      setMotor(leftPWM, rightPWM);

    } else {   
      stopMotors();
      whiteFound = 1;
      return;
    }
  }
  limit = 1;
}

// Go Straight Until a Specific Colour is Detected
void gostraight3(int Speed) {
  long diff = 0;
  long prevDiff = 0;
  long s_derivative = 0;
  long s_correction = 0;

  Serial.print(PPR);
  Serial.println();
  
  Serial.println();


  // === PID VARIABLES ===
  float Ksp = 1.5;
  float Ksd = 0.3;

  float error = 0, prevError = 0, derivative = 0;
  float correction = 0;

  leftCount = 0;
  rightCount = 0;

  while (colour != neededColour) {
    colourSense3();
    // get encoder counts for the last 100ms
    long L_count = leftCount;
    long R_count = rightCount;

    // calculate error (difference)
    diff = L_count - R_count;

    // PID calculations
    s_derivative = diff - prevDiff;

    s_correction = Ksp * diff + Ksd * s_derivative;
    prevDiff = diff;

    int leftPWM = 0;
    int rightPWM = 0;

    if (Speed < 0) {
      leftPWM  = Speed*1.27681 + s_correction;
      rightPWM = Speed - s_correction;
      leftPWM  = constrain(leftPWM,  -255, -90);
      rightPWM = constrain(rightPWM, -255, -90);

    } else {
      leftPWM  = Speed*1.27681 - s_correction;
      rightPWM = Speed + s_correction;
      leftPWM  = constrain(leftPWM,  90, 255);
      rightPWM = constrain(rightPWM, 90, 255);
    }
    
    // Apply motor commands
    setMotor(leftPWM, rightPWM);
  } 
  
  stopMotors();
  return;

}



void gostraightTask1(float d, int Speed) {

  long diff = 0;
  long prevDiff = 0;
  long s_derivative = 0;
  long s_correction = 0;

  long tickCount = (PPR * d) / wheelCircumference;

  // === PID VARIABLES ===
  float Ksp = 3;
  float Ksd = 1;

  float error = 0, prevError = 0, derivative = 0;
  float correction = 0;

  leftCount = 0;
  rightCount = 0;

  if (d == 0) {

    total = 8;

    while (total != 0) {
      IRsense();
      // get encoder counts for the last 100ms
      long L_count = leftCount;
      long R_count = rightCount;

      // calculate error (difference)
      diff = L_count - R_count;

      // PID calculations
      s_derivative = diff - prevDiff;

      s_correction = Ksp * diff + Ksd * s_derivative;
      prevDiff = diff;

      int leftPWM = 0;
      int rightPWM = 0;

      if (Speed < 0) {
        leftPWM  = 1.27681*Speed + s_correction;
        rightPWM = Speed - s_correction;
        leftPWM  = constrain(leftPWM,  -255, -90);
        rightPWM = constrain(rightPWM, -255, -90);

      } else {
        leftPWM  = 1.27681*Speed - s_correction;
        rightPWM = Speed + s_correction;
        leftPWM  = constrain(leftPWM,  90, 255);
        rightPWM = constrain(rightPWM, 90, 255);
      }
      
      // Apply motor commands
      setMotor(leftPWM, rightPWM);
    } 
    
    stopMotors();
    return;
    
    
  } else {

    while (leftCount < tickCount || rightCount < tickCount) {
    
      // get encoder counts for the last 100ms
      long L_count = leftCount;
      long R_count = rightCount;

      // calculate error (difference)
      diff = L_count - R_count;

      // PID calculations
      s_derivative = diff - prevDiff;

      s_correction = Ksp * diff + Ksd * s_derivative;
      prevDiff = diff;

      int leftPWM = 0;
      int rightPWM = 0;

      if (Speed < 0) {
        leftPWM  = 1.27681*Speed + s_correction;
        rightPWM = Speed - s_correction;
        leftPWM  = constrain(leftPWM,  -255, -90);
        rightPWM = constrain(rightPWM, -255, -90);

      } else {
        leftPWM  = 1.27681*Speed - s_correction;
        rightPWM = Speed + s_correction;
        leftPWM  = constrain(leftPWM,  90, 255);
        rightPWM = constrain(rightPWM, 90, 255);
      }

      // Apply motor commands
      setMotor(leftPWM, rightPWM);
    }
  } 
  stopMotors();
  return;
  
}




float ultrasonic() {

  //for (int i = 0; i < 10; i++) {
    // ----- Send Trigger Pulse -----
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // ----- Read Echo Time -----
  ultra_duration = pulseIn(echoPin, HIGH);

  // ----- Convert to Distance -----
  float distance = ultra_duration * 0.00034 / 2; // m
  
  return distance ; 
}
  
void rotateAngle(float angle) {

  // 2048 steps per 360 degrees
  float stepsPerDegree = 2048.0 / 360.0;

  int totalSteps = abs(angle * stepsPerDegree);
  int direction = (angle >= 0) ? 1 : -1;

  for (int s = 0; s < totalSteps; s++) {
    stepMotor(direction);
    delay(stepDelay);
  }
  motorOff();   // important to stop holding torque
}

// Run one step in given direction
void stepMotor(int dir) {
  stepIndex += dir;

  if (stepIndex > 3) stepIndex = 0;
  if (stepIndex < 0) stepIndex = 3;

  digitalWrite(IN1, stepSequence[stepIndex][0]);
  digitalWrite(IN2, stepSequence[stepIndex][1]);
  digitalWrite(IN3, stepSequence[stepIndex][2]);
  digitalWrite(IN4, stepSequence[stepIndex][3]);
}

// Turn off coils (reduces power consumption)
void motorOff() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
  delay(200);
  return;
}


void colourSense1() {

  tcaSelect(7);
  uint16_t r, g, b, c;
  colorSensor.getRawData(&r, &g, &b, &c);

  Serial.print("Color 1 (CH7) - R:");
  Serial.print(r);
  Serial.print(" G:");
  Serial.print(g);
  Serial.print(" B:");
  Serial.print(b);
  Serial.print(" C:");
  Serial.println(c);

  if ( r > g && r > b && c < 3000 ) {
    colour = "Red";
  } else if ( g > r && g > b && c < 3000 ) {
    colour = "Green";
  } else if ( b > g && b > r && c < 3000 ) {
    colour = "Blue";
  } else {
    colour = "White";
  }
  delay(50);
}


void colourSense2() {

  tcaSelect(3);
  uint16_t r, g, b, c;
  colorSensor.getRawData(&r, &g, &b, &c);

  Serial.print("Color 2 (CH3) - R:");
  Serial.print(r);
  Serial.print(" G:");
  Serial.print(g);
  Serial.print(" B:");
  Serial.print(b);
  Serial.print(" C:");
  Serial.println(c);

  if ( r > g && r > b && c < 3000 ) {
    colour = "Red";
  } else if ( g > r && g > b && c < 3000 ) {
    colour = "Green";
  } else if ( b > g && b > r && c < 3000 ) {
    colour = "Blue";
  } else {
    colour = "White";
  }
  delay(50);
}


void colourSense3() {

  tcaSelect(3);
  uint16_t r, g, b, c;
  colorSensor.getRawData(&r, &g, &b, &c);

  Serial.print("Color 3 (CH3) - R:");
  Serial.print(r);
  Serial.print(" G:");
  Serial.print(g);
  Serial.print(" B:");
  Serial.print(b);
  Serial.print(" C:");
  Serial.println(c);

  if ( r > 300 && b > 300 && c < 300 ) {
    colour = "Yes";

  } else {
    colour = "No";
  }
  delay(50);
}

void ToFSense1() {
  // ======================
  // READ ToF #1 (CH5)
  // ======================
  tcaSelect(5);
  VL53L0X_RangingMeasurementData_t measure2;
  tof2.rangingTest(&measure2, false);
  Serial.print("ToF2 (CH5): ");
  if (measure2.RangeStatus != 4) {
    Serial.println(measure2.RangeMilliMeter);
    dist1 = measure2.RangeMilliMeter;
  } else {
    Serial.println("Out of range");
  }
}

void ToFSense2() {
  // ======================
  // READ ToF #2 (CH4)
  // ======================
  // ------- Read ToF 1 -------
  tcaSelect(4);
  VL53L0X_RangingMeasurementData_t measure1;
  tof1.rangingTest(&measure1, false);
  Serial.print("ToF1 (CH4): ");
  if (measure1.RangeStatus != 4)
    Serial.println(measure1.RangeMilliMeter);
  else
    Serial.println("Out of range");
}



// startAngle = starting angle
// endAngle   = ending angle
// stepDelay  = delay in ms between each degree
void servoMotor1(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos++) {
      servo1.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos--) {
      servo1.write(pos);
      delay(stepDelay);
    }
  }
}
void servoMotor2(int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int pos = startAngle; pos <= endAngle; pos++) {
      servo2.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = startAngle; pos >= endAngle; pos--) {
      servo2.write(pos);
      delay(stepDelay);
    }
  }
}

void ballPicking(int Delay) {
  servoMotor2(60, 175, Delay);
  delay(500);
  servoMotor2(175, 60, Delay);
  delay(500);
}

void IRsense() {

  total = 0;
  weightedSum = 0;

  /* ---- READ ALL 8 IR SENSORS ---- */
  for (int i = 0; i < 8; i++) {
    sensor[i] = analogRead(irPins[i]); 
  }

  /* ---- WEIGHTED POSITION ---- */
  int digital_value=1;

  for (int i = 0; i < 8; i++) {
    int analog_value = map(sensor[i], 0, 1023, 0, 1000);

    if (analog_value > 500) digital_value = 1;
    else digital_value = 0;
    weightedSum += digital_value * (Weight[i]);
    total += digital_value;
  }

}

int readRightSideSensors() {

  int Total = 0;

  int sensor[3];
  for (int i = 0; i <= 2; i++) {
    sensor[i] = analogRead(irPins[i]); 
  }

  /* ---- WEIGHTED POSITION ---- */
  int digital_value=1;

  for (int i = 0; i <= 2; i++) {
    int analog_value = map(sensor[i], 0, 1023, 0, 1000);

    if (analog_value > 500)  {digital_value = 1;}
    else {digital_value = 0;}
    Total += digital_value;
  }
  return Total;
}

int readLeftSideSensors() {

  int Total = 0;

  int sensor[3];
  for (int i = 5; i < 8; i++) {
    sensor[i] = analogRead(irPins[i]); 
  }

  /* ---- WEIGHTED POSITION ---- */
  int digital_value=1;

  for (int i = 5; i < 8; i++) {
    int analog_value = map(sensor[i], 0, 1023, 0, 1000);

    if (analog_value > 500) {digital_value = 1;}
    else {digital_value = 0;}
    Total += digital_value;
  }
  return Total;
}

// =================== READ ONE SIDE SENSOR =============
int readSideIR() {
  int raw = analogRead(irPins[6]);  // A sensor which is not centered to avoid the straight line for line following at the end 
  int mapped = map(raw, 0, 1023, 0, 1000);
  return (mapped > 500) ? 1 : 0;
}


// =============== CLASSIFY WIDTH INTO BIT ==================
char classifyWidth(float width) {

  if (abs(width - BIT_1_WIDTH_CM) <= BIT_1_WIDTH_CM * WIDTH_TOLERANCE)
    return '1';

  if (abs(width - BIT_0_WIDTH_CM) <= BIT_0_WIDTH_CM * WIDTH_TOLERANCE)
    return '0';

  return 'X';   // error/unclassified
}



void waveScan(String Side) {
  
  if (total == 8 && Side == "Left") {

    // Turn Left slowly forward
    leftCount = 0;
    stopMotors();
    while (leftCount < 500) {     
      IRsense();
      if (total != 8) {
        return;
      } else {
        setMotor_low(160, 0);
      }
    }
    stopMotors();
   
  } else {
    // Turn Right slowly forward
    rightCount = 0;
    stopMotors();
    while (rightCount < 500) {
      IRsense();
      if (total != 8) {
        whiteFound = 1;
        return;
      } else {
        setMotor_low(0, 160);
      }
    }
    stopMotors();
  }
  //gostraight(0.02,-120);
 
}


void wiggleStraighten(int counts, int distance) {
  for (int i = 0; i < counts; i++) {
    linefollow(distance,130);
    gostraight(distance,-130);
  }
}

void juncStraighten(int counts) {

  const int turnSpeed = 100;      // very slow turning
  const int sampleTime = 50;     // ms
  const int tolerance = 1;       // difference allowed between sides

  int diff = 1;
  while (diff != 0) {    
    
    int leftWhite = readLeftSideSensors();    // sum of left IR values
    Serial.print("Left = ");
    Serial.print(leftWhite);
    int rightWhite = readRightSideSensors();  // sum of right IR values
    Serial.print("   Right = ");
    Serial.print(rightWhite);
    diff = leftWhite - rightWhite;
    Serial.print("   diff = ");
    Serial.print(diff);
    Serial.println();

    if (abs(diff) <= tolerance) {
      // robot is now perpendicular
      stopMotors();
      break;
    }

    if (diff > 0) {
      // left sees more white → robot tilted left → turn right
      setMotor(-turnSpeed, turnSpeed);
    }
    else {
      // right sees more white → robot tilted right → turn left
      setMotor(turnSpeed, -turnSpeed);
    }
    
    delay(sampleTime);
  }
  stopMotors();
}

void dashedLineFollow(String side,int baseSpeed) {

  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  digitalWrite(M2_IN2, LOW);

  leftCount = 0;
  rightCount = 0;

  IRsense();
  
  if (total == 0) {
    juncDetected = 1;
    stopMotors();
    return;   

  } else if (total == 8) {
    gostraight2(0.1,100);
    gostraight2(0.07,-100);

    if (total == 8) {
      waveScan(side);
      Serial.print("waveScan completed");
      Serial.println();
      Serial.print("wiggling started");
      Serial.println();
      wiggleStraighten(4,0.03);
      Serial.print("wiggling completed");
      Serial.println();

      if (side == "Left") {
        setMotor(100*1.27681,-100);
        delay(1000);
        stopMotors();
      } else {
        setMotor(-100*1.27681, 100);
        delay(1000);
        stopMotors();
      }
    }
    

  } else {
    float position;
    if (total > 0 && total < 8) position = weightedSum / total;
    else position = 4500;

    /* ---- ERROR ---- */      
    float d_error = position - 4500;

    /* ---- PD CONTROL ---- */      
    float P = d_error;
    float D = d_error - d_lastError;
    float correction = (Kdp * P) + (Kdd * D);

    delay(5);
    d_lastError = d_error;

    correction = constrain(correction, -100, 100);

    int leftSpeed  = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed  = constrain(leftSpeed,  90, 255);
    rightSpeed = constrain(rightSpeed, 90, 255);

    analogWrite(M1_PWM, leftSpeed);
    analogWrite(M2_PWM, rightSpeed);
  } 
}

// ULTRASONIC READING FUNCTION
// -------------------------
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1; // no reading
  delay(30);

  float distance = duration * 0.0343 / 2;
  return distance;
}

// ULTRASONIC READING FUNCTION
// -------------------------
float getDistance1() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1; // no reading
  delay(30);

  float distance = duration * 0.0343 / 2;
  return distance;
}


void barcodeRead(float d, int Speed) {

  long diff = 0;
  long prevDiff = 0;
  long s_derivative = 0;
  long s_correction = 0;

  long tickCount = (PPR * d) / wheelCircumference;

  // === PID VARIABLES ===
  float Ksp = 3;
  float Ksd = 1;

  float error = 0, prevError = 0, derivative = 0;
  float correction = 0;

  leftCount = 0;
  rightCount = 0;

  int state = 0;
  int prevState = 1;
  long startTick;
  long endTick;

  while (leftCount < tickCount || rightCount < tickCount) {
    
    state = readSideIR();

    if (state == 0 && prevState == 1) {
      startTick = (leftCount + rightCount) / 2;
      prevState = 0;

    } else if (state == 1 && prevState == 0) {
      endTick = (leftCount + rightCount) / 2;
      int tickCount = endTick - startTick;
      float whiteWidth = tickCount * cmPerTick;

      char bit = classifyWidth(whiteWidth);
      barcode += bit;

      prevState = 1;
    }
    
    // get encoder counts for the last 100ms
    long L_count = leftCount;
    long R_count = rightCount;

    // calculate error (difference)
    diff = L_count - R_count;

    // PID calculations
    s_derivative = diff - prevDiff;

    s_correction = Ksp * diff + Ksd * s_derivative;
    prevDiff = diff;

    int leftPWM = 0;
    int rightPWM = 0;

    if (Speed < 0) {
      leftPWM  = Speed*1.27681 + s_correction;
      rightPWM = Speed - s_correction;
      leftPWM  = constrain(leftPWM,  -255, -90);
      rightPWM = constrain(rightPWM, -255, -90);

    } else {
      leftPWM  = Speed*1.27681 - s_correction;
      rightPWM = Speed + s_correction;
      leftPWM  = constrain(leftPWM,  90, 255);
      rightPWM = constrain(rightPWM, 90, 255);
    }

    // Apply motor commands
    setMotor(leftPWM, rightPWM);
    Serial.println();
  }
}








/*

// ============================================
// ADDITIONAL HELPER FUNCTIONS
// ============================================

// Reset PID state (call when starting wall following)
void resetWallFollowing() {
  w_integral = 0;
  w_previousError = 0;
  previousDerivative = 0;
  isInBlindMode = false;

  // Clear filter buffer
  for (int i = 0; i < FILTER_WINDOW; i++) {
    distanceBuffer[i] = 0;
  }
  bufferIndex = 0;
}

// Check if wall following is stable
bool isWallFollowingStable() {
  return !isInBlindMode && abs(w_previousError) < 2.0; // Within 2cm of target
}



// ============================================
// CALCULATE DIFFERENTIAL DRIVE TURN RATE
// ============================================
float calculateCircularTurnRate() {
  // For a circular path, the robot's center follows a circle of radius (WALL_RADIUS + DESIRED_DISTANCE)
  float robotPathRadius = WALL_RADIUS + DESIRED_DISTANCE;

  // Differential drive: inner wheel radius vs outer wheel radius
  float innerRadius = robotPathRadius - (ROBOT_TRACK_WIDTH / 2.0);
  float outerRadius = robotPathRadius + (ROBOT_TRACK_WIDTH / 2.0);

  // Speed ratio = innerRadius / outerRadius
  return innerRadius / outerRadius;
}

// ============================================
// FILTERED DISTANCE READING
// ============================================
float getFilteredDistance() {
  float rawDistance = getDistance();

  // Validate reading
  if (rawDistance == 0 || rawDistance > MAX_VALID_DISTANCE * 100 || 
      rawDistance < MIN_VALID_DISTANCE * 10) {
    return -1; // Invalid reading
  }

  // Convert to cm
  float distanceCm = rawDistance / 10.0;

  // Add to circular buffer
  distanceBuffer[bufferIndex] = distanceCm;
  bufferIndex = (bufferIndex + 1) % FILTER_WINDOW;

  // Calculate moving average
  float sum = 0;
  int validCount = 0;
  for (int i = 0; i < FILTER_WINDOW; i++) {
    if (distanceBuffer[i] > 0) {
      sum += distanceBuffer[i];
      validCount++;
    }
  }

  if (validCount == 0) return -1;

  return sum / validCount;
}


// ============================================
// MAIN WALL FOLLOWING FUNCTION
// ============================================
/*void wallFollowing() {
  float distance = getFilteredDistance();

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | ");

  // ============================================
  // BLIND MODE - ULTRASONIC LOST
  // ============================================
  if (distance < 0) {
    if (!isInBlindMode) {
      isInBlindMode = true;
      lastValidReading = millis();
      Serial.println("ENTERING BLIND MODE - Following circular path");
    }
    
    unsigned long blindDuration = millis() - lastValidReading;
    
    if (blindDuration > BLIND_MODE_TIMEOUT) {
      // Lost wall for too long - stop
      Serial.println("WALL LOST - STOPPING");
      setMotor(0, 0);
      return;
    }
    
    // Follow circular path using differential steering
    int leftSpeed = BLIND_MODE_SPEED * TURN_RATE_CIRCULAR;
    int rightSpeed = BLIND_MODE_SPEED;
    
    leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
    
    Serial.print("BLIND MODE | Left: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right: ");
    Serial.println(rightSpeed);
    
    setMotor(leftSpeed, rightSpeed);
    return;
  }

  // ============================================
  // NORMAL MODE - VALID ULTRASONIC READING
  // ============================================
  isInBlindMode = false;
  lastValidDistance = distance;
  lastValidReading = millis();

  // -------------------------
  // PID CALCULATION
  // -------------------------
  float w_error = DESIRED_DISTANCE - distance;

  // Anti-windup for integral
  if (abs(w_error) < 3.0) {  // Only integrate when close to setpoint
    w_integral += w_error;
    w_integral = constrain(w_integral, -50, 50); // Prevent integral windup
  } else {
    w_integral *= 0.95; // Decay integral when far from setpoint
  }

  // Derivative with filtering (reduce noise)
  float w_derivative = w_error - w_previousError;
  w_derivative = DERIVATIVE_FILTER * w_derivative + 
                  (1 - DERIVATIVE_FILTER) * previousDerivative;
  previousDerivative = w_derivative;

  // PID output
  float correction = Kwp * w_error + Kwi * w_integral + Kwd * w_derivative;

  w_previousError = w_error;

  // -------------------------
  // MOTOR SPEED CALCULATION
  // -------------------------
  // Add circular path bias to maintain the curve
  float circularBias = BASE_SPEED * (1.0 - TURN_RATE_CIRCULAR) * 0.5;

  int leftMotorSpeed  = BASE_SPEED * 1.27681 - correction - circularBias;
  int rightMotorSpeed = BASE_SPEED + correction + circularBias;

  // Constrain speeds
  leftMotorSpeed  = constrain(leftMotorSpeed, MIN_SPEED, MAX_SPEED);
  rightMotorSpeed = constrain(rightMotorSpeed, MIN_SPEED, MAX_SPEED);

  // -------------------------
  // DEBUG OUTPUT
  // -------------------------
  Serial.print("Error: ");
  Serial.print(w_error, 2);
  Serial.print(" | Correction: ");
  Serial.print(correction, 2);
  Serial.print(" | Left: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" | Right: ");
  Serial.println(rightMotorSpeed);

  // -------------------------
  // SEND TO MOTORS
  // -------------------------
  setMotor(rightMotorSpeed, leftMotorSpeed);
}*/







// ===============================================================
//  FUNCTION: Read Tilt Angles (Roll, Pitch, Yaw)
//  Call this ANYTIME to get the latest angle values
// ===============================================================
bool getTiltAngles(float &roll, float &pitch, float &yaw) {
  if (!dmpReady) return false;

  uint16_t fifoCount = mpu.getFIFOCount();

  if (fifoCount < 42) return false;   // Not enough data yet

  if (fifoCount >= 1024) {            // FIFO overflow protection
    mpu.resetFIFO();
    return false;
  }

  // Read exactly 42 bytes from FIFO
  mpu.getFIFOBytes(fifoBuffer, 42);

  // Convert data to readable orientation
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Convert radians → degrees
  yaw   = ypr[0] * 180 / M_PI;
  pitch = ypr[1] * 180 / M_PI;
  roll  = ypr[2] * 180 / M_PI;

  return true;
}




void beforeRamp( int baseSpeed) {

  long diff = 0;
  long prevDiff = 0;
  long s_derivative = 0;
  long s_correction = 0;

  // === PID VARIABLES ===
  float Ksp = 3;
  float Ksd = 1;

  float error = 0, prevError = 0, derivative = 0;
  float correction = 0;
    
  while (pitch <= 3 && pitch > -1) {

    getTiltAngles(roll, pitch, yaw);
    
    leftCount = 0;
    rightCount = 0;

    // get encoder counts for the last 100ms
    long L_count = leftCount;
    long R_count = rightCount;

    // calculate error (difference)
    diff = L_count - R_count;

    // PID calculations
    s_derivative = diff - prevDiff;

    s_correction = Ksp * diff + Ksd * s_derivative;
    prevDiff = diff;

    int leftPWM = 0;
    int rightPWM = 0;

    if (baseSpeed < 0) {
      leftPWM  = baseSpeed*1.27681 + s_correction;
      rightPWM = baseSpeed - s_correction;
      leftPWM  = constrain(leftPWM,  -255, -90);
      rightPWM = constrain(rightPWM, -255, -90);

    } else {
      leftPWM  = baseSpeed*1.27681 - s_correction;
      rightPWM = baseSpeed + s_correction;
      leftPWM  = constrain(leftPWM,  90, 255);
      rightPWM = constrain(rightPWM, 90, 255);
    }
    
    // Apply motor commands
    setMotor(leftPWM, rightPWM);
  } 
}


void rampClimbUp(int baseSpeed) {

  leftCount = 0;
  rightCount = 0;
  
  // get encoder counts for the last 100ms
  long L_count = leftCount;
  long R_count = rightCount;
  long prevDiff = 0;

  int leftPWM = 0;
  int rightPWM = 0;

  // === PID VARIABLES ===
  float Ksp = 3;
  float Ksd = 1;

  //tcaSelect();
  getTiltAngles(roll, pitch, yaw);

  float startPitch = pitch;
  float currentPitch = pitch;

  while (currentPitch - startPitch > -5) {
    
    //tcsSelect();
    getTiltAngles(roll, pitch, yaw);
    delay(10);
    currentPitch = pitch;

    // calculate error (difference)
    long diff = L_count - R_count;

    // PID calculations
    long s_derivative = diff - prevDiff;

    long s_correction = Ksp * diff + Ksd * s_derivative;
    prevDiff = diff;

    if (baseSpeed < 0) {
      leftPWM  = baseSpeed * (currentPitch - startPitch) * 5 * 1.27681 + s_correction;
      rightPWM = baseSpeed * (currentPitch - startPitch) * 5 - s_correction;
      leftPWM  = constrain(leftPWM,  -255, -90);
      rightPWM = constrain(rightPWM, -255, -90);

    } else {
      leftPWM  = baseSpeed * (currentPitch - startPitch) * 5 * 1.27681 - s_correction;
      rightPWM = baseSpeed * (currentPitch - startPitch) * 5 + s_correction;
      leftPWM  = constrain(leftPWM,  90, 255);
      rightPWM = constrain(rightPWM, 90, 255);
    }
  
    // Apply motor commands
    setMotor(leftPWM, rightPWM);
  }
  return;
}


    
void onTheTop(int baseSpeed) {

  leftCount = 0;
  rightCount = 0;
  
  // get encoder counts for the last 100ms
  long L_count = leftCount;
  long R_count = rightCount;
  long prevDiff = 0;

  int leftPWM = 0;
  int rightPWM = 0;

  // === PID VARIABLES ===
  float Ksp = 3;
  float Ksd = 1;

  while (pitch >= -3) {

    getTiltAngles(roll, pitch, yaw);

    // calculate error (difference)
    long diff = L_count - R_count;

    // PID calculations
    long s_derivative = diff - prevDiff;

    long s_correction = Ksp * diff + Ksd * s_derivative;
    prevDiff = diff;

    int leftPWM = 0;
    int rightPWM = 0;

    if (baseSpeed < 0) {
      leftPWM  = baseSpeed*1.27681 + s_correction;
      rightPWM = baseSpeed - s_correction;
      leftPWM  = constrain(leftPWM,  -255, -90);
      rightPWM = constrain(rightPWM, -255, -90);

    } else {
      leftPWM  = baseSpeed*1.27681 - s_correction;
      rightPWM = baseSpeed + s_correction;
      leftPWM  = constrain(leftPWM,  90, 255);
      rightPWM = constrain(rightPWM, 90, 255);
    }
    
    // Apply motor commands
    setMotor(leftPWM, rightPWM); 
  }
}


void rampClimbDown(int baseSpeed) {

  //tcaSelect();
  getTiltAngles(roll, pitch, yaw);
  
  float startPitch = pitch;
  float currentPitch = pitch;

  while (currentPitch - startPitch < -5) {
    getTiltAngles(roll, pitch, yaw);
    gostraight(0.05,baseSpeed);
    delay(100);
  }
}



void outerWallFollow(int baseSpeed, float desiredDistance, float distanceToWall) {

  float distToWall = distanceToWall; //cm

  float Kwp = 10.0;     // proportional gain
  float Kwi = 0.0;     // usually small or zero for wall following
  float Kwd = 40.0;     // derivative gain

  float w_previousError = 0;
  float w_integral = 0;

  IRsense();
  delay(10);

  while (distToWall < 20 && total != 0) {

    distToWall = getDistance();
    IRsense();
    delay(10);
   
    if (distToWall <= 0) {
      Serial.println("Ultrasonic error: No reading");
      gostraight(0.02,100);

    } else {

      // PID ERROR CALCULATION
      // -------------------------
      float w_error = desiredDistance - distToWall;
      w_integral += w_error;
      float w_derivative = w_error - w_previousError;

      float correction = Kwp * w_error + Kwi * w_integral + Kwd * w_derivative;
      w_previousError = w_error;

      int leftMotorSpeed  = baseSpeed * 1.27681 + correction;
      int rightMotorSpeed = baseSpeed - correction;

      // LIMIT SPEEDS 0–255
      leftMotorSpeed  = constrain(leftMotorSpeed , 90, 255);
      rightMotorSpeed = constrain(rightMotorSpeed, 90, 255);
      Serial.print(leftMotorSpeed);
      Serial.print(" | ");
      Serial.print(rightMotorSpeed);
      Serial.println();

      setMotor(leftMotorSpeed,rightMotorSpeed);
    }
  }
  stopMotors();
}


void outerWallFollowTask5(int baseSpeed, float desiredDistance, float distanceToWall) {

  float distToWall = distanceToWall; //cm

  float Kwp = 15.0;     // proportional gain
  float Kwi = 0.0;     // usually small or zero for wall following
  float Kwd = 80.0;     // derivative gain

  float w_previousError = 0;
  float w_integral = 0;

  while (distToWall < 20) {

    distToWall = getDistance();
   
    if (distToWall <= 0) {
      Serial.println("Ultrasonic error: No reading");
      gostraight(0.02,100);

    } else {

      float w_error = desiredDistance - distToWall;
      w_integral += w_error;
      float w_derivative = w_error - w_previousError;

      float correction = Kwp * w_error + Kwi * w_integral + Kwd * w_derivative;
      w_previousError = w_error;

      int leftMotorSpeed  = baseSpeed * 1.27681 + correction;
      int rightMotorSpeed = baseSpeed - correction;

      // LIMIT SPEEDS 0–255
      leftMotorSpeed  = constrain(leftMotorSpeed , 90, 255);
      rightMotorSpeed = constrain(rightMotorSpeed, 90, 255);
      Serial.print(leftMotorSpeed);
      Serial.print(" | ");
      Serial.print(rightMotorSpeed);
      Serial.println();

      setMotor(leftMotorSpeed,rightMotorSpeed);
    }
  }
  stopMotors();
}



void circularMotion(int baseSpeed, float radiusCM) {
                    

  // PID variables for maintaining speed ratio
  float leftIntegral = 0, rightIntegral = 0;
  float leftPrevError = 0, rightPrevError = 0;

  // PID gains - adjust these if needed
  float Kip = 2.0;
  float Kii = 0.5;
  float Kid = 0.1;
  
  float turnRatio = (radiusCM + WHEEL_BASE/2) / (radiusCM - WHEEL_BASE/2);

  leftCount = 0;
  rightCount = 0;

  // Target speeds
  int leftTarget = baseSpeed * turnRatio;
  int rightTarget = baseSpeed;
  
  // Calculate speed difference (error) between wheels
  // We want to maintain the ratio between left and right encoder increments
  static long prevLeft = 0, prevRight = 0;
  
  while (true) {

    long leftDiff = leftCount - prevLeft;
    long rightDiff = rightCount - prevRight;
    
    prevLeft = leftCount;
    prevRight = rightCount;
    
    // Calculate errors
    float leftError = leftTarget - (leftDiff * 10);  // Scale factor
    float rightError = rightTarget - (rightDiff * 10);
    
    // PID for left motor
    leftIntegral += leftError;
    leftIntegral = constrain(leftIntegral, -100, 100);
    float leftDerivative = leftError - leftPrevError;
    float leftPID = Kip * leftError + Kii * leftIntegral + Kid * leftDerivative;
    leftPrevError = leftError;
    
    // PID for right motor
    rightIntegral += rightError;
    rightIntegral = constrain(rightIntegral, -100, 100);
    float rightDerivative = rightError - rightPrevError;
    float rightPID = Kip * rightError + Kii * rightIntegral + Kid * rightDerivative;
    rightPrevError = rightError;
    
    // Calculate final motor speeds
    int leftSpeed = constrain(leftTarget + leftPID, -255, 255);
    int rightSpeed = constrain(rightTarget + rightPID, -255, 255);
    
    setMotor(leftSpeed,rightSpeed);
  }
}

void alignPerpendicular(float distance, int baseSpeed) {

  IRsense();
  if (total <= 2) {

    while (total == 0) {
      gostraight(distance, baseSpeed);
      IRsense();
      int leftTotal = 0;
      int rightTotal = 0;
      for (int i=0; i<3; i++) {
        if (sensor[i] < 500) {
          rightTotal++;
        }
      }
      for (int i=5; i<8 ; i++) {
        if (sensor[i] < 500) {
          leftTotal++;
        }
      }

      if (leftTotal <= rightTotal) {
        setMotor(-100,100);
      } else {
        setMotor(100,-100);
      }
    }
    
  }

  if (total == 0) {
    gostraight(distance, baseSpeed);
    IRsense();
    int leftTotal = 0;
    int rightTotal = 0;
    for (int i=0; i<3; i++) {
      if (sensor[i] < 500) {
        rightTotal++;
      }
    }
    for (int i=5; i<8 ; i++) {
      if (sensor[i] < 500) {
        leftTotal++;
      }
    }

    if (leftTotal <= rightTotal) {
      setMotor(-100,100);
    } else {
      setMotor(100,-100);
    }
  }
  stopMotors();
}

// Returns distance in mm, or -1 on error / out of range.
int16_t readToF_mm(uint8_t channel) {
  tcaSelect(channel);
  delay(10); // allow bus settle

  // Make sure sensor is initialized on this channel.
  // Some designs re-init every time (robust); if you already
  // initialized at setup and won't power-cycle, you can skip re-init.
  if (!tof.begin()) {
    // sensor did not respond
    return -1;
  }

  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false); // blocking single measurement

  if (measure.RangeStatus == 4) {   // 4 means out of range according to library examples
    return -1;
  } else {
    return (int16_t)measure.RangeMilliMeter;
  }
}