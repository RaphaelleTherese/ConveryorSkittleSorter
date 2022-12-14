#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>

// ----- VARIABLES -----
// === MICRO SERVOS ===
int numPins = 4;
#define SERVO1 8
#define SERVO2 7
#define SERVO3 6
#define SERVO4 5
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servos[4] = {servo1, servo2, servo3, servo4};

// === COLOR SENSOR === 
/* Initialise with specific int time and gain values */
// SCL = A5, SDA = A4
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// === MOTOR SHIELD === 
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *conveyor = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

// === POSITIONS ===
// RED, ORANGE, YELLOW, GREEN, PURPLE
#define COLORS_SIZE 40
int colors[COLORS_SIZE];
int wait_times[5] = {450, 300, 200, 100, 600};
int input_pos = 0;
int output_pos = 0;
int beg = 1;

// === FEEDER ===
Servo myservo;  // create servo object to control a servo
int pos1 = 100;    // variable to store the servo position
int pos2 = 180;    // variable to store the servo position

#define NUM_COLORS  6

// Skittle colours to indices
#define COL_RED     0
#define COL_GREEN   1
#define COL_ORANGE  2
#define COL_YELLOW  3
#define COL_PURPLE  4
#define COL_NOTHING 5

// Names for colours
#define COLNAME_RED     "RED"
#define COLNAME_GREEN   "GREEN"
#define COLNAME_ORANGE  "ORANGE"
#define COLNAME_YELLOW  "YELLOW"
#define COLNAME_PURPLE  "PURPLE"
#define COLNAME_NOTHING "NOTHING"

// RGB channels in the array
#define CHANNEL_R   0
#define CHANNEL_G   1
#define CHANNEL_B   2

// Training colours (populate these manually, but these vectors must be of unit length (i.e. length 1))
float trainingColors[3][NUM_COLORS];    // 3(rgb) x NUM_COLORS.

// Last read colour
float rNorm = 0.0f;
float gNorm = 0.0f;
float bNorm = 0.0f;
float hue = 0.0f;
float saturation = 0.0f;
float brightness = 0.0f;

// Last classified class
int lastClass = -1;
float lastCosine = 0;

void setup() {
  // Setup servos
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  myservo.attach(4);  // attaches the servo on pin 9 to the servo object
  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  while (!Serial);
  Serial.println("Stepper test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  myMotor->setSpeed(10);  // 10 rpm
  conveyor->setSpeed(20);  // 10 rpm
  
  // Populate array of training colours for classification. 
  initializeTrainingColors();
  
  // Setup color sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}

void loop() {
  resetServos();
  // Step 1: Get normalized colour vector
  delay(1000);
  getNormalizedColor();
  int colClass = getColorClass();   

  // Step 2: Output colour
  Serial.print("R: "); Serial.print(rNorm, 3); Serial.print("  ");
  Serial.print("G: "); Serial.print(gNorm, 3); Serial.print("  ");
  Serial.print("B: "); Serial.print(bNorm, 3); Serial.print("  ");  
  Serial.print("H: "); Serial.print(hue, 3); Serial.print("  ");
  Serial.print("S: "); Serial.print(saturation, 3); Serial.print("  ");
  Serial.print("B: "); Serial.print(brightness, 3); Serial.print("  ");
  
  printColourName(colClass);  
  Serial.print(" (cos: "); Serial.print(lastCosine); Serial.print(") ");
  Serial.println("");
  myMotor->step(25, BACKWARD, MICROSTEP);
  delay(500);
  myservo.write(pos2);              // tell servo to go to position in variable 'pos'
  delay(500);
  myservo.write(pos1);              // tell servo to go to position in variable 'pos'

  Serial.println(colClass);
  pushSingleFull(colClass);

  // == TESTING ==
  // Testing with colors value
//  if (beg == 1) {
//    resetServos();
//    setUpTestColors();
//    beg = 0;
//  } else {
//    sortColor();
//  }

  // Test individual 
//  pushSingle(0);

  // Test all servos
//    testServos();

  // All the way around
//  resetServos();
//  myMotor -> step(1500, BACKWARD, DOUBLE);
//  delay(500);
}

void testServos() {
  resetServos();
  conveyor -> step(100, BACKWARD, DOUBLE);
  
    for (int i = 0; i < numPins; i++) {
//      servos[i].write(45);

      servos[i].write(135);
      delay(1000);
      servos[i].write(45);
  } 
}

/*
 * Colour sensing
 */
void initializeTrainingColors() {
  // Skittle: red
  trainingColors[CHANNEL_R][COL_RED] = 0.915;
  trainingColors[CHANNEL_G][COL_RED] = 0.286;
  trainingColors[CHANNEL_B][COL_RED] = 0.286;

  // Skittle: green
  trainingColors[CHANNEL_R][COL_GREEN] = 0.523;
  trainingColors[CHANNEL_G][COL_GREEN] = 0.785;
  trainingColors[CHANNEL_B][COL_GREEN] = 0.331;

  // Skittle: orange
  trainingColors[CHANNEL_R][COL_ORANGE] = 0.920;
  trainingColors[CHANNEL_G][COL_ORANGE] = 0.323;
  trainingColors[CHANNEL_B][COL_ORANGE] = 0.224;

  // Skittle: yellow
  trainingColors[CHANNEL_R][COL_YELLOW] = 0.795;
  trainingColors[CHANNEL_G][COL_YELLOW] = 0.558;
  trainingColors[CHANNEL_B][COL_YELLOW] = 0.238;

  // Skittle: purple
  trainingColors[CHANNEL_R][COL_PURPLE] = 0.735;
  trainingColors[CHANNEL_G][COL_PURPLE] = 0.515;
  trainingColors[CHANNEL_B][COL_PURPLE] = 0.441;

  // Nothing
  trainingColors[CHANNEL_R][COL_NOTHING] = 0.684;
  trainingColors[CHANNEL_G][COL_NOTHING] = 0.570;
  trainingColors[CHANNEL_B][COL_NOTHING] = 0.456;
}


void getNormalizedColor() {
  uint16_t r, g, b, c, colorTemp, lux;  
  tcs.getRawData(&r, &g, &b, &c);

  float lenVec = sqrt((float)r*(float)r + (float)g*(float)g + (float)b*(float)b);

  // Note: the Arduino only has 2k of RAM, so rNorm/gNorm/bNorm are global variables. 
  rNorm = (float)r/lenVec;
  gNorm = (float)g/lenVec;
  bNorm = (float)b/lenVec;

  // Also convert to HSB:
  RGBtoHSV(rNorm, gNorm, bNorm, &hue, &saturation, &brightness);
}


int getColorClass() {
  float distances[NUM_COLORS] = {0.0f};

  // Step 1: Compute the cosine similarity between the query vector and all the training colours. 
  for (int i=0; i<NUM_COLORS; i++) {
    // For normalized (unit length) vectors, the cosine similarity is the same as the dot product of the two vectors.
    float cosineSimilarity = rNorm*trainingColors[CHANNEL_R][i] + gNorm*trainingColors[CHANNEL_G][i] + bNorm*trainingColors[CHANNEL_B][i];
    distances[i] = cosineSimilarity;

    // DEBUG: Output cosines
    Serial.print("   C"); Serial.print(i); Serial.print(": "); Serial.println(cosineSimilarity, 3);
  }

  // Step 2: Find the vector with the highest cosine (meaning, the closest to the training color)
  float maxVal = distances[0];
  int maxIdx = 0;
  for (int i=0; i<NUM_COLORS; i++) {
    if (distances[i] > maxVal) {
      maxVal = distances[i];
      maxIdx = i;
    }
  }

  // Step 3: Return the index of the minimum color
  lastCosine = maxVal;
  lastClass = maxIdx;
  return maxIdx;
}


// Convert from colour index to colour name.
void printColourName(int colIdx) {
  switch (colIdx) {
    case COL_RED:
      Serial.print(COLNAME_RED);
      break;
    case COL_GREEN:
      Serial.print(COLNAME_GREEN);
      break;
    case COL_ORANGE:
      Serial.print(COLNAME_ORANGE);
      break;
    case COL_YELLOW:
      Serial.print(COLNAME_YELLOW);
      break;
    case COL_PURPLE:
      Serial.print(COLNAME_PURPLE);
      break;
    case COL_NOTHING:
      Serial.print(COLNAME_NOTHING);
      break;
    default:
      Serial.print("ERROR");
      break;
  }
}

/*
 * Colour converstion
 */

// RGB to HSV.  From https://www.cs.rit.edu/~ncs/color/t_convert.html . 
void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v ) {  
  float minVal = min(min(r, g), b);
  float maxVal = max(max(r, g), b);
  *v = maxVal;       // v
  float delta = maxVal - minVal;
  if( maxVal != 0 )
    *s = delta / maxVal;   // s
  else {
    // r = g = b = 0    // s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }
  if( r == maxVal )
    *h = ( g - b ) / delta;   // between yellow & magenta
  else if( g == maxVal )
    *h = 2 + ( b - r ) / delta; // between cyan & yellow
  else
    *h = 4 + ( r - g ) / delta; // between magenta & cyan
  *h *= 60;       // degrees
  if( *h < 0 )
    *h += 360;
}

void pushSingle(int i) {
  resetServos();
  delay( 5000 );
  
  if (i == 4) {
    conveyor -> step(wait_times[i], BACKWARD, DOUBLE);
    return;
  }
  conveyor -> step(wait_times[i] - 10, BACKWARD, DOUBLE);
  servos[i].write(135);
  delay(1000);
  servos[i].write(45);

  delay(500);
  conveyor -> step(10, BACKWARD, DOUBLE);
  servos[i].write(135);
  delay(1000);
  servos[i].write(45);
  
  delay(500);
  conveyor -> step(10, BACKWARD, DOUBLE);
  servos[i].write(135);
  delay(1000);
  servos[i].write(45);
  
  delay( 5000 );
}

// 1500
void pushSingleFull(int i) {
  resetServos();
  int fullRound = 1000;

  if (i >= 4) {
    conveyor -> step(fullRound, BACKWARD, DOUBLE);
    return;
  }
  
  fullRound = wait_times[i];
  int moveTime = 30;
  int tolerance = 150;
  for (int j = 0; j <= fullRound + tolerance; j += moveTime) {
    delay(500);
    conveyor -> step(moveTime, BACKWARD, DOUBLE);
    servos[i].write(135);
    delay(500);
    servos[i].write(45);
  }
}

void resetServos() {
    for (int i = 0; i < numPins; i++) {
      servos[i].write(45);
  }
}

void setUpTestColors() {
  Serial.println("Setting up test colors");
  for (int i = 0; i < COLORS_SIZE; i++) {
    int color = random(0, 5);
    colors[i] = color;
  }
  
}

void sortColor() {
  int cur = colors[output_pos];
  Serial.println(cur);
//  pushSingle(cur);
  pushSingleFull(cur);
  output_pos++;
}
