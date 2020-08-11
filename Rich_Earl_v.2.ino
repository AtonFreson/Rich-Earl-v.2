#define DEBUG false  //increase performance by turning to false. WARNING: Will thusly not print to serial.

#define serial if(DEBUG) SerialUSB  //defines a local serial if DEBUG is enabled.

#include <Servo.h>


// Motors

Servo servoLeft; //min: 950(49.45 rpm), 0°: -=-,  180°: -=-,  max: 2130(49.25 rpm),  dödzon: 1443-1541    MOTOR REVERSED
int leftReverse = 950;
int leftForward = 2130;
int leftReverseDead = 1445;
int leftForwardDead = 1539;

float leftTable[40] = {0, 0.4510, 0.7356, 1.2909, 2.0538, 2.8594, 3.5986, 4.2498, 4.8532, 5.4691, 6.1435, 6.8906, 7.6911, 8.5041, 9.2835, 9.9956, 10.6302, 11.2041, 11.7560, 12.3351, 12.9849, 13.7293, 14.5616, 15.4429, 16.3097, 17.0897, 17.7249, 18.1943, 18.5297, 18.8183, 19.1829, 19.7410, 20.5451, 21.5251, 22.4672, 23.0918, 23.3251, 23.9022, 27.4960, 40.6293};


Servo servoRight; //min: 2060(55.80 rpm), 0°: 1876,  180°: 1100,  max: 940(53.40 rpm),  dödzon: 1443-1542
int rightReverse = 1876;
int rightForward = 1100;
int rightReverseDead = 1540;
int rightForwardDead = 1445;

float rightTable[40] = {0, 0.471, 0.9218, 1.9024, 2.9863, 4.0932, 5.1878, 6.2612, 7.3173, 8.3649, 9.4131, 10.4683, 11.5339, 12.6104, 13.6954, 14.7855, 15.8763, 16.9639, 18.045, 19.1175, 20.1806, 21.235, 22.2824, 23.3252, 24.3663, 25.408, 26.4516, 27.4967, 28.5402, 29.5756, 30.5927, 31.5769, 32.5102, 33.3724, 34.1443, 34.8124, 35.3779, 35.8685, 36.3573, 36.9873};



// Garmin LIDAR-Lite v4 LED

#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"

LIDARLite_v4LED myLidarLite;

#define MonitorPin    10
#define TriggerPin    11

long duration;
int distance;

unsigned long timer1;
unsigned long timer2;



// Servo motor

Servo servo;
float angleTable[37] = {0, 4.4, 8.74, 13.14, 17.63, 22.18, 26.78, 31.4, 36.06, 40.75, 45.48, 50.23, 54.98, 59.72, 64.47, 69.28, 74.24, 79.45, 84.96, 90.74, 96.69, 102.62, 108.36, 113.78, 118.91, 123.88, 128.88, 134.02, 139.29, 144.51, 149.53, 154.41, 159.38, 164.63, 169.87, 174.78, 180}; //value given to servo for every 5th degree



// Mapping

const int avrAmount = 3;
int avrArray[avrAmount] = { };
int avrCounter = 0;

const int measurements = 60;//30, 45, 60, 90, 180
unsigned int avrArrayTot [measurements+1];

int minDistance, maxDistance, minDistCone, bestAngle;

Servo servoDisp;


// Curve Fitting

const int nrValuePoints = measurements;    //To find the size of arrays that will store x,y, and z values
int valuePoints[nrValuePoints+1];

const int polyDeg = 8;    // n is the degree of Polynomial
int tolerance = 100;     //The tolerance of the maximization calculation. Lower is more accurate, higher gives more performance.

double coeffs[polyDeg+1];
int turnAngle;



// Rich Earl v.2

const int width = 111; // Width of car in mm
const int maxSpeed = 39; //Maximum allowed speed, determined by maximum rotational motor speed.

const float cruiseSpeed = 38; //Sets the standard maximum speed. 0-39 cm/s

//const float convConst = 0.75; //Experimentally determined conversion constant, 
               //between engine voltage and vehicle velocity.

const double degToRadAndMS = 17.45329252/10; // = 2pi/360*1000 (1000 milliseconds for delaytimer)

//const float totConsts = convConst * degToRadAndMS;




void setup() {  
  delay(1000);  //IMPORTANT
  if(DEBUG){
    serial.begin(9600);
    while(!SerialUSB);
  }
  serial.println("\nDEBUG mode turned ON.");
  serial.println("\nStarting...");
  
  
  servoLeft.attach(6,440,2510);  // (pin, min, max)
  servoRight.attach(5,440,2510);  // (pin, min, max)
  
    
  Wire.begin();
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif


  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);

  pinMode(MonitorPin, INPUT);
  pinMode(TriggerPin, OUTPUT);
  digitalWrite(TriggerPin, LOW);

  myLidarLite.configure(0);

  uint8_t dataByte = 0x00; //standard 0x20
  myLidarLite.write(0xEB, &dataByte, 1, 0x62); // Turn off high accuracy mode

 
  servo.attach(7,362,2302);  // (pin, min, max)          min: 337,  0°: 362,  180°: 2302,  max: 2587
  
  servoDisp.attach(2,480,2302);  // (pin, min, max)          min: 337,  0°: 362,  180°: 2302,  max: 2587
  
  delay(100);
}



void loop() {
  servo.write(calibrateAngle(90));
  waitWall(20);
  servo.write(calibrateAngle(0));
  delay(100);
  /*
  servo.write(calibrateAngle(180));
  for (int i=17;i<=19;i++) {
    forward(i*2+1, 250, false);

    serial.print("Speed: ");
    serial.print(i*2);
    serial.print("  RPM: ");
    serial.println((i*2+1)*49.25/39);
    
    waitWall(15);
  }
  
  
  for (int i = 1; i<=10;i++) {
    forward(i*4, 250, false);
  }
  forward(38, 750, false);
  turn(38, 180, 230);
  turn(38, -180, 230);
  
  
  //accelerate(1, cruiseSpeed, 8);
  //forward(cruiseSpeed, 750, false);
  */
  while (true) {
    LIDARmapping();
    if (minDistance < 20 or maxDistance < 40) {
      break;
    }

    if (nrValuePoints != measurements) {
      for (int i = 0; i < nrValuePoints; i++) {
        valuePoints[i] = avrArrayTot[int(i*floor((measurements+1)/nrValuePoints)+floor((measurements+1)/nrValuePoints/2))];
      }
    } else {
      for (int i = 0; i < nrValuePoints; i++) {
        valuePoints[i] = avrArrayTot[i];
      }
    }
    
    calcLeastSquare(valuePoints, polyDeg, nrValuePoints, coeffs);
    turnAngle = 90 - calcMax(coeffs, polyDeg, tolerance);

    servoDisp.write(constrain(90 - turnAngle,0,180));

    serial.print("Turning ");
    serial.print(abs(turnAngle));
    serial.print("° to the ");
    if (DEBUG and turnAngle<0){serial.println("left.");}else{serial.println("right.");}
    
    if (maxDistance < 60) {
      turn(cruiseSpeed, 180, 111);
      forward(cruiseSpeed, 10, false);
      
    } else if (minDistCone > 40) {
      if (turnAngle > -5 and turnAngle < 5) {
        forward(cruiseSpeed, 10, false);
      } else {
        turn(cruiseSpeed, turnAngle, 280);   //111*2.5
        forward(cruiseSpeed, 10, false);
      }
      
    } else {
      if (turnAngle > -5 and turnAngle < 5) {
        forward(cruiseSpeed, 10, false);
      } else {
        turn(cruiseSpeed, turnAngle, 111);
        forward(cruiseSpeed, 10, false);
      }
    }
  }
  
  


  // Now turn off motors

  //accelerate(cruiseSpeed, 10, 16);

  forward(0, 1000, false);
}



void turn(float speed1, signed int angle, float radius) {
  float speed2 = speed1 * (1 - width/radius);
  int turnTime = abs(angle) * radius * degToRadAndMS / speed1;
  

  if (DEBUG) {
    serial.println();
    serial.print("Left motor: ");
    if (angle>0){serial.print(speed1);}else{serial.print(speed2);}
    serial.print("    Right motor: ");
    if (angle<0){serial.println(speed1);}else{serial.println(speed2);}
  }

  
  if (angle < 0) {
    leftMotor(speed2);
    rightMotor(speed1);
    serial.print("Turning left for ");
  } else {
    leftMotor(speed1);
    rightMotor(speed2);
    serial.print("Turning right for ");
  }
  
  serial.print(turnTime);
  serial.print("ms...");
  
  delay(turnTime); //delays for the duration of the turn (in ms);
  
  serial.println("  Done!");
  serial.println();
  
  leftMotor(speed1);
  rightMotor(speed1);
}


void forward(float speedIn, int delayTime, bool untilWall) {  //delayTime is used as the trigger distance (in cm) to the wall if untilWall is set to true.
  leftMotor(speedIn);
  rightMotor(speedIn);
  
  if (untilWall == false) {
    serial.print("Going forward for ");
    serial.print(delayTime);
    serial.print("ms...");
  
    delay(delayTime);

    serial.println("  Done!");
  } else {
  
    serial.print("Going forward until a wall is hit...");

    distance = 1000;
    //delayTime is used as the trigger distance (in cm) to the wall if untilWall is set to true.
    do {
      delay(5);
      uint16_t distance;
      uint8_t  newDistance;
      do {
        newDistance = distanceContinuousGpio(&distance);
      } while (!newDistance);
    } while (distance > delayTime);
  
    serial.println("  Stopping!");

    leftMotor(0);
    rightMotor(0);
  }
}


void waitWall(int maxDistValue) {
  leftMotor(0);
  rightMotor(0);
  
  serial.print("Waiting until a blockage is created and/or cleared...");

  distance = 1000;
  bool wallDelay = true;
  bool checkBool = true;
  
  for (int i=0;i<20;i++){
    delay(10);
    uint16_t distance;
    uint8_t  newDistance;
    do {
      newDistance = distanceContinuousGpio(&distance);
    } while (!newDistance);
  }
  
  do {
    delay(100);
    uint16_t distance;
    uint8_t  newDistance;
    do {
      newDistance = distanceContinuousGpio(&distance);
    } while (!newDistance);

    if (distance <= maxDistValue and wallDelay == true) {
      wallDelay = false;
    }
    
    checkBool = (wallDelay == true or distance <= maxDistValue);
  } while (checkBool);
  
  serial.println("  Continuing!");
  delay(300);
  
  for (int i=0;i<20;i++){
    delay(10);
    uint16_t distance;
    uint8_t  newDistance;
    do {
      newDistance = distanceContinuousGpio(&distance);
    } while (!newDistance);
  }
}


void leftMotor(float speedIn) {
  int sign = (speedIn >= 0) - (speedIn < 0);
  if (abs(speedIn) > maxSpeed) {
    speedIn = sign*maxSpeed;
  }
  
  if (sign == 1) {
    servoLeft.writeMicroseconds(round(mapFloat(calibrateSpeed(speedIn,leftTable),0,maxSpeed,leftForwardDead,leftForward)));
  } else {
    servoLeft.writeMicroseconds(round(mapFloat(abs(speedIn),0,maxSpeed,leftReverseDead,leftReverse)));
  }
}


void rightMotor(float speedIn) {
  int sign = (speedIn >= 0) - (speedIn < 0);
  if (abs(speedIn) > maxSpeed) {
    speedIn = sign*maxSpeed;
  }
  
  if (sign == 1) {
    servoRight.writeMicroseconds(round(mapFloat(calibrateSpeed(speedIn,rightTable),0,maxSpeed,rightForwardDead,rightForward)));
  } else {
    servoRight.writeMicroseconds(round(mapFloat(abs(speedIn),0,maxSpeed,rightReverseDead,rightReverse)));
  }
}


uint8_t distanceContinuousGpio(uint16_t * distance) {
  uint8_t newDistance = 0;

  if (myLidarLite.getBusyFlagGpio(MonitorPin) == 0) {
    // Trigger the next range measurement
    myLidarLite.takeRangeGpio(TriggerPin, MonitorPin);

    // Read new distance data from device registers
    *distance = myLidarLite.readDistance();

    // Report to calling function that we have new data
    newDistance = 1;
  }

  return newDistance;
}


void accelerate(int beginningSpeed, int endSpeed, int acceleration) {
  int h = 1;
  int currentSpeed = beginningSpeed;
  
  if (beginningSpeed > endSpeed) {
    h = -1;
  }

  while (currentSpeed != endSpeed) {
    currentSpeed += h*acceleration;
    
    if (h > 0) {
      if (currentSpeed > endSpeed) {
        currentSpeed = endSpeed;
      }
    } else {
      if (currentSpeed < endSpeed) {
        currentSpeed = endSpeed;
      }
    }
    
    leftMotor(currentSpeed);
    rightMotor(currentSpeed);

    delay(10);
  }
}


double * calcLeastSquare(int *y, int n, int N, double *a) {
  //source: https://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/
  //Updated: https://www.bragitoff.com/2018/06/polynomial-fitting-c-program/
  
  int i,j,k;
  
  double x[N];
  for (i=0;i<N;i++) {
    x[i] = i*floor(181/N)+floor(181/N/2);
  }
  
  double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  for (i=0;i<2*n+1;i++)
  {
    X[i]=0;
    for (j=0;j<N;j++)
      X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  }
  double B[n+1][n+2];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
  for (i=0;i<=n;i++)
    for (j=0;j<=n;j++)
    B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  for (i=0;i<n+1;i++)
  {    
    Y[i]=0;
    for (j=0;j<N;j++)
      Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  }
  for (i=0;i<=n;i++)
    B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
  n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
  
  for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
    for (k=i+1;k<n;k++)
    if (B[i][i]<B[k][i])
    for (j=0;j<=n;j++)
  {
    double temp=B[i][j];
    B[i][j]=B[k][j];
    B[k][j]=temp;
  }

  for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
    for (k=i+1;k<n;k++)
  {
    double t=B[k][i]/B[i][i];
    for (j=0;j<=n;j++)
      B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
  }
  for (i=n-1;i>=0;i--)                //back-substitution
  {                        //x is an array whose values correspond to the values of x,y,z..
    a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
    for (j=0;j<n;j++)
      if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
      a[i]=a[i]-B[i][j]*a[j];
      a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
  }

  if (DEBUG) {
    serial.println("\nThe values of the coefficients are as follows:\n");
    for (i=0;i<n;i++) {
      serial.print("x^");
      serial.print(i);
      serial.print(" = ");
      //serial.println(a[i]);
      serial.println(float2s(a[i], 5));            // Print the values of x^0,x^1,x^2,x^3,....    
    }
  }
}


int calcMax(double *funcCoeffs, int polyDeg, int tolerance) {
  double derivCoeffs[polyDeg], secDerivCoeffs[polyDeg-1];
  int i;
  int maxVal = 0;
  double es, derivVal, secDerivVal, tempScore, polyVal;
  double maxScore, bestMin = 0;
  
  //formulates coeffs for first and second derivative of function.
  for (i=1;i<=polyDeg;i++) {
    derivCoeffs[i-1] = funcCoeffs[i]*i;
    
    if (i>1) {
      secDerivCoeffs[i-2] = derivCoeffs[i-1]*(i-1);
    }

    if (DEBUG) {
      if (i==1){serial.println("");}
      serial.print("x^");
      serial.print(i-1);
      serial.print(" = ");
      serial.println(float2s(derivCoeffs[i-1], 5));
    }
    
  }
  
  //Perform Newton iterations: https://mathworld.wolfram.com/NewtonsMethod.html
  for (i=0;i<polyDeg-1;i++) {
    es = (110+i*220) / (polyDeg-1) - 20; //220/(2*(polyDeg-1))+i*220/(polyDeg-1)-20
    
    int times = 0;
    
    do {
      derivVal = polyCalc(derivCoeffs, es, polyDeg-1);
      secDerivVal = polyCalc(secDerivCoeffs, es, polyDeg-2);

      es = es - derivVal / secDerivVal;

      times++;
    } while (abs(derivVal) > abs(secDerivVal) * tolerance / 100);

    serial.println(float2s(es,5));
    serial.println(times+1);
    
    //estimate comparison logic
    if (es > -20 and es < 200) {
      secDerivVal = polyCalc(secDerivCoeffs, es, polyDeg-2);
      
      if (secDerivVal < 0) {
        tempScore = polyCalc(funcCoeffs, es, polyDeg);// / secDerivVal;  //creates a function of the value and the second derivative. ********************************************************************** modified!!!!!!! ********************
        
        if (tempScore > maxScore) { //***************************** flipped(previous: tempScore < maxScore     secDerivVal returns neg val) ********************
          maxScore = tempScore;
          maxVal = round(es);
        }
      } else if (maxVal == 0) {                                                       //ta bort detta kanske.
        polyVal = polyCalc(funcCoeffs, es, polyDeg);
        
        if (bestMin == 0) {
          bestMin = es;
        } else if (polyVal > polyCalc(funcCoeffs, bestMin, polyDeg)) {
          bestMin = es;
        }
      }
    }
  }
  
  if (maxVal == 0) {
    //maxVal = round(bestMin);
    maxVal = bestAngle;
  }
  
  return maxVal;
}


double polyCalc(double *polyCoeffs, double es, int polyDeg) {
  double value = polyCoeffs[0];
  
  for (int i=1;i<=polyDeg;i++) {
    value = value + polyCoeffs[i] * pow( es, i);
  }
  
  return value;
}


char * float2s(float f, unsigned int digits)
{ // https://forum.arduino.cc/index.php?topic=46931.0
 int index = 0;
 static char s[16];                    // buffer to build string representation
 // handle sign
 if (f < 0.0)
 {
   s[index++] = '-';
   f = -f;
 }
 // handle infinite values
 if (isinf(f))
 {
   strcpy(&s[index], "INF");
   return s;
 }
 // handle Not a Number
 if (isnan(f))
 {
   strcpy(&s[index], "NaN");
   return s;
 }

 // max digits
 if (digits > 6) digits = 6;
 long multiplier = pow(10, digits);     // fix int => long

 int exponent = int(log10(f));
 float g = f / pow(10, exponent);
 if ((g < 1.0) && (g != 0.0))      
 {
   g *= 10;
   exponent--;
 }

 long whole = long(g);                     // single digit
 long part = long((g-whole)*multiplier);   // # digits
 char format[20];
 sprintf(format, "%%ld.%%0%dld*10^%%+d", digits);
 sprintf(&s[index], format, whole, part, exponent);
 
 return s;
}


void LIDARmapping() {
  avrCounter = 0;
  minDistance = 300;
  minDistCone = 300;
  maxDistance = 0;
  bestAngle = 90;
  int iFactor;
  int average;
  
  timer1 = millis();
  if (servo.read() < 90) {
    servo.write(calibrateAngle(0));
    //delay(30);
    avrCounter = 0;
    
    for (int i = 0; i <= measurements; ++i) {
      servo.write(calibrateAngle(i*180/measurements));
      delay(5);  //minimum delay: 5
      uint16_t distance;
      uint8_t  newDistance;
      do {
        newDistance = distanceContinuousGpio(&distance);
      } while(!newDistance);
    
      avrArray[avrCounter] = distance;
      avrCounter++;
      
      if (avrCounter >= avrAmount) {
       avrCounter = 0; 
      }
    
      unsigned int average = averageDist(avrArray);
      if (average < minDistance) {
        minDistance = average;
      }
      if (average > maxDistance) {
        maxDistance = average;
        bestAngle = i*180/measurements;
      }
      if (average < minDistCone and (i*180/measurements >= 60 and i*180/measurements <= 120)) {
        minDistCone = average;
      }
      avrArrayTot[i] = average;
    }
  } else {
    servo.write(calibrateAngle(180));
    //delay(30);
    avrCounter = 0;
    
    for (int i = measurements; i >= 0; --i) {
      servo.write(calibrateAngle(i*180/measurements));
      delay(5);  //minimum delay: 5ms
      uint16_t distance;
      uint8_t  newDistance;
      do {
        newDistance = distanceContinuousGpio(&distance);
      } while(!newDistance);
    
      avrArray[avrCounter] = distance;
      avrCounter++;
      
      if (avrCounter >= avrAmount) {
       avrCounter = 0; 
      }
    
      unsigned int average = averageDist(avrArray);
      if (average < minDistance) {
        minDistance = average;
      }
      if (average > maxDistance) {
        maxDistance = average;
        bestAngle = i*180/measurements;
      }
      if (average < minDistCone and (i*180/measurements >= 60 and i*180/measurements <= 120)) {
        minDistCone = average;
      }
      avrArrayTot[i] = average;
    }
  }
  timer2 = millis();
}


int averageDist(int *avrArray) {
  int averageN = 0;
  
  for (int i = 0; i < avrAmount; ++i) {
    averageN += avrArray[i];
  }
  
  averageN = averageN/avrAmount;
  
  return averageN;
}


float mapFloat(float inFl, float in_min, float in_max, float out_min, float out_max) {
  return (inFl - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int calibrateAngle(float wantedAngle) {
  int closestTableInt = floor(wantedAngle/5);
  if (float(closestTableInt*5) != wantedAngle) {
    return round((angleTable[closestTableInt+1] - angleTable[closestTableInt])*(wantedAngle - closestTableInt*5)/5 + angleTable[closestTableInt]);
  } else {  
    return round(angleTable[closestTableInt]);
  }
}


float calibrateSpeed(float wantedSpeed, float* speedTable) {
  int closestTableFloat = floor(wantedSpeed);
  if (closestTableFloat >= 39) {
    closestTableFloat = 38;
  }
  return (speedTable[closestTableFloat+1] - speedTable[closestTableFloat])*(wantedSpeed - closestTableFloat) + speedTable[closestTableFloat];
}
