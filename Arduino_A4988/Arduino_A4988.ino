#include <Servo.h>
#include <SoftwareSerial.h>

#define limitSWPin 7  // pin for reading the limit switches (Pull Down; active high)

#define MS1 11         // Pins for configuring the A4988 drivers
#define MS2 10
#define MS3  9
#define stepEn 12

#define stepPin 6    // Pins to send instructions to the A4988 driver
#define dirPin  5

#define servoPin 4    // Pin for connecting the servo
#define servoMin 110  // Minimum Pulse Width in microseconds
#define servoMax 200  // Maximum Pulse Width in microseconds

// Spherical Vector Struct to store given angles
struct spVector{
    float phi;
    float theta;
}; typedef spVector spVector;

// Variables for driving the stepper motor
int  dely    = 1000;
int  dir     = 0;  // 0 for clockwise rot, 1 for anticlockwise rot 
int  stpSize = 4;

bool homeMode = 0;
int  limitSW  = 0;
int  totSteps = 0;

const float totAng = 315;
float stpsPerAng = 0;

Servo servo;

spVector currVec = {.phi=0, .theta=0};

spVector P0T0 = {.phi=90 , .theta=0  };

spVector P0T45     = {.phi= 90.0  , .theta=   45.0};
spVector P0T90     = {.phi= 90.0  , .theta=   90.0};
spVector P0T135    = {.phi= 90.0  , .theta=  135.0};
spVector P0TM45    = {.phi= 90.0  , .theta=  -45.0};
spVector P0TM90    = {.phi= 90.0  , .theta=  -90.0};
spVector P0TM135   = {.phi= 90.0  , .theta= -135.0};

spVector P45T45    = {.phi= 135.0 , .theta=   45.0};
spVector P45T90    = {.phi= 135.0 , .theta=   90.0};
spVector P45T135   = {.phi= 135.0 , .theta=  135.0};
spVector P45TM45   = {.phi= 135.0 , .theta=  -45.0};
spVector P45TM90   = {.phi= 135.0 , .theta=  -90.0};
spVector P45TM135  = {.phi= 135.0 , .theta= -135.0};

spVector PM45T45   = {.phi= 45.0  , .theta=   45.0};
spVector PM45T90   = {.phi= 45.0  , .theta=   90.0};
spVector PM45T135  = {.phi= 45.0  , .theta=  135.0};
spVector PM45TM45  = {.phi= 45.0  , .theta=  -45.0};
spVector PM45TM90  = {.phi= 45.0  , .theta=  -90.0};
spVector PM45TM135 = {.phi= 45.0  , .theta= -135.0};


SoftwareSerial ESPSerial(3, 2);

void setup() {
    String readSerial;
    pinMode(stepPin,  OUTPUT);
    pinMode(dirPin,   OUTPUT);
    pinMode(MS1,      OUTPUT);
    pinMode(MS2,      OUTPUT);
    pinMode(MS3,      OUTPUT);
    pinMode(servoPin, OUTPUT);
    pinMode(stepEn,   OUTPUT);

    pinMode(limitSWPin, INPUT);

    digitalWrite(stepEn,1);

    Serial.begin(9600);
    ESPSerial.begin(9600);

    Serial.println("Beginning...");
    delay(5000);
    servo.attach(servoPin, servoMin, servoMax);
    digitalWrite(stepEn, 0);

    while (1) {
      if (Serial.available()) {
        Serial.println("Type 'start' to begin calibration step");
        readSerial = Serial.readString();
        readSerial.trim();
        if (readSerial=="start") {
          Serial.println("Starting Calibration Step...");
          home();
          Serial.println("Finished Calibration...");
          break; 
        }
      }
    }
}

void loop() {
    String readSerial;
    
    if (Serial.available()) {
        Serial.println("Type start to run through all the nodes");
        readSerial = Serial.readString();
        readSerial.trim();

        if (readSerial == "start") {
          drive(P0T0     );
          drive(P0TM135  );
          waitForInput()  ;
          drive(P0TM90   );
          waitForInput()  ;
          drive(P0TM45   );
          waitForInput()  ;
          drive(P0T0     );
          waitForInput()  ;
          drive(P0T45    );
          waitForInput()  ;
          drive(P0T90    );
          waitForInput()  ;
          drive(P0T135   );
          waitForInput()  ;

          drive(P0T0     );
          drive(P45TM135 );
          waitForInput()  ;
          drive(P45TM90  );
          waitForInput()  ;
          drive(P45TM45  );
          waitForInput()  ;
          drive(P45T45   );
          waitForInput()  ;
          drive(P45T90   );
          waitForInput()  ;
          drive(P45T135  );
          waitForInput()  ;

          drive(P0T0     );
          drive(PM45TM135);
          waitForInput()  ;
          drive(PM45TM90 );
          waitForInput()  ;
          drive(PM45TM45 );
          waitForInput()  ;
          drive(PM45T45  );
          waitForInput()  ;
          drive(PM45T90  );
          waitForInput()  ;
          drive(PM45T135 );
          waitForInput()  ;        
        }
        Serial.println("Ran though all nodes, switching to manual mode...");
 
        if (readSerial == "P0T0"   ) {drive(P0T0   );}

        else if (readSerial == "P0T45"    ) {drive(P0T45    );}
        else if (readSerial == "P0T90"    ) {drive(P0T90    );}
        else if (readSerial == "P0T135"   ) {drive(P0T135   );}
        else if (readSerial == "P0TM45"   ) {drive(P0TM45   );}
        else if (readSerial == "P0TM90"   ) {drive(P0TM90   );}
        else if (readSerial == "P0TM135"  ) {drive(P0TM135  );}
        
        else if (readSerial == "P45T45"   ) {drive(P45T45   );}
        else if (readSerial == "P45T90"   ) {drive(P45T90   );}
        else if (readSerial == "P45T135"  ) {drive(P45T135  );}
        else if (readSerial == "P45TM45"  ) {drive(P45TM45  );}
        else if (readSerial == "P45TM90"  ) {drive(P45TM90  );}
        else if (readSerial == "P45TM135" ) {drive(P45TM135 );}

        else if (readSerial == "PM45T45"  ) {drive(PM45T45  );}
        else if (readSerial == "PM45T90"  ) {drive(PM45T90  );}
        else if (readSerial == "PM45T135" ) {drive(PM45T135 );}
        else if (readSerial == "PM45TM45" ) {drive(PM45TM45 );}
        else if (readSerial == "PM45TM90" ) {drive(PM45TM90 );}
        else if (readSerial == "PM45TM135") {drive(PM45TM135);}
        
    }
}

void home() {
    const int buffer=10;
    int i;

    // drive one direction at full speed until limitA is hit
    Serial.println("Homing Step A");
    digitalWrite(dirPin, 1); setStpSize(2);
    while (SWCheck()!="LIMIT_SW_INTERRUPT_HIGH") {step();}

    // drive to the other side at slow speed until limitB
    Serial.println("Homing Step D");
    digitalWrite(dirPin, 0); setStpSize(stpSize);
    while (SWCheck()!="LIMIT_SW_INTERRUPT_LOW" ) {step(); totSteps++;} // Buffer to allow for 'unclick'
    while (SWCheck()!="LIMIT_SW_INTERRUPT_HIGH") {step(); totSteps++;}
    Serial.println(totSteps);
    
    // Set known variables
    currVec.theta = totAng/2.0;
    stpsPerAng    = totSteps/totAng;

    // drive to home
    drive(P0T0);

}

int drive(spVector vector) {
    /*
    function to drive the Stepper Motor and Servo
    Servo will drive phi, stepper motor drives theta
    returns: 
        0 for error (not implemented)
        1 for completion of instructions
        2 for hitting the limit switch while in not in home mode
    */

    int i; bool interrupt = 0;

    // ---------- Driving Servo Motor ----------
    Serial.println("Driving Servo");
    servo.write(vector.phi);

    // ---------- Driving Stepper Motor ----------
    // Calculate Variables
    float deltaTheta = (vector.theta - currVec.theta);
    int   noSteps    = round(abs(deltaTheta)*stpsPerAng)%(totSteps);

    if   (deltaTheta >= 0) {dir = 0;}
    else                   {dir = 1;}
  
    Serial.print(deltaTheta);
    Serial.print(", ");
    Serial.println(noSteps);

    digitalWrite(dirPin, dir); setStpSize(stpSize);
    for (i=0; i<noSteps; i++) {
        if (SWCheck()=="LIMIT_SW_INTERRUPT_HIGH") {interrupt=1; break;} // if limit switch is pressed too early, raise interrupt, break
        step();
    }

    // update new position
    if (dir==0) {
        currVec.theta += i/stpsPerAng;
    } else {
        currVec.theta -= i/stpsPerAng;
    }

    // return with correct code
    if (interrupt) {return(2);}
    return(1);

}
    
String SWCheck() {
    if (ESPSerial.available()) {
        String interruptString = ESPSerial.readString();
        interruptString.trim();
        if      (interruptString == "LIMIT_SW_INTERRUPT_HIGH") {return(interruptString);}
        else if (interruptString == "LIMIT_SW_INTERRUPT_LOW" ) {return(interruptString);}
        Serial.println(interruptString);
    }
    return("NO_CHANGE");
}

void setStpSize(int size) {
    // set step size pins on the A4988
    if      (size==1) {digitalWrite(MS1,  LOW); digitalWrite(MS2,  LOW); digitalWrite(MS3,  LOW);}
    else if (size==2) {digitalWrite(MS1, HIGH); digitalWrite(MS2,  LOW); digitalWrite(MS3,  LOW);}
    else if (size==4) {digitalWrite(MS1,  LOW); digitalWrite(MS2, HIGH); digitalWrite(MS3,  LOW);}
    else if (size==8) {digitalWrite(MS1, HIGH); digitalWrite(MS2, HIGH); digitalWrite(MS3,  LOW);}
}

void step() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(dely);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(dely);
}

void waitForInput() {
    String readSerial;
    digitalWrite(stepEn, 1);
    while (1) {
      if (Serial.available()) {
        readSerial = Serial.readString();
        readSerial.trim();
        if (readSerial == "next") {digitalWrite(stepEn, 0); break;};
      }
    }
}
