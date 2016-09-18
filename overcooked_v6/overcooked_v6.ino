#include <NewPing.h>
#if defined(ARDUINO)
SYSTEM_MODE(MANUAL);
#endif
/**** Pins ****/
#define LP    A3   //Left Pad
#define RP    A0   //Right Pad
#define FP    A1   //Front Pad
#define BP    A2   //Back Pad
#define B1    A6   //Board pin
#define B2    A7   //Board pin
#define TRIG  D2   //Trigger
#define ECHO  D3   //Echo
#define AX    A5   // X-axis
#define AY    A4   //Y-Axis
#define ALCAL  D5  //Calibrate
#define SCHEF D4   //Switch Chefs
#define PAUSE D6  //Pause
/**** Charachters to be sent for different actions ****/
#define CUT   'c'
#define FWD   'w'
#define BWD   's'
#define LWD   'a'
#define RWD   'd'
#define POUR  'l'
#define LIFT  'l'
#define NOFB 'u'
#define NOLR 'j'
#define SCHEF 'q'
/**** Threshold definitions ****/
#define MAXD 200      //Max distance of ping
#define LTR  100      //Left pad Treshold
#define RTR  100      //Right pad Treshold
#define FTR  100      //Front pad Treshold
#define BTR  35       //Back pad Treshold
#define XTR  280      //X-axis tresh
#define YTR  280      //Y-axis tresh
#define BRDTR 200     //Board cutting threshold
#define DTR  5        //Ping distance tresh (in cm)
#define CALSAMP 40.0  //Samples for calibration of accelerometers
#define BRDSAMP 40.0  //Samples for calibration of board FSRs
#define FTSAMP 150.0  //Samples for calibration of foot FSRs
#define KDELAY 95     //Minimum time between two consecutive commands
//#define DEBUG 1

int basel = 0;        //Variable for base value of the left pad
int baser = 0;        //Variable for base value of the right pad
int basef = 0;        //Variable for base value of the front pad
int baseb = 0;        //Variable for base value of the back pad
float basex = 0;      //Variable for base value of X-axis acceleration
float basey = 0;      //Variable for base value of Y-axis acceleration
float basebrd = 0;    //Variable for base value of board FSR
int dist = 0;         //Ping distance in cm
int acx = 0;          //Variable for storing acceleration values in the x-direction
int acy = 0;          //Variable for storing acceleration values in the y-direction
int ctcnt = 0;        //Variable for counting the number of cuts

unsigned long t0 = millis(); //Variable for storing start time of each loop
NewPing sonar(TRIG, ECHO, MAXD); //Initialising the sonar

int lift_flag = 0;    //Flag that indicates lift status of carton
int tilt_flag = 0;    //Flag that indicates tilt status of carton

void setup() {
  Serial.begin(57600);
  pinMode(LP, INPUT);
  pinMode(RP, INPUT);
  pinMode(FP, INPUT);
  pinMode(BP, INPUT);
  pinMode(AX, INPUT);
  pinMode(AY, INPUT);
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(ALCAL, INPUT_PULLDOWN);
  calibrate_brd();
  calibrate_foot();
  calibrate_acc();

#if defined(DEBUG)
  Serial.print("In calibration - "); Serial.print(basex); Serial.print(" "); Serial.print(basey); Serial.println();
#endif
  delay(3000);
#if defined(DEBUG)
  Serial.print("In setup - "); Serial.print(basebrd); Serial.println();
  Serial.println("Start!!!!");
#endif

}

void loop() {
  if (digitalRead(ALCAL) == HIGH) {
    calibrate_brd();
    calibrate_foot();
    calibrate_acc();
  }
  if ((millis() - t0) > KDELAY) {

    if (isLifted()) {

#if defined(DEBUG)
      if (lift_flag != 1) {
        Serial.println("Carton Lifted");
        lift_flag = 1;
      }
#endif
      if (lift_flag != 1) {
        Serial.write(LIFT);
        lift_flag = 1;
      }


      if (isTilted()) {
#if defined(DEBUG)
        if (tilt_flag != 1) {
          Serial.println("Carton Tilted - Command to be sent to computer");
        }
#endif
        if (tilt_flag != 1) {
          Serial.write(POUR);
        }
        tilt_flag = 1;
#if defined(DEBUG)
        Serial.print("Tilted "); Serial.print(basex); Serial.print(" "); Serial.print(basey); Serial.print(" AX - "); Serial.print(basex - acx); Serial.print(" AY - "); Serial.print(basex - acy); Serial.print(" XTR - "); Serial.print(XTR); Serial.print(" YTR - "); Serial.println(YTR);
#endif
      }
      else {
        tilt_flag = 0;
#if defined(DEBUG)
        Serial.print(" AX - "); Serial.print(basex - acx); Serial.print(" AY - "); Serial.print(basex - acy); Serial.print(" XTR - "); Serial.print(XTR); Serial.print(" YTR - "); Serial.println(YTR);
#endif
      }
    }
    else {
      if (lift_flag == 1) {
#if defined(DEBUG)
        Serial.println("Carton Put down");
#endif
        Serial.write(LIFT);
        lift_flag = 0;
      }

#if defined(DEBUG)
      Serial.print("Distance - ");
      Serial.println(dist);
#endif
    }


    isCut();
    updatePads();
    t0 = millis();
  }
  delay(1);
}

void calibrate_acc() {
  basex = 0;
  basey = 0;
  for (int i = 0; i < CALSAMP; i++) {
    basex += analogRead(AX);
    basey += analogRead(AY);
  }
#if defined(DEBUG)
  Serial.print("In calibration - "); Serial.print(basex); Serial.print(" "); Serial.print(basey); Serial.println();
#endif
  delay(1000);
  basex = basex / CALSAMP;
  basey = basey / CALSAMP;
#if defined(DEBUG)
  Serial.print("After calibration - "); Serial.print(basex); Serial.print(" "); Serial.print(basey); Serial.println();
#endif
}

bool calibrate_brd() {
  basebrd = 0;

  for (int i = 0; i < BRDSAMP; i++) {
    basebrd += analogRead(B1);

  }
#if defined(DEBUG)
  Serial.print("In calibration - "); Serial.print(basebrd); Serial.println();
#endif

  basebrd = basebrd / BRDSAMP;

#if defined(DEBUG)
  Serial.print("After calibration - "); Serial.print(basebrd); Serial.println();
#endif
}
bool calibrate_foot() {
  basel = 0;
  baser = 0;
  basef = 0;
  baseb = 0;

  for (int i = 0; i < FTSAMP; i++) {
    basel += analogRead(LP);
    baser += analogRead(RP);
    basef += analogRead(FP);
    baseb += analogRead(BP);

  }
#if defined(DEBUG)
  Serial.print("In calibration - "); Serial.print(basebrd); Serial.println();
#endif

  basel = basel / FTSAMP;
  baser = baser / FTSAMP;
  basef = basef / FTSAMP;
  baseb = baseb / FTSAMP;

#if defined(DEBUG)
  Serial.print("After calibration - "); Serial.print(basebrd); Serial.println();
#endif
}

bool isLifted() {
  // Debug
  //return(1);

  dist = sonar.ping_cm();

  if (dist >= DTR || dist == 0) {
    return (1);
  }

  else if (dist <= DTR && dist != 0) {
    return (0);

  }
}

bool isTilted() {

  acx = analogRead(AX);
  acy = analogRead(AY);

  if ( (mod(basex - acx) >= XTR) || (mod(basey - acy) >= YTR) ) {
    return (1);
  }
  else {
    return (0);
  }
}
bool isCut() {
  int p1 = analogRead(B1);
  //int p2 = analogRead(B2);

  if ( p1 - basebrd >= BRDTR) {

    Serial.write(CUT);
    ctcnt++;
    return (1);
  }
  else {
    return (0);
  }

}

void updatePads() {
  int fwd = analogRead(FP);
  int bwd = analogRead(BP);
  int lwd = analogRead(LP);
  int rwd = analogRead(RP);
#if defined(DEBUG)
  Serial.print("Val -- > "); Serial.print(fwd); Serial.print(" "); Serial.print(lwd); Serial.print(" "); Serial.print(bwd); Serial.print(" "); Serial.print(rwd); Serial.print(" --> ");
  Serial.print("Base -- > "); Serial.print(basef); Serial.print(" "); Serial.print(basel); Serial.print(" "); Serial.print(baseb); Serial.print(" "); Serial.print(baser); Serial.print(" --> ");
#endif
  if ( (fwd > basef + FTR) )
  {
#if defined(DEBUG)
    Serial.write(FWD); Serial.print(" "); Serial.print(basef); Serial.print(" "); Serial.print(fwd); Serial.print(" ");
#endif
    Serial.write(FWD);


  }
  else if ((bwd > baseb + BTR)) {
#if defined(DEBUG)
    Serial.write(BWD); Serial.print(" "); Serial.print(baseb); Serial.print(" "); Serial.print(bwd); Serial.print(" ");
#endif
    Serial.write(BWD);
  }
  else {
#if defined(DEBUG)
    Serial.write(NOFB);
#endif
  }

  if ( (lwd > basel + LTR))
  {

#if defined(DEBUG)
    Serial.write(LWD); Serial.print(" "); Serial.print(basel); Serial.print(" "); Serial.print(lwd); Serial.print(" ");
#endif
    Serial.write(LWD);
  }
  else if ((rwd > baser + RTR)) {
#if defined(DEBUG)
    Serial.write(RWD); Serial.print(" "); Serial.print(baser); Serial.print(" "); Serial.print(fwd); Serial.print(" ");
#endif
    Serial.write(RWD);
  }
  else {
#if defined(DEBUG)
    Serial.write(NOLR);
#endif
  }

#if defined(DEBUG)
  Serial.println();
#endif
}


int mod(int a) {
  if (a < 0) {
    return (-a);
  }
  return (a);
}
