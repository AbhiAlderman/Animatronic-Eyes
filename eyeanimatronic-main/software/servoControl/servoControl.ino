#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32PWM.h>

Servo upperEyelid1;
Servo lowerEyelid2;
Servo rightUD3;
Servo rightLR4;
Servo leftUD5;
Servo leftLR6;

// U Eyelid, L Eylid, Right up/down, Right left/right, Left up/down, Left left/right
Servo servos[6] = {upperEyelid1, lowerEyelid2, rightUD3, rightLR4, leftUD5, leftLR6};
int pins[6] = {12,13,14,15,22,23};

// Define initial positions for each servo
int startpos[6] = {150,60,100,80,95,100};
int currpos[6] = {150,60,100,80,95,100};

// Define bounds of movement 
int minBound[6] = {110, 30, 85, 60, 110, 85}; // Down, right, or closed is LOW
int maxBound[6] = {150, 60, 120, 100, 65, 125}; // Up, left, or open is HIGH

void moveServo(int ind, int targetpos) {
  if (targetpos > currpos[ind]) {
    for (currpos[ind]; currpos[ind] < targetpos; currpos[ind]++){
      servos[ind].write(currpos[ind]);
      Serial.println(currpos[ind]);
      delay(15);
    }
  } else {
    for (currpos[ind]; currpos[ind] > targetpos; currpos[ind]--){
      servos[ind].write(currpos[ind]);
      delay(15);
    }
  }
}

void blink(int dt) {
    int uOpen = maxBound[0];
    int lOpen = maxBound[1];
    int uClosed = minBound[0];
    int lClosed = minBound[1];

    int uInt = uOpen-uClosed;
    int lInt = lOpen-lClosed;
    int numsteps = min(uInt, lInt);

    int uStart = uOpen;
    int lStart = lOpen;
    
    for (int i = 0; i<numsteps; i++) {
      currpos[0] = uStart-(i*uInt)/numsteps;
      servos[0].write(currpos[0]);
      Serial.println(currpos[0]);
      currpos[1] = lStart-(i*lInt)/numsteps;
      servos[1].write(currpos[1]);
      Serial.println(currpos[1]);
      delay(5);
    }
    
    delay(dt);
}

void moveEye(float xPercent, float yPercent) {
  int lLeft = maxBound[5];
  int lRight = minBound[5];
  int rLeft = maxBound[3];
  int rRight = minBound[3];

  int ltargetX = lLeft - int(xPercent*float(lLeft-lRight));
  int rtargetX = rLeft - int(xPercent*float(rLeft-rRight));
  
  int lUp = maxBound[4];
  int lDown = minBound[4];
  int rUp = maxBound[2];
  int rDown = minBound[2];

  int ltargetY = yPercent*(lUp-lDown)+lDown;
  int rtargetY = yPercent*(rUp-rDown)+rDown;


  int leftLRint = ltargetX-currpos[5];
  int rightLRint = rtargetX-currpos[3];
  int xnumsteps = min(abs(leftLRint), abs(rightLRint));
  int lLRstart = currpos[5];
  int rLRstart = currpos[3];

  int leftUDint = ltargetY-currpos[4];
  int rightUDint = rtargetY-currpos[2];
  int ynumsteps = min(abs(leftUDint), abs(rightUDint));
  int lUDstart = currpos[4];
  int rUDstart = currpos[2];

  Serial.print("LTarget Y: ");
  Serial.println(ltargetY);
  Serial.print("RTarget Y: ");
  Serial.println(rtargetY);

  int numsteps = min(xnumsteps, ynumsteps);
  
  for (int i = 0; i<numsteps; i++) {
    // Left Eye Left/Right
    currpos[5] = lLRstart+(i*leftLRint)/numsteps;
    servos[5].write(currpos[5]);
    Serial.print("L Left/Right: ");
    Serial.println(currpos[5]);
    // Right Eye Left/Right
    currpos[3] = rLRstart+(i*rightLRint)/numsteps;
    servos[3].write(currpos[3]);
    Serial.print("R Left/Right: ");
    Serial.println(currpos[3]);
    // Left Eye Up/Down
    currpos[4] = lUDstart+(i*leftUDint)/numsteps;
    servos[4].write(currpos[4]);
    Serial.print("L Up/Down: ");
    Serial.println(currpos[4]);
    // Right Eye Up/Down
    currpos[2] = rUDstart+(i*rightUDint)/numsteps;
    servos[2].write(currpos[2]);
    Serial.print("L Up/Down: ");
    Serial.println(currpos[2]);
    delay(30);
  }

  
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i<6; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(pins[i],500,2400);
    servos[i].write(startpos[i]);
  }

}

void loop() {
  for(int i = 0; i < 6; i++){
    Serial.print(currpos[i]);
    Serial.print(",");
  }
  Serial.print("\n");
  delay(1000);
  if (Serial.available()) {
    int temp = Serial.parseInt();
    if (temp != 0) {
      float targetpos = float(temp)/100; 
      Serial.println(targetpos);
      moveEye(targetpos, targetpos);
    }
  }
//  delay(1000);
//  for (int i = 4; i<6; i++) {
//    Serial.println(i);
//    delay(1000);
//    moveServo(i, maxBound[i]);
//    delay(1000);
//  }
//  delay(1000);
//  blink(75);

}
