#include "gs2d_robotis.h"
#include "gs2d_b3m.h"
#include "gs2d_futaba.h"
#include "gs2d_krs.h"
#include "ArduinoSerial.h"

using namespace gs2d;

void setup() 
{
  delay(2000);
  Serial.begin(115200);
  Serial.println("Hello World 2");
  delay(1000);
  
//  Driver* servo = new RobotisP20<ArduinoSerial>();
//  Driver* servo = new B3M<ArduinoSerial>();
//  Driver* servo = new Futaba<ArduinoSerial>();
  Driver* servo = new KRS<ArduinoSerial>();

  Serial.println("Target Positon");
  servo->writeTargetPosition(1, 90.0);
  delay(1000);
  Serial.println("Target Positon");
  servo->writeTargetPosition(1, -45.0);
  delay(1000);
  Serial.print("Position : ");
  Serial.print((int)servo->readCurrentPosition(1));
  Serial.println();
  
}

void loop() 
{

}
