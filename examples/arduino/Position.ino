#include <Arduino.h>
#include <gs2d.h>

class ArduinoSerial;
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

void loop() {}

class ArduinoSerial
{
  public:
    ArduinoSerial() {}
    ~ArduinoSerial() {}

    int open(void)
    {
        pinMode(2, OUTPUT);
        Serial1.begin(115200, SERIAL_8E1);
        //      Serial1.begin(115200, SERIAL_8N1);
    }

    int isConnected(void) { return 1; }

    void close(void) {}

    int read(void)
    {
        if (Serial1.available())
        {
            return Serial1.read();
        } else
        {
            return -1;
        }
    }

    int write(unsigned char *data, uint8_t size)
    {
        digitalWrite(2, HIGH);
        int retval = Serial1.write(data, size);
        Serial1.flush();
        digitalWrite(2, LOW);
    }

    unsigned long long int time(void) { return millis(); }
};
