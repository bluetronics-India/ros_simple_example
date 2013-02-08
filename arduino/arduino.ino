#include <Servo.h> 
#include <NewPing.h>

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define SERVO_PIN 3      // Which PWM pin is the servo attached
#define FORWARDS 0
#define BACKWARDS 1

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int pos = 25;    // variable to store the servo position
int dir = FORWARDS;

int vals[25]; //25 Values stored
int scanIndex = -1;

void setup()
{
  myservo.attach(SERVO_PIN);
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  myservo.write(pos);
}

void loop()
{
  
  //Wait for data
  if ((pos == 25) || (pos == 155))
  {
    while (Serial.available() == 0);

    //Send ack
    Serial.println("OK");
  }
  
  while (Serial.available() != 0)
  { //Ignore data during a sweep
    Serial.read();
  }
  
  if (dir == FORWARDS) 
  {
    pos+= 5;
    scanIndex++;
  }
  if (dir == BACKWARDS)
  {
    pos-= 5;
    scanIndex--;
  }
  
  myservo.write(pos);
  delay(15);

  if ((pos > 25) && (pos < 155))
  {
    //Take a reading here.
    unsigned int uS = sonar.ping_median(); // Send ping, get ping time in microseconds (uS).
    vals[scanIndex] = sonar.convert_cm(uS);
    
    if (((pos == 30) && (dir == BACKWARDS)) || 
        ((pos == 150) && (dir == FORWARDS)))
        {
          //End of the sweep. Send the data.
          for (int i = 0; i < 25; i++)
          {
            Serial.println(vals[i]);
          }
          Serial.println("END");
        }
  }
  
  //Setup direction for next run
  if (pos == 155) dir = BACKWARDS;
  if (pos == 25) dir = FORWARDS;
}

