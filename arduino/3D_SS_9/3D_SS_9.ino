// HOW TO:
// ------------------------------
// Serial input = 2 -> Steps through gait sequence once and stops at the end.
// Serial input = 1 -> Steps through gait sequence in a loop.
// Serial input = 0 -> Stops infinite loop at the node where it is at that moment

//Gait template
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

#define SERVO_FREQ 300
#define SER0 0 //Hip 1
#define SER1 1 //Knee 1
#define SER2 2 //Hip 2
#define SER3 3 //Knee 2
#define SER4 4 //Hip 3
#define SER5 5 //Knee 3
#define SER6 6 //Hip 4
#define SER7 7 //Knee 4



// Insert servo angles here:
// For 3D gaits, these angles are obtained from linearisation.py
// For 2D gaits, these angles are obtained from 2D_trajectory_reader.py

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=100000;
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);
int one_print = 0;

// -------------------Joint configurations-------------------
int servo0=0;
int servo0_deg=0;
int servo1=0;
int servo1_deg=0;
int servo2=0;
int servo2_deg=0;
int servo3=0;
int servo3_deg=0;
int servo4=0;
int servo4_deg=0;
int servo5=0;
int servo5_deg=0;
int servo6=0;
int servo6_deg=0;
int servo7=0;
int servo7_deg=0;

void setup() {
  Serial.begin(115200);
  pca9685.begin();

  pca9685.setOscillatorFrequency(27000000); //Value between 24MHz and 27MHz, tuned for this setup
  pca9685.setPWMFreq(SERVO_FREQ);
  delay (1000);

  servo0_deg=SS_servo0[0]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[0]/3.14159265*180;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[0]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[0]/3.14159265*180;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[0]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[0]/3.14159265*180;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[0]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[0]/3.14159265*180;
        servo7=map(servo7_deg,-90,90,920,2620);
        
        pca9685.setPWM(SER0,0,servo0);
        pca9685.setPWM(SER1,0,servo1);
        pca9685.setPWM(SER2,0,servo2);
        pca9685.setPWM(SER3,0,servo3);
        pca9685.setPWM(SER4,0,servo4);
        pca9685.setPWM(SER5,0,servo5);
        pca9685.setPWM(SER6,0,servo6);
        pca9685.setPWM(SER7,0,servo7);

        Serial.print("Node:");
        Serial.println(node);
        Serial.print("servo0 -> rads:");
        Serial.print(SS_servo0[node]);
        Serial.print("        deg:");
        Serial.print(servo0_deg);
        Serial.print("        us:");
        Serial.println(servo0);
        Serial.print("servo1 -> rads:");
        Serial.print(SS_servo1[node]);
        Serial.print("        deg:");
        Serial.print(servo1_deg);
        Serial.print("        us:");
        Serial.println(servo1);
        Serial.print("servo2 -> rads:");
        Serial.print(SS_servo2[node]);
        Serial.print("        deg:");
        Serial.print(servo2_deg);
        Serial.print("        us:");
        Serial.println(servo2);
        Serial.print("servo3 -> rads:");
        Serial.print(SS_servo3[node]);
        Serial.print("        deg:");
        Serial.print(servo3_deg);
        Serial.print("        us:");
        Serial.println(servo3);
        Serial.print("servo4 -> rads:");
        Serial.print(SS_servo4[node]);
        Serial.print("        deg:");
        Serial.print(servo4_deg);
        Serial.print("        us:");
        Serial.println(servo4);
        Serial.print("servo5 -> rads:");
        Serial.print(SS_servo5[node]);
        Serial.print("        deg:");
        Serial.print(servo5_deg);
        Serial.print("        us:");
        Serial.println(servo5);
        Serial.print("servo6 -> rads:");
        Serial.print(SS_servo6[node]);
        Serial.print("        deg:");
        Serial.print(servo6_deg);
        Serial.print("        us:");
        Serial.println(servo6);
        Serial.print("servo7 -> rads:");
        Serial.print(SS_servo7[node]);
        Serial.print("        deg:");
        Serial.print(servo7_deg);
        Serial.print("        us:");
        Serial.println(servo7);

        Serial.println("10");
        delay(1000);
        Serial.println("9");
        delay(1000);
        Serial.println("8");
        delay(1000);
        Serial.println("7");
        delay(1000);
        Serial.println("6");
        delay(1000);
        Serial.println("5");
        delay(1000);
        Serial.println("4");
        delay(1000);
        Serial.println("3");
        delay(1000);
        Serial.println("2");
        delay(1000);
        Serial.println("1");
        delay(1000);

}

void loop() 
{
  if(Serial.available()>0)
  {
    first=Serial.read();
    if(first=='2')
    {
      input=2;
      node=0;
      one_print=1;
    }
    if(first=='1')
    {
      input=1;
      node=0;
      one_print=1;
    }
    if(first=='0')
    {
      input=0;
    }
  }
  if(input==1 or input==2)
  {
      if(esp_timer_get_time()>=end_time)
      {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180;
        servo7=map(servo7_deg,-90,90,920,2620);
        
        pca9685.setPWM(SER0,0,servo0);
        pca9685.setPWM(SER1,0,servo1);
        pca9685.setPWM(SER2,0,servo2);
        pca9685.setPWM(SER3,0,servo3);
        pca9685.setPWM(SER4,0,servo4);
        pca9685.setPWM(SER5,0,servo5);
        pca9685.setPWM(SER6,0,servo6);
        pca9685.setPWM(SER7,0,servo7);
        
        node+=1;
        if(node>=steps and input==1)
        {
          Serial.print("Input:");
          Serial.println(input);
          node=0;
        }
        if(node>=steps and input==2)
        {
          Serial.print("Input:");
          Serial.println(input);
          input=0;
        }
         end_time=esp_timer_get_time()+time_step;
      }
   }

  if(input == 0)
  {        
        pca9685.setPWM(SER0,0,servo0);
        pca9685.setPWM(SER1,0,servo1);
        pca9685.setPWM(SER2,0,servo2);
        pca9685.setPWM(SER3,0,servo3);
        pca9685.setPWM(SER4,0,servo4);
        pca9685.setPWM(SER5,0,servo5);
        pca9685.setPWM(SER6,0,servo6);
        pca9685.setPWM(SER7,0,servo7);

        if(one_print==1)
        {
          Serial.print("Input:");
          Serial.println(input);
          Serial.print("Node:");
          Serial.println(node);
          Serial.print("servo0 -> rads:");
          Serial.print(SS_servo0[node]);
          Serial.print("        deg:");
          Serial.print(servo0_deg);
          Serial.print("        us:");
          Serial.println(servo0);
          Serial.print("servo1 -> rads:");
          Serial.print(SS_servo1[node]);
          Serial.print("        deg:");
          Serial.print(servo1_deg);
          Serial.print("        us:");
          Serial.println(servo1);
          Serial.print("servo2 -> rads:");
          Serial.print(SS_servo2[node]);
          Serial.print("        deg:");
          Serial.print(servo2_deg);
          Serial.print("        us:");
          Serial.println(servo2);
          Serial.print("servo3 -> rads:");
          Serial.print(SS_servo3[node]);
          Serial.print("        deg:");
          Serial.print(servo3_deg);
          Serial.print("        us:");
          Serial.println(servo3);
          Serial.print("servo4 -> rads:");
          Serial.print(SS_servo4[node]);
          Serial.print("        deg:");
          Serial.print(servo4_deg);
          Serial.print("        us:");
          Serial.println(servo4);
          Serial.print("servo5 -> rads:");
          Serial.print(SS_servo5[node]);
          Serial.print("        deg:");
          Serial.print(servo5_deg);
          Serial.print("        us:");
          Serial.println(servo5);
          Serial.print("servo6 -> rads:");
          Serial.print(SS_servo6[node]);
          Serial.print("        deg:");
          Serial.print(servo6_deg);
          Serial.print("        us:");
          Serial.println(servo6);
          Serial.print("servo7 -> rads:");
          Serial.print(SS_servo7[node]);
          Serial.print("        deg:");
          Serial.print(servo7_deg);
          Serial.print("        us:");
          Serial.println(servo7);
          one_print=0;
        }
  
  }
}