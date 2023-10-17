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
float SS_servo0[] = {-0.44717396, -0.46227239, -0.47575685, -0.43709776, -0.36253599, -0.28078688, -0.19468392, -0.12418845, -0.07481472, -0.03962981, -0.01332907, -0.01837182, -0.05255555, -0.08448842, -0.10672175, -0.12302022, -0.13758571, -0.14829132, -0.15927913, -0.17285321, -0.18365548, -0.19351963, -0.20251045, -0.21378508, -0.22659228, -0.2402809 , -0.25425284, -0.27046273, -0.28963956, -0.31037304, -0.33208567, -0.35486282, -0.375995  , -0.39421852, -0.4081079 , -0.41753713, -0.42596944, -0.43217705, -0.43876515, -0.44346229};
float SS_servo1[] = {0.90084986, 0.89734806, 0.92023816, 0.97358771, 1.05604396, 1.09555156, 1.03325834, 0.96412363, 0.8924761 , 0.81609417, 0.75723006, 0.75846788, 0.8187828 , 0.87337183, 0.90852396, 0.92966858, 0.94236871, 0.94426846, 0.92970893, 0.90055268, 0.86603922, 0.83505859, 0.81302386, 0.80648573, 0.81620477, 0.82800979, 0.83193993, 0.83960779, 0.85560012, 0.87735936, 0.90345906, 0.93032014, 0.95051897, 0.96438062, 0.9714361 , 0.97023629, 0.96659794, 0.96224169, 0.94616869, 0.90955838};
float SS_servo2[] = {-0.23389169, -0.24621611, -0.26186835, -0.2715343 , -0.27955164, -0.28565977, -0.29343852, -0.30610103, -0.3231788 , -0.34294288, -0.36324844, -0.38521939, -0.40729184, -0.42587611, -0.43836855, -0.44441833, -0.44739454, -0.44766243, -0.45060792, -0.45682079, -0.46773816, -0.46660345, -0.43660483, -0.38343859, -0.3100036 , -0.23760535, -0.18204056, -0.14795085, -0.11425897, -0.08299572, -0.0651748 , -0.07840673, -0.11274532, -0.14350685, -0.16418779, -0.18033248, -0.19389671, -0.20212442, -0.21337907, -0.22705211};
float SS_servo3[] = {0.81484331, 0.8044398 , 0.78832994, 0.77693864, 0.77961015, 0.78390423, 0.78971684, 0.80652977, 0.82956561, 0.85588209, 0.88553809, 0.91811307, 0.94715882, 0.96814231, 0.9793846 , 0.97770925, 0.96663247, 0.94861656, 0.92084059, 0.88211521, 0.85336367, 0.84135025, 0.86340917, 0.91312686, 0.99149984, 1.02797816, 0.96718924, 0.89135939, 0.81640968, 0.74405857, 0.6968877 , 0.70724385, 0.7593036 , 0.80287718, 0.82593739, 0.83945841, 0.84644001, 0.84389548, 0.83615921, 0.81748859};
float SS_servo4[] = {0.47950432, 0.47316238, 0.46442315, 0.45696154, 0.45386584, 0.44977852, 0.44200115, 0.43054143, 0.41394214, 0.39405982, 0.37450929, 0.35576884, 0.33627722, 0.31978115, 0.30737841, 0.29494836, 0.28356719, 0.27484267, 0.26236902, 0.24433201, 0.2244549 , 0.20652006, 0.19129604, 0.17786002, 0.16816828, 0.15578224, 0.13450607, 0.10695649, 0.07203224, 0.04218929, 0.04468074, 0.07423211, 0.11119805, 0.16386749, 0.23303012, 0.30429616, 0.37194142, 0.4444608 , 0.4842256 , 0.48165405};
float SS_servo5[] = {-1.00733781, -1.02902099, -1.06101261, -1.07881249, -1.08686591, -1.08856254, -1.08480416, -1.072733  , -1.05302654, -1.02753172, -1.00004831, -0.97487084, -0.95163852, -0.93434916, -0.92227169, -0.90910934, -0.90227237, -0.90355764, -0.91281536, -0.93035777, -0.94529971, -0.95965142, -0.97010393, -0.97392544, -0.97266357, -0.96516071, -0.94572423, -0.9123002 , -0.85979068, -0.81380988, -0.83611881, -0.91211667, -0.99631634, -1.08032708, -1.14627134, -1.15917017, -1.11167078, -1.02725821, -0.97572852, -0.99734891};
float SS_servo6[] = {0.17743422, 0.17110532, 0.15946807, 0.1453468 , 0.1338374 , 0.12011188, 0.10289697, 0.08206929, 0.05150287, 0.02363071, 0.02416339, 0.05338169, 0.09045391, 0.13729983, 0.2025593 , 0.27320001, 0.34063093, 0.41056069, 0.44619966, 0.4456229 , 0.43879631, 0.43454706, 0.42982316, 0.42335812, 0.41813713, 0.41110724, 0.397694  , 0.37998641, 0.35792712, 0.33390776, 0.30984023, 0.28772717, 0.26756342, 0.25112084, 0.23726568, 0.22473566, 0.21310267, 0.2049803 , 0.193172  , 0.17972617};
float SS_servo7[] = {-0.87992852, -0.90370288, -0.93250813, -0.93758172, -0.92785682, -0.90684211, -0.88038947, -0.84377939, -0.7884491 , -0.73975201, -0.74955094, -0.81438686, -0.89441136, -0.97903382, -1.04655208, -1.0503161 , -0.98944242, -0.89581677, -0.83118743, -0.86919462, -0.90556817, -0.94406378, -0.97304337, -0.98916002, -0.99467065, -0.99770742, -0.9968045 , -0.98845093, -0.96824327, -0.94041052, -0.90993215, -0.88520388, -0.86665122, -0.8553276 , -0.84738834, -0.84069632, -0.83707189, -0.83872754, -0.8467231 , -0.86648038};

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
  servo2=map(servo2_deg,-38,90,1510,2730);
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
        servo2=map(servo2_deg,-38,90,1510,2730);
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
