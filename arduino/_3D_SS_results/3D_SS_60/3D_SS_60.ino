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
float SS_servo0[] = {-0.66629001, -0.66367321, -0.60887753, -0.55020883, -0.49063779, -0.42332091, -0.34347099, -0.26293923, -0.18177884, -0.10074805, -0.0193673 ,  0.0029923 , -0.02462851, -0.05023984, -0.07610882, -0.1098961 , -0.1495389 , -0.19003109, -0.23210146, -0.27031988, -0.29689249, -0.31172629, -0.31720996, -0.32482118, -0.33337349, -0.34649347, -0.37443992, -0.40470626, -0.44184677, -0.47568917, -0.51385395, -0.54417619, -0.56538832, -0.58443256, -0.60867541, -0.63085213, -0.63361773, -0.63509286, -0.63303727, -0.62895851, -0.6358403 };
float SS_servo1[] = {0.82545985, 0.85854721, 0.88251796, 0.90502665, 0.92618439, 0.94080506, 0.94162847, 0.94166229, 0.94145331, 0.94123914, 0.93249718, 0.97308295, 1.06363132, 1.13219243, 1.17656512, 1.20696111, 1.20140025, 1.17221137, 1.13928418, 1.11239625, 1.06645451, 1.00279052, 0.9324595 , 0.86815083, 0.81878648, 0.79749021, 0.82471997, 0.86852956, 0.91890155, 0.95161374, 0.99658195, 1.04622247, 1.07618335, 1.09710515, 1.12499235, 1.14126596, 1.07794731, 0.99980463, 0.91701027, 0.83427126, 0.79628123};
float SS_servo2[] = {-0.24559441, -0.2551936 , -0.26544866, -0.27009466, -0.27324659, -0.28307256, -0.31597785, -0.35614686, -0.40077127, -0.44438652, -0.4848815 , -0.50793559, -0.52255934, -0.53554351, -0.55186382, -0.5693598 , -0.59072149, -0.60774731, -0.62945786, -0.64198729, -0.64263063, -0.58455918, -0.51465849, -0.44607485, -0.37335918, -0.29381423, -0.21334584, -0.13270793, -0.05200002,  0.02848664,  0.0979184 ,  0.08297184,  0.03031061, -0.01335183, -0.05512789, -0.09293676, -0.12239963, -0.15087294, -0.17579361, -0.19710358, -0.22504385};
float SS_servo3[] = {0.89215564, 0.83724695, 0.75834432, 0.67584493, 0.60063289, 0.56392442, 0.60820947, 0.68397142, 0.77888524, 0.88173716, 0.99142275, 1.06895717, 1.11261869, 1.1362654 , 1.14816031, 1.13970542, 1.10630902, 1.04555786, 0.98675295, 0.92369855, 0.88386768, 0.89099027, 0.90180726, 0.91372451, 0.92156668, 0.91822372, 0.8901874 , 0.83605195, 0.77903437, 0.70655551, 0.65183209, 0.68585902, 0.79729559, 0.88769782, 0.97258765, 1.03472538, 1.01413593, 0.9780763 , 0.93756367, 0.89198841, 0.89430914};
float SS_servo4[] = { 0.57513914,  0.58659007,  0.58193776,  0.57903021,  0.57519558,  0.57036487,  0.55760402,  0.54670613,  0.54111119,  0.53624054,  0.52602592,  0.50215083,  0.47083202,  0.44254989,  0.42306393,  0.4150186 ,  0.41378547,  0.40664219,  0.39167354,  0.36582184,  0.32491343,  0.27621234,  0.22301201,  0.17465813,  0.12462344,  0.08139366,  0.03661752, -0.0054577 , -0.05150075, -0.10145903, -0.11455216, -0.04614582,  0.03471811,  0.11613349,  0.19718391,  0.27542539,  0.34128125,  0.40085242,  0.46602797,  0.54027948,  0.56391487};
float SS_servo5[] = {-0.73980979, -0.82221032, -0.89612379, -0.97346938, -1.04666862, -1.09565844, -1.09944665, -1.08983814, -1.07690192, -1.05445653, -1.00832868, -0.93215692, -0.85594928, -0.79907967, -0.7763345 , -0.7988951 , -0.87012398, -0.95265896, -1.031595  , -1.07803963, -1.08539572, -1.07026439, -1.03389792, -1.01112892, -0.97132198, -0.92659666, -0.85654189, -0.77840464, -0.69992464, -0.6248865 , -0.61356952, -0.6752726 , -0.69329575, -0.72688132, -0.76221647, -0.78973624, -0.77887059, -0.7560222 , -0.73732047, -0.73597509, -0.73197877};
float SS_servo6[] = {0.38585475, 0.38198722, 0.36544653, 0.35028555, 0.33101964, 0.30656065, 0.27849769, 0.25279441, 0.22203696, 0.18965189, 0.19429957, 0.26678105, 0.34747985, 0.42816278, 0.50864852, 0.58894797, 0.6546845 , 0.66292083, 0.65180378, 0.64870962, 0.64879587, 0.64533332, 0.65379144, 0.66122501, 0.66392921, 0.66017746, 0.65098463, 0.6404839 , 0.63370419, 0.642347  , 0.63836766, 0.59477274, 0.54265244, 0.49083314, 0.44106679, 0.40627651, 0.40806861, 0.41464853, 0.4246264 , 0.4276551 , 0.40782885};
float SS_servo7[] = {-0.82470216, -0.88571139, -0.94345262, -1.00406775, -1.0547549 , -1.07064386, -1.03692768, -0.9868831 , -0.91197699, -0.82290875, -0.76345214, -0.75927925, -0.75955599, -0.75934155, -0.75924731, -0.75854688, -0.7437322 , -0.67073151, -0.59713626, -0.57296436, -0.48976755, -0.47959301, -0.56290834, -0.64413369, -0.70845707, -0.74773643, -0.76222352, -0.76318388, -0.77635072, -0.82549567, -0.8487906 , -0.77954762, -0.69036449, -0.60274418, -0.5194545 , -0.4711864 , -0.53842641, -0.62927885, -0.72669525, -0.81165561, -0.82597855};

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=120000; 
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
