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
float SS_servo0[] = {-0.37022152, -0.3783601 , -0.39690981, -0.39019181, -0.32593642, -0.24785146, -0.17361001, -0.12601606, -0.08873115, -0.05284973, -0.0303255 , -0.03939734, -0.06870134, -0.09060879, -0.10620742, -0.11943539, -0.12889666, -0.14002271, -0.15601559, -0.16765814, -0.1783301 , -0.18876436, -0.19860911, -0.20656202, -0.21188782, -0.21527783, -0.22164771, -0.22858045, -0.24062235, -0.25530683, -0.27176153, -0.29003535, -0.30849144, -0.32372476, -0.33582694, -0.34662647, -0.35006049, -0.35263628, -0.35954534, -0.36643289};
float SS_servo1[] = {1.0560164 , 1.04349133, 1.0397853 , 1.07162749, 1.15032042, 1.19271614, 1.13288458, 1.06148412, 0.98626504, 0.91399719, 0.86554199, 0.88352167, 0.94574294, 0.99242783, 1.02475834, 1.04780175, 1.06090376, 1.05644592, 1.03964199, 1.01361774, 0.98746459, 0.96271346, 0.93893263, 0.92189337, 0.92440279, 0.92462943, 0.92091395, 0.91886897, 0.93303932, 0.95564558, 0.98416523, 1.01606392, 1.04530854, 1.06593121, 1.07701844, 1.08307831, 1.09017062, 1.09194761, 1.08429577, 1.06151464};
float SS_servo2[] = {-0.16997362, -0.17904545, -0.19304444, -0.20483888, -0.20997386, -0.2128646 , -0.21764077, -0.22688553, -0.24134745, -0.25778547, -0.2754458 , -0.29512255, -0.31447718, -0.32930699, -0.33947363, -0.34555712, -0.34737751, -0.35412762, -0.367513  , -0.37893241, -0.38818842, -0.40156144, -0.41846307, -0.39926029, -0.32776748, -0.25843028, -0.20472163, -0.16636104, -0.12600111, -0.08882502, -0.05815483, -0.05612821, -0.08418373, -0.10599478, -0.12245998, -0.1371925 , -0.14147171, -0.14347704, -0.15241172, -0.1642205 };
float SS_servo3[] = {0.93153832, 0.92003748, 0.90214963, 0.89101215, 0.8911108 , 0.89371141, 0.89683494, 0.90656501, 0.9269975 , 0.9538174 , 0.98650907, 1.02528292, 1.0608599 , 1.08792535, 1.10556807, 1.11173927, 1.10761654, 1.09490399, 1.07634683, 1.05491624, 1.03131466, 1.01831635, 1.01686035, 1.03626971, 1.12000073, 1.16821986, 1.10490368, 1.02840114, 0.95427489, 0.88091827, 0.81728386, 0.80748549, 0.86124048, 0.89925563, 0.9226812 , 0.94017406, 0.94878967, 0.94734591, 0.94401074, 0.93284895};
float SS_servo4[] = {0.4161279 , 0.41051356, 0.40059439, 0.38957443, 0.38984376, 0.38744912, 0.38219631, 0.3684997 , 0.35308969, 0.33492414, 0.3173947 , 0.30244026, 0.28789976, 0.27823723, 0.27349612, 0.26648848, 0.26338215, 0.24948506, 0.23303772, 0.21935084, 0.20649198, 0.19155751, 0.17442289, 0.16149997, 0.15884587, 0.15284865, 0.14035309, 0.12161874, 0.10311122, 0.07983593, 0.05717716, 0.05985442, 0.09083813, 0.13103639, 0.17772817, 0.22787375, 0.30001833, 0.37871281, 0.42491564, 0.41898908};
float SS_servo5[] = {-1.09861347, -1.11646452, -1.14379039, -1.15966473, -1.17326326, -1.17323945, -1.17033038, -1.15297605, -1.13184653, -1.10193531, -1.07038883, -1.04383701, -1.02036074, -1.00502944, -0.99950841, -0.99049674, -0.99148645, -0.98709384, -0.99903418, -1.01816414, -1.03867477, -1.05386519, -1.06228503, -1.06785473, -1.0730871 , -1.06787581, -1.05791453, -1.03202339, -1.00078153, -0.95500704, -0.90722565, -0.91843483, -0.99244411, -1.07008672, -1.15093051, -1.20146821, -1.13349775, -1.07544712, -1.07652137, -1.09113367};
float SS_servo6[] = {0.16022457, 0.15662543, 0.1486786 , 0.1365632 , 0.13111075, 0.1217741 , 0.11106497, 0.0926191 , 0.06934061, 0.04344891, 0.04400194, 0.07875103, 0.12021898, 0.17660986, 0.2482638 , 0.31797126, 0.37917289, 0.39954383, 0.37711844, 0.36798928, 0.36355161, 0.35619489, 0.34743023, 0.34019507, 0.33834007, 0.33385634, 0.32413038, 0.30954716, 0.29499709, 0.27755587, 0.25791747, 0.23850765, 0.22173434, 0.20779713, 0.19943325, 0.18855237, 0.18297832, 0.17569602, 0.17132609, 0.16214992};
float SS_servo7[] = {-0.94849456, -0.97217945, -1.00628106, -1.01954784, -1.01996202, -1.00290533, -0.98634152, -0.954458  , -0.91149248, -0.85971555, -0.86482393, -0.93823783, -1.01946563, -1.10320309, -1.17331104, -1.18144785, -1.09596446, -1.00838763, -0.99186451, -1.01515005, -1.04938822, -1.07831744, -1.10419511, -1.12111303, -1.12553823, -1.12267283, -1.1201528 , -1.10668199, -1.08695152, -1.05754425, -1.02059339, -0.9855593 , -0.96031504, -0.94208136, -0.93814148, -0.92956418, -0.91748874, -0.90601187, -0.92066134, -0.9377969 };

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
