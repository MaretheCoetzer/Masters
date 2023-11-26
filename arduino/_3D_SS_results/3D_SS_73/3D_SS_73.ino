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
float SS_servo0[] = {-0.66531888, -0.66644879, -0.64210311, -0.6266247 , -0.60068599, -0.53093483, -0.45082768, -0.36991595, -0.28884647, -0.20775846, -0.12719646, -0.12580262, -0.16067473, -0.19407302, -0.221797  , -0.2440337 , -0.25823464, -0.27670422, -0.29168063, -0.30826684, -0.32666173, -0.34657846, -0.35878398, -0.3534191 , -0.34629498, -0.3369149 , -0.3358763 , -0.36838135, -0.40559426, -0.44554614, -0.48392949, -0.52568206, -0.54972711, -0.56389491, -0.58716848, -0.60286437, -0.61861995, -0.63536168, -0.64236733, -0.64766761, -0.65316028, -0.66053794, -0.66220539};
float SS_servo1[] = {0.64730048, 0.76969105, 0.82156545, 0.8170574 , 0.86664957, 0.87750785, 0.87816491, 0.87847776, 0.8786645 , 0.87810401, 0.87882158, 0.91984289, 0.97605733, 1.00713467, 1.04432655, 1.08511287, 1.06994536, 1.04450875, 0.99302062, 0.94221055, 0.90184763, 0.88832189, 0.85662629, 0.78703523, 0.7105818 , 0.63357526, 0.58969084, 0.64833911, 0.73059011, 0.81154878, 0.88043621, 0.95696612, 0.98028617, 0.96910511, 0.98253501, 0.99021162, 0.98803018, 0.97056557, 0.9162364 , 0.8511742 , 0.78413145, 0.7309897 , 0.67962178};
float SS_servo2[] = {-0.25649844, -0.27627702, -0.29450952, -0.30691623, -0.31243508, -0.31775816, -0.3278251 , -0.34339845, -0.37167788, -0.40425178, -0.4414636 , -0.47336011, -0.48471097, -0.49356221, -0.51108832, -0.52343894, -0.52754075, -0.53009193, -0.52325409, -0.51655415, -0.51236893, -0.51374945, -0.49964978, -0.45245752, -0.38720717, -0.31943405, -0.24495995, -0.16485291, -0.08409946, -0.00323949,  0.07743874,  0.17260309,  0.16539304,  0.11484206,  0.07356107,  0.03293446, -0.00702339, -0.04456476, -0.07993311, -0.11967681, -0.15827469, -0.19822643, -0.24073853};
float SS_servo3[] = {0.8632612 , 0.88671652, 0.87791464, 0.8344045 , 0.76554736, 0.71290542, 0.68939317, 0.68706569, 0.71397843, 0.75380411, 0.81289412, 0.86258092, 0.86549825, 0.84545475, 0.85683461, 0.87024978, 0.83776435, 0.785651  , 0.70240285, 0.61690281, 0.53931335, 0.56059429, 0.52393908, 0.52674401, 0.54211086, 0.55490044, 0.55449991, 0.52981744, 0.51747854, 0.51432836, 0.50693055, 0.49006053, 0.53386283, 0.60632039, 0.66755209, 0.73725105, 0.79232054, 0.82175334, 0.82005026, 0.81932212, 0.81441836, 0.82409832, 0.85075748};
float SS_servo4[] = {0.6274467 , 0.62625175, 0.62581221, 0.6255438 , 0.63191735, 0.63520015, 0.629378  , 0.62851883, 0.62440248, 0.62400754, 0.62556952, 0.61725057, 0.60905433, 0.61258817, 0.59744437, 0.55537449, 0.5213419 , 0.4958644 , 0.48639176, 0.48081248, 0.47429252, 0.45308574, 0.43007043, 0.39916176, 0.38020601, 0.359414  , 0.33762745, 0.30684383, 0.27471317, 0.24630035, 0.21553146, 0.22766407, 0.28345397, 0.28778855, 0.32043239, 0.39769941, 0.47858454, 0.55599351, 0.60966208, 0.61982838, 0.62677197, 0.65048809, 0.63758854};
float SS_servo5[] = {-0.36089307, -0.3761968 , -0.41780002, -0.4774782 , -0.56066474, -0.6262845 , -0.65989203, -0.6930223 , -0.71696531, -0.74278131, -0.76271686, -0.76227634, -0.76526248, -0.80765639, -0.80238584, -0.73266744, -0.70365652, -0.70613311, -0.75660703, -0.81985853, -0.87996253, -0.88941808, -0.89606847, -0.89026233, -0.91503478, -0.93386859, -0.93575478, -0.87912469, -0.80256447, -0.74087894, -0.68250147, -0.66880003, -0.65554929, -0.65808777, -0.70046435, -0.70452767, -0.70300192, -0.69918656, -0.67163439, -0.59933115, -0.5226693 , -0.51554919, -0.40335009};
float SS_servo6[] = { 0.41686697,  0.38611506,  0.33583219,  0.2879784 ,  0.24478427,  0.20322504,  0.1582702 ,  0.11156722,  0.06038396,  0.00634606, -0.03868038, -0.04679376,  0.00656246,  0.0156162 ,  0.05152036,  0.12946474,  0.2088157 ,  0.28240962,  0.35009916,  0.42338886,  0.47891338,  0.48000001,  0.48737756,  0.50127078,  0.51514612,  0.53055231,  0.53795194,  0.51070381,  0.48011143,  0.45166432,  0.45265908,  0.4408829 ,  0.43710838,  0.44555992,  0.43264361,  0.39848355,  0.38429366,  0.39568853,  0.41031404,  0.42316919,  0.43399103,  0.43438684,  0.42632571};
float SS_servo7[] = {-0.97423648, -0.92992588, -0.87113712, -0.83778917, -0.82732564, -0.80505842, -0.75486109, -0.68975133, -0.61072813, -0.52053312, -0.41199736, -0.35044567, -0.34021393, -0.35002511, -0.41625862, -0.46909391, -0.47116085, -0.46407261, -0.45031778, -0.4420214 , -0.3917265 , -0.31521055, -0.3256782 , -0.40422216, -0.48839493, -0.57433801, -0.63133659, -0.5897912 , -0.52737833, -0.4728759 , -0.47990158, -0.46142466, -0.47677984, -0.5280791 , -0.53178689, -0.48514252, -0.48697452, -0.55432165, -0.64999981, -0.75246496, -0.85437837, -0.92760114, -0.97312215};

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
