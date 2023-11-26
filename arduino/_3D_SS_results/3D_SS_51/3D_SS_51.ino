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
float SS_servo0[] = {-0.62737952, -0.6000911 , -0.54142538, -0.50621476, -0.44700177, -0.38513229, -0.31916066, -0.24685994, -0.21316261, -0.19019563, -0.12766952, -0.07869737, -0.06245759, -0.08740957, -0.11493268, -0.14014095, -0.15871608, -0.17593048, -0.18921576, -0.20571312, -0.22311774, -0.24472545, -0.26639935, -0.28470033, -0.30280793, -0.31612188, -0.32682616, -0.3402286 , -0.3481603 , -0.34970676, -0.35380993, -0.36594741, -0.3968742 , -0.43275222, -0.46861234, -0.50659367, -0.53725127, -0.55823694, -0.56341792, -0.57223606, -0.58169968, -0.5922441 , -0.60057017, -0.60714574, -0.61837143, -0.62732832, -0.63151155};
float SS_servo1[] = {0.68098648, 0.66605883, 0.70106043, 0.67066552, 0.68841889, 0.70913671, 0.71939679, 0.6679247 , 0.58798383, 0.63905375, 0.65704958, 0.6534508 , 0.68019397, 0.74427627, 0.81863718, 0.91343332, 1.01008869, 1.08356473, 1.10140408, 1.09912309, 1.09267205, 1.06690227, 1.02773492, 0.98956792, 0.96397684, 0.92781962, 0.89572632, 0.87868899, 0.83732287, 0.77704576, 0.71952686, 0.69301558, 0.76365631, 0.84476837, 0.91595938, 0.97998028, 1.0213358 , 1.04971093, 1.03160288, 1.01802779, 1.0002297 , 0.97240161, 0.92829742, 0.87434802, 0.82629539, 0.77518253, 0.73537717};
float SS_servo2[] = {-0.24908306, -0.25944482, -0.2718168 , -0.28697428, -0.2942144 , -0.29891774, -0.30454661, -0.34387624, -0.39601733, -0.44943731, -0.4879276 , -0.49676692, -0.49737255, -0.49700285, -0.50260771, -0.51056498, -0.5194685 , -0.52816311, -0.5282862 , -0.52992765, -0.53172115, -0.53421907, -0.5368032 , -0.53056686, -0.52429248, -0.5009199 , -0.43944537, -0.40161728, -0.36625081, -0.30237579, -0.23345687, -0.16022042, -0.07999914,  0.00074973,  0.08149089,  0.13848283,  0.13813753,  0.1363454 ,  0.09415582,  0.06041755,  0.02572985, -0.01000593, -0.0450203 , -0.08224735, -0.11903268, -0.16276438, -0.21016029};
float SS_servo3[] = {0.8269769 , 0.78852059, 0.73736844, 0.69328187, 0.6232751 , 0.55038789, 0.48468704, 0.52310192, 0.60606365, 0.69471187, 0.75233488, 0.75915755, 0.7617066 , 0.76417542, 0.78414446, 0.83013512, 0.88895577, 0.92730531, 0.91343959, 0.88142473, 0.8451566 , 0.7891843 , 0.72200406, 0.6466256 , 0.5796177 , 0.54729861, 0.58497045, 0.58105969, 0.52442869, 0.53283025, 0.54512226, 0.53796834, 0.48909564, 0.48466486, 0.48124549, 0.45813319, 0.4804952 , 0.542442  , 0.60829932, 0.65351696, 0.69402072, 0.72134638, 0.73267877, 0.73993491, 0.74087172, 0.76004669, 0.80705912};
float SS_servo4[] = {0.62060277, 0.64096037, 0.65060963, 0.65505829, 0.65798924, 0.66189209, 0.66221019, 0.65196529, 0.63352541, 0.61969579, 0.60406335, 0.58698265, 0.56492107, 0.54399535, 0.52742252, 0.50921208, 0.48754679, 0.46999504, 0.47167776, 0.48436002, 0.4969585 , 0.50916886, 0.52119018, 0.51475338, 0.49662409, 0.47409832, 0.45344646, 0.4300841 , 0.40730779, 0.38968093, 0.3682322 , 0.34214161, 0.31146861, 0.27965066, 0.24529856, 0.20810435, 0.2078889 , 0.2757762 , 0.35708869, 0.43831471, 0.51907203, 0.59940415, 0.64877933, 0.6620604 , 0.6618447 , 0.62859168, 0.62817044};
float SS_servo5[] = {-0.53594745, -0.62608708, -0.71077394, -0.78680416, -0.87390916, -0.96280733, -1.04127088, -1.06748574, -1.06007236, -1.0553781 , -1.04547606, -1.02483676, -0.98199989, -0.93968945, -0.8979482 , -0.83487491, -0.75195774, -0.69427124, -0.70921296, -0.76680523, -0.82928127, -0.91438244, -1.01301215, -1.06976367, -1.09356109, -1.10852058, -1.12034303, -1.11924725, -1.13247749, -1.16619063, -1.19727817, -1.20496124, -1.12711745, -1.04540959, -0.96894051, -0.8988333 , -0.88093948, -0.87540902, -0.87612676, -0.87629793, -0.87622131, -0.87592892, -0.84495545, -0.77787723, -0.69857765, -0.65705261, -0.58905246};
float SS_servo6[] = { 0.39461351,  0.37859737,  0.35135057,  0.32596175,  0.30186489,  0.27706436,  0.25066374,  0.21619397,  0.17930666,  0.14415021,  0.10715658,  0.06620592,  0.03642368,  0.00387121, -0.02789212, -0.00100861,  0.07874266,  0.15925814,  0.2394531 ,  0.31689342,  0.39450586,  0.47291568,  0.51598153,  0.54356566,  0.55466374,  0.54997976,  0.56788726,  0.58744883,  0.60060355,  0.62051859,  0.64048842,  0.64704221,  0.6188385 ,  0.5856831 ,  0.56875903,  0.5600962 ,  0.55005303,  0.50542065,  0.45686477,  0.41592093,  0.38490952,  0.38036551,  0.39467113,  0.40538192,  0.41344101,  0.42057293,  0.40806307};
float SS_servo7[] = {-0.88740683, -0.91405755, -0.93562129, -0.96084618, -1.00454993, -1.04773382, -1.0863031 , -1.06258372, -1.00528788, -0.94606452, -0.88273589, -0.80139957, -0.704614  , -0.60136469, -0.48659432, -0.4788666 , -0.5055524 , -0.50760879, -0.506549  , -0.49929928, -0.49246221, -0.48753681, -0.48924311, -0.49480601, -0.45511796, -0.38096127, -0.43058547, -0.50689412, -0.58377348, -0.67847291, -0.7758898 , -0.83615237, -0.77946051, -0.71215804, -0.68087515, -0.67438101, -0.67668152, -0.60569835, -0.53655736, -0.48229923, -0.4524287 , -0.48710144, -0.57106795, -0.65655082, -0.74269349, -0.82660282, -0.85383335};

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
