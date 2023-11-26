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
float SS_servo0[] = {-0.66751891, -0.62999191, -0.5966646 , -0.54491661, -0.49120451, -0.43494705, -0.37091723, -0.29152587, -0.21100907, -0.12978161, -0.04823845,  0.0328405 ,  0.03931779,  0.01979309,  0.00804217, -0.00279998, -0.01780595, -0.04487992, -0.07246271, -0.09631854, -0.12033084, -0.14972012, -0.18513761, -0.22116631, -0.25163657, -0.27919259, -0.30842305, -0.33966432, -0.36939358, -0.3980973 , -0.43074893, -0.46493763, -0.50082614, -0.53582661, -0.55974757, -0.57885561, -0.59956771, -0.62110973, -0.63651304, -0.64092794, -0.64661895, -0.65195522, -0.6554325 , -0.65524258, -0.6638141 };
float SS_servo1[] = {0.89141453, 0.91003717, 0.87841926, 0.9046026 , 0.93372737, 0.95864336, 0.97598305, 0.97454352, 0.95820541, 0.92818548, 0.91497589, 0.89322166, 0.98387646, 1.12234822, 1.22775975, 1.31384235, 1.3883963 , 1.43970215, 1.47040592, 1.47777217, 1.46462967, 1.43601063, 1.40238534, 1.36248867, 1.32917732, 1.3061868 , 1.28118402, 1.26164901, 1.2578345 , 1.26363373, 1.27282415, 1.27619014, 1.27848487, 1.31020969, 1.33356888, 1.33931413, 1.35252835, 1.36866543, 1.34176304, 1.26295366, 1.18712021, 1.11223243, 1.03594148, 0.96466652, 0.93120082};      
float SS_servo2[] = {-0.26182531, -0.2763959 , -0.29473139, -0.30582503, -0.3164848 , -0.32803049, -0.3461203 , -0.38725491, -0.43125587, -0.47889333, -0.52247214, -0.5587697 , -0.57998128, -0.59165515, -0.60014452, -0.61160598, -0.628996  , -0.64947675, -0.66270714, -0.6632414 , -0.6577521 , -0.65319162, -0.65303886, -0.63511739, -0.57576386, -0.49907078, -0.4247744 , -0.35046865, -0.27256065, -0.19192016, -0.11123701, -0.03046842,  0.05029276,  0.12349529,  0.0814563 ,  0.03665795, -0.00142659, -0.03788799, -0.07545262, -0.10847665, -0.138979  , -0.16427274, -0.1877572 , -0.21610138, -0.24569636};
float SS_servo3[] = {0.93472918, 0.91658664, 0.86861899, 0.79525789, 0.72255229, 0.65679757, 0.63017739, 0.69378793, 0.77961981, 0.88416938, 0.99569417, 1.11488497, 1.22117996, 1.29949693, 1.35710641, 1.40495469, 1.44699752, 1.46271295, 1.45550226, 1.4200264 , 1.36181619, 1.28844492, 1.20971125, 1.18537732, 1.20615449, 1.21013564, 1.21640893, 1.22266885, 1.19143287, 1.11423404, 1.03311628, 0.95194579, 0.87243728, 0.80364431, 0.88582551, 0.96519999, 1.03623803, 1.10475562, 1.12421207, 1.09011775, 1.05177575, 1.00337692, 0.95400681, 0.93186905, 0.93657803};      
float SS_servo4[] = { 0.60163669,  0.60057741,  0.59048276,  0.58217762,  0.57144625,  0.55567569,  0.54177775,  0.5251552 ,  0.51457037,  0.5048551 ,  0.49996918,  0.49617006,  0.49326995,  0.49192734,  0.49557579,  0.50464303,  0.5205835 ,  0.53584366,  0.54199743,  0.53378633,  0.5211072 ,  0.51176303,  0.50005983,  0.48005654,  0.43924953,  0.38669543,  0.33411978,  0.28073391,  0.22537389,  0.16582143,  0.10712194,  0.03834689, -0.03280361, -0.03059343,  0.04741793,  0.12848342,  0.20944301,  0.29056098,  0.36166564,  0.42463136,  0.49176782,  0.56608989,  0.61781332,  0.60185869,  0.60085732};
float SS_servo5[] = {-0.90602673, -0.94669065, -1.0055632 , -1.0840743 , -1.16027388, -1.22477278, -1.27013473, -1.26507031, -1.25318941, -1.22554188, -1.18929713, -1.13424218, -1.06232842, -1.00547322, -0.97439651, -0.9703908 , -0.99750588, -1.05341471, -1.09857901, -1.11560922, -1.13479077, -1.18162263, -1.24301486, -1.30178495, -1.30047032, -1.25340123, -1.21340234, -1.17287204, -1.11162807, -1.02384887, -0.9440177 , -0.84970769, -0.75516235, -0.75171433, -0.80178868, -0.82462068, -0.86843263, -0.93231624, -0.96791264, -0.94723991, -0.930367  , -0.92235281, -0.90553372, -0.89403345, -0.89692822};
float SS_servo6[] = {0.35194823, 0.342336  , 0.32496149, 0.30739738, 0.28745406, 0.2620457 , 0.23586658, 0.21100815, 0.19201345, 0.17022246, 0.14705244, 0.1582004 , 0.23291337, 0.31368713, 0.39439712, 0.4749022 , 0.55544024, 0.55550051, 0.56105181, 0.56979873, 0.5735361 , 0.59109945, 0.59057662, 0.59863636, 0.62790444, 0.6403986 , 0.6510418 , 0.66260469, 0.66364505, 0.65674795, 0.65345271, 0.65627007, 0.65694587, 0.63430016, 0.58624382, 0.53492247, 0.4863083 , 0.43812416, 0.41187437, 0.41438099, 0.41538024, 0.41273916, 0.4071601 , 0.39260333, 0.36497629};      
float SS_servo7[] = {-0.9239946 , -0.95198173, -1.00167752, -1.06973628, -1.13607022, -1.19199608, -1.21944058, -1.18897621, -1.14774441, -1.08139363, -0.99378221, -0.95477466, -1.00123634, -1.01132569, -1.02718991, -1.028555  , -1.02856216, -0.49889663, -0.23031506, -0.29456833, -0.330895  , -0.33482094, -0.26017697, -0.25080772, -0.36930621, -0.44929785, -0.53008845, -0.61358541, -0.66680209, -0.69755027, -0.73999852, -0.80053864, -0.8613794 , -0.85669896, -0.78680128, -0.71164744, -0.63589518, -0.55895756, -0.5539163 , -0.63811224, -0.72023226, -0.79787609, -0.87158203, -0.91781271, -0.91621839};

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
