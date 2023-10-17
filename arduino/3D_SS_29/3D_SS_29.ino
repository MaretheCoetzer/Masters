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
float SS_servo0[] = {-0.52268076, -0.50230405, -0.44735548, -0.38584096, -0.31797342, -0.24394104, -0.1421561 , -0.06391864,  0.0092784 ,  0.07848483,  0.20863414,  0.32597666,  0.40337936,  0.39035325,  0.31552265,  0.25392504,  0.19371731,  0.13465974,  0.07431729,  0.0156166 , -0.04873808, -0.10942083, -0.15241047, -0.18106218, -0.18876465, -0.19748855, -0.22182848, -0.25712343, -0.29445925, -0.32500114, -0.36341915, -0.41061616, -0.45437578, -0.4820292 , -0.51449849, -0.52150184, -0.52461582, -0.52133091, -0.52051527, -0.51994654, -0.52250881, -0.52580671, -0.51973412, -0.52010285};
float SS_servo1[] = {0.23662274, 0.24700208, 0.32098006, 0.39912221, 0.48274434, 0.55297415, 0.5198304 , 0.47159809, 0.44473004, 0.47934969, 0.44351783, 0.37456473, 0.30055498, 0.31476776, 0.42430775, 0.51274165, 0.59879234, 0.68034295, 0.75915773, 0.82699603, 0.89510038, 0.8935148 , 0.84464772, 0.79569279, 0.74451695, 0.70701628, 0.70585188, 0.72501866, 0.73578601, 0.72394602, 0.72305503, 0.74998685, 0.80417093, 0.83875696, 0.88190887, 0.841387  , 0.76986742, 0.69151857, 0.61295385, 0.53484368, 0.47304467, 0.41653059, 0.33906898, 0.27237309};
float SS_servo2[] = {-0.27128429, -0.29050904, -0.30441279, -0.31158491, -0.31075095, -0.32073891, -0.33912279, -0.35049649, -0.36374165, -0.40466852, -0.44755086, -0.48289441, -0.51505862, -0.5231786 , -0.51517354, -0.50884697, -0.49977189, -0.49539048, -0.4947194 , -0.49424684, -0.4910958 , -0.5123233 , -0.52322947, -0.48724825, -0.40989334, -0.32686322, -0.24741271, -0.14778277, -0.05512627,  0.02093557,  0.08815406,  0.17335427,  0.26711251,  0.33986776,  0.36793371,  0.30769862,  0.22536483,  0.16588193,  0.09236856,  0.01535825, -0.05607616, -0.11700748, -0.16344845, -0.2299289 };
float SS_servo3[] = {0.76697847, 0.74409802, 0.67969828, 0.60917726, 0.53654033, 0.49286842, 0.47164523, 0.4370212 , 0.41140445, 0.44498525, 0.52658378, 0.61644028, 0.70890824, 0.71226443, 0.65520021, 0.60064228, 0.53909645, 0.48283978, 0.42850805, 0.36769291, 0.29109065, 0.22966684, 0.2325112 , 0.29733602, 0.37199261, 0.45346105, 0.5392319 , 0.55055635, 0.49225001, 0.42321233, 0.38026169, 0.34506839, 0.27379803, 0.19504314, 0.13721185, 0.19949017, 0.2927941 , 0.3469006 , 0.42464965, 0.50898849, 0.58883538, 0.64931275, 0.67397079, 0.7343483 };
float SS_servo4[] = { 0.49513406,  0.50140204,  0.50138517,  0.50041398,  0.50063313,  0.50363105,  0.50803876,  0.51238461,  0.51829269,  0.52553705,  0.51947053,  0.50915415,  0.48795194,  0.45040617,  0.41961774,  0.38310656,  0.35179325,  0.33047945,  0.31518186,  0.30710259,  0.30100154,  0.26560203,  0.22137681,  0.15126278,  0.08202375,  0.02002552, -0.04215104, -0.10640261, -0.17761248, -0.2537864 , -0.33393799, -0.35743797, -0.28108326, -0.16097682,  0.00110656,  0.11481541,  0.19033235,  0.27036099,  0.34408981,  0.39862427,  0.47018776,  0.52491232,  0.52356556,  0.50857097};
float SS_servo5[] = {-0.31765305, -0.38535955, -0.46972674, -0.55035167, -0.62495081, -0.69841013, -0.76948847, -0.84364695, -0.91511309, -0.97973007, -0.97267231, -0.9354394 , -0.87197773, -0.81415662, -0.79519249, -0.7628464 , -0.74111382, -0.74195876, -0.76070122, -0.80123405, -0.85791935, -0.89738006, -0.94423235, -0.91494069, -0.8321769 , -0.75464801, -0.67377487, -0.59151781, -0.50873411, -0.42646764, -0.34296279, -0.36143554, -0.4494668 , -0.51197508, -0.54149129, -0.60369419, -0.65599881, -0.70167621, -0.74893626, -0.70928307, -0.61402756, -0.47306348, -0.40782924, -0.35887906};
float SS_servo6[] = { 0.01355185, -0.02420952, -0.08009042, -0.14687503, -0.19984727, -0.25907564, -0.31170661, -0.37247512, -0.43753395, -0.48851492, -0.44676477, -0.34726531, -0.21294668, -0.09847008, -0.01662601,  0.05924113,  0.13178787,  0.2024144 ,  0.27390484,  0.34823865,  0.44093469,  0.49181197,  0.47941691,  0.46122142,  0.46135467,  0.47127628,  0.48462793,  0.49817789,  0.50476178,  0.50645655,  0.52446739,  0.51800002,  0.47729505,  0.43443097,  0.38929337,  0.34827969,  0.30067736,  0.26503959,  0.22984742,  0.20422271,  0.19573814,  0.17972264,  0.12424558,  0.07101517};
float SS_servo7[] = {-0.50377131, -0.48391824, -0.46379587, -0.41543122, -0.38339819, -0.33100136, -0.28657274, -0.22632254, -0.15249009, -0.09798265, -0.16515009, -0.23895798, -0.27718182, -0.32018755, -0.38355011, -0.41199438, -0.40097289, -0.35118552, -0.27491775, -0.19122187, -0.17880628, -0.1310602 , -0.10254352, -0.16921459, -0.23336453, -0.3045857 , -0.38020256, -0.45840402, -0.53643216, -0.61516818, -0.72723489, -0.77945529, -0.73294143, -0.67435739, -0.60897064, -0.58080818, -0.55560772, -0.54834181, -0.54780246, -0.5684631 , -0.61918456, -0.65582878, -0.61158072, -0.57156346};

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
