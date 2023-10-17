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
float SS_servo0[] = {-0.51403284, -0.52109395, -0.49135339, -0.3784791 , -0.32054708, -0.24529982, -0.17605555, -0.10384406, -0.03107378,  0.01423362,  0.10144367,  0.24884026,  0.36082997,  0.4023068 ,  0.3917102 ,  0.37337033,  0.37423218,  0.38785955,  0.39926885,  0.40636067,  0.40804864,  0.40295536,  0.3928483 ,  0.3818159 ,  0.36479382,  0.34219159,  0.31434711,  0.27791058,  0.23437476,  0.18069455,  0.11479434,  0.05877286, -0.01515475, -0.08302613, -0.13608573, -0.19141451, -0.24656122, -0.29796563, -0.34572485, -0.38826766, -0.43003184, -0.4680139 , -0.50292614};
float SS_servo1[] = {1.88648513, 1.92367849, 1.91086365, 1.85262717, 1.76848249, 1.68913259, 1.60674185, 1.52420349, 1.45006456, 1.36207774, 1.28358283, 1.22356191, 1.1422649 , 1.05663163, 1.09201575, 1.1252213 , 1.10709161, 1.04970021, 0.99885115, 0.95593196, 0.92051852, 0.89932462, 0.88067831, 0.85503813, 0.83584117, 0.82624526, 0.82484199, 0.83583761, 0.85975621, 0.89328236, 0.96238317, 1.00457991, 1.07576287, 1.16177701, 1.24438325, 1.3133196 , 1.39220985, 1.4756631 , 1.55658127, 1.63013508, 1.71242108, 1.79103883, 1.86401367};
float SS_servo2[] = {-0.19162266, -0.18912133, -0.20490811, -0.23109016, -0.26293276, -0.29248688, -0.32842526, -0.36419168, -0.39661937, -0.44589345, -0.48717944, -0.51368432, -0.52537767, -0.52212145, -0.52111925, -0.51191084, -0.4970702 , -0.47137754, -0.44746496, -0.42921666, -0.41576697, -0.4049999 , -0.39575134, -0.34061009, -0.27367941, -0.20906467, -0.14504115, -0.08435355, -0.02574706,  0.0310141 ,  0.0859144 ,  0.16709052,  0.22689771,  0.23390866,  0.19093511,  0.11424024,  0.04168111, -0.00908063, -0.04902683, -0.08446264, -0.11577762, -0.14533183, -0.17776447};
float SS_servo3[] = {1.67749986, 1.6889902 , 1.68567517, 1.66790513, 1.62653305, 1.60505396, 1.6007658 , 1.59160863, 1.56972979, 1.57726469, 1.58942165, 1.60172469, 1.6118518 , 1.61851296, 1.61364489, 1.58267732, 1.55644394, 1.49932338, 1.44254012, 1.39264255, 1.34423087, 1.29318118, 1.24926474, 1.20821423, 1.16007274, 1.11494424, 1.07148558, 1.0258855 , 0.99919014, 0.95607538, 0.94526022, 0.89850473, 0.83491518, 0.82925994, 0.89701551, 1.02595894, 1.15914654, 1.25941748, 1.34256487, 1.42643801, 1.51118765, 1.59384551, 1.65817198};
float SS_servo4[] = { 0.52578354,  0.52620855,  0.51005375,  0.48302049,  0.43439176,  0.38135379,  0.33091982,  0.27512553,  0.22290052,  0.1625668 ,  0.11321995,  0.08200069,  0.06444366,  0.05938232,  0.04715397,  0.02879447,  0.01862322,  0.01294823,  0.00514261, -0.00662032, -0.02208745, -0.03773771, -0.05823966, -0.07635971, -0.10001275, -0.12362634, -0.14630696, -0.18673177, -0.22420783, -0.28324952, -0.34048633, -0.4071663 , -0.45295372, -0.40576927, -0.27436979, -0.1732933 , -0.07329459,  0.02433018,  0.11845304,  0.21294772,  0.30610303,  0.39833819,  0.48498896};
float SS_servo5[] = {-1.84314488, -1.84433621, -1.83320997, -1.80991869, -1.78737383, -1.72910118, -1.70771161, -1.66627193, -1.64059319, -1.62795527, -1.59244464, -1.58810104, -1.54825994, -1.48306489, -1.41436288, -1.35026612, -1.29252131, -1.24413579, -1.20348427, -1.16400966, -1.12788156, -1.10470996, -1.08011978, -1.0750856 , -1.06703988, -1.07144234, -1.09303136, -1.06944021, -1.07858932, -1.05277613, -1.02532467, -0.95288857, -1.00681722, -1.09835407, -1.16342086, -1.24542124, -1.3251467 , -1.40382402, -1.4839648 , -1.56395212, -1.64299137, -1.72607782, -1.80188892};
float SS_servo6[] = {-0.03760955, -0.04095994, -0.07289421, -0.10845726, -0.15625963, -0.20349343, -0.25736858, -0.31440123, -0.37375836, -0.44341658, -0.49757224, -0.51962358, -0.45302221, -0.30113722, -0.18961185, -0.08433354,  0.02124354,  0.1016588 ,  0.17836152,  0.2548599 ,  0.33035129,  0.40019815,  0.47154583,  0.490558  ,  0.48806925,  0.486966  ,  0.48838343,  0.48769507,  0.48280299,  0.46876823,  0.42572597,  0.3929766 ,  0.33649521,  0.27896094,  0.2227915 ,  0.16654596,  0.12333705,  0.08977936,  0.06132867,  0.0393092 ,  0.02208702,  0.00227087, -0.02517855};
float SS_servo7[] = {-1.40243917, -1.38329164, -1.31867993, -1.25002633, -1.18917368, -1.11937713, -1.04162536, -0.96401775, -0.8872126 , -0.81173491, -0.76090152, -0.74574301, -0.76436815, -0.80729975, -0.86198576, -0.93091855, -1.00552375, -1.0427976 , -1.05982144, -1.06850075, -1.07806255, -1.06845968, -1.08985808, -1.12543199, -1.16978459, -1.22481579, -1.29362599, -1.37015247, -1.44910746, -1.53023225, -1.55139984, -1.60216314, -1.63814756, -1.63242974, -1.58436195, -1.52233993, -1.49980501, -1.46876996, -1.43611475, -1.41980102, -1.41459308, -1.40121372, -1.43313114};

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
