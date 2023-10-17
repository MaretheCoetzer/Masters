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
float SS_servo0[] = {-0.49551003, -0.48652238, -0.45355282, -0.38459677, -0.29538039, -0.19404846, -0.08184322,  0.01726587,  0.09217173,  0.14122255,  0.18945297,  0.20452815,  0.16671339,  0.13620267,  0.12004552,  0.1077694 ,  0.09898012,  0.09060476,  0.07999408,  0.06527331,  0.05058327,  0.03493932,  0.01440852, -0.00893182, -0.0350005 , -0.06496117, -0.0993422 , -0.1397795 , -0.18703461, -0.2351891 , -0.28989032, -0.3500875 , -0.40181363, -0.44905088, -0.49260029, -0.51959636, -0.51517154, -0.51351914, -0.5054206 , -0.49834691};
float SS_servo1[] = {0.66693019, 0.63206588, 0.61563271, 0.67915753, 0.75580323, 0.80096982, 0.76715451, 0.70465904, 0.63884956, 0.56207013, 0.48833954, 0.47019211, 0.54701405, 0.60674391, 0.63953871, 0.66247143, 0.67611033, 0.68204948, 0.68210713, 0.67304647, 0.64689084, 0.62383279, 0.62088827, 0.62471015, 0.63229279, 0.64382117, 0.65868598, 0.68230768, 0.714742  , 0.74397419, 0.79398875, 0.86351897, 0.9066548 , 0.93339934, 0.94654106, 0.93219549, 0.86648159, 0.79751641, 0.73272012, 0.68267177};
float SS_servo2[] = {-0.33101257, -0.3309655 , -0.33845035, -0.34119793, -0.34551818, -0.35211157, -0.36074455, -0.37281156, -0.3893875 , -0.40672998, -0.42403819, -0.44155455, -0.45689993, -0.46336525, -0.46402924, -0.45925551, -0.45428806, -0.44821552, -0.44971639, -0.46119465, -0.46519506, -0.45039674, -0.40199144, -0.34106701, -0.27738567, -0.21003295, -0.14480507, -0.08905835, -0.056323  , -0.05290548, -0.07205977, -0.13037785, -0.20287816, -0.27485125, -0.33710827, -0.37056924, -0.36705508, -0.36400998, -0.35129929, -0.33507782};
float SS_servo3[] = {0.5941954 , 0.57763182, 0.57737968, 0.58595783, 0.60949406, 0.64375162, 0.68288906, 0.72669345, 0.77472302, 0.81751172, 0.86395672, 0.90640534, 0.93037396, 0.93515711, 0.93208228, 0.91651795, 0.89921807, 0.87428115, 0.85554934, 0.84123486, 0.79750582, 0.7799349 , 0.82324569, 0.86460979, 0.89281816, 0.88233892, 0.82803685, 0.76481957, 0.70124525, 0.63116961, 0.61679469, 0.68701031, 0.77928144, 0.8637073 , 0.91745556, 0.91043626, 0.83865986, 0.76387735, 0.68625744, 0.61330944};
float SS_servo4[] = {0.43145208, 0.41573142, 0.38241781, 0.36780543, 0.36756035, 0.36882795, 0.36860576, 0.36541282, 0.35419117, 0.33901129, 0.32497411, 0.31036426, 0.28960322, 0.27073816, 0.25508467, 0.24161957, 0.23271514, 0.22775345, 0.22425805, 0.21950081, 0.2098449 , 0.1994951 , 0.19278112, 0.18717083, 0.17812068, 0.16345141, 0.1409011 , 0.10901199, 0.06595109, 0.02306514, 0.04788199, 0.13073378, 0.21171804, 0.29141115, 0.36817212, 0.45432726, 0.52218895, 0.48145024, 0.45221774, 0.43449131};
float SS_servo5[] = {-1.61018076, -1.59828972, -1.53577917, -1.49021776, -1.46075549, -1.42555125, -1.38695959, -1.34653494, -1.29618115, -1.24740954, -1.19730924, -1.1499813 , -1.10702382, -1.06989203, -1.03477021, -1.00540955, -0.98719116, -0.98419358, -0.99527662, -1.02332889, -1.06057509, -1.0958085 , -1.13238414, -1.17490874, -1.21477162, -1.24959434, -1.27621209, -1.28748753, -1.27896919, -1.27546751, -1.36882137, -1.44948127, -1.531714  , -1.61003248, -1.70207971, -1.74755775, -1.65942365, -1.64016511, -1.62604272, -1.60937818};
float SS_servo6[] = { 0.02673304,  0.00840871, -0.01635853, -0.02914124, -0.03752472, -0.04452065, -0.05402366, -0.06861687, -0.09089418, -0.11476113, -0.10947383, -0.06706815, -0.02328279,  0.03908222,  0.11908296,  0.19341691,  0.27355723,  0.35376936,  0.43234602,  0.48478433,  0.49280277,  0.49520475,  0.50052988,  0.50651752,  0.51169442,  0.51367237,  0.50958249,  0.49673651,  0.47037299,  0.42975981,  0.38775878,  0.35148479,  0.31486791,  0.27896788,  0.24034432,  0.20065289,  0.17466817,  0.12035538,  0.07612499,  0.03615865};
float SS_servo7[] = {-1.38142902, -1.36050326, -1.31043142, -1.26307411, -1.20727025, -1.1474175 , -1.08244603, -1.01038794, -0.92866292, -0.85447974, -0.84244637, -0.91439792, -0.99778921, -1.0778017 , -1.10831306, -1.0731013 , -1.01229339, -0.95427126, -0.92023965, -0.93441379, -0.99888956, -1.05895378, -1.11613455, -1.17604805, -1.23861008, -1.30162553, -1.36222595, -1.41338124, -1.44860902, -1.46371104, -1.4589681 , -1.45579765, -1.45751151, -1.47119018, -1.48742357, -1.49406253, -1.51912907, -1.48122758, -1.44292655, -1.38994438};

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
