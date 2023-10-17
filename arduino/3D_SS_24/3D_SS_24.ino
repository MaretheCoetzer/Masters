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
float SS_servo0[] = {-0.52407291, -0.49021802, -0.4011576 , -0.33692541, -0.27169592, -0.20207127, -0.11893662, -0.02689783,  0.06514097,  0.15717977,  0.24921856,  0.35622895,  0.46788593,  0.57954291,  0.68911854,  0.75501782,  0.73823177,  0.66211786,  0.57660867,  0.62914076,  0.68167407,  0.67827292,  0.63911576,  0.58212616,  0.52533786,  0.4651754 ,  0.42879681,  0.39042622,  0.35551432,  0.32696095,  0.29297619,  0.27249286,  0.26324717,  0.25387886,  0.21427577,  0.17777728,  0.14869815,  0.12325108,  0.09628608,  0.06892677,  0.0302168 , -0.00100424, -0.03243623, -0.06481915, -0.09777339, -0.13072763, -0.16368187, -0.19663611, -0.22981186, -0.26298939, -0.29616691, -0.33777755, -0.36693487, -0.39916066, -0.42043133, -0.44090668, -0.45462586, -0.4686307 , -0.48350488, -0.4973921 , -0.50947387, -0.52115225};
float SS_servo1[] = {1.75695259, 1.74715237, 1.67896675, 1.60790689, 1.52720349, 1.44625951, 1.36099697, 1.27288817, 1.18477937, 1.09667056, 1.00856176, 0.92836666, 0.85062765, 0.77288864, 0.69935329, 0.6494597 , 0.57707507, 0.53445383, 0.58108658, 0.52364449, 0.45325931, 0.41364472, 0.45771108, 0.54143241, 0.6208989 , 0.68501169, 0.69433839, 0.72088807, 0.73993082, 0.70680495, 0.67740186, 0.6232537 , 0.55534639, 0.50629714, 0.56258272, 0.64236391, 0.72359494, 0.80638877, 0.89070393, 0.98143173, 1.10621945, 1.21233828, 1.30339696, 1.38764556, 1.46104513, 1.53444471, 1.60784428, 1.68124385, 1.70994193, 1.73828273, 1.76662353, 1.77948146, 1.75686676, 1.73726352, 1.70794322, 1.6935954 , 1.68304406, 1.68320466, 1.69346345, 1.70572899, 1.72301372, 1.74520272};
float SS_servo2[] = { 0.50359521,  0.46617207,  0.39435722,  0.34699861,  0.29184173,  0.23681724,  0.18145257,  0.12586368,  0.07027479,  0.0146859 , -0.04090299, -0.10063619, -0.16165562, -0.22267504, -0.28433114, -0.34649595, -0.42340385, -0.49920536, -0.52456092, -0.52268881, -0.52301145, -0.52328487, -0.5139068 , -0.50950401, -0.51520682, -0.52273525, -0.52648241, -0.52607696, -0.52233878, -0.52036124, -0.52257417, -0.51816415, -0.4804406 , -0.41061825, -0.33490659, -0.24430491, -0.14482027, -0.04068091,  0.06209788,  0.1503599 ,  0.22566475,  0.31518496,  0.41812882,  0.51653624,  0.59497007,  0.67340391,  0.75183774,  0.83027157,  0.86194603,  0.89324677,  0.92454751,  0.89926859,  0.81330341,  0.74891129,  0.69763433,  0.65617866,  0.62738994,  0.60149368,  0.57497856,  0.54977144,  0.52690862,  0.50950705};
float SS_servo3[] = {0.98874006, 1.01002691, 1.05149906, 1.02811322, 1.03507277, 1.06240429, 1.08996599, 1.11767939, 1.1453928 , 1.17310621, 1.20081961, 1.24286816, 1.28936576, 1.33586337, 1.38337074, 1.41188226, 1.43649627, 1.4251553 , 1.35328795, 1.3088182 , 1.28753703, 1.25545049, 1.20156487, 1.15292863, 1.12035517, 1.07996599, 1.02972878, 0.97856145, 0.92582367, 0.856215  , 0.78091549, 0.71517739, 0.70041568, 0.73977526, 0.79237209, 0.8335559 , 0.86087207, 0.84887093, 0.78211824, 0.73807657, 0.78953601, 0.85654074, 0.85462294, 0.82708711, 0.76537716, 0.70366721, 0.64195725, 0.5802473 , 0.49037647, 0.40028058, 0.31018468, 0.3095474 , 0.43474836, 0.51635448, 0.58839682, 0.650195  , 0.6968767 , 0.7457557 , 0.80619   , 0.86792717, 0.93303631, 0.9859996 };
float SS_servo4[] = { 0.53003416,  0.5292782 ,  0.49998452,  0.50117181,  0.50592498,  0.49898722,  0.49734707,  0.49919845,  0.50104984,  0.50290122,  0.5047526 ,  0.50919443,  0.51444024,  0.51968604,  0.52311203,  0.50612226,  0.4666643 ,  0.41827229,  0.36998011,  0.33578764,  0.29060456,  0.24129825,  0.1930062 ,  0.14150976,  0.08926453,  0.03335373, -0.01937308, -0.06000351, -0.08539675, -0.11705006, -0.17055345, -0.20356563, -0.23324656, -0.26804552, -0.30755876, -0.33344457, -0.33810478, -0.32895902, -0.33472193, -0.34031126, -0.35533389, -0.39426033, -0.43907599, -0.4662798 , -0.45333325, -0.44038669, -0.42744013, -0.41449357, -0.34652805, -0.2781228 , -0.20971755, -0.11803822, -0.02479183,  0.04859235,  0.12031256,  0.18644445,  0.24745176,  0.30981136,  0.36840231,  0.42497392,  0.47778096,  0.51847202};
float SS_servo5[] = {-0.11927679, -0.15980609, -0.17817685, -0.25490646, -0.34039755, -0.40209609, -0.47229004, -0.54808314, -0.62387624, -0.69966934, -0.77546244, -0.85978575, -0.9467565 , -1.03372725, -1.11958166, -1.18578774, -1.23384879, -1.28873206, -1.32165341, -1.30158376, -1.23062977, -1.1531542 , -1.07535882, -0.99673408, -0.92313081, -0.85157876, -0.78971181, -0.75130455, -0.74582541, -0.75168481, -0.72438421, -0.74584896, -0.76824167, -0.76876016, -0.72208139, -0.66801015, -0.64563133, -0.65127103, -0.62492369, -0.59864229, -0.55303461, -0.45469196, -0.37166935, -0.31618867, -0.31048505, -0.30478142, -0.2990778 , -0.29337418, -0.34549036, -0.39806866, -0.45064696, -0.51995034, -0.59024354, -0.63058712, -0.66094748, -0.64943407, -0.57011345, -0.48781967, -0.39738046, -0.29761038, -0.17930896, -0.10133399};
float SS_servo6[] = { 0.42949732,  0.40112879,  0.32007818,  0.23784371,  0.15874315,  0.10121347,  0.04980977,  0.00244358, -0.04492261, -0.09228879, -0.13965498, -0.16152544, -0.17548302, -0.18944061, -0.19978506, -0.21791764, -0.26912336, -0.35211805, -0.38870698, -0.36145655, -0.297049  , -0.21307322, -0.12612494, -0.04286818,  0.0408437 ,  0.10821097,  0.16670724,  0.2486894 ,  0.35605953,  0.43743881,  0.49475872,  0.53595318,  0.53596482,  0.50899716,  0.50849834,  0.48948506,  0.48937723,  0.49686018,  0.50138032,  0.50533267,  0.50356377,  0.51551298,  0.53028009,  0.53790601,  0.53280582,  0.52770564,  0.52260545,  0.51750527,  0.48054072,  0.4433215 ,  0.40610229,  0.38735079,  0.38099352,  0.35004408,  0.33438772,  0.32075505,  0.31862745,  0.32906112,  0.35082212,  0.37634799,  0.40986218,  0.43101796};
float SS_servo7[] = {-0.40389916, -0.39132887, -0.30560759, -0.21558358, -0.13526961, -0.08878615, -0.05514237, -0.02996094, -0.00477951,  0.02040193,  0.04558336,  0.01856937, -0.024644  , -0.06785738, -0.1190464 , -0.16800816, -0.18002459, -0.14210145, -0.17324375, -0.26626131, -0.36028821, -0.44310939, -0.52341515, -0.60267724, -0.68796469, -0.72294649, -0.65246586, -0.56668647, -0.47022828, -0.37998136, -0.2955232 , -0.19217989, -0.11789498, -0.10558128, -0.13752812, -0.11104794, -0.108059  , -0.1142605 , -0.11767413, -0.1182346 , -0.1013088 , -0.11794331, -0.1531511 , -0.17650379, -0.18452125, -0.1925387 , -0.20055615, -0.2085736 , -0.17411768, -0.13932231, -0.10452693, -0.10844616, -0.13457035, -0.11516154, -0.11712545, -0.11627055, -0.12896454, -0.16208259, -0.21449456, -0.27250279, -0.34460074, -0.39673752};

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
