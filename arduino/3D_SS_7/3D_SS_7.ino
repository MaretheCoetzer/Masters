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
float SS_servo0[] = {-0.52567634, -0.52318542, -0.52118645, -0.5130689 , -0.48602837, -0.42520421, -0.35834175, -0.29147928, -0.18705867, -0.06957381,  0.05813786,  0.18681635,  0.28627281,  0.37848616,  0.45189792,  0.4575764 ,  0.41161248,  0.31153327,  0.23982557,  0.21604706,  0.19866511,  0.18373691,  0.1706335 ,  0.16210284,  0.12295692,  0.07941513,  0.04658718,  0.01212309, -0.02234101, -0.04594516, -0.06442933, -0.08172267, -0.11000428, -0.1318995 , -0.15557955, -0.17717617, -0.19689186, -0.21387561, -0.22621521, -0.23928384, -0.24768136, -0.25588541, -0.27166182, -0.3000008 , -0.33659183, -0.36156674, -0.38828693, -0.40795069, -0.42538324, -0.44183293, -0.45828262, -0.4747323 , -0.49148948, -0.50832052, -0.52245216, -0.51063351, -0.50726583, -0.51099018, -0.51471453, -0.51843887, -0.52216322, -0.52350621, -0.52353712, -0.52356803, -0.52269752};
float SS_servo1[] = {0.15810259, 0.11555433, 0.11962038, 0.12080447, 0.19055371, 0.27088957, 0.35225055, 0.43361154, 0.52933292, 0.63004942, 0.71021588, 0.67075886, 0.59275336, 0.51478086, 0.45806844, 0.4028778 , 0.43324562, 0.57967795, 0.70676128, 0.79102225, 0.8788657 , 0.96708475, 1.05659221, 1.1269249 , 1.19528217, 1.21717064, 1.22025353, 1.2123902 , 1.20452686, 1.17975625, 1.14701459, 1.14241262, 1.20066582, 1.2554936 , 1.32786868, 1.39856559, 1.46981037, 1.54324885, 1.61970108, 1.69407667, 1.74164607, 1.75949309, 1.76638861, 1.77114425, 1.75729183, 1.70032277, 1.63726351, 1.56088138, 1.48885803, 1.42070401, 1.35254999, 1.28439597, 1.22149203, 1.15984914, 1.0939568 , 0.98721287, 0.89492058, 0.81475603, 0.73459148, 0.65442693, 0.57426238, 0.49148731, 0.40727388, 0.32306046, 0.2366286 };
float SS_servo2[] = {-0.35223361, -0.35345738, -0.3529004 , -0.35021747, -0.34161217, -0.34492983, -0.35044289, -0.35595594, -0.3773672 , -0.40430849, -0.43805387, -0.47974192, -0.49274304, -0.50270187, -0.51916399, -0.52438496, -0.52323273, -0.52152868, -0.52526674, -0.52167152, -0.50834662, -0.49700985, -0.48710431, -0.48000162, -0.48940306, -0.52834354, -0.55391032, -0.55876141, -0.5636125 , -0.52314872, -0.46132097, -0.37830246, -0.30305005, -0.22415658, -0.13414197, -0.03533571,  0.06803579,  0.16455308,  0.262377  ,  0.36240908,  0.43697208,  0.45033827,  0.42691876,  0.41506691,  0.36942369,  0.33736479,  0.30681459,  0.28569391,  0.26608384,  0.24721378,  0.22834372,  0.20947366,  0.19027583,  0.17099928,  0.15126656,  0.12714862,  0.0944218 ,  0.05447046,  0.01451913, -0.02543221, -0.06538355, -0.12160944, -0.18680228, -0.25199513, -0.31787271};
float SS_servo3[] = {0.30380733, 0.27498863, 0.22439649, 0.16648347, 0.1165939 , 0.10155795, 0.09240916, 0.08326037, 0.11676951, 0.16511679, 0.22628739, 0.2881097 , 0.2889138 , 0.28210888, 0.28041034, 0.23970121, 0.18903103, 0.12376547, 0.10319599, 0.10802536, 0.10207533, 0.10044873, 0.10091028, 0.10271851, 0.10110586, 0.1292453 , 0.18965677, 0.27884987, 0.36804296, 0.47919321, 0.60069529, 0.67999723, 0.68616448, 0.72626038, 0.75643173, 0.73038188, 0.65851608, 0.58393392, 0.50911129, 0.43791175, 0.38000316, 0.35175109, 0.3817913 , 0.37671371, 0.42837935, 0.43967532, 0.43020909, 0.39518096, 0.35803822, 0.32042755, 0.28281688, 0.2452062 , 0.20580736, 0.16597901, 0.1273157 , 0.09985239, 0.08968275, 0.09402589, 0.09836903, 0.10271217, 0.10705532, 0.14395189, 0.19878477, 0.25361765, 0.30975168};
float SS_servo4[] = { 0.51752251,  0.48454306,  0.42729176,  0.36658841,  0.31737737,  0.28043733,  0.24545381,  0.2104703 ,  0.18717279,  0.16794014,  0.15137039,  0.13357511,  0.10630596,  0.07046239,  0.03201031, -0.01790109, -0.06433813, -0.12240566, -0.1575812 , -0.16450291, -0.16030175, -0.15867929, -0.15653609, -0.15098607, -0.18812022, -0.23700956, -0.27509138, -0.32395511, -0.37281883, -0.4156704 , -0.4556875 , -0.49589443, -0.53202967, -0.56194427, -0.59076777, -0.62053141, -0.65090224, -0.6811832 , -0.70995093, -0.73192447, -0.72081335, -0.69636322, -0.71678672, -0.73186498, -0.68166742, -0.60428492, -0.52957455, -0.45583025, -0.38544484, -0.31744997, -0.24945511, -0.18146025, -0.12031745, -0.0608205 , -0.00122515,  0.05931626,  0.1169741 ,  0.17221206,  0.22745003,  0.28268799,  0.33792595,  0.38770249,  0.4344699 ,  0.48123731,  0.52689091};
float SS_servo5[] = {-1.92587134, -1.92837805, -1.89979095, -1.85619285, -1.8088476 , -1.76322907, -1.71792003, -1.67261098, -1.6285252 , -1.58486492, -1.54684076, -1.53599759, -1.52307451, -1.47798556, -1.45018105, -1.41065422, -1.38053988, -1.34401655, -1.31099301, -1.26763168, -1.22509125, -1.1717037 , -1.12904971, -1.1109714 , -1.04906011, -0.99832809, -0.97309176, -0.93109031, -0.88908886, -0.85494745, -0.8245117 , -0.77460857, -0.70550438, -0.63241173, -0.55853985, -0.48496468, -0.41177932, -0.33838795, -0.26501752, -0.20382343, -0.20412243, -0.24629383, -0.22888677, -0.22899931, -0.32074237, -0.40051002, -0.48163492, -0.56020772, -0.63675732, -0.71194319, -0.78712906, -0.86231493, -0.93887934, -1.01577486, -1.09290318, -1.17226944, -1.25205993, -1.33220643, -1.41235292, -1.49249942, -1.57264592, -1.65246222, -1.73209659, -1.81173096, -1.89042346};
float SS_servo6[] = { 0.46267779,  0.43084663,  0.37445277,  0.31011196,  0.26542973,  0.2226502 ,  0.17960569,  0.13656118,  0.10059545,  0.067092  ,  0.03609274, -0.0018591 , -0.04680895, -0.09343893, -0.14413306, -0.19244204, -0.19286499, -0.17985344, -0.09847101,  0.01215499,  0.10876083,  0.20138743,  0.29476477,  0.39266549,  0.46508177,  0.53523492,  0.59539238,  0.5991618 ,  0.60293122,  0.57538059,  0.53306391,  0.51217525,  0.5228995 ,  0.51052452,  0.500174  ,  0.49359573,  0.48918289,  0.49611013,  0.50649121,  0.52154657,  0.51126592,  0.50046315,  0.52210303,  0.52561048,  0.50970929,  0.49196077,  0.48206497,  0.48607128,  0.49135506,  0.49716152,  0.50296798,  0.50877443,  0.51397194,  0.51902317,  0.52431652,  0.53193748,  0.53767364,  0.54182808,  0.54598252,  0.55013696,  0.5542914 ,  0.54946458,  0.53968927,  0.52991395,  0.51873742};
float SS_servo7[] = {-1.9122172 , -1.90996758, -1.87076687, -1.81126304, -1.75770444, -1.67661571, -1.58979002, -1.50296434, -1.41460519, -1.32571264, -1.24333198, -1.16408427, -1.08337171, -1.00309503, -0.92465018, -0.86416438, -0.85045956, -0.9345406 , -1.00030319, -1.03421945, -1.05819086, -1.0391053 , -0.98017639, -0.91297646, -0.85241173, -0.78444702, -0.72073561, -0.55824885, -0.39576209, -0.25668533, -0.12864535, -0.10024555, -0.13834374, -0.12560147, -0.11119602, -0.10324611, -0.09793425, -0.11102473, -0.12521786, -0.14097854, -0.11218698, -0.09614807, -0.15459643, -0.19264866, -0.20892334, -0.22963042, -0.27947578, -0.35503307, -0.43169184, -0.50923894, -0.58678604, -0.66433314, -0.74655619, -0.8299024 , -0.91439391, -1.00989555, -1.10513631, -1.20015814, -1.29517996, -1.39020179, -1.48522362, -1.58441924, -1.68591453, -1.78740983, -1.88849519};

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
