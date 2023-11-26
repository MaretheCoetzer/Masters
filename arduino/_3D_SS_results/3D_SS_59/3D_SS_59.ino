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
float SS_servo0[] = {-0.5948556 , -0.59017429, -0.53317752, -0.47422899, -0.41302065, -0.34350571, -0.26351814, -0.18271588, -0.10120136, -0.02000773,  0.06134192,  0.07025642,  0.04030064,  0.01298495, -0.01411558, -0.04952016, -0.08760607, -0.12916544, -0.17023584, -0.20874081, -0.23398875, -0.24952935, -0.25418759, -0.25834998, -0.26025741, -0.27047895, -0.29373155, -0.32714669, -0.36594   , -0.40105077, -0.44141201, -0.46079913, -0.47618724, -0.49716277, -0.51710065, -0.53654606, -0.53720819, -0.53678876, -0.53476967, -0.53101884, -0.5334911 , -0.56666903};
float SS_servo1[] = {0.73282881, 0.76768186, 0.79039762, 0.81287268, 0.83245194, 0.84488707, 0.84561875, 0.84569562, 0.84422987, 0.84341385, 0.83149442, 0.89004647, 0.98406983, 1.05507364, 1.10020643, 1.13168182, 1.12086461, 1.0980405 , 1.06875421, 1.04876524, 1.00012055, 0.93849884, 0.86566974, 0.79157921, 0.72049683, 0.68658272, 0.70237779, 0.75398271, 0.81392627, 0.85690335, 0.91458872, 0.94004982, 0.95186703, 0.98061628, 1.00776944, 1.01849273, 0.94734712, 0.87099025, 0.79225813, 0.71153303, 0.65377651, 0.68831764};
float SS_servo2[] = {-0.29461463, -0.30125079, -0.30200184, -0.30218861, -0.3005299 , -0.31220445, -0.35573006, -0.40308801, -0.45350996, -0.50065868, -0.543745  , -0.56643693, -0.57869151, -0.58928113, -0.60080291, -0.61554326, -0.62865418, -0.63933155, -0.65391762, -0.65703319, -0.65887457, -0.63700443, -0.57179645, -0.50963594, -0.44606528, -0.36878573, -0.28848517, -0.20799998, -0.12734672, -0.04671084,  0.04395705,  0.03673737, -0.01143297, -0.05422606, -0.09557108, -0.13170328, -0.15541003, -0.1757955 , -0.19541823, -0.21405799, -0.23584202, -0.27713494};
float SS_servo3[] = {0.80956159, 0.75411438, 0.66752401, 0.58425066, 0.50523114, 0.47827119, 0.54854968, 0.64000252, 0.74364889, 0.85008276, 0.96379344, 1.03715907, 1.07295431, 1.08958118, 1.09065411, 1.07600623, 1.02818898, 0.95961488, 0.89392447, 0.82009352, 0.77846958, 0.82588299, 0.84107959, 0.85944722, 0.87643704, 0.8665893 , 0.80271847, 0.72287609, 0.64089648, 0.570631  , 0.54987858, 0.61501103, 0.70375544, 0.78775467, 0.87071377, 0.92151035, 0.89030681, 0.85060869, 0.80891291, 0.76528593, 0.74162087, 0.7991184 };
float SS_servo4[] = { 0.55216599,  0.5635771 ,  0.55969233,  0.55691694,  0.55437137,  0.54825101,  0.53486834,  0.52632545,  0.51870057,  0.51263748,  0.5010449 ,  0.47639337,  0.44592105,  0.41998453,  0.4032846 ,  0.39692319,  0.39574581,  0.38930887,  0.37349951,  0.34999539,  0.31008724,  0.26263544,  0.20931876,  0.1610486 ,  0.1107289 ,  0.0659152 ,  0.01982907, -0.0230109 , -0.07038351, -0.12290134, -0.13875892, -0.06453194,  0.01638525,  0.0973985 ,  0.17856131,  0.25567782,  0.31979508,  0.38679979,  0.45907037,  0.52436434,  0.54278888,  0.54355567};
float SS_servo5[] = {-0.75711073, -0.8379945 , -0.91038593, -0.98777252, -1.06107528, -1.10331855, -1.09987512, -1.08993838, -1.07205619, -1.04790775, -0.99540397, -0.91732706, -0.84355464, -0.7925081 , -0.77730975, -0.80607623, -0.87987344, -0.96359593, -1.039625  , -1.08719986, -1.09719068, -1.08531963, -1.04932171, -1.02973763, -0.99896128, -0.96023977, -0.88963346, -0.80819978, -0.72007392, -0.63171506, -0.59891195, -0.66311171, -0.70816982, -0.72533484, -0.77174028, -0.79887826, -0.78328588, -0.76563893, -0.75439317, -0.73990319, -0.74994221, -0.74659145};
float SS_servo6[] = {0.37528133, 0.37386736, 0.35948747, 0.34699176, 0.33079133, 0.30607286, 0.27908166, 0.25433518, 0.22410797, 0.19173021, 0.20023472, 0.27463289, 0.3554928 , 0.43611337, 0.51660201, 0.59674115, 0.65839737, 0.66269323, 0.64924296, 0.64693366, 0.64409737, 0.64321848, 0.64956187, 0.65689662, 0.66287959, 0.66062789, 0.64912785, 0.63497182, 0.62166016, 0.62136814, 0.61403509, 0.56660897, 0.5163211 , 0.46525444, 0.41433609, 0.3784055 , 0.38641269, 0.40004878, 0.40994762, 0.41578057, 0.41344396, 0.38681479};
float SS_servo7[] = {-0.83029254, -0.89500339, -0.95305814, -1.01826844, -1.07244647, -1.08263783, -1.04477833, -0.99281508, -0.91941119, -0.83034664, -0.77319212, -0.77230193, -0.77280455, -0.77265966, -0.77255347, -0.77149477, -0.75265266, -0.67557184, -0.60090098, -0.56730249, -0.47586907, -0.47781246, -0.55632218, -0.63916201, -0.71519272, -0.76216284, -0.77421792, -0.7685825 , -0.76580983, -0.79392518, -0.80565308, -0.73011909, -0.65062827, -0.56387045, -0.47567534, -0.42867093, -0.5113656 , -0.60893562, -0.70142431, -0.78836204, -0.85123157, -0.82693118};

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
