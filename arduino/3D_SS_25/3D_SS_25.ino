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
float SS_servo0[] = {-0.47737933, -0.46070658, -0.40111336, -0.32413701, -0.24255309, -0.15584724, -0.08786458, -0.06078887, -0.03758777, -0.03299934, -0.04535131, -0.06715805, -0.10752283, -0.1369628 , -0.15981795, -0.17924435, -0.19665187, -0.21256325, -0.2260861 , -0.23598882, -0.24725058, -0.25872762, -0.27004536, -0.28043387, -0.28871429, -0.29642409, -0.30654751, -0.3198864 , -0.3378717 , -0.35878773, -0.382865  , -0.40632697, -0.42804867, -0.4437793 , -0.45571094, -0.4647308 , -0.47483907, -0.48309492, -0.48160812, -0.47783334};
float SS_servo1[] = {0.60289815, 0.61435019, 0.67995485, 0.75409439, 0.82378154, 0.85472749, 0.81244256, 0.74177964, 0.67260504, 0.63777736, 0.64816374, 0.67037727, 0.73410568, 0.78300244, 0.81858779, 0.84155979, 0.85033494, 0.83927338, 0.81024156, 0.77581959, 0.7468546 , 0.71937645, 0.69234066, 0.66514639, 0.63751286, 0.6188328 , 0.61232727, 0.61746762, 0.63456263, 0.66031544, 0.68978498, 0.71154997, 0.72988222, 0.73774242, 0.73797012, 0.73215078, 0.72718663, 0.70954522, 0.66357651, 0.61513435};
float SS_servo2[] = {-0.27376386, -0.2822769 , -0.29061965, -0.29742322, -0.30960595, -0.32380178, -0.33767665, -0.35312695, -0.37059679, -0.38925498, -0.41338296, -0.43427011, -0.45235853, -0.46348093, -0.46804722, -0.46947968, -0.47086601, -0.47064665, -0.46195774, -0.4523412 , -0.45455483, -0.47245933, -0.48313509, -0.50528092, -0.50431206, -0.46160658, -0.39574603, -0.31473976, -0.22600037, -0.1428139 , -0.08971023, -0.08558832, -0.12031695, -0.15535576, -0.18039829, -0.19986983, -0.21692051, -0.23604721, -0.25451075, -0.26805031};
float SS_servo3[] = {0.66204283, 0.65501902, 0.64710299, 0.64279011, 0.65239178, 0.66609034, 0.67144745, 0.67029042, 0.67187071, 0.67826974, 0.70312878, 0.72099245, 0.73569133, 0.74489969, 0.7427668 , 0.73114255, 0.71139657, 0.67293268, 0.60645745, 0.54031852, 0.49886558, 0.48982633, 0.47060267, 0.47492057, 0.49675967, 0.56030781, 0.62153176, 0.65936558, 0.65699341, 0.6231087 , 0.56653862, 0.53953375, 0.58761466, 0.63617054, 0.66307129, 0.67692284, 0.68360701, 0.68701597, 0.68065078, 0.66522382};
float SS_servo4[] = {0.49133566, 0.49229898, 0.49292821, 0.4904336 , 0.49164128, 0.4890634 , 0.48180434, 0.47075783, 0.45129603, 0.43113138, 0.4102438 , 0.3914212 , 0.37251804, 0.35568364, 0.34207632, 0.33426526, 0.32832379, 0.31645255, 0.300983  , 0.28510711, 0.26976713, 0.25196699, 0.23077687, 0.20621774, 0.18154522, 0.15768259, 0.13368628, 0.10644516, 0.07146434, 0.05264758, 0.07335569, 0.11536984, 0.17938059, 0.25278449, 0.32807308, 0.4018665 , 0.46939739, 0.50380558, 0.50359703, 0.49114173};
float SS_servo5[] = {-0.71049099, -0.73714592, -0.76201083, -0.7737321 , -0.78873106, -0.79629901, -0.80387957, -0.81451498, -0.80944862, -0.80117364, -0.7836595 , -0.77003914, -0.75356003, -0.73283923, -0.7173732 , -0.71735142, -0.72964637, -0.7455307 , -0.76773919, -0.78890201, -0.80942286, -0.82427108, -0.83256711, -0.83303716, -0.82852745, -0.81639322, -0.79633249, -0.76318885, -0.70954961, -0.68483002, -0.74510278, -0.83468474, -0.89967882, -0.91079161, -0.88362795, -0.82236431, -0.73002448, -0.65672122, -0.68755913, -0.69776319};
float SS_servo6[] = {0.21219084, 0.20708227, 0.19933402, 0.1828643 , 0.16753576, 0.14957917, 0.12728614, 0.09908548, 0.06128879, 0.03914784, 0.06163998, 0.09722967, 0.14918165, 0.21891831, 0.29534019, 0.37446262, 0.43634807, 0.4585278 , 0.45222578, 0.44712747, 0.45387911, 0.46062842, 0.46445163, 0.46649509, 0.46424673, 0.45480988, 0.44323435, 0.42872051, 0.40869511, 0.38502895, 0.36265707, 0.3415945 , 0.32140038, 0.30216317, 0.28397148, 0.27015039, 0.25818033, 0.24677796, 0.2338032 , 0.21575511};
float SS_servo7[] = {-0.70847684, -0.72300638, -0.73254915, -0.71691048, -0.69976936, -0.67687724, -0.65368018, -0.627883  , -0.58296106, -0.56682051, -0.63140951, -0.71590848, -0.78013029, -0.76615889, -0.70953948, -0.63958518, -0.54398262, -0.47386072, -0.50050961, -0.53650421, -0.59443031, -0.65296396, -0.70654776, -0.75752582, -0.79582509, -0.81009037, -0.81314017, -0.80548157, -0.78447497, -0.75362475, -0.72787007, -0.71013697, -0.6936494 , -0.67786412, -0.66452174, -0.66092508, -0.66319535, -0.67593111, -0.69415828, -0.70042132};

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
