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
float SS_servo0[] = {-0.4847066 , -0.46908796, -0.41080674, -0.33454255, -0.25353817, -0.1652717 , -0.09295734, -0.0637404 , -0.03941956, -0.03655396, -0.06634028, -0.10532634, -0.13556056, -0.1597224 , -0.17999584, -0.19774379, -0.21418144, -0.23021559, -0.24493567, -0.25778551, -0.27083477, -0.28282778, -0.29311661, -0.30140083, -0.3112352 , -0.32309448, -0.33836689, -0.35401764, -0.37148914, -0.39059542, -0.41390135, -0.43639974, -0.45509113, -0.46791587, -0.4765709 , -0.4836675 , -0.48948246, -0.49532511, -0.49042391, -0.48476429};
float SS_servo1[] = {0.57805644, 0.58974466, 0.65530448, 0.72959514, 0.80181585, 0.8355577 , 0.79209095, 0.7181823 , 0.6496056 , 0.62019237, 0.66162421, 0.72808469, 0.7841307 , 0.82762939, 0.86060203, 0.88134375, 0.88893144, 0.87778787, 0.84820901, 0.81489122, 0.78592465, 0.75783216, 0.73057496, 0.71187115, 0.70260784, 0.70078137, 0.70550708, 0.70645969, 0.70667424, 0.71614128, 0.73771104, 0.75720413, 0.76997057, 0.76763915, 0.75512918, 0.74048279, 0.72586046, 0.70164804, 0.64516136, 0.5903652 };
float SS_servo2[] = {-0.22469015, -0.23426905, -0.24438604, -0.25341269, -0.26777719, -0.28452882, -0.30165152, -0.32061504, -0.34212478, -0.36549351, -0.39051898, -0.40943044, -0.42407455, -0.431618  , -0.43795732, -0.44051943, -0.44354486, -0.44785966, -0.44623944, -0.44926687, -0.45746451, -0.46904285, -0.45528637, -0.39658433, -0.32497162, -0.24544954, -0.15852972, -0.09778142, -0.06971927, -0.04058499, -0.0194574 , -0.02978258, -0.07145923, -0.10578576, -0.13155722, -0.15281526, -0.16847336, -0.18650969, -0.20529349, -0.21860033};
float SS_servo3[] = {0.62060117, 0.61471497, 0.60961061, 0.61018039, 0.62603284, 0.64826162, 0.66456703, 0.67433771, 0.68810996, 0.7068279 , 0.73436248, 0.75581987, 0.77637506, 0.78337914, 0.78660927, 0.77790093, 0.7618272 , 0.7315043 , 0.67521268, 0.62782281, 0.59464173, 0.57090436, 0.57640978, 0.64492079, 0.72415621, 0.7778696 , 0.76601082, 0.71523688, 0.63984887, 0.56792439, 0.5096135 , 0.5097842 , 0.57231242, 0.6154626 , 0.63833497, 0.65104872, 0.65299706, 0.65213696, 0.64235074, 0.62384989};
float SS_servo4[] = {0.47982471, 0.47886358, 0.47737185, 0.47562379, 0.47702372, 0.47541018, 0.46681946, 0.45321333, 0.43080578, 0.40609531, 0.38174187, 0.3586352 , 0.33773991, 0.31886464, 0.30374665, 0.29317043, 0.2834419 , 0.26791053, 0.2469524 , 0.22622285, 0.21096197, 0.19424066, 0.17690299, 0.1627326 , 0.14863935, 0.13267678, 0.11185862, 0.08119117, 0.03555268, 0.01326617, 0.04234134, 0.09758136, 0.17835418, 0.25945377, 0.33286436, 0.40122323, 0.47167826, 0.51009861, 0.50210701, 0.48010179};
float SS_servo5[] = {-0.82994454, -0.85423502, -0.87660359, -0.8898424 , -0.9042464 , -0.91094803, -0.91217921, -0.91454271, -0.89974812, -0.87956074, -0.85512485, -0.82568854, -0.79205612, -0.76121303, -0.73917074, -0.73156166, -0.73473672, -0.74268677, -0.75589697, -0.77181089, -0.79716928, -0.81720566, -0.82998086, -0.83824646, -0.8408703 , -0.83595674, -0.82031252, -0.78737094, -0.7265806 , -0.70774598, -0.78880148, -0.87832784, -0.94813824, -1.00349807, -1.01880908, -0.97114759, -0.87700148, -0.80450025, -0.82463833, -0.81755462};
float SS_servo6[] = {0.20769814, 0.20227016, 0.19525384, 0.18062048, 0.16778255, 0.15115379, 0.12948664, 0.09935606, 0.05985453, 0.03918332, 0.06669827, 0.12182181, 0.20147835, 0.27790599, 0.3570744 , 0.43470621, 0.49311194, 0.50984759, 0.48519463, 0.4599128 , 0.46373272, 0.47210349, 0.47566575, 0.47407465, 0.4706651 , 0.46644839, 0.45738174, 0.44378531, 0.42316653, 0.39619505, 0.37153959, 0.34566386, 0.32368497, 0.30470625, 0.28882388, 0.27123314, 0.25817952, 0.24633798, 0.23017541, 0.21157647};
float SS_servo7[] = {-0.77861419, -0.79451526, -0.80762817, -0.79601705, -0.78277666, -0.75953107, -0.73325413, -0.6994402 , -0.6460652 , -0.62988723, -0.70679829, -0.79643996, -0.85271571, -0.85627788, -0.82757936, -0.77606796, -0.68342092, -0.60666758, -0.59866387, -0.59876942, -0.6546605 , -0.71973121, -0.77052574, -0.79952731, -0.81987189, -0.83580233, -0.84306464, -0.84769013, -0.84408535, -0.82082866, -0.79640236, -0.76863464, -0.74852465, -0.73735687, -0.73415323, -0.72756748, -0.72948524, -0.74391783, -0.76096578, -0.76989583};

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
