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
float SS_servo0[] = {-0.66600998, -0.65472467, -0.5990525 , -0.54006364, -0.47998522, -0.41152958, -0.33161588, -0.25103701, -0.16968865, -0.08839001, -0.00704839,  0.018258  , -0.00793451, -0.0297113 , -0.0514481 , -0.08253142, -0.12162894, -0.16227454, -0.20469615, -0.24083666, -0.26933973, -0.29501336, -0.30910301, -0.31747848, -0.32867964, -0.33781176, -0.35435689, -0.38196496, -0.41267577, -0.44839985, -0.48204096, -0.51955304, -0.54602617, -0.56724104, -0.58900708, -0.61683369, -0.64061283, -0.65016049, -0.65447534, -0.65804146, -0.65940357, -0.65761861};
float SS_servo1[] = {0.83768298, 0.851122  , 0.87327761, 0.89571408, 0.916365  , 0.9298365 , 0.93057988, 0.93064586, 0.93044364, 0.9295273 , 0.92804361, 0.98839423, 1.09681771, 1.17729051, 1.23238712, 1.26976036, 1.25114133, 1.22258651, 1.19206155, 1.16284818, 1.13451706, 1.09439231, 1.03271002, 0.96703456, 0.91164903, 0.86286854, 0.84990937, 0.87966727, 0.92473437, 0.97103881, 1.00206873, 1.05062501, 1.09388072, 1.12130553, 1.14656312, 1.18442106, 1.19908146, 1.14802959, 1.07368714, 0.99833975, 0.92058252, 0.84931054};
float SS_servo2[] = {-0.26009341, -0.26473315, -0.27672004, -0.2815508 , -0.28505353, -0.29540315, -0.32976576, -0.37001888, -0.41308653, -0.4564922 , -0.49906255, -0.52623547, -0.54201452, -0.55688711, -0.57172231, -0.59178781, -0.60846612, -0.62625932, -0.6469716 , -0.65654815, -0.6599725 , -0.65177384, -0.58844169, -0.51919736, -0.45039954, -0.37804083, -0.29800182, -0.21750568, -0.13679742, -0.05610253,  0.02441583,  0.08903143,  0.05899274,  0.01111963, -0.03043646, -0.06948729, -0.10798792, -0.14279539, -0.17124142, -0.19454742, -0.21618039, -0.24503996};
float SS_servo3[] = {0.89765311, 0.83909736, 0.76261479, 0.67893663, 0.6040704 , 0.57058276, 0.62298351, 0.70361544, 0.79767765, 0.89655799, 1.01193995, 1.11296191, 1.17264652, 1.21303842, 1.23448494, 1.23754933, 1.18680914, 1.12800363, 1.06935016, 1.00407803, 0.94047416, 0.91777259, 0.93170622, 0.94346933, 0.95521976, 0.96345007, 0.95255512, 0.91153066, 0.86315887, 0.8125703 , 0.74208979, 0.68043438, 0.74525266, 0.84515112, 0.93125516, 1.01460901, 1.0732402 , 1.06571165, 1.02681505, 0.97851744, 0.92822797, 0.91006878};
float SS_servo4[] = { 0.63362032,  0.63809644,  0.6319874 ,  0.62640508,  0.62091649,  0.61194041,  0.59527119,  0.58168982,  0.57233383,  0.56329061,  0.55140817,  0.53298259,  0.50728738,  0.48553494,  0.47055589,  0.47630214,  0.48203944,  0.4850433 ,  0.47878591,  0.45527658,  0.4176328 ,  0.37321773,  0.32767336,  0.27696679,  0.23167588,  0.18653413,  0.14445328,  0.10317788,  0.06311366,  0.0227662 , -0.02061617, -0.01139964,  0.06428234,  0.1456289 ,  0.22681533,  0.30784554,  0.38730265,  0.45724737,  0.51664364,  0.58325655,  0.63715373,  0.62749356};
float SS_servo5[] = {-0.78742072, -0.85416909, -0.92750895, -1.00064571, -1.07046246, -1.10969824, -1.10097298, -1.08110273, -1.05818818, -1.03059084, -0.98079339, -0.90145875, -0.82582701, -0.77139086, -0.74631321, -0.78804946, -0.87717619, -0.97409866, -1.06425649, -1.10940516, -1.11012894, -1.10094505, -1.08828518, -1.05866216, -1.04068627, -1.01261479, -0.96967212, -0.90484197, -0.83187567, -0.76884099, -0.71034741, -0.73213314, -0.79082304, -0.79979797, -0.81160836, -0.83191762, -0.85794947, -0.84884971, -0.82483112, -0.80700973, -0.79126511, -0.77996652};
float SS_servo6[] = {0.42941655, 0.41749256, 0.39631336, 0.37674842, 0.35598483, 0.32660479, 0.29548355, 0.26505426, 0.23174509, 0.19602296, 0.1968403 , 0.26776888, 0.34832698, 0.42903552, 0.5095483 , 0.58691053, 0.64135484, 0.66230723, 0.63610207, 0.64529297, 0.64408708, 0.6262141 , 0.63495524, 0.64542406, 0.65585969, 0.66294733, 0.66009464, 0.65073178, 0.63944027, 0.63601306, 0.64565893, 0.63531996, 0.58959817, 0.54121095, 0.49120986, 0.44509257, 0.42047347, 0.42857833, 0.43912159, 0.45036854, 0.45227346, 0.44288034};
float SS_servo7[] = {-0.914885  , -0.95636572, -1.00714429, -1.0609799 , -1.11028459, -1.11364847, -1.06697355, -1.00052891, -0.91721395, -0.8243611 , -0.76285509, -0.75762363, -0.75807484, -0.75790411, -0.75773159, -0.75372324, -0.72636844, -0.66554673, -0.59071895, -0.5651276 , -0.50276877, -0.40248392, -0.45020681, -0.53659002, -0.62182553, -0.694899  , -0.7343886 , -0.74544346, -0.74421909, -0.76411424, -0.81630627, -0.82472806, -0.74980659, -0.66896529, -0.58495996, -0.50532069, -0.48074132, -0.55877242, -0.65835905, -0.76033235, -0.84680738, -0.9005949 };

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
