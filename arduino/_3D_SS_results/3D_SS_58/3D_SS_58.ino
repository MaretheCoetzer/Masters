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
float SS_servo0[] = {-0.62223548, -0.61702867, -0.55519717, -0.49194158, -0.42786028, -0.35672325, -0.27671051, -0.19584169, -0.11458248, -0.03306795,  0.06301603,  0.03296182, -0.00982665, -0.04421177, -0.07363937, -0.10152155, -0.12593319, -0.15014119, -0.17370575, -0.19598594, -0.21676535, -0.24184006, -0.25324225, -0.25485616, -0.25547315, -0.25466972, -0.25340176, -0.26430709, -0.30200211, -0.34472224, -0.38829283, -0.43041705, -0.46466224, -0.48641994, -0.49972377, -0.51500532, -0.53383448, -0.55271677, -0.55421964, -0.55335185, -0.55267056, -0.54920929, -0.54933787, -0.57586446, -0.59722806};
float SS_servo1[] = {0.69751369, 0.73252916, 0.74992496, 0.76740478, 0.78402391, 0.78308556, 0.74692337, 0.68896771, 0.61072137, 0.55586681, 0.54686721, 0.62793741, 0.72545502, 0.83173548, 0.92650877, 1.00177014, 1.02973219, 1.01127041, 0.97341669, 0.93713984, 0.90322295, 0.89743877, 0.85511641, 0.78874014, 0.71592233, 0.64284296, 0.57166137, 0.54116577, 0.6067382 , 0.69232424, 0.77299761, 0.8361752 , 0.87613594, 0.91090046, 0.92814684, 0.95084501, 0.97541286, 0.98642464, 0.92431532, 0.8499509 , 0.77605484, 0.69686262, 0.63532186, 0.65493279, 0.66296023};      
float SS_servo2[] = {-0.24248612, -0.24952204, -0.25304959, -0.25843625, -0.2615022 , -0.2751793 , -0.31876247, -0.36815857, -0.42042206, -0.47193904, -0.51530183, -0.53281713, -0.53840795, -0.54058171, -0.54638271, -0.55323597, -0.5569573 , -0.55115641, -0.54597472, -0.5325492 , -0.51819462, -0.50927631, -0.46526347, -0.39681452, -0.33091506, -0.26355741, -0.19667679, -0.11945901, -0.03854751,  0.04250223,  0.10836729,  0.09863921,  0.08535751,  0.09511302,  0.05582599,  0.01927036, -0.01411977, -0.04834987, -0.07239656, -0.09266455, -0.11280911, -0.13335624, -0.156085  , -0.18898688, -0.22772065};
float SS_servo3[] = {0.77675948, 0.73445112, 0.65822292, 0.58334663, 0.50734754, 0.47415706, 0.5332082 , 0.61743209, 0.71479681, 0.80590438, 0.87714129, 0.88855569, 0.89818983, 0.92441368, 0.95466913, 0.97323504, 0.95486255, 0.88433797, 0.8016042 , 0.70519396, 0.60892054, 0.54471933, 0.53974992, 0.55315363, 0.56879054, 0.58235693, 0.59606754, 0.59080801, 0.54202038, 0.49778719, 0.43366655, 0.44349376, 0.53174236, 0.57734211, 0.65721205, 0.73424169, 0.79810849, 0.84398114, 0.81936723, 0.78208409, 0.7449673 , 0.70746529, 0.68559304, 0.72343839, 0.77259859};      
float SS_servo4[] = {0.63488669, 0.66037095, 0.66297746, 0.66249555, 0.66265563, 0.65733891, 0.63685763, 0.61874074, 0.60006512, 0.58219126, 0.56476196, 0.54429438, 0.51665454, 0.48370659, 0.44782489, 0.4272664 , 0.43688609, 0.46164647, 0.48009144, 0.50158403, 0.51305817, 0.51448671, 0.49370051, 0.46511961, 0.43880475, 0.40813685, 0.38393757, 0.35332722, 0.31619887, 0.27788799, 0.23817061, 0.19397513, 0.18243771, 0.24818319, 0.32909598, 0.41011773, 0.49080848, 0.55993913, 0.57298273, 0.57455753, 0.57638715, 0.58573622, 0.61245731, 0.61994006, 0.62545864};      
float SS_servo5[] = {-0.47557164, -0.57212546, -0.6511663 , -0.72966441, -0.80771127, -0.85737214, -0.85276317, -0.83764138, -0.81130941, -0.78788788, -0.76724179, -0.74761254, -0.69662701, -0.61269299, -0.52476539, -0.4792383 , -0.52066026, -0.62586726, -0.7338733 , -0.84678779, -0.93842759, -0.99800612, -1.01876014, -1.02762278, -1.04823191, -1.06003958, -1.08529397, -1.08248698, -1.0174545 , -0.93619041, -0.85844021, -0.7857913 , -0.78266972, -0.83845233, -0.85198486, -0.85295458, -0.86265375, -0.87796999, -0.8149471 , -0.72928202, -0.64338653, -0.56794086, -0.57488254, -0.53220638, -0.4807539 };
float SS_servo6[] = { 0.33077923,  0.31677421,  0.28290828,  0.25106033,  0.21714257,  0.18088667,  0.14089753,  0.10063505,  0.05667399,  0.00619581, -0.03718384, -0.08290232, -0.0753196 ,  0.00205426,  0.08273285,  0.1632157 ,  0.23838606,  0.29999827,  0.35699416,  0.42005146,  0.49333752,  0.51848005,  0.51042789,  0.53109319,  0.54857227,  0.56387464,  0.57593006,  0.58467148,  0.56487253,  0.54482076,  0.526042  ,  0.52165823,  0.51200845,  0.47143353,  0.42320502,  0.37675596,  0.33487884,  0.31398538,  0.32131861,  0.33808363,  0.35488987,  0.36179712,  0.3633669 ,  0.34976414,  0.33732647};
float SS_servo7[] = {-0.93057961, -0.96161527, -0.97988629, -1.00901575, -1.03336856, -1.03017036, -0.97552014, -0.90078491, -0.80673775, -0.70233138, -0.59789388, -0.48879087, -0.47985363, -0.54952339, -0.58684887, -0.58859564, -0.58295112, -0.55997789, -0.5337785 , -0.51214993, -0.51543185, -0.46938985, -0.39341731, -0.49313974, -0.59300565, -0.68649748, -0.77354729, -0.84160752, -0.81743698, -0.78179599, -0.75339115, -0.76768553, -0.77831988, -0.70868328, -0.62327077, -0.53769732, -0.46517277, -0.44882777, -0.52650948, -0.63128884, -0.7361239 , -0.82698826, -0.89965142, -0.90533783, -0.91501315};

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
