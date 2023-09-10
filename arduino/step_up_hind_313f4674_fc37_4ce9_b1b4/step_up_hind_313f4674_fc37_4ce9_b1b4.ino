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
float SS_servo0[] = { 0.36339664,  0.28014626,  0.23139848,  0.19900989,  0.18001651,  0.14764563,  0.12457601,  0.11079071,  0.10406515,  0.08764095,  0.06829144,  0.05925307,  0.0536932 ,  0.04872352,  0.04340696,  0.03884319,  0.03320969,  0.02555209,  0.01471436,  0.00348723, -0.00636541, -0.01044154, -0.007354  , -0.00260049,  0.00157346,  0.00628932,  0.01172869,  0.01615834,  0.01742177,  0.01417123,  0.00241476, -0.00850347, -0.02384767, -0.03758913, -0.05020375, -0.06212258, -0.07030152, -0.07105462, -0.06181506, -0.03847511};
float SS_servo1[] = {0.91446374, 1.04050552, 1.09319479, 1.13538878, 1.20609462, 1.21109729, 1.11652305, 1.0231255 , 0.95768581, 0.94771202, 0.98336069, 1.02857686, 1.07313567, 1.11100472, 1.14230802, 1.16718593, 1.18017884, 1.17041251, 1.12779302, 1.05921191, 0.97366153, 0.89337742, 0.82786589, 0.77364825, 0.72730747, 0.68382696, 0.64078211, 0.60333401, 0.57707903, 0.57519201, 0.60543045, 0.64413934, 0.67994804, 0.71064316, 0.73683385, 0.75766569, 0.76515505, 0.74113706, 0.68246977, 0.60017268};
float SS_servo2[] = { 0.12818518,  0.0215472 , -0.03028032, -0.05165744, -0.06923429, -0.08356159, -0.09089983, -0.09989023, -0.11376052, -0.12921382, -0.1386138 , -0.14244755, -0.14161046, -0.14056697, -0.14075855, -0.14034449, -0.13795276, -0.13529973, -0.14083007, -0.15646867, -0.1743141 , -0.19393807, -0.21753358, -0.23996204, -0.26154561, -0.26967108, -0.25886609, -0.24405399, -0.22385415, -0.19668696, -0.17082736, -0.16726331, -0.18898229, -0.21316569, -0.2353928 , -0.25423035, -0.26801533, -0.27318888, -0.26287705, -0.23690094};
float SS_servo3[] = {0.60371474, 0.7312031 , 0.77429246, 0.75140606, 0.72009298, 0.68078297, 0.6347562 , 0.5977386 , 0.57434   , 0.56988605, 0.58870955, 0.62140442, 0.65280782, 0.68061217, 0.70363906, 0.71908885, 0.72132045, 0.70301881, 0.66659948, 0.62176172, 0.57131063, 0.54160824, 0.5391485 , 0.5513787 , 0.57275666, 0.56711292, 0.51025418, 0.45029873, 0.39594365, 0.3545038 , 0.3379785 , 0.35936024, 0.40638065, 0.45372235, 0.49574834, 0.52949267, 0.54974024, 0.54122089, 0.49281899, 0.4203675 };
float SS_servo4[] = { 0.48848437,  0.51502563,  0.51990029,  0.50951867,  0.49160387,  0.46574846,  0.43608062,  0.40433835,  0.37040984,  0.336129  ,  0.30614387,  0.27828646,  0.24767795,  0.2138279 ,  0.18353067,  0.15382239,  0.12033583,  0.08368477,  0.04479753,  0.01017165, -0.00980187, -0.01823378, -0.01555769, -0.01230885, -0.01282672, -0.01587075, -0.02307795, -0.03660981, -0.06767139, -0.07631395, -0.02027361,  0.04778196,  0.1146302 ,  0.18325234,  0.25192173,  0.32114665,  0.38614841,  0.4460881 ,  0.51155666,  0.5307017 };
float SS_servo5[] = {-0.6325975 , -0.66641654, -0.69540389, -0.73016556, -0.76059977, -0.78573129, -0.80381   , -0.81228103, -0.81004654, -0.79055092, -0.74487192, -0.68077878, -0.61166815, -0.54430897, -0.48767744, -0.43584642, -0.38814417, -0.35404539, -0.34695244, -0.36569294, -0.40819357, -0.4565844 , -0.50213041, -0.53866977, -0.56592551, -0.58799795, -0.60504911, -0.61186025, -0.59444857, -0.57660382, -0.5991508 , -0.61199656, -0.62644089, -0.63805766, -0.64966943, -0.66157038, -0.66480826, -0.60752842, -0.45431234, -0.32406122};
float SS_servo6[] = { 0.04928583,  0.13239539,  0.15964804,  0.16727305,  0.15958679,  0.13902954,  0.11090009,  0.07064022,  0.02010745, -0.00944623,  0.01601276,  0.07943039,  0.14954539,  0.22138941,  0.29190608,  0.36176545,  0.41518385,  0.45482611,  0.51173619,  0.5288307 ,  0.47506238,  0.43548875,  0.45209509,  0.47387643,  0.49254929,  0.50607801,  0.51531374,  0.52089025,  0.51950181,  0.5018763 ,  0.47014333,  0.4398357 ,  0.40622937,  0.37261363,  0.34101051,  0.31918633,  0.31334114,  0.3100965 ,  0.31472666,  0.33963718};
float SS_servo7[] = {-0.22759391, -0.31119351, -0.35860927, -0.40748881, -0.44359897, -0.46881342, -0.48360596, -0.47844217, -0.45432766, -0.43429708, -0.43717658, -0.45106386, -0.46205543, -0.47184224, -0.48117268, -0.49155623, -0.47846988, -0.39097448, -0.246544  , -0.12337462, -0.0941005 , -0.11953904, -0.17696997, -0.23001615, -0.27570531, -0.31499034, -0.35031603, -0.37941976, -0.39551931, -0.3815876 , -0.33361109, -0.27896031, -0.2297994 , -0.18325954, -0.14084876, -0.11035598, -0.10048265, -0.11045942, -0.14675286, -0.20701339};

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=100000;
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);

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
  delay (500);

  servo0_deg=SS_servo0[0]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[0]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[0]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[0]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[0]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[0]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[0]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[0]/3.14159265*180-servo6_deg;
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
}

void loop() 
{
  if(Serial.available()>0)
  {
    first=Serial.read();
    if(first=='1')
    {
      input=1;
    }
    if(first=='0')
    {
      input=0;
    }
  }
  if(input==1)
  {
      if(esp_timer_get_time()>=end_time)
      {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
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
        if(node>=steps)
           {
              node=0;
            }
         end_time=esp_timer_get_time()+time_step;
      }
   }

  if(input == 0)
  {
   servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
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
  
  }
}
