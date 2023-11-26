// This is a 3cm clearance height gait
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
float SS_servo0[] = {-0.64888498, -0.62594736, -0.56193101, -0.49911347, -0.4376159 , -0.37474964, -0.30310297, -0.22363618, -0.14315435, -0.06243161,  0.01864285,  0.09969607,  0.11574984,  0.08125312,  0.05852538,  0.03457007,  0.00927851, -0.02636109, -0.05765533, -0.08647697, -0.10892048, -0.12884885, -0.15269803, -0.17871921, -0.20181213, -0.22131727, -0.24051874, -0.2585772 , -0.27602255, -0.29791733, -0.32306918, -0.35312717, -0.38982835, -0.42748027, -0.4660454 , -0.499129  , -0.51895396, -0.529826  , -0.54244148, -0.55674163, -0.57318506, -0.59253372, -0.61667062, -0.63761083, -0.6544286 , -0.66154953, -0.66188878, -0.65672115, -0.65173988};
float SS_servo1[] = {1.07621824, 1.06091296, 1.07421587, 1.09493559, 1.11786873, 1.13714843, 1.13571739, 1.07977777, 1.0026524 , 0.92419647, 0.84504547, 0.77210743, 0.82116326, 0.95194559, 1.04827578, 1.13146545, 1.19394328, 1.24109363, 1.26440891, 1.28360996, 1.29211577, 1.28088151, 1.25619098, 1.21773844, 1.16356429, 1.10640448, 1.05035293, 0.99713543, 0.9497373 , 0.91939643, 0.90164629, 0.92777071, 1.013534  , 1.11310714, 1.21554942, 1.28614137, 1.29498363, 1.29821732, 1.31471997, 1.33826617, 1.36653061, 1.38613081, 1.39580852, 1.38209228, 1.36277994, 1.32955607, 1.28524543, 1.21949953, 1.14247163};
float SS_servo2[] = {-0.29211052, -0.29907373, -0.30622129, -0.31584329, -0.32406284, -0.33125759, -0.34328252, -0.36794823, -0.39963038, -0.44506094, -0.49119944, -0.53718469, -0.56917704, -0.58344223, -0.59742223, -0.60990798, -0.62859456, -0.65107362, -0.66136855, -0.66751542, -0.66300228, -0.65155585, -0.64044894, -0.63245837, -0.59852766, -0.53763525, -0.47473947, -0.41044066, -0.34475017, -0.27847251, -0.20784428, -0.1295935 , -0.04933598,  0.00967294,  0.02587967,  0.02223682, -0.01115775, -0.01868811, -0.03720523, -0.05587858, -0.07570796, -0.09452718, -0.12259544, -0.15129245, -0.17854195, -0.20334951, -0.2265514 , -0.24912803, -0.27522436};
float SS_servo3[] = {1.05000962, 0.99914693, 0.92531104, 0.8541331 , 0.78148336, 0.71237859, 0.67178192, 0.67448934, 0.70684952, 0.78178547, 0.86874621, 0.97481072, 1.07871896, 1.13434262, 1.18082436, 1.20985181, 1.23437234, 1.23990804, 1.22121543, 1.19732119, 1.15997814, 1.10032948, 1.02584426, 0.94354915, 0.91314011, 0.93591247, 0.95710851, 0.97520471, 0.9916314 , 1.00622242, 1.0161894 , 0.99169334, 0.92475583, 0.84267975, 0.78322335, 0.87185562, 0.91658075, 0.94148448, 0.97630187, 1.01892322, 1.06751859, 1.09802932, 1.12731336, 1.133664  , 1.13805635, 1.13917963, 1.13705621, 1.11940092, 1.09568524};
float SS_servo4[] = {0.61856707, 0.6446219 , 0.65125923, 0.65630858, 0.66038009, 0.66244929, 0.66289152, 0.65809077, 0.65402231, 0.64957554, 0.64765666, 0.64050145, 0.62633947, 0.61063908, 0.60914765, 0.61152426, 0.63141831, 0.65435412, 0.66362078, 0.66477729, 0.64507856, 0.62458073, 0.60979935, 0.60059682, 0.58021364, 0.55129244, 0.52123688, 0.49022443, 0.46002399, 0.43276395, 0.4032119 , 0.37342429, 0.34383058, 0.31422401, 0.28219031, 0.24406923, 0.22361458, 0.27430155, 0.3550269 , 0.4359143 , 0.51688183, 0.5973839 , 0.65403127, 0.63361562, 0.6353288 , 0.6366043 , 0.63089682, 0.61134969, 0.59537893};
float SS_servo5[] = {-0.40018466, -0.50155059, -0.58469124, -0.67005466, -0.75501236, -0.83549158, -0.90046862, -0.94114038, -0.96872578, -0.98025259, -0.98319713, -0.95761043, -0.89223278, -0.83577656, -0.81419618, -0.81262275, -0.85983279, -0.93754085, -0.99291069, -1.03045211, -1.02220774, -1.02070565, -1.04621479, -1.09889638, -1.14118075, -1.16464073, -1.18679518, -1.20825396, -1.22871745, -1.25256228, -1.26831216, -1.24925826, -1.17454455, -1.08641425, -0.99344415, -0.9054598 , -0.88972369, -0.95485693, -1.03567991, -1.11613912, -1.19694515, -1.26897227, -1.12607932, -0.60599988, -0.37787208, -0.35445117, -0.40810315, -0.40409859, -0.35778019};
float SS_servo6[] = {0.34344513, 0.32884185, 0.29145805, 0.2561347 , 0.22004561, 0.18606721, 0.15565813, 0.12311491, 0.08908028, 0.05646813, 0.01588514, 0.00156966, 0.0684756 , 0.14903532, 0.23003835, 0.31057491, 0.38967704, 0.3989608 , 0.41069416, 0.41908361, 0.41831051, 0.40463493, 0.4150511 , 0.42956535, 0.47235822, 0.5389022 , 0.59394079, 0.61643368, 0.63731613, 0.65000064, 0.66236441, 0.66330523, 0.64345812, 0.62051657, 0.60067349, 0.58252027, 0.56638757, 0.53409443, 0.49097012, 0.45194486, 0.41287846, 0.38635735, 0.38768697, 0.39099976, 0.38881706, 0.37637574, 0.35797233, 0.34405493, 0.34377733};
float SS_servo7[] = {-0.82694359, -0.85750588, -0.8627877 , -0.87897018, -0.89535614, -0.91431714, -0.92300206, -0.90638397, -0.86644002, -0.81012467, -0.72209201, -0.66500171, -0.73113669, -0.80924489, -0.89000169, -0.97162696, -1.03842157, -0.48516789, -0.23990958, -0.08374415, -0.15169008, -0.17711304, -0.19335486, -0.1682977 , -0.20985376, -0.41477886, -0.59211198, -0.70512705, -0.8131612 , -0.90198717, -0.98916885, -1.02734531, -0.98706145, -0.92893424, -0.87630777, -0.84044511, -0.837272  , -0.79379358, -0.71691602, -0.6429271 , -0.56648002, -0.52501309, -0.55580121, -0.6060327 , -0.64412531, -0.65977014, -0.66348712, -0.68880703, -0.75584145};

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
