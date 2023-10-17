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
float SS_servo0[] = {-0.51201453, -0.4972617 , -0.43946054, -0.36101674, -0.28048318, -0.20571929, -0.10557714, -0.00559533,  0.0824946 ,  0.17428616,  0.27813152,  0.35520343,  0.34314702,  0.27884627,  0.23353361,  0.18820004,  0.13430943,  0.08875551,  0.03764917, -0.00169566, -0.05047771, -0.08874081, -0.10766026, -0.13564546, -0.16439621, -0.19138088, -0.23317046, -0.28545971, -0.32683889, -0.37911666, -0.43356082, -0.48684993, -0.51850305, -0.52419959, -0.52318405, -0.52301836, -0.52302949, -0.51957963, -0.52703934, -0.52523695, -0.5135011 , -0.50966788};
float SS_servo1[] = {0.45210533, 0.45885161, 0.52882517, 0.61268874, 0.69652035, 0.78668762, 0.79213662, 0.7190138 , 0.63957433, 0.55772759, 0.49697909, 0.42856197, 0.44712999, 0.55363938, 0.62522117, 0.68966598, 0.77184704, 0.83265837, 0.86190738, 0.84717177, 0.83225111, 0.80018932, 0.7394567 , 0.73279749, 0.74645393, 0.76501228, 0.81527334, 0.88274821, 0.93014117, 0.99272391, 1.06037499, 1.13488629, 1.14606099, 1.08256189, 1.0081585 , 0.93233548, 0.85387691, 0.77792516, 0.70762715, 0.63019472, 0.54398311, 0.46922271};
float SS_servo2[] = {-0.07697887, -0.1119401 , -0.15887116, -0.18173411, -0.20620854, -0.23223066, -0.26494916, -0.30771429, -0.35648947, -0.41357885, -0.4709293 , -0.51410066, -0.52293357, -0.51780698, -0.51477197, -0.50482291, -0.5024057 , -0.50108686, -0.49688568, -0.49833107, -0.51276861, -0.50737759, -0.45877001, -0.39533019, -0.31236796, -0.22779355, -0.14489096, -0.05554693,  0.0330424 ,  0.11877574,  0.20925382,  0.28934064,  0.30861711,  0.27259979,  0.2225825 ,  0.18179631,  0.13174708,  0.10216603,  0.02975433, -0.02090152, -0.03994831, -0.0600972 };
float SS_servo3[] = {0.49188964, 0.50139553, 0.51601849, 0.49102045, 0.47491333, 0.47012353, 0.49072712, 0.53482867, 0.5928332 , 0.67074408, 0.77699586, 0.87181143, 0.8749186 , 0.83627965, 0.79900939, 0.74178429, 0.69618467, 0.65281156, 0.58606529, 0.5123423 , 0.45369195, 0.46665711, 0.54204237, 0.61489794, 0.69569995, 0.77875172, 0.83127561, 0.76236355, 0.68180242, 0.59888495, 0.52136648, 0.46211418, 0.39179089, 0.39576369, 0.43203685, 0.44714421, 0.47679825, 0.46614959, 0.52610008, 0.55063229, 0.51746636, 0.48635672};
float SS_servo4[] = { 0.51945452,  0.52373201,  0.52080167,  0.52031525,  0.51984387,  0.52224346,  0.52341079,  0.52046678,  0.52035937,  0.52431766,  0.52163079,  0.49848377,  0.46089873,  0.42474756,  0.39512126,  0.35629646,  0.32541024,  0.29998733,  0.26934953,  0.25221074,  0.2407771 ,  0.22119942,  0.18321774,  0.12843325,  0.08922296,  0.05813597,  0.00938937, -0.04911751, -0.10429729, -0.16708545, -0.21408344, -0.17509593, -0.09166678, -0.00832134,  0.07498433,  0.14970949,  0.21457466,  0.27270725,  0.33506849,  0.41548376,  0.499019  ,  0.52067727};
float SS_servo5[] = {-0.47243659, -0.53976543, -0.61116836, -0.67974153, -0.74550946, -0.81121835, -0.86400319, -0.90747528, -0.9547646 , -1.00427259, -1.01741313, -0.96268908, -0.8955486 , -0.84989635, -0.81628668, -0.77062046, -0.74360507, -0.72845543, -0.72946435, -0.77907255, -0.85817669, -0.92835797, -0.95782784, -0.9119283 , -0.87659297, -0.84913864, -0.78675168, -0.70468766, -0.62364132, -0.53351747, -0.48039664, -0.56945054, -0.6566285 , -0.74257641, -0.82143664, -0.89950282, -0.94052229, -0.87437287, -0.79899532, -0.71568485, -0.6148852 , -0.47227863};
float SS_servo6[] = { 0.12868616,  0.09211754,  0.04487938,  0.00413839, -0.03401935, -0.07299875, -0.12202541, -0.17891825, -0.23406071, -0.28819826, -0.28666168, -0.20223087, -0.10459076, -0.01383553,  0.07689433,  0.14766257,  0.21130227,  0.28748823,  0.36357091,  0.43153263,  0.48404577,  0.50198413,  0.49237361,  0.4875036 ,  0.49540096,  0.50483152,  0.5142386 ,  0.52286645,  0.52155838,  0.5187278 ,  0.52944746,  0.50163946,  0.44355037,  0.38629395,  0.35424976,  0.32231732,  0.28722748,  0.27943905,  0.23131673,  0.20043954,  0.18537107,  0.14934106};
float SS_servo7[] = {-0.6558204 , -0.64116927, -0.62734436, -0.61915636, -0.61124465, -0.5934412 , -0.54621704, -0.47156683, -0.39725908, -0.32054216, -0.32714484, -0.40329   , -0.48488998, -0.56615586, -0.6485144 , -0.68277427, -0.6392757 , -0.56417406, -0.48403629, -0.40366783, -0.29666116, -0.24094202, -0.28800788, -0.34172118, -0.40257939, -0.45919107, -0.51434233, -0.57262684, -0.61205855, -0.65430136, -0.72123601, -0.70267479, -0.62949883, -0.57576054, -0.57277614, -0.5719602 , -0.57019217, -0.61704949, -0.60408508, -0.61912057, -0.6593142 , -0.66551315};

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
