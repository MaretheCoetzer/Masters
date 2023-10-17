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
float SS_servo0[] = {-0.47803394, -0.44254997, -0.32894859, -0.27280114, -0.19732923, -0.11646822, -0.03334659,  0.08497434,  0.20399026,  0.29614017,  0.36492201,  0.44614138,  0.51876397,  0.58452613,  0.63468676,  0.67549508,  0.7361921 ,  0.71899619,  0.65159287,  0.63855081,  0.62451842,  0.61541027,  0.59863913,  0.57573926,  0.54671835,  0.52303959,  0.50337711,  0.47785948,  0.44966448,  0.4166891 ,  0.3791806 ,  0.34543081,  0.31248423,  0.27955978,  0.24671478,  0.21620016,  0.18036857,  0.14228513,  0.11583737,  0.08535941,  0.04244603, -0.00908888, -0.06737215, -0.10990889, -0.13996876, -0.17024875, -0.19410191, -0.21613211, -0.23204569, -0.24683159, -0.26731611, -0.28839837, -0.31171071, -0.33812692, -0.36564714, -0.39400934, -0.42556164, -0.45771775};
float SS_servo1[] = {1.79184799, 1.77020761, 1.70288207, 1.61965729, 1.54351609, 1.4692128 , 1.39569977, 1.33449164, 1.27357067, 1.20234293, 1.12165267, 1.04506379, 0.95991083, 0.86902378, 0.77052708, 0.66939463, 0.60705383, 0.60332674, 0.69306401, 0.71198235, 0.76076847, 0.79327856, 0.85238262, 0.91568505, 0.94868961, 0.94651704, 0.94723354, 0.95860143, 0.96070482, 0.96991178, 0.9898825 , 1.01405469, 1.06235394, 1.1185293 , 1.18511438, 1.24414863, 1.28484574, 1.33671347, 1.36786714, 1.4361263 , 1.48946086, 1.52424615, 1.53134588, 1.5288126 , 1.51183446, 1.50143482, 1.4920091 , 1.48326408, 1.47622338, 1.47116986, 1.47355208, 1.48147971, 1.50100075, 1.53891192, 1.58363456, 1.63441779, 1.70816351, 1.77700381};
float SS_servo2[] = {-0.13031982, -0.16780276, -0.21314669, -0.24247071, -0.26452215, -0.28597864, -0.30742043, -0.32863335, -0.3498563 , -0.36212351, -0.37193662, -0.39089233, -0.41268618, -0.43411929, -0.45343126, -0.4717542 , -0.49987306, -0.52579316, -0.52129004, -0.50582744, -0.48810769, -0.47047245, -0.45405038, -0.45252115, -0.44323949, -0.44069757, -0.44669305, -0.45234696, -0.45788972, -0.43106837, -0.36511884, -0.28238032, -0.1960275 , -0.10969205, -0.02283878,  0.07045986,  0.16184911,  0.24763177,  0.32437506,  0.42188867,  0.48188999,  0.50769416,  0.49476136,  0.45296696,  0.39772376,  0.34278843,  0.29419638,  0.24751067,  0.20590954,  0.16635986,  0.12800555,  0.0890192 ,  0.05228274,  0.02059785, -0.0092036 , -0.03757929, -0.06055304, -0.08776539};
float SS_servo3[] = {1.62167254, 1.62884121, 1.6000152 , 1.54059547, 1.48852621, 1.43959554, 1.39183539, 1.36230094, 1.33308693, 1.28694107, 1.24416284, 1.23356262, 1.23125789, 1.22657663, 1.21535579, 1.20149335, 1.21878317, 1.20150003, 1.14375174, 1.10711701, 1.07419513, 1.04384973, 1.01757129, 1.01207174, 0.98077603, 0.95196079, 0.93458437, 0.91003176, 0.88206935, 0.88347631, 0.90747546, 0.89668539, 0.89449897, 0.89493425, 0.90281715, 0.88595174, 0.84407998, 0.81300208, 0.81980417, 0.76013694, 0.6836778 , 0.60274984, 0.53938524, 0.56777125, 0.64268942, 0.7190575 , 0.79203843, 0.86416875, 0.93618083, 1.00623887, 1.0671418 , 1.13616519, 1.20767466, 1.27768322, 1.3471202 , 1.41752708, 1.49160878, 1.56898046};
float SS_servo4[] = { 0.48888325,  0.49291875,  0.48360869,  0.48402939,  0.48358958,  0.48268828,  0.48198544,  0.48437268,  0.4868643 ,  0.47935936,  0.47083177,  0.47179415,  0.46927056,  0.46330043,  0.44894625,  0.42886981,  0.40641703,  0.35606329,  0.31404318,  0.29759761,  0.28696887,  0.27652149,  0.25483694,  0.22995385,  0.19998697,  0.16643198,  0.13879378,  0.10608817,  0.06756189,  0.02716058, -0.02292455, -0.06964751, -0.1116213 , -0.15378609, -0.1959526 , -0.23866325, -0.29137332, -0.34691029, -0.3963015 , -0.44380843, -0.51724547, -0.5959668 , -0.64699408, -0.58659589, -0.49410212, -0.41123786, -0.31662185, -0.21713379, -0.10097138,  0.01734579,  0.11339725,  0.19611154,  0.26291295,  0.3204409 ,  0.37559881,  0.42876109,  0.4743623 ,  0.50543974};
float SS_servo5[] = {-0.68176201, -0.73589562, -0.78506226, -0.86365334, -0.94134879, -1.01803825, -1.09505905, -1.17723904, -1.26043461, -1.33192903, -1.38862147, -1.46637608, -1.54548554, -1.62148198, -1.686725  , -1.74437279, -1.79732955, -1.81887193, -1.81853222, -1.79881234, -1.75916955, -1.71403498, -1.64610593, -1.5651259 , -1.49001809, -1.41756936, -1.36762015, -1.309685  , -1.24777585, -1.19433281, -1.1273588 , -1.05583192, -0.97889538, -0.90104271, -0.82252523, -0.74525641, -0.65681363, -0.57127795, -0.49188115, -0.42021648, -0.27662942, -0.1519224 , -0.09300535, -0.13642048, -0.21379624, -0.27182943, -0.32519595, -0.38005369, -0.4387807 , -0.50157193, -0.57485226, -0.6318572 , -0.66391313, -0.68175637, -0.69576832, -0.70635783, -0.70398033, -0.69213261};
float SS_servo6[] = {-0.0679262 , -0.08973189, -0.11794937, -0.11761768, -0.12502535, -0.13467403, -0.14534286, -0.17189602, -0.19891363, -0.22437241, -0.24454893, -0.26714187, -0.29112747, -0.31608195, -0.34065188, -0.36527801, -0.4006981 , -0.464115  , -0.46938315, -0.39619492, -0.31902998, -0.23382253, -0.12998945, -0.03078664,  0.04738656,  0.11842592,  0.19524906,  0.26830784,  0.3366074 ,  0.3702096 ,  0.36519936,  0.35283123,  0.37149349,  0.3894668 ,  0.41153404,  0.43677131,  0.46167621,  0.48699925,  0.51273699,  0.53832024,  0.54014713,  0.53089198,  0.4994928 ,  0.44754533,  0.3851476 ,  0.32265172,  0.26543553,  0.2101425 ,  0.16007115,  0.11245307,  0.06883214,  0.0319532 ,  0.00418775, -0.01224035, -0.02457733, -0.03353612, -0.02969555, -0.03350032};
float SS_servo7[] = {-0.09895256, -0.10079616, -0.10906329, -0.17685982, -0.22544709, -0.26864824, -0.30944209, -0.31275274, -0.31498991, -0.32028761, -0.33212108, -0.3305134 , -0.32513341, -0.3192634 , -0.31697027, -0.31570407, -0.28126362, -0.21732024, -0.25333909, -0.32413084, -0.33833642, -0.33132895, -0.36810943, -0.42883376, -0.41533688, -0.35034128, -0.28738632, -0.21713298, -0.13471657, -0.09778739, -0.11325101, -0.1298706 , -0.1963111 , -0.25895209, -0.32657263, -0.40008634, -0.47718454, -0.55336023, -0.6270539 , -0.70215263, -0.75765236, -0.81119724, -0.83275094, -0.78469142, -0.70178263, -0.61743419, -0.53589391, -0.45579658, -0.37909627, -0.30578383, -0.24457121, -0.19417677, -0.15827308, -0.13945741, -0.1266668 , -0.118839  , -0.12981441, -0.12982402};

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
