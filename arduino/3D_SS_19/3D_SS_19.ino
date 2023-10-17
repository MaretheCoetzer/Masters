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
float SS_servo0[] = {-0.51250845, -0.49959067, -0.44666935, -0.37505003, -0.30059907, -0.21662536, -0.12228693, -0.04021519,  0.04374613,  0.12724258,  0.192141  ,  0.19733523,  0.14099019,  0.1063687 ,  0.08158299,  0.06008573,  0.04102283,  0.02374855,  0.00625824, -0.01371456, -0.03710636, -0.05774706, -0.08381009, -0.11358263, -0.1464247 , -0.18052033, -0.21637938, -0.25463727, -0.29682034, -0.33512771, -0.3735453 , -0.40895065, -0.4404426 , -0.46732336, -0.48595569, -0.50682223, -0.52035426, -0.51618127, -0.51326873, -0.51183963};
float SS_servo1[] = {0.65298871, 0.65904166, 0.72676329, 0.8050754 , 0.88906289, 0.92568526, 0.86010187, 0.78192067, 0.70404371, 0.63546832, 0.56970681, 0.55424614, 0.65239463, 0.70364733, 0.72882695, 0.74267114, 0.74996067, 0.74984664, 0.74424649, 0.73763094, 0.73253704, 0.72422713, 0.73405022, 0.7544929 , 0.78299338, 0.81086989, 0.83393081, 0.85739221, 0.87981928, 0.89125288, 0.92355351, 0.95864328, 0.97291653, 0.97034909, 0.94522793, 0.91845689, 0.88219326, 0.819274  , 0.74879939, 0.67207672};
float SS_servo2[] = {-0.25159038, -0.26267787, -0.27289975, -0.2841291 , -0.29868134, -0.31788157, -0.33966116, -0.36525821, -0.39765387, -0.43309739, -0.46831776, -0.50066302, -0.51891498, -0.52013376, -0.51663005, -0.50974489, -0.50077414, -0.49018555, -0.48029719, -0.47815509, -0.46813276, -0.41752985, -0.34657344, -0.27307751, -0.19706432, -0.11569687, -0.03673581,  0.02676528,  0.06319608,  0.07449017,  0.09240078,  0.08195026,  0.02005328, -0.04231545, -0.09490535, -0.14439201, -0.18212136, -0.20216926, -0.22207599, -0.24273984};
float SS_servo3[] = {0.66592335, 0.64270963, 0.60745681, 0.58342566, 0.57455456, 0.58211505, 0.59795241, 0.62328172, 0.66570096, 0.72388925, 0.79196082, 0.84339674, 0.85575446, 0.83700436, 0.80578876, 0.76530903, 0.72039452, 0.66985144, 0.61569059, 0.57019655, 0.57001545, 0.63425274, 0.71019535, 0.78671943, 0.85647848, 0.87006554, 0.80408966, 0.73684505, 0.67148793, 0.59411164, 0.52416922, 0.51482391, 0.59843885, 0.6751571 , 0.72306396, 0.75508456, 0.76227059, 0.73790684, 0.70773312, 0.67289756};
float SS_servo4[] = { 0.51436761,  0.50748802,  0.49288647,  0.48291616,  0.47722488,  0.47018148,  0.46192507,  0.45148923,  0.43970612,  0.42716502,  0.40967138,  0.38232193,  0.35004138,  0.32035976,  0.29182906,  0.26315153,  0.23694877,  0.21533833,  0.19714401,  0.17931887,  0.1571457 ,  0.13685594,  0.11807188,  0.09863003,  0.07751986,  0.05322569,  0.0219397 , -0.0163879 , -0.07056911, -0.11161678, -0.06878371,  0.02018096,  0.10705212,  0.19339113,  0.27372606,  0.34015907,  0.4105679 ,  0.48694877,  0.52044843,  0.51597636};
float SS_servo5[] = {-0.90909245, -0.94017826, -0.96636569, -0.99222966, -1.01952037, -1.03846747, -1.05290165, -1.06158276, -1.06259125, -1.05249499, -1.01939857, -0.97425602, -0.9311553 , -0.89143299, -0.85729189, -0.82554341, -0.79987552, -0.78730439, -0.78793153, -0.79666194, -0.80357746, -0.81280677, -0.81990529, -0.82331794, -0.82180862, -0.81663435, -0.80447529, -0.78094436, -0.73098172, -0.7126158 , -0.80276481, -0.88280159, -0.96296054, -1.04212694, -1.11139106, -1.12154485, -1.05165031, -0.94990043, -0.87020463, -0.8957904 };
float SS_servo6[] = { 0.10424399,  0.08717618,  0.06267843,  0.0307581 ,  0.00142255, -0.02972875, -0.0630445 , -0.10222387, -0.14754412, -0.17035347, -0.14521102, -0.10521651, -0.05475092,  0.02049765,  0.0945055 ,  0.15906858,  0.22720211,  0.30216503,  0.37468254,  0.43189021,  0.45637104,  0.46621635,  0.47588927,  0.48789831,  0.49891067,  0.50760067,  0.51296806,  0.51437017,  0.50659755,  0.48414011,  0.44385921,  0.39841135,  0.35711538,  0.32252686,  0.29180611,  0.24981501,  0.21564752,  0.19248326,  0.15794555,  0.11466209};
float SS_servo7[] = {-0.76849011, -0.78116956, -0.79245134, -0.77801915, -0.76030365, -0.73018849, -0.69054776, -0.63427229, -0.55864842, -0.51876084, -0.56276488, -0.64259467, -0.73253799, -0.80813237, -0.85567298, -0.83766493, -0.76854654, -0.69107193, -0.61242567, -0.51468574, -0.45129966, -0.50766534, -0.56660469, -0.6280564 , -0.68636495, -0.74342488, -0.80249916, -0.85856138, -0.90676961, -0.93285706, -0.90050158, -0.84472414, -0.80702775, -0.78913941, -0.78499892, -0.76439223, -0.75845133, -0.77253256, -0.77318947, -0.76384618};

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
