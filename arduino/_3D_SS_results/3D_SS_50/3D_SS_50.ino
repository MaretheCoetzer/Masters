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
float SS_servo0[] = {-5.28864757e-01, -5.17603862e-01, -4.50445460e-01, -3.83297471e-01, -3.15879801e-01, -2.41717551e-01, -1.62189090e-01, -8.12571721e-02,  5.73400497e-05,  8.13054590e-02,  1.58422448e-01,  1.73723100e-01,  1.24600280e-01,  8.28413246e-02,  5.44506117e-02,  3.05592334e-02,  1.22724443e-02, -7.12316851e-03, -2.72129283e-02, -4.94862079e-02, -7.17098618e-02, -9.38566946e-02, -1.19188599e-01, -1.45913878e-01, -1.58190196e-01, -1.56469120e-01, -1.53758479e-01, -1.53605368e-01, -1.55994352e-01, -1.63917313e-01, -1.95453659e-01, -2.38401863e-01, -2.79528994e-01, -3.24395662e-01, -3.53128471e-01, -3.73476756e-01, -3.90856649e-01, -4.06597758e-01, -4.26024756e-01, -4.42420980e-01, -4.55975395e-01, -4.52787876e-01, -4.39279993e-01, -4.33833856e-01, -4.28387720e-01, -4.31303558e-01, -4.48250242e-01, -4.67350917e-01, -4.92257769e-01, -5.02207063e-01};
float SS_servo1[] = {0.65073221, 0.67550726, 0.68715262, 0.70065894, 0.71398772, 0.72052739, 0.72150805, 0.71840931, 0.69213358, 0.62847936, 0.59093828, 0.59111473, 0.67936679, 0.76791974, 0.86512154, 0.94694228, 0.9576337 , 0.94085136, 0.9224114 , 0.88830761, 0.85184665, 0.84090285, 0.84705553, 0.8526901 , 0.80402955, 0.73641369, 0.66660706, 0.60396951, 0.5510735 , 0.52440793, 0.56156736, 0.63360904, 0.69449594, 0.75657796, 0.78804261, 0.79937431, 0.80077171, 0.80597335, 0.83416597, 0.85965054, 0.87033569, 0.80152919, 0.71373362, 0.65566707, 0.59760051, 0.56682588, 0.58184529, 0.60333606, 0.63463635, 0.62645117};
float SS_servo2[] = {-0.24214866, -0.24325396, -0.23743526, -0.23169295, -0.2242336 , -0.2293198 , -0.26012672, -0.30451505, -0.35203792, -0.4023922 , -0.44846758, -0.47870479, -0.48841459, -0.49784863, -0.50859491, -0.52171095, -0.52304203, -0.52249551, -0.52210934, -0.5224416 , -0.52170609, -0.51682621, -0.51362547, -0.52661013, -0.49147675, -0.41876552, -0.34601245, -0.27334696, -0.19952087, -0.12060703, -0.04016022,  0.02990725,  0.02916875,  0.01195951,  0.05007178,  0.05123463,  0.01364034, -0.0199435 , -0.05270959, -0.08546294, -0.11575756, -0.13282491, -0.14404413, -0.15688745, -0.16973078, -0.18270818, -0.19591058, -0.20866992, -0.2228899 , -0.22988062};
float SS_servo3[] = {0.74294719, 0.71390713, 0.64288078, 0.56629564, 0.48964107, 0.45474529, 0.48975763, 0.5660954 , 0.65329301, 0.74317242, 0.82673675, 0.87321241, 0.87258318, 0.88290153, 0.92677068, 0.96663209, 0.94077141, 0.88969939, 0.83634929, 0.76847989, 0.69695507, 0.63811028, 0.59001421, 0.58740024, 0.60139532, 0.61322331, 0.62082811, 0.62866019, 0.63538963, 0.62938662, 0.57420619, 0.50280105, 0.48764138, 0.57390148, 0.58385455, 0.59899281, 0.6447567 , 0.69131149, 0.7538129 , 0.8199676 , 0.87099796, 0.83842212, 0.79495246, 0.77077614, 0.74659983, 0.73495074, 0.74432151, 0.75502768, 0.76638066, 0.75147743};
float SS_servo4[] = { 0.40395847,  0.42542328,  0.43954462,  0.45251742,  0.46525927,  0.47457977,  0.47689315,  0.47610258,  0.48081004,  0.48934658,  0.50047258,  0.50682395,  0.5206469 ,  0.52062573,  0.4925877 ,  0.45811834,  0.42958211,  0.42172134,  0.41688308,  0.41624919,  0.41508714,  0.40190643,  0.38336694,  0.36231029,  0.33050835,  0.28560294,  0.24144765,  0.19851635,  0.15708368,  0.11961499,  0.07541664,  0.03009549, -0.0204859 , -0.07581118, -0.114036  , -0.11214957, -0.09616448, -0.06921756,  0.00066947,  0.08226381,  0.16157099,  0.22870919,  0.28072707,  0.31445067,  0.34817427,  0.37117669,  0.37618969,  0.38190233,  0.38723148,  0.39855377};
float SS_servo5[] = {-0.22207246, -0.29183973, -0.3725009 , -0.45778733, -0.54096866, -0.60486622, -0.63942618, -0.65428579, -0.67449137, -0.70288422, -0.73223861, -0.75694507, -0.80476908, -0.81719594, -0.74468224, -0.6638139 , -0.63206557, -0.66291383, -0.70210237, -0.76801412, -0.83676498, -0.86327104, -0.87060456, -0.87610621, -0.87885568, -0.84357231, -0.8118829 , -0.7829491 , -0.75412041, -0.7189068 , -0.65315936, -0.57275497, -0.48857617, -0.40110044, -0.33991117, -0.34046431, -0.3759389 , -0.42146027, -0.50428958, -0.57264153, -0.62132721, -0.62873147, -0.61415876, -0.56366043, -0.51316211, -0.46632297, -0.4256237 , -0.40265449, -0.35094687, -0.29972639};
float SS_servo6[] = { 0.14903223,  0.12968298,  0.10143233,  0.08592626,  0.0666162 ,  0.04571686,  0.01865491, -0.02047157, -0.06209804, -0.1061487 , -0.14822892, -0.16366521, -0.1466631 , -0.11183105, -0.03852767,  0.04191089,  0.11909382,  0.18969131,  0.25907143,  0.32318079,  0.39316345,  0.42294392,  0.42890421,  0.43286357,  0.41914673,  0.43942678,  0.45427998,  0.46915132,  0.48398017,  0.4938069 ,  0.48892522,  0.48073073,  0.48067541,  0.48793226,  0.48849627,  0.48409422,  0.48518977,  0.4729361 ,  0.43340478,  0.39055063,  0.3509575 ,  0.33992904,  0.33407314,  0.32119544,  0.30831774,  0.28796468,  0.25506845,  0.22160603,  0.19115775,  0.16899138};
float SS_servo7[] = {-0.52986307, -0.52018559, -0.51910032, -0.55272053, -0.57676504, -0.58120921, -0.55306483, -0.48315952, -0.40287154, -0.31914654, -0.22558943, -0.16585976, -0.18066871, -0.22577578, -0.30975042, -0.35378855, -0.35693029, -0.34346181, -0.32838564, -0.30936906, -0.36559208, -0.39437957, -0.36069415, -0.31697173, -0.20527927, -0.29800454, -0.38193935, -0.46634704, -0.54943236, -0.61142075, -0.6323995 , -0.63502052, -0.66053608, -0.70524823, -0.73119183, -0.75107099, -0.78725863, -0.79090756, -0.72548628, -0.64773266, -0.58244525, -0.61792165, -0.66619239, -0.68761142, -0.70903044, -0.70408277, -0.65489351, -0.60226943, -0.55579634, -0.53821248};

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
