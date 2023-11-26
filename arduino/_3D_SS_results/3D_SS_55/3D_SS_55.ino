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
float SS_servo0[] = {-0.66684107, -0.63143991, -0.59771067, -0.54367899, -0.48808372, -0.42951672, -0.36055804, -0.28028935, -0.19975499, -0.11852944, -0.03723451,  0.04349571,  0.01531399, -0.01334125, -0.02346735, -0.02559821, -0.04177472, -0.07275441, -0.10697021, -0.14219148, -0.17826925, -0.21444898, -0.2439218 , -0.27032206, -0.28996343, -0.3021945 , -0.31576684, -0.32865389, -0.34275919, -0.3663774 , -0.39293401, -0.42544454, -0.45695233, -0.49296165, -0.52614888, -0.5533297 , -0.57690015, -0.6033298 , -0.63282446, -0.64702402, -0.65233889, -0.65673953, -0.65860123, -0.65885476, -0.66342919};
float SS_servo1[] = {0.88391589, 0.90701691, 0.87627874, 0.9000534 , 0.9283145 , 0.95090139, 0.96309823, 0.96330655, 0.96330701, 0.962976  , 0.96081616, 0.93445662, 1.02615093, 1.12581144, 1.23614538, 1.32732057, 1.36807284, 1.35139795, 1.33301509, 1.30694795, 1.27147344, 1.23085371, 1.20135494, 1.16941133, 1.11201543, 1.05439528, 1.00430301, 0.95592616, 0.9174829 , 0.93370357, 0.97446058, 1.02288166, 1.05726205, 1.09421799, 1.14837287, 1.19066179, 1.22293259, 1.26211407, 1.30795749, 1.27630765, 1.202413  , 1.12784384, 1.05011906, 0.97437595, 0.93451828};      
float SS_servo2[] = {-0.28151967, -0.29138206, -0.3009631 , -0.30786024, -0.30964888, -0.31429813, -0.32827652, -0.36296008, -0.40260535, -0.44440902, -0.48524982, -0.52226681, -0.55071927, -0.57003239, -0.58015459, -0.59188116, -0.60349955, -0.6134431 , -0.62291587, -0.63397979, -0.6469173 , -0.66164267, -0.65996813, -0.66335985, -0.62640972, -0.55850221, -0.48969087, -0.42176972, -0.34920937, -0.2705674 , -0.18953613, -0.10876925, -0.02806207,  0.05128717,  0.07863576,  0.02153435, -0.02843787, -0.07007599, -0.10930736, -0.14351055, -0.16909402, -0.1933236 , -0.21354352, -0.23351113, -0.25986434};
float SS_servo3[] = {0.95816329, 0.93228883, 0.86979181, 0.78852538, 0.70242461, 0.63252215, 0.61421385, 0.67525693, 0.76180302, 0.86233193, 0.96631004, 1.07010951, 1.13440469, 1.18603075, 1.25769002, 1.32974866, 1.34521457, 1.29814811, 1.24416099, 1.18732535, 1.1272798 , 1.06361637, 0.99175338, 0.9329987 , 0.92495891, 0.93983093, 0.95180881, 0.96463954, 0.97198338, 0.94111206, 0.88646937, 0.84974626, 0.78837007, 0.74420986, 0.73689582, 0.86249231, 0.9707545 , 1.06392386, 1.15551287, 1.15958435, 1.11408802, 1.06739889, 1.01223447, 0.96398858, 0.96190971};      
float SS_servo4[] = { 0.61878973,  0.62738898,  0.6306038 ,  0.63638727,  0.63843652,  0.63807094,  0.6327486 ,  0.62341217,  0.61726131,  0.61321794,  0.61395201,  0.6159661 ,  0.61678475,  0.60876269,  0.58643803,  0.55809676,  0.53203899,  0.52013976,  0.51574895,  0.51050613,  0.50442932,  0.49138229,  0.46026788,  0.41836131,  0.37697292,  0.33222881,  0.28986128,  0.25124051,  0.21044301,  0.16882057,  0.12825168,  0.08372615,  0.03556524, -0.00358223,  0.03856676,  0.11852967,  0.19994698,  0.28111044,  0.36174712,  0.42657733,  0.4867116 ,  0.55132275,  0.62471542,  0.61590961,  0.62310098};
float SS_servo5[] = {-0.73675168, -0.79183983, -0.86834965, -0.9622443 , -1.05046175, -1.12822276, -1.1688296 , -1.16629112, -1.15351282, -1.13229633, -1.1132112 , -1.0894832 , -1.08536048, -1.06095537, -0.97340295, -0.87636358, -0.8284987 , -0.86241173, -0.91913731, -0.98326021, -1.05680589, -1.12706922, -1.14121229, -1.12939391, -1.12825681, -1.10869614, -1.09351537, -1.08663356, -1.06911798, -1.01164192, -0.93233852, -0.84941492, -0.77073786, -0.71921309, -0.76593392, -0.79812145, -0.80000431, -0.80452048, -0.8428723 , -0.84647368, -0.82047754, -0.7987412 , -0.79002985, -0.7654919 , -0.7579802 };
float SS_servo6[] = {0.40209813, 0.39105417, 0.37709814, 0.36346786, 0.34708332, 0.32973979, 0.30692046, 0.28211367, 0.2592932 , 0.23196854, 0.2019619 , 0.18389278, 0.16303754, 0.14264689, 0.18628527, 0.26654826, 0.34056222, 0.4067484 , 0.47456499, 0.54638265, 0.62053743, 0.61907662, 0.63543867, 0.62983024, 0.6083438 , 0.62765667, 0.63977629, 0.65269317, 0.66482836, 0.6551806 , 0.63859301, 0.62305367, 0.62680188, 0.62823597, 0.59916017, 0.552224  , 0.50723165, 0.46461176, 0.42483582, 0.4148317 , 0.43016443, 0.44411198, 0.44975615, 0.44981196, 0.42544763};      
float SS_servo7[] = {-0.86481384, -0.8862982 , -0.935757  , -0.99978105, -1.06136511, -1.1169144 , -1.12620556, -1.08271273, -1.02319339, -0.94131334, -0.84756664, -0.74487207, -0.67416688, -0.59870085, -0.62172208, -0.66084188, -0.65433026, -0.63644583, -0.61723624, -0.60328247, -0.59569144, -0.60835897, -0.5543112 , -0.50051854, -0.38343884, -0.48390657, -0.56937142, -0.65532643, -0.73655916, -0.75029595, -0.73497277, -0.7255008 , -0.76068161, -0.79693645, -0.75776544, -0.67837695, -0.60263621, -0.52811435, -0.45605295, -0.48735367, -0.59243022, -0.69511974, -0.78493436, -0.86332361, -0.86682659};

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
