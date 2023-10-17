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
float SS_servo0[] = {-0.51171401, -0.49993748, -0.47026327, -0.44096888, -0.38808686, -0.34624653, -0.3070126 , -0.23088439, -0.13595798, -0.05817479,  0.04621614,  0.16362595,  0.24589119,  0.25747467,  0.20784978,  0.16711634,  0.12219967,  0.07533078,  0.0273456 , -0.02287987, -0.07339243, -0.12692253, -0.17858024, -0.21310471, -0.23049041, -0.2436698 , -0.25631732, -0.27291926, -0.30633458, -0.34531976, -0.37760393, -0.40913472, -0.4519298 , -0.48734177, -0.51238617, -0.52457266, -0.51846743, -0.51358055, -0.51059049, -0.51420725, -0.52272302, -0.52376686, -0.51976746, -0.51226402};
float SS_servo1[] = {0.17681871, 0.11460583, 0.09170655, 0.13116924, 0.10831753, 0.09374044, 0.154486  , 0.23660993, 0.32806931, 0.41010372, 0.44212866, 0.39706951, 0.35607909, 0.36033417, 0.46371112, 0.54743088, 0.63617836, 0.72524182, 0.81397774, 0.9019839 , 0.98201166, 1.03605664, 1.02317177, 0.97132136, 0.89768239, 0.840446  , 0.79632531, 0.76922017, 0.77283385, 0.77760873, 0.76562201, 0.74610745, 0.76399392, 0.79027263, 0.80386495, 0.79135013, 0.71032046, 0.62525281, 0.54511585, 0.48036843, 0.42976493, 0.36994427, 0.30097453, 0.21639757};
float SS_servo2[] = {-0.35076251, -0.38591963, -0.41315662, -0.42925029, -0.43738371, -0.44071693, -0.44142061, -0.44553188, -0.44752448, -0.45619665, -0.47291364, -0.4927327 , -0.51094557, -0.52420364, -0.49862455, -0.46983607, -0.45558107, -0.44725575, -0.45891991, -0.48499302, -0.51056488, -0.51308705, -0.52744127, -0.51456898, -0.46849032, -0.40211896, -0.32332215, -0.24711927, -0.16536702, -0.09215053, -0.01590923,  0.06132111,  0.14200473,  0.21868547,  0.26810438,  0.26427062,  0.1827378 ,  0.10361304,  0.02726716, -0.04730158, -0.12087813, -0.19416797, -0.26049209, -0.31989334};
float SS_servo3[] = {0.86828151, 0.86082244, 0.79473108, 0.71762425, 0.6339075 , 0.54923772, 0.47503262, 0.41961528, 0.37030268, 0.33231797, 0.32921484, 0.367766  , 0.41288029, 0.45416173, 0.4050976 , 0.34424659, 0.30423845, 0.27035358, 0.27109809, 0.29373983, 0.30561456, 0.2507651 , 0.19793656, 0.22219394, 0.29588024, 0.37171256, 0.45222202, 0.54562486, 0.56003521, 0.52317705, 0.47974466, 0.41919337, 0.36473352, 0.2967786 , 0.22667908, 0.20273331, 0.29347754, 0.38320708, 0.46956309, 0.55527611, 0.64118237, 0.7262366 , 0.79446293, 0.84236912};
float SS_servo4[] = { 5.25261413e-01,  5.24424733e-01,  5.18975641e-01,  5.11270000e-01,  5.05472395e-01,  5.00579916e-01,  5.00341582e-01,  5.06384747e-01,  5.16290005e-01,  5.23134099e-01,  5.20004143e-01,  5.07315098e-01,  4.84188809e-01,  4.51541954e-01,  4.30313390e-01,  3.99537195e-01,  3.74092862e-01,  3.47234014e-01,  3.21503999e-01,  3.03114171e-01,  3.00525536e-01,  2.96632885e-01,  2.54669464e-01,  2.14813734e-01,  1.41636923e-01,  6.79551109e-02,  5.04151387e-04, -6.52289414e-02, -1.34789526e-01, -2.10272703e-01, -2.88127349e-01, -3.71251152e-01, -4.04765934e-01, -3.36333433e-01, -2.11979862e-01, -6.04039604e-02,  3.69624575e-02,  1.20890593e-01,  1.98152430e-01,  2.62364256e-01,  3.34999559e-01,  4.19307773e-01,  4.89869708e-01,  5.20573847e-01};
float SS_servo5[] = {-0.37731848, -0.44153553, -0.54536577, -0.6350439 , -0.72326036, -0.81107334, -0.8962354 , -0.97869678, -1.058515  , -1.13677747, -1.1771854 , -1.15748051, -1.10881525, -1.03530566, -0.99351051, -0.93670684, -0.89588232, -0.85678762, -0.82337737, -0.81093928, -0.83946363, -0.8903663 , -0.91219459, -0.94550948, -0.90483607, -0.8290381 , -0.75414245, -0.67503361, -0.59319121, -0.51091105, -0.42873782, -0.34458213, -0.34983226, -0.41776347, -0.45549208, -0.48119693, -0.55607863, -0.6374075 , -0.70045331, -0.68628951, -0.61566124, -0.53255355, -0.4282161 , -0.35123131};
float SS_servo6[] = {-0.10370654, -0.12094141, -0.1766264 , -0.23689442, -0.28668648, -0.33978677, -0.38428272, -0.43961141, -0.50517789, -0.58196543, -0.60512114, -0.5310631 , -0.42204064, -0.2760854 , -0.16709525, -0.07038866,  0.01661336,  0.08745056,  0.16365939,  0.24723835,  0.33085243,  0.4124951 ,  0.4532656 ,  0.45901598,  0.4598202 ,  0.4653078 ,  0.47800165,  0.49400647,  0.50485443,  0.50767107,  0.50853851,  0.52385658,  0.51149524,  0.46777957,  0.41029029,  0.35442092,  0.29506338,  0.24278463,  0.18664623,  0.12815598,  0.07385349,  0.01681364, -0.04089244, -0.09603496};
float SS_servo7[] = {-0.3096837 , -0.34474162, -0.35156395, -0.34423182, -0.35343919, -0.34967608, -0.34832609, -0.30954348, -0.23770965, -0.14681666, -0.13920341, -0.23449743, -0.32552965, -0.39430856, -0.42898354, -0.47592105, -0.51335347, -0.48530509, -0.41610589, -0.33758221, -0.25736556, -0.20016314, -0.10550818, -0.1035675 , -0.20018249, -0.28647105, -0.37337634, -0.45932771, -0.54098247, -0.61860584, -0.6959595 , -0.80599573, -0.8555288 , -0.81070048, -0.73394309, -0.6650318 , -0.61633686, -0.58174718, -0.5373421 , -0.48507636, -0.43863081, -0.38603433, -0.33330681, -0.29066803};

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
