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
float SS_servo0[] = {-0.6676992 , -0.62945067, -0.59554008, -0.54028469, -0.48347582, -0.42511456, -0.35673069, -0.27654799, -0.19605193, -0.11493437, -0.03361594,  0.04723602,  0.03168801,  0.01838336,  0.01283173,  0.01233364, -0.00795356, -0.04907805, -0.08879392, -0.12900269, -0.16022038, -0.18602706, -0.21744325, -0.24778054, -0.26785429, -0.28915771, -0.3103122 , -0.32998171, -0.35305061, -0.37543405, -0.40386178, -0.43313271, -0.46775786, -0.49692471, -0.52251279, -0.54442587, -0.57086799, -0.5979245 , -0.61592941, -0.62310649, -0.62731454, -0.63188068, -0.64008539, -0.65376116};
float SS_servo1[] = {0.95200008, 0.96851023, 0.93583258, 0.95901171, 0.98622596, 1.00885946, 1.02164962, 1.02199693, 1.02200271, 1.0219716 , 0.99255249, 0.92819901, 1.05552752, 1.18588495, 1.29237466, 1.38206928, 1.41811882, 1.38936335, 1.35329934, 1.31508036, 1.31298364, 1.32086694, 1.32941243, 1.30019339, 1.25739833, 1.2177575 , 1.17788578, 1.1418442 , 1.12867076, 1.12651108, 1.13710463, 1.14340968, 1.15683681, 1.18715959, 1.21491418, 1.2344749 , 1.26533049, 1.29700336, 1.26582406, 1.18738887, 1.10477181, 1.02485013, 0.98269528, 0.98106311};
float SS_servo2[] = {-0.2829993 , -0.29189694, -0.30237039, -0.30973644, -0.31744417, -0.32511109, -0.34176932, -0.37275   , -0.40945947, -0.45187034, -0.49350942, -0.53580563, -0.56163992, -0.57298405, -0.58214352, -0.59220405, -0.60369791, -0.61545112, -0.62278945, -0.63227295, -0.64061456, -0.64768978, -0.66520391, -0.63463502, -0.56305737, -0.49104526, -0.42054687, -0.34768972, -0.26943789, -0.18851722, -0.10757492, -0.02689482,  0.05339951,  0.09896776,  0.04372169, -0.00476671, -0.04757711, -0.08893901, -0.12967459, -0.16268797, -0.1939408 , -0.22184751, -0.24382364, -0.26510594};
float SS_servo3[] = {1.01696819, 0.98508002, 0.92300711, 0.84785324, 0.77336796, 0.70669778, 0.68477748, 0.72188321, 0.78657751, 0.87430087, 0.96282234, 1.06937811, 1.17418241, 1.25666703, 1.32722748, 1.39272053, 1.39620698, 1.33201592, 1.25722733, 1.18417972, 1.14498785, 1.12002505, 1.10768097, 1.11355263, 1.1273819 , 1.13592537, 1.1462216 , 1.15349021, 1.10875642, 1.02975118, 0.95688104, 0.89806674, 0.83098404, 0.76741746, 0.87407454, 0.96806986, 1.05343337, 1.13679322, 1.14908919, 1.11275976, 1.07403807, 1.02880664, 1.01166549, 1.02503452};
float SS_servo4[] = { 0.53426151,  0.54391818,  0.54738433,  0.5544234 ,  0.55870982,  0.55758502,  0.55564326,  0.5483021 ,  0.54676932,  0.54684742,  0.5476001 ,  0.54622734,  0.53496472,  0.51483154,  0.4887926 ,  0.46541572,  0.45129943,  0.450786  ,  0.4521396 ,  0.44634689,  0.42739197,  0.40677695,  0.37896394,  0.33897653,  0.28565295,  0.23460787,  0.1844512 ,  0.1361352 ,  0.08608485,  0.03791815, -0.01126297, -0.06523576, -0.10601681, -0.0655521 ,  0.01379432,  0.09483612,  0.17604741,  0.25692025,  0.32319579,  0.38466716,  0.45061478,  0.52604319,  0.5491906 ,  0.53563855};
float SS_servo5[] = {-0.64795027, -0.70740257, -0.78549926, -0.87975392, -0.97116   , -1.04968376, -1.10422689, -1.12175626, -1.13480536, -1.13690065, -1.13776581, -1.11567322, -1.04222491, -0.94863032, -0.85144901, -0.76567517, -0.75099359, -0.82815748, -0.91450399, -0.99418078, -1.01432887, -1.0121248 , -1.00598031, -0.99557521, -0.94590006, -0.90427155, -0.86720407, -0.83008758, -0.77607288, -0.71475668, -0.65253291, -0.58594158, -0.55280844, -0.60901193, -0.64454591, -0.67192451, -0.70428109, -0.76530027, -0.78281537, -0.75942816, -0.74037743, -0.73613043, -0.7101376 , -0.66938342};
float SS_servo6[] = { 0.18315367,  0.16835131,  0.15548174,  0.14116313,  0.12412513,  0.10733477,  0.0833295 ,  0.05667559,  0.0310964 ,  0.00155017, -0.03025643, -0.05661351, -0.02898314,  0.05054236,  0.13173672,  0.21299903,  0.28165982,  0.33883861,  0.39992039,  0.46925625,  0.48837566,  0.4857536 ,  0.48980991,  0.47392506,  0.49178327,  0.50374907,  0.51583322,  0.5285369 ,  0.53219485,  0.53154834,  0.53552773,  0.5458689 ,  0.55023046,  0.52267693,  0.46975416,  0.42030539,  0.37151807,  0.3257037 ,  0.30648904,  0.31099125,  0.31413885,  0.30928767,  0.27890542,  0.22741841};
float SS_servo7[] = {-0.63697438, -0.65156752, -0.70307405, -0.76380898, -0.82335241, -0.88145748, -0.89674498, -0.86690367, -0.81883661, -0.7460925 , -0.66579978, -0.5588317 , -0.51388741, -0.51409981, -0.51560437, -0.51605596, -0.50365704, -0.47865994, -0.45694169, -0.44914119, -0.42687782, -0.37822803, -0.32893136, -0.2056304 , -0.29849598, -0.38199818, -0.46726525, -0.55234914, -0.61138857, -0.65229545, -0.70227824, -0.77114605, -0.83298107, -0.80972229, -0.72684507, -0.64860474, -0.5680175 , -0.49096953, -0.51014886, -0.60371318, -0.69530775, -0.77455914, -0.77012856, -0.68990884};

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
