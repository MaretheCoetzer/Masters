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

// normal travel = 16.5 cm over 3 gaits
// left right swopped travel = 17cm over 3 gaits, 11 deg turn to left
// front back swopped travel = 

// Insert servo angles here:
// For 3D gaits, these angles are obtained from linearisation.py
// For 2D gaits, these angles are obtained from 2D_trajectory_reader.py
float SS_servo0[] = {0.32277883, 0.2755763 , 0.21692904, 0.18634526, 0.18239162, 0.20756325, 0.25200672, 0.28786171, 0.31481241, 0.33629525, 0.35995506, 0.35657659, 0.34629368, 0.33808414, 0.31992628, 0.29642505, 0.27539715, 0.2576394 , 0.24181438, 0.23218506, 0.22213229, 0.203858  , 0.19829118, 0.21062881, 0.24360154, 0.28600253, 0.34012088, 0.39975041, 0.45965036, 0.51273783, 0.55935756, 0.60556751, 0.64915724, 0.68580263, 0.6989178 , 0.67701471, 0.6273424 , 0.55795745, 0.48158105, 0.41141905, 0.34613517};
float SS_servo1[] = {0.76923259, 0.7812891 , 0.83416281, 0.89739416, 0.9743512 , 1.00507156, 0.9323621 , 0.85669361, 0.78410595, 0.71428378, 0.64777226, 0.68557029, 0.75517144, 0.8076408 , 0.85265947, 0.87834606, 0.86902344, 0.83293202, 0.77895394, 0.70312619, 0.63378424, 0.58601181, 0.52845294, 0.47196567, 0.3963957 , 0.32508538, 0.24973743, 0.18436076, 0.13853291, 0.12135785, 0.13373309, 0.17075936, 0.23285175, 0.32070223, 0.42826746, 0.55222694, 0.66726093, 0.75589697, 0.80691159, 0.80839333, 0.77619278};
float SS_servo2[] = {-0.46448441, -0.49785806, -0.54378389, -0.58381651, -0.61667367, -0.64111882, -0.66153917, -0.68153877, -0.7043023 , -0.73009523, -0.76085356, -0.78605905, -0.80539082, -0.81747301, -0.82630791, -0.83210782, -0.83248166, -0.82834042, -0.8224335 , -0.80943071, -0.80890368, -0.8140316 , -0.79841238, -0.75414747, -0.68339209, -0.60144404, -0.52019161, -0.44150452, -0.36415145, -0.28673822, -0.21213502, -0.1493313 , -0.1037936 , -0.07446475, -0.09917619, -0.15730985, -0.21983582, -0.28453541, -0.34438494, -0.39804607, -0.44870547};
float SS_servo3[] = {1.48043628, 1.43603931, 1.38628453, 1.36052696, 1.36029145, 1.37577652, 1.38798447, 1.40251973, 1.42397943, 1.45161026, 1.49494115, 1.53403769, 1.57011045, 1.59394954, 1.5998385 , 1.58283185, 1.53794091, 1.47310588, 1.39540362, 1.3119367 , 1.3014415 , 1.35952766, 1.42325723, 1.48073306, 1.49804057, 1.46425489, 1.40085976, 1.3240152 , 1.25076918, 1.17964974, 1.13389279, 1.15627441, 1.22671838, 1.31578358, 1.43890179, 1.55291223, 1.62383605, 1.65160239, 1.63534942, 1.58355594, 1.5092895 };
float SS_servo4[] = { 0.19176424,  0.22569296,  0.2638546 ,  0.31350475,  0.34477078,  0.32491914,  0.30082353,  0.27359443,  0.24233721,  0.20656703,  0.17020304,  0.13892303,  0.11014601,  0.0813829 ,  0.04865212,  0.01121966, -0.02985608, -0.07392048, -0.11718396, -0.15587106, -0.18797584, -0.21480769, -0.23392911, -0.24671274, -0.257386  , -0.27429088, -0.29768444, -0.32992174, -0.37165507, -0.42450305, -0.45628059, -0.445854  , -0.41001714, -0.35162707, -0.28057801, -0.20848596, -0.13498973, -0.06227273,  0.01074136,  0.08861124,  0.16707284};
float SS_servo5[] = {-1.28640792, -1.38057729, -1.49031529, -1.58326739, -1.63035587, -1.6194653 , -1.59656453, -1.55777034, -1.50403576, -1.43575696, -1.3564148 , -1.28053244, -1.1981353 , -1.11290498, -1.0329027 , -0.96373762, -0.9094511 , -0.86425092, -0.83649683, -0.83090155, -0.84376028, -0.87553545, -0.9204127 , -0.9605359 , -0.99371018, -1.00431747, -0.98967257, -0.94443273, -0.86817861, -0.76184069, -0.71948889, -0.77817671, -0.83831516, -0.89393502, -0.94960391, -1.015024  , -1.09161448, -1.16416046, -1.20239797, -1.22672765, -1.26239028};
float SS_servo6[] = { 0.01581706, -0.02290972, -0.08104999, -0.12969664, -0.15144486, -0.14555477, -0.1687826 , -0.19987432, -0.23709918, -0.28234505, -0.30534394, -0.28202017, -0.23087608, -0.15779765, -0.07879161, -0.00213657,  0.0717757 ,  0.14651302,  0.22470215,  0.30437761,  0.38255007,  0.45909986,  0.52984321,  0.60008781,  0.64541227,  0.64347162,  0.6300267 ,  0.61162562,  0.58800154,  0.55871707,  0.5283528 ,  0.50185164,  0.47189844,  0.43652471,  0.39019046,  0.33419119,  0.27281294,  0.21124419,  0.14928336,  0.09299536,  0.03601035};
float SS_servo7[] = {-1.13056816, -1.16615514, -1.19492867, -1.21054932, -1.22165875, -1.22397958, -1.18880855, -1.12915192, -1.04811035, -0.9440226 , -0.90429646, -0.96969597, -1.03895622, -1.10725385, -1.1788936 , -1.25190416, -1.30613584, -1.32536363, -1.32979297, -1.33340503, -1.34785714, -1.41625277, -1.49650421, -1.5594309 , -1.60460742, -1.63628294, -1.64860348, -1.64772446, -1.63271299, -1.60436054, -1.56636606, -1.52044854, -1.44985958, -1.35711515, -1.25513848, -1.16508729, -1.09656405, -1.06047973, -1.04514416, -1.06714987, -1.11417552};

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
