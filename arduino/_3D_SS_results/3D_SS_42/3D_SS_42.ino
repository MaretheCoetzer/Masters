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
float SS_servo0[] = {-0.509379  , -0.52234202, -0.5232867 , -0.5232733 , -0.52242093, -0.49215298, -0.41408182, -0.3328258 , -0.25285925, -0.17314953, -0.13433909, -0.04199926,  0.07629147,  0.19458221,  0.304367  ,  0.37636625,  0.44836549,  0.44746242,  0.40655481,  0.36792521,  0.33123972,  0.30279246,  0.27334163,  0.24179285,  0.21138646,  0.17848425,  0.14487747,  0.1109022 ,  0.07059512,  0.02700117, -0.0248371 , -0.064752  , -0.09200754, -0.10794097, -0.12775176, -0.15367076, -0.18800498, -0.22173296, -0.26055975, -0.29645368, -0.34033001, -0.37632142, -0.4020512 , -0.42555846, -0.44906571, -0.47257297, -0.49136373, -0.50244755, -0.51353138, -0.52335198, -0.5231531 , -0.52124145, -0.51994404, -0.51797654, -0.51445253, -0.50897053, -0.50236049, -0.49721676, -0.48966878, -0.49171861};
float SS_servo1[] = {0.37543193, 0.42146526, 0.50078195, 0.58141967, 0.66127591, 0.71130836, 0.68719485, 0.63502172, 0.60285335, 0.54974288, 0.51392867, 0.46940165, 0.42082275, 0.37224386, 0.32661637, 0.29409976, 0.26158314, 0.29078491, 0.36072828, 0.43171208, 0.48572196, 0.50146638, 0.49544049, 0.48270869, 0.46578491, 0.47790052, 0.49418695, 0.51255873, 0.54366511, 0.57968585, 0.63523815, 0.69856275, 0.73059386, 0.71933515, 0.71293977, 0.72471893, 0.78185139, 0.84923937, 0.92071636, 0.97770027, 1.0534424 , 1.11788767, 1.14294059, 1.15946168, 1.17598276, 1.19250384, 1.18993576, 1.15617519, 1.12241462, 1.08486358, 1.01164819, 0.9347834 , 0.86372226, 0.79383643, 0.72127188, 0.63854357, 0.55422643, 0.47310657, 0.39572662, 0.3575632 };
float SS_servo2[] = { 0.01170489, -0.00315559, -0.01266734, -0.01549475, -0.02777728, -0.05434578, -0.08498334, -0.13836968, -0.18949015, -0.24048123, -0.26179222, -0.30207903, -0.35384397, -0.4056089 , -0.45302818, -0.481143  , -0.50925782, -0.51545626, -0.51444174, -0.51095629, -0.5144886 , -0.51788831, -0.51901556, -0.52112174, -0.52258076, -0.51991336, -0.51048727, -0.49740231, -0.50014033, -0.50656631, -0.5226923 , -0.51008921, -0.47226163, -0.45844328, -0.45798075, -0.45421571, -0.39883398, -0.3183532 , -0.2374905 , -0.15674652, -0.07398968,  0.0143581 ,  0.0958006 ,  0.17574751,  0.25569441,  0.33564132,  0.40406415,  0.45365616,  0.50324817,  0.54908809,  0.5098898 ,  0.44945845,  0.40119919,  0.35204574,  0.2998851 ,  0.24408139,  0.18482967,  0.11989067,  0.07039454,  0.02952298};
float SS_servo3[] = {0.4990355 , 0.49527185, 0.44436543, 0.3760064 , 0.33090105, 0.32809816, 0.34854351, 0.41833027, 0.47775663, 0.52777685, 0.51485955, 0.56281377, 0.64398706, 0.72516034, 0.80003366, 0.84692101, 0.89380837, 0.88862554, 0.86799535, 0.84571456, 0.82554105, 0.79111972, 0.7344684 , 0.67043379, 0.60366598, 0.54916935, 0.48243024, 0.39567129, 0.41645777, 0.48612693, 0.55697459, 0.67345223, 0.70132308, 0.69868769, 0.77750773, 0.85274262, 0.85351465, 0.84142205, 0.833351  , 0.78659381, 0.75724423, 0.73550954, 0.67111741, 0.59748627, 0.52385512, 0.45022398, 0.37174623, 0.28534893, 0.19895164, 0.10714906, 0.1130012 , 0.16217464, 0.19153453, 0.22466224, 0.26272357, 0.30028328, 0.34628066, 0.40679906, 0.44208526, 0.48212262};
float SS_servo4[] = { 0.46875822,  0.48762943,  0.49766933,  0.50671449,  0.51624082,  0.51933384,  0.51554046,  0.51669988,  0.5175057 ,  0.52289099,  0.52403404,  0.5162191 ,  0.50494046,  0.49366182,  0.47865441,  0.44708292,  0.41551143,  0.37973914,  0.35075376,  0.32592854,  0.31482939,  0.31595864,  0.32410927,  0.32755763,  0.32340911,  0.30271832,  0.27246925,  0.24601676,  0.22320573,  0.20081397,  0.17976975,  0.14678343,  0.10965115,  0.07601712,  0.0468374 ,  0.01430615, -0.0233396 , -0.06851449, -0.11805058, -0.16653271, -0.20124648, -0.21784001, -0.19656363, -0.16708515, -0.13760668, -0.1081282 , -0.06314424,  0.00717632,  0.07749687,  0.14939813,  0.22409769,  0.2930491 ,  0.36313731,  0.43655753,  0.50638378,  0.53337938,  0.48721254,  0.43672006,  0.46302413,  0.46607028};
float SS_servo5[] = {-0.21745086, -0.28598953, -0.36954583, -0.45516335, -0.54015879, -0.60324236, -0.64079912, -0.68507194, -0.73439989, -0.79851949, -0.85850731, -0.87705766, -0.87827791, -0.87949815, -0.87067936, -0.81726465, -0.76384994, -0.70922944, -0.66711228, -0.63014163, -0.63233263, -0.67552811, -0.75436302, -0.83594178, -0.90637014, -0.92121585, -0.91322648, -0.91284657, -0.92330748, -0.93777928, -0.9557051 , -0.90743449, -0.85209125, -0.82223816, -0.80871792, -0.78319021, -0.71729786, -0.6203376 , -0.52273002, -0.4372306 , -0.37717424, -0.33620081, -0.32720472, -0.32513447, -0.32306423, -0.32099399, -0.32323664, -0.33252674, -0.34181684, -0.34964596, -0.34424233, -0.33253024, -0.32096396, -0.31289781, -0.30193084, -0.25646286, -0.19166507, -0.20811763, -0.2580488 , -0.22404795};
float SS_servo6[] = { 0.1635305 ,  0.13073412,  0.08009026,  0.03503379, -0.00704604, -0.05461007, -0.10635904, -0.15990508, -0.21418784, -0.25604627, -0.23902576, -0.20020761, -0.15420172, -0.10819583, -0.05743425,  0.01445329,  0.08634083,  0.1648088 ,  0.24613461,  0.32709995,  0.40768266,  0.48426992,  0.52967421,  0.52891753,  0.50473442,  0.52472774,  0.52921645,  0.53038809,  0.53028545,  0.51681688,  0.50245278,  0.49581104,  0.47149905,  0.4893196 ,  0.50858925,  0.5254938 ,  0.50054405,  0.45603691,  0.43187265,  0.44601727,  0.43833527,  0.41706833,  0.40230504,  0.38895035,  0.37559566,  0.36224096,  0.33879602,  0.2988632 ,  0.25893037,  0.21611605,  0.19804424,  0.20033214,  0.20220832,  0.20635878,  0.21221796,  0.21546962,  0.21547188,  0.21128749,  0.20352957,  0.17839782};
float SS_servo7[] = {-0.68876583, -0.65429503, -0.61755304, -0.59792177, -0.58294997, -0.54282998, -0.47709544, -0.40526107, -0.34240998, -0.31337179, -0.40377003, -0.46392941, -0.50776023, -0.55159105, -0.58820497, -0.59275965, -0.59731433, -0.60550774, -0.60791387, -0.60801473, -0.60790756, -0.60390982, -0.5733458 , -0.52604214, -0.51965478, -0.4918335 , -0.4539438 , -0.41498733, -0.37310153, -0.24462092, -0.14036447, -0.143197  , -0.12397123, -0.20056359, -0.28178098, -0.35386532, -0.32210263, -0.24294873, -0.20884755, -0.25526001, -0.25669844, -0.22669069, -0.22228222, -0.2234182 , -0.22455418, -0.22569016, -0.21397812, -0.1812719 , -0.14856567, -0.1111962 , -0.13904614, -0.2103747 , -0.27727838, -0.34815337, -0.42525865, -0.50651655, -0.58243571, -0.65055763, -0.7044348 , -0.69855355};

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
