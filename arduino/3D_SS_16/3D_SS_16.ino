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
float SS_servo0[] = {-0.51316214, -0.51897993, -0.510275  , -0.44528774, -0.37155904, -0.29135485, -0.19706992, -0.10278498,  0.00378049,  0.12385947,  0.24106328,  0.32343962,  0.40843893,  0.49365119,  0.57567958,  0.64726378,  0.71872154,  0.81051501,  0.84693397,  0.8439906 ,  0.84522097,  0.85874157,  0.87351778,  0.89331527,  0.9038306 ,  0.91294225,  0.92517094,  0.93739963,  0.9550462 ,  0.97475399,  0.97244888,  0.94492953,  0.92243635,  0.90433973,  0.88250628,  0.85287521,  0.81945515,  0.78370912,  0.75023634,  0.71474527,  0.7078452 ,  0.70541143,  0.70093168,  0.68116859,  0.65796212,  0.62728061,  0.59659909,  0.56591758,  0.53523607,  0.49759442,  0.44748368,  0.39737294,  0.3472622 ,  0.28679867,  0.2183469 ,  0.14777255,  0.0771982 ,  0.00426407, -0.07435726, -0.1529786 , -0.22711051, -0.28898838, -0.34987064, -0.40379051, -0.43741803, -0.46803365, -0.49974562, -0.5042074 };
float SS_servo1[] = {1.93167226, 1.89912154, 1.85864021, 1.79209581, 1.71042482, 1.62830386, 1.54865537, 1.46900688, 1.39356533, 1.3227531 , 1.25194714, 1.17318487, 1.09385566, 1.00759405, 0.91825932, 0.82261537, 0.72803709, 0.65682407, 0.62323667, 0.65558243, 0.65163806, 0.64185417, 0.60787803, 0.5783612 , 0.55145622, 0.52801263, 0.51654319, 0.50507375, 0.50929919, 0.51949569, 0.52061473, 0.4972785 , 0.46167831, 0.43615547, 0.42359211, 0.41856314, 0.4283387 , 0.44616411, 0.45549566, 0.46573101, 0.46204883, 0.49239258, 0.51407267, 0.54334076, 0.576862  , 0.61573281, 0.65460362, 0.69347443, 0.73234524, 0.78107083, 0.8474513 , 0.91383177, 0.98021224, 1.05525162, 1.12072721, 1.18173575, 1.2427443 , 1.29795964, 1.33921297, 1.38046631, 1.40909934, 1.44303913, 1.5209233 , 1.60685828, 1.6902899 , 1.76587337, 1.85415644, 1.90998392};
float SS_servo2[] = {-0.28973197, -0.30882053, -0.31653745, -0.3294975 , -0.33637561, -0.3426639 , -0.35240063, -0.36213735, -0.37598814, -0.39436606, -0.41377875, -0.43282747, -0.45544424, -0.47548965, -0.49374453, -0.50667633, -0.52021156, -0.52281782, -0.52343389, -0.50717104, -0.48417055, -0.45872282, -0.44307449, -0.41860142, -0.39967293, -0.38573788, -0.36954368, -0.35334947, -0.33497458, -0.31577006, -0.30032646, -0.30215108, -0.31665568, -0.30605665, -0.25335902, -0.20363307, -0.14631241, -0.08819746, -0.03025172,  0.0376708 ,  0.11137904,  0.18911909,  0.26885817,  0.34222034,  0.40750783,  0.45253591,  0.49756399,  0.54259207,  0.58762014,  0.61766364,  0.62086219,  0.62406074,  0.62725929,  0.57455883,  0.49340398,  0.40642843,  0.31945288,  0.23233327,  0.14486648,  0.05739969, -0.02183871, -0.09253885, -0.14696592, -0.19106339, -0.21583059, -0.2442906 , -0.26489301, -0.26599386};
float SS_servo3[] = {1.79073122, 1.79729876, 1.79251494, 1.77352316, 1.74252902, 1.71024858, 1.69669021, 1.68313184, 1.68783245, 1.71262527, 1.73633184, 1.73223085, 1.77443779, 1.82416702, 1.86628455, 1.8853567 , 1.90953752, 1.92823228, 1.90813099, 1.86436012, 1.79592058, 1.72720526, 1.68508157, 1.62683347, 1.56579335, 1.51958441, 1.48282474, 1.44606508, 1.42191195, 1.40255492, 1.36579099, 1.30516912, 1.2655143 , 1.23842908, 1.21072378, 1.17868712, 1.14971027, 1.11598213, 1.05183124, 0.96048994, 0.90603034, 0.92310041, 0.97574176, 0.9718105 , 0.95582526, 0.91764834, 0.87947142, 0.84129449, 0.80311757, 0.75576667, 0.69198055, 0.62819443, 0.56440832, 0.5804121 , 0.67615787, 0.79524681, 0.91433575, 1.02582726, 1.11900848, 1.2121897 , 1.27517502, 1.35849649, 1.44195359, 1.52475648, 1.60620056, 1.68573845, 1.72929923, 1.78322298};
float SS_servo4[] = {-0.6163669 , -0.6449712 , -0.64882342, -0.66311343, -0.68728793, -0.71473347, -0.74311866, -0.77150384, -0.80565207, -0.84614195, -0.8878689 , -0.92113694, -0.95304617, -0.9801511 , -1.00475031, -1.02320807, -1.03288702, -1.01260022, -1.0062517 , -1.00805919, -1.0175487 , -1.01909697, -1.02826086, -1.02721935, -1.04303661, -1.05686129, -1.05178925, -1.04671722, -1.01853279, -0.98155535, -0.96836502, -1.00288597, -1.03083596, -1.04890455, -1.07245011, -1.11028571, -1.14953381, -1.18904688, -1.23262197, -1.28858889, -1.32114153, -1.33812042, -1.34063907, -1.30288784, -1.2492803 , -1.16340164, -1.07752299, -0.99164434, -0.90576568, -0.80995049, -0.69633393, -0.58271737, -0.46910081, -0.37120049, -0.32029205, -0.28600315, -0.25171425, -0.24131817, -0.28850536, -0.33569255, -0.41393704, -0.49521574, -0.55791871, -0.57874045, -0.55499978, -0.54233893, -0.55710757, -0.56742075};
float SS_servo5[] = {-0.79480647, -0.77952372, -0.77865323, -0.75363132, -0.71910917, -0.68102748, -0.64012001, -0.59921253, -0.54663278, -0.48120884, -0.41972881, -0.41170527, -0.39398546, -0.37651115, -0.36508527, -0.37094714, -0.39129635, -0.45115492, -0.49847833, -0.48362418, -0.4283257 , -0.36924917, -0.29823138, -0.25456904, -0.19706018, -0.15303755, -0.14034372, -0.12764989, -0.15564389, -0.19911741, -0.2145214 , -0.19654489, -0.20852373, -0.22607911, -0.23526951, -0.22837542, -0.22248161, -0.22311379, -0.22336001, -0.20585777, -0.17974118, -0.11887609, -0.08995222, -0.18604475, -0.29445488, -0.40830208, -0.52214929, -0.63599649, -0.7498437 , -0.8460965 , -0.91082886, -0.97556122, -1.04029357, -1.08768718, -1.09525274, -1.08909997, -1.08294719, -1.0726952 , -1.05256384, -1.03243248, -0.979347  , -0.89482024, -0.86529912, -0.94036883, -0.88460677, -0.82506831, -0.83611242, -0.81835782};
float SS_servo6[] = {-0.58388404, -0.60287677, -0.61201465, -0.6328137 , -0.65108615, -0.67277385, -0.69612589, -0.71947793, -0.74577124, -0.77530112, -0.80534653, -0.83158675, -0.85383204, -0.87664473, -0.89973208, -0.92336107, -0.94364732, -0.94363433, -0.91435454, -0.82369589, -0.72568596, -0.63156765, -0.54803904, -0.45588351, -0.3780484 , -0.29088262, -0.18015314, -0.06942366,  0.07334476,  0.22830227,  0.36594785,  0.43426337,  0.4820008 ,  0.52091051,  0.52221131,  0.51430483,  0.50944625,  0.50470483,  0.49740844,  0.49400645,  0.49905101,  0.51801452,  0.52520109,  0.5150255 ,  0.49959575,  0.47025761,  0.44091947,  0.41158134,  0.3822432 ,  0.34457997,  0.29200229,  0.23942461,  0.18684693,  0.12861394,  0.06771402,  0.0063066 , -0.05510082, -0.11997353, -0.19319782, -0.26642212, -0.34451828, -0.41438529, -0.46588252, -0.50527723, -0.52593768, -0.54828655, -0.56944283, -0.57263716};
float SS_servo7[] = {-0.84024677, -0.87777946, -0.89539714, -0.86376262, -0.82387703, -0.77094233, -0.72248675, -0.67403117, -0.6243549 , -0.57333537, -0.52672383, -0.5160608 , -0.50019901, -0.46936702, -0.4367565 , -0.39936925, -0.36121795, -0.33424967, -0.39958855, -0.48626082, -0.56346068, -0.63999059, -0.72222105, -0.80160315, -0.87619905, -0.94467656, -1.00249948, -1.06032239, -1.09922772, -1.13093593, -1.16852018, -1.23416079, -1.31016046, -1.37055824, -1.42770013, -1.48267881, -1.54827493, -1.62025456, -1.69640203, -1.77607784, -1.82405628, -1.83937165, -1.83658904, -1.84282277, -1.84068922, -1.81048396, -1.7802787 , -1.75007343, -1.71986817, -1.67743279, -1.6130871 , -1.54874141, -1.48439572, -1.41616919, -1.36414718, -1.31870692, -1.27326666, -1.23422504, -1.2106046 , -1.18698415, -1.1513938 , -1.08790577, -1.03249917, -0.9916621 , -0.94017727, -0.8874735 , -0.86177988, -0.81554659};

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
