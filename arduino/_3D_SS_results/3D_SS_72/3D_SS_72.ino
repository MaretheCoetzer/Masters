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
float SS_servo0[] = {-0.59798426, -0.58538585, -0.51742132, -0.44865079, -0.37882374, -0.30172246, -0.22139364, -0.14061868, -0.0593449 ,  0.0216598 ,  0.07833615,  0.03826703, -0.00689369, -0.04917828, -0.08130688, -0.10271258, -0.12648104, -0.15117085, -0.17590926, -0.20072939, -0.22713036, -0.25230996, -0.27587528, -0.29587087, -0.30446276, -0.30468437, -0.30317684, -0.2986667 , -0.29432566, -0.30176961, -0.32076326, -0.34537857, -0.36836153, -0.38332269, -0.39807764, -0.40514532, -0.40901325, -0.42192489, -0.44371563, -0.4692599 , -0.49253343, -0.51534923, -0.53703798, -0.55484483, -0.57195126, -0.58955399, -0.60310031, -0.60619007, -0.59648522};
float SS_servo1[] = {0.61126398, 0.62717862, 0.63819116, 0.65053322, 0.66148334, 0.66504235, 0.66530996, 0.6652157 , 0.66487305, 0.66395402, 0.68758303, 0.80231006, 0.9040471 , 0.99257116, 1.07771897, 1.13796684, 1.14134352, 1.12969875, 1.10904745, 1.07255723, 1.03833077, 1.02597761, 1.0146847 , 0.98645446, 0.93760007, 0.86338664, 0.78534026, 0.70591354, 0.64026821, 0.61518155, 0.62175089, 0.64963084, 0.67132642, 0.67821733, 0.69085069, 0.67752649, 0.65070433, 0.64831824, 0.67847751, 0.72585792, 0.75286266, 0.77481679, 0.78870702, 0.77482305, 0.75612058, 0.73390426, 0.71827235, 0.68516129, 0.62119939};
float SS_servo2[] = {-0.41159437, -0.41772071, -0.40702517, -0.39467277, -0.38132171, -0.37421387, -0.3945041 , -0.4302392 , -0.47122878, -0.51399149, -0.55518855, -0.58383781, -0.60552734, -0.62532382, -0.64830579, -0.66200692, -0.66669699, -0.66590529, -0.66481298, -0.66324303, -0.66243723, -0.6595327 , -0.64987893, -0.63661635, -0.61799082, -0.59589429, -0.56786995, -0.53839337, -0.48923429, -0.40897795, -0.32788343, -0.23864317, -0.18637173, -0.17557745, -0.13242049, -0.14537117, -0.16211983, -0.17817152, -0.20032538, -0.22578228, -0.24513056, -0.26413525, -0.28242273, -0.29823992, -0.3146982 , -0.33860417, -0.36005017, -0.3813959 , -0.4033799 };
float SS_servo3[] = {0.80718058, 0.79778671, 0.71864762, 0.63111739, 0.54747776, 0.49551993, 0.51544771, 0.58036078, 0.66220231, 0.74818055, 0.83587466, 0.89231799, 0.92612597, 0.94991077, 0.99386745, 1.0179647 , 0.98544544, 0.93241954, 0.87164301, 0.79732268, 0.72192456, 0.66353174, 0.59339432, 0.5994336 , 0.5879605 , 0.64400473, 0.69673087, 0.74793536, 0.75794991, 0.72253099, 0.70647204, 0.72759015, 0.73286195, 0.73894954, 0.72567967, 0.72456426, 0.72218576, 0.72389478, 0.75703094, 0.80842764, 0.83033715, 0.84729052, 0.85614288, 0.83709322, 0.81538424, 0.80629102, 0.80446191, 0.80641985, 0.8003825 };
float SS_servo4[] = {0.60760394, 0.59685014, 0.60315195, 0.61168306, 0.62063176, 0.62329973, 0.6194707 , 0.61162381, 0.60714105, 0.60563629, 0.6002382 , 0.60061613, 0.61166762, 0.63093879, 0.63106614, 0.60124038, 0.58769596, 0.58915439, 0.59265358, 0.59976   , 0.60434546, 0.59365802, 0.57512152, 0.55487085, 0.51983501, 0.47228725, 0.4273459 , 0.38667172, 0.3462445 , 0.30065913, 0.25609704, 0.21527035, 0.17006828, 0.12951991, 0.10943115, 0.11018872, 0.12218   , 0.14187906, 0.19123311, 0.26919289, 0.34849566, 0.42729976, 0.50618086, 0.58532713, 0.65526787, 0.65424013, 0.66345022, 0.65715448, 0.62391493};
float SS_servo5[] = {-0.34765088, -0.34765165, -0.41457846, -0.49180669, -0.56577497, -0.61190912, -0.62937154, -0.62447427, -0.61746413, -0.61262935, -0.59407994, -0.59397099, -0.6214392 , -0.67035206, -0.67261797, -0.61602852, -0.62822224, -0.67933512, -0.74120573, -0.82207268, -0.90433867, -0.93928363, -0.95634196, -0.97720738, -0.96686348, -0.94022555, -0.92124691, -0.90827401, -0.88438778, -0.83188637, -0.77232693, -0.70924144, -0.63923831, -0.57732087, -0.52123796, -0.51662477, -0.54669522, -0.58047343, -0.66092471, -0.68985504, -0.7002496 , -0.70841744, -0.71490373, -0.71559833, -0.71204001, -0.66971285, -0.61881786, -0.54047305, -0.39479438};
float SS_servo6[] = {0.42512523, 0.39939149, 0.36601073, 0.33667516, 0.30651683, 0.27818012, 0.24854089, 0.21409112, 0.17891074, 0.14074139, 0.10696012, 0.07740323, 0.06009377, 0.06579382, 0.11530622, 0.1929971 , 0.2712372 , 0.34985063, 0.42888944, 0.50865101, 0.55153182, 0.57605386, 0.57907971, 0.55659713, 0.56457589, 0.57360862, 0.58334678, 0.5938895 , 0.59677555, 0.59068466, 0.58366638, 0.57663603, 0.56187265, 0.54206203, 0.52213535, 0.50555561, 0.50483259, 0.50356127, 0.48119174, 0.4488294 , 0.43475825, 0.42530737, 0.42017041, 0.42989169, 0.4414623 , 0.45437224, 0.45456738, 0.44242766, 0.43212514};
float SS_servo7[] = {-0.98111005, -0.95088071, -0.94322391, -0.95114739, -0.95229961, -0.93569826, -0.89533735, -0.82653235, -0.74748779, -0.65961562, -0.55070745, -0.45103593, -0.38655663, -0.37059253, -0.43741977, -0.46758519, -0.46454056, -0.45818015, -0.45290421, -0.44980461, -0.48139727, -0.47773146, -0.42525457, -0.30275063, -0.35819125, -0.44041851, -0.52453906, -0.60896397, -0.66986194, -0.6968722 , -0.71520704, -0.72539575, -0.7215146 , -0.70739243, -0.68458687, -0.67969035, -0.7121282 , -0.74002751, -0.71249827, -0.65553644, -0.64539585, -0.64742537, -0.66398069, -0.73056674, -0.80720906, -0.89115097, -0.93663891, -0.95438721, -0.98540408};

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
