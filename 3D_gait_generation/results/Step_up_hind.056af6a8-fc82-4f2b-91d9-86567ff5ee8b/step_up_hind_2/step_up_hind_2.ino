//Gait template
//FIRST STEP OF LEG 1 MODIFIED
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

float SS_servo0[] = { 0.61750753,  0.55129978,  0.39650269,  0.29070554,  0.20680457,  0.13510698,  0.07346175,  0.02084582, -0.02429755, -0.0632518 , -0.09466622, -0.12002249, -0.14254266, -0.16157121, -0.17754231, -0.1902858 , -0.19891233, -0.20416584, -0.20721832, -0.20371508, -0.19679382, -0.18551144, -0.17353702, -0.15995169, -0.1458606 , -0.13170225, -0.11810836, -0.10659043, -0.09981465, -0.09674693, -0.0960338 , -0.09488208, -0.08954113, -0.08085842, -0.06974981, -0.05689964, -0.03953606, -0.01642004,  0.01474453,  0.05892179};
float SS_servo1[] = {1.05960513, 1.01100843, 1.08285289, 1.09563111, 1.06795914, 1.0213883 , 0.96624207, 0.9093993 , 0.85700472, 0.81434791, 0.78360419, 0.76321044, 0.74784915, 0.73198251, 0.71329093, 0.69010834, 0.66230945, 0.63109257, 0.60193582, 0.5649649 , 0.51528721, 0.47076737, 0.43672879, 0.41123774, 0.39423848, 0.38512244, 0.38388214, 0.39214336, 0.41533486, 0.45681218, 0.5151619 , 0.5795087 , 0.63919465, 0.69210658, 0.73723387, 0.77024271, 0.78793849, 0.79129679, 0.77342341, 0.72955539};
float SS_servo2[] = { 0.50010685,  0.31458348,  0.1681266 ,  0.07572738, -0.0032864 , -0.073666  , -0.13565167, -0.18935045, -0.2352524 , -0.2736895 , -0.30433875, -0.32878039, -0.34923563, -0.36707556, -0.38285731, -0.39635108, -0.40788894, -0.41543377, -0.41994591, -0.42508302, -0.42191051, -0.42030145, -0.41342254, -0.40311288, -0.3903602 , -0.37585923, -0.36041339, -0.34569398, -0.33173571, -0.31530688, -0.29979524, -0.28610909, -0.26931117, -0.25113804, -0.23144745, -0.21120859, -0.18784072, -0.15981627, -0.12351352, -0.07621443};
float SS_servo3[] = {0.86871047, 1.02091138, 1.10465087, 1.09859486, 1.06363097, 1.01538326, 0.96103604, 0.90576108, 0.85401138, 0.8096554 , 0.77670721, 0.75393985, 0.73437177, 0.7154627 , 0.69514172, 0.67179451, 0.64671428, 0.61741285, 0.58746237, 0.56065416, 0.51816606, 0.48738904, 0.46047096, 0.43963028, 0.42487539, 0.41605431, 0.41345012, 0.4191515 , 0.43561175, 0.46208939, 0.50184094, 0.54893853, 0.59286472, 0.63267093, 0.66640155, 0.69057786, 0.7021659 , 0.70255054, 0.68263638, 0.6389874 };
float SS_servo4[] = { 0.39314502,  0.44727786,  0.4521372 ,  0.42613923,  0.3831469 ,  0.32932951,  0.26941787,  0.20684636,  0.14390841,  0.08360627,  0.03224255, -0.01084619, -0.04867918, -0.08217795, -0.11190451, -0.13791754, -0.15960567, -0.17973341, -0.19931819, -0.2165758 , -0.23455197, -0.24668755, -0.2537208 , -0.2545536 , -0.25071666, -0.24306502, -0.2334039 , -0.22333489, -0.21660054, -0.20869503, -0.15435003, -0.07484438,  0.00880695,  0.09311907,  0.1766078 ,  0.25256607,  0.32496524,  0.39919331,  0.47310193,  0.52506464};
float SS_servo5[] = {-0.47268261, -0.62420733, -0.75527725, -0.85861833, -0.94806502, -1.02481882, -1.08814138, -1.13658807, -1.16755326, -1.17989913, -1.17933952, -1.16825983, -1.15330015, -1.13948306, -1.13022292, -1.1276217 , -1.13207962, -1.13855774, -1.14012028, -1.1472778 , -1.16792092, -1.18463426, -1.19335416, -1.19680236, -1.19390986, -1.18482745, -1.16785944, -1.14075402, -1.09587345, -1.03313212, -1.02235655, -1.02210445, -1.01823276, -1.0137475 , -1.00798426, -1.01338848, -1.00758095, -0.92391173, -0.77025546, -0.60925955};
float SS_servo6[] = {0.3921652 , 0.46325307, 0.47852169, 0.46193631, 0.42954736, 0.38608419, 0.33543829, 0.28125379, 0.22515232, 0.17495464, 0.16303382, 0.17785408, 0.20191146, 0.23284545, 0.26787884, 0.30786969, 0.36199699, 0.41909578, 0.47557915, 0.5232515 , 0.50998985, 0.49604825, 0.48168904, 0.47415287, 0.47047847, 0.47080807, 0.47390046, 0.47868827, 0.48323954, 0.4874313 , 0.4912473 , 0.49248137, 0.49042231, 0.48561906, 0.47872356, 0.46882127, 0.45948036, 0.45033862, 0.43561077, 0.42300124};
float SS_servo7[] = {-0.37663806, -0.53877239, -0.67348755, -0.78179646, -0.87934619, -0.96607   , -1.04017156, -1.09986989, -1.14103225, -1.16851875, -1.22866375, -1.29019042, -1.34448178, -1.39171009, -1.43481793, -1.47399392, -1.4945751 , -1.459735  , -1.33329631, -1.21964201, -1.24126293, -1.26906593, -1.28167559, -1.28714311, -1.28286432, -1.270776  , -1.25041287, -1.2211127 , -1.17766354, -1.11612762, -1.03970178, -0.95749458, -0.87585646, -0.79892545, -0.72914874, -0.66909784, -0.61945287, -0.57596993, -0.53411895, -0.50266536};

int node=0;
int first=0;
int input=2;
int end_time=0;
int time_step=80000; // WAS 200 000
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);
int first_print=1;

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
  delay (500);

        servo0_deg=SS_servo0[0]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[0]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[0]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[0]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[0]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[0]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[0]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[0]/3.14159265*180-servo6_deg;
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
  
}

void loop() 
{
  if(Serial.available()>0)
  {
    first=Serial.read();
    if(first=='1')
    {
      input=1;
    }
    if(first=='0')
    {
      input=0;
    }
  }
  if(input==1)
  {
      if(esp_timer_get_time()>=end_time)
      {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
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
        Serial.print(node);
        Serial.print("       SS_servo0:");
        Serial.println(SS_servo3[node]);
        node+=1;
        if(node>=steps)
           {
              input=0;
              node=steps-1;
            }
         end_time=esp_timer_get_time()+time_step;
      }
   }

  if(input == 0)
  {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
        servo7=map(servo7_deg,-90,90,920,2620);
        
        pca9685.setPWM(SER0,0,servo0);
        pca9685.setPWM(SER1,0,servo1);
        pca9685.setPWM(SER2,0,servo2);
        pca9685.setPWM(SER3,0,servo3);
        pca9685.setPWM(SER4,0,servo4);
        pca9685.setPWM(SER5,0,servo5);
        pca9685.setPWM(SER6,0,servo6);
        pca9685.setPWM(SER7,0,servo7);

        if(first_print==1)
        {
        first_print=0;
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
        }
  
  }
}
