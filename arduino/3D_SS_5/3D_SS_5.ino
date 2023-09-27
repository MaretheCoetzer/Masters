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
float SS_servo0[] = {-0.50995656, -0.49349666, -0.43581509, -0.36089668, -0.28287344, -0.197465  , -0.10331767, -0.01636785,  0.06053985,  0.11159735,  0.1399592 ,  0.13684691,  0.08452413,  0.03647158, -0.00169669, -0.03591819, -0.06966407, -0.10234688, -0.13547898, -0.16704366, -0.19193181, -0.21500304, -0.23542445, -0.25857578, -0.28538076, -0.31375723, -0.34180047, -0.36857546, -0.39238331, -0.41448747, -0.43726327, -0.45767191, -0.47377103, -0.48479291, -0.49050112, -0.49477376, -0.49957327, -0.50585003, -0.51025413, -0.51028757};
float SS_servo1[] = {0.580561  , 0.5878297 , 0.65428052, 0.73063601, 0.80914105, 0.86034723, 0.84586767, 0.79585886, 0.73411851, 0.67156687, 0.59871194, 0.58601832, 0.67296932, 0.75120929, 0.80364883, 0.8351525 , 0.85175891, 0.85402653, 0.84132094, 0.81343463, 0.77504042, 0.75206246, 0.74488723, 0.75343899, 0.77350086, 0.79793629, 0.81856258, 0.82643212, 0.81964662, 0.81477383, 0.81678418, 0.81118134, 0.79629899, 0.77946408, 0.75276734, 0.72492389, 0.69854034, 0.67303592, 0.64205483, 0.5961252 };
float SS_servo2[] = {-0.24819628, -0.26070375, -0.27366699, -0.28476454, -0.30089675, -0.3212234 , -0.34354969, -0.36759421, -0.39579105, -0.42504921, -0.450111  , -0.47453915, -0.49338856, -0.50121871, -0.50673603, -0.50772552, -0.50854553, -0.50536945, -0.50116096, -0.50113516, -0.48826392, -0.44433173, -0.37397172, -0.29855119, -0.22110973, -0.13634897, -0.048381  ,  0.00540209,  0.02276571,  0.04022482,  0.04930205,  0.02659632, -0.02656045, -0.07023993, -0.10423821, -0.1344666 , -0.16215786, -0.18857971, -0.21412619, -0.23956658};
float SS_servo3[] = {0.68751323, 0.67506363, 0.65469681, 0.63696777, 0.63375096, 0.64197508, 0.65450818, 0.6689857 , 0.6903461 , 0.71751054, 0.74448074, 0.76872514, 0.78006158, 0.7708367 , 0.75445015, 0.7213017 , 0.67794714, 0.61842549, 0.54216819, 0.46242734, 0.42763784, 0.48768963, 0.56360072, 0.6383186 , 0.71099502, 0.75302758, 0.73427144, 0.68222311, 0.60708051, 0.53637902, 0.48174833, 0.48516849, 0.54905879, 0.59932837, 0.62945137, 0.65244746, 0.66944504, 0.68104607, 0.68811055, 0.68952295};
float SS_servo4[] = { 0.49869339,  0.50506329,  0.51146485,  0.5110567 ,  0.51169806,  0.51147248,  0.50985642,  0.50679917,  0.49610393,  0.47440102,  0.44684086,  0.41372858,  0.38081079,  0.3512164 ,  0.32911467,  0.31490142,  0.30634626,  0.29705411,  0.28038385,  0.25611286,  0.2271344 ,  0.20381446,  0.18472821,  0.16420227,  0.14052944,  0.11397666,  0.08253377,  0.0398675 , -0.01933963, -0.05455041, -0.04113406, -0.01605332,  0.03201897,  0.10759035,  0.17899523,  0.25034882,  0.32678715,  0.40437077,  0.47303522,  0.49783638};
float SS_servo5[] = {-0.48314076, -0.53175918, -0.58627829, -0.62200914, -0.65596217, -0.68550547, -0.71240581, -0.73835857, -0.75102214, -0.74095736, -0.70995652, -0.66663644, -0.62657663, -0.59230304, -0.57504438, -0.58182283, -0.61094739, -0.64937516, -0.68805774, -0.72483248, -0.75081768, -0.77405673, -0.7872728 , -0.78633328, -0.77367364, -0.75291651, -0.72399649, -0.68059489, -0.6113618 , -0.58674983, -0.6549679 , -0.74180408, -0.83050117, -0.88106316, -0.88023544, -0.82877702, -0.75549981, -0.67907601, -0.58851038, -0.48010575};
float SS_servo6[] = { 0.26251084,  0.24795143,  0.226346  ,  0.19606247,  0.16749461,  0.13706822,  0.10475177,  0.06659821,  0.01805926, -0.00411819,  0.02926884,  0.08421869,  0.15394552,  0.22962035,  0.31456113,  0.3936179 ,  0.45848528,  0.50475468,  0.51512666,  0.49894437,  0.46755386,  0.44663133,  0.45736622,  0.47295796,  0.48442627,  0.48974685,  0.49139645,  0.49263375,  0.489018  ,  0.47394337,  0.45527188,  0.43560049,  0.41330969,  0.38858656,  0.3623431 ,  0.33784957,  0.31768852,  0.30252713,  0.28895852,  0.26997117};
float SS_servo7[] = {-0.67368111, -0.68078659, -0.68347781, -0.66241132, -0.64028162, -0.61091785, -0.57690289, -0.53203375, -0.46660774, -0.44937677, -0.53118063, -0.61117249, -0.62716509, -0.58537609, -0.53773806, -0.4853574 , -0.40876017, -0.31047843, -0.24856937, -0.27747791, -0.28259934, -0.30073428, -0.36313007, -0.42635358, -0.47786832, -0.51635756, -0.55052473, -0.59431814, -0.63838669, -0.65630208, -0.66219864, -0.66751844, -0.66919177, -0.65945686, -0.64583646, -0.63420699, -0.63126359, -0.64074129, -0.65660169, -0.66925219};

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
        servo2=map(servo2_deg,-37,90,1500,2670);
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
        servo2=map(servo2_deg,-37,90,1500,2670);
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
