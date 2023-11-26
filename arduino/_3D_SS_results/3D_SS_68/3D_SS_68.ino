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
float SS_servo0[] = {-0.62209064, -0.59196747, -0.52673967, -0.46203314, -0.39062914, -0.31155222, -0.23111066, -0.15018136, -0.06921757,  0.01165105,  0.0657441 ,  0.02351671, -0.02487941, -0.07044006, -0.10412586, -0.12999312, -0.15679787, -0.18513809, -0.21365259, -0.24286005, -0.27270916, -0.30381082, -0.33348496, -0.36286795, -0.37592357, -0.37395077, -0.37170302, -0.36953875, -0.37723974, -0.39695292, -0.42403875, -0.4549627 , -0.48757607, -0.52420976, -0.55060021, -0.56849433, -0.58458968, -0.60080248, -0.61581948, -0.63046939, -0.64442808, -0.65604117, -0.66016441, -0.65100286, -0.63621561};
float SS_servo1[] = {0.65084663, 0.63123423, 0.64371231, 0.66199298, 0.67176061, 0.6733238 , 0.67345561, 0.67328296, 0.67271747, 0.67206385, 0.69901734, 0.8121466 , 0.91173855, 1.00125419, 1.08767913, 1.14974735, 1.15467396, 1.14292926, 1.1265511 , 1.09174973, 1.07114657, 1.08634629, 1.09692986, 1.09391999, 1.05373853, 0.98499581, 0.91591002, 0.85439023, 0.83219892, 0.85334634, 0.90394924, 0.96406768, 1.018723  , 1.07360217, 1.08772752, 1.07507746, 1.07108153, 1.05581428, 1.03538351, 1.01010486, 0.9777147 , 0.92884131, 0.86902062, 0.79876478, 0.71926476};      
float SS_servo2[] = {-0.3207203 , -0.31583484, -0.3037318 , -0.28767504, -0.27070485, -0.26208893, -0.2807248 , -0.31145643, -0.35011442, -0.39003153, -0.4312105 , -0.46478071, -0.49132371, -0.51390096, -0.54075667, -0.56057459, -0.57090705, -0.57787008, -0.58499859, -0.59278505, -0.60139464, -0.62124114, -0.63566462, -0.63205561, -0.60635748, -0.54260902, -0.48674167, -0.4165822 , -0.33740838, -0.25695512, -0.17602575, -0.09529742, -0.01462892,  0.04499181,  0.01715595, -0.02579298, -0.06609459, -0.10199135, -0.13744866, -0.17385564, -0.20414046, -0.23054786, -0.25820667, -0.28074254, -0.30619263};
float SS_servo3[] = {0.83618932, 0.77671732, 0.68033629, 0.57834789, 0.4936584 , 0.44555978, 0.46913961, 0.53006922, 0.61016043, 0.69092187, 0.77822392, 0.84226677, 0.87964389, 0.90513263, 0.95829337, 0.99032066, 0.965588  , 0.92158525, 0.87348433, 0.80908859, 0.75572131, 0.74865084, 0.73139819, 0.76489351, 0.73909006, 0.74793976, 0.77271952, 0.78315422, 0.77928737, 0.76705971, 0.76249472, 0.75736514, 0.75398193, 0.73330114, 0.76301489, 0.80687649, 0.86183433, 0.89046391, 0.91241734, 0.93251821, 0.93024002, 0.90348512, 0.88448435, 0.86826801, 0.86164897};      
float SS_servo4[] = {0.60273663, 0.6202169 , 0.62749502, 0.63666242, 0.6394068 , 0.63314036, 0.62789834, 0.62123955, 0.61886899, 0.62170287, 0.62634161, 0.62901497, 0.64691222, 0.66080019, 0.6536928 , 0.61958775, 0.59640944, 0.58748331, 0.57961025, 0.5759243 , 0.56695055, 0.54725518, 0.52599507, 0.50521637, 0.47369384, 0.43068534, 0.39625388, 0.36425423, 0.33220837, 0.29801073, 0.26514524, 0.2294749 , 0.19777911, 0.18916202, 0.22365468, 0.2842877 , 0.33033167, 0.40958518, 0.49123895, 0.57240159, 0.64526964, 0.66890794, 0.6516784 , 0.64861168, 0.62526229};      
float SS_servo5[] = {-0.39155727, -0.46658344, -0.54436024, -0.62404   , -0.67875208, -0.70066937, -0.70888847, -0.70178177, -0.69892801, -0.70528498, -0.71025259, -0.71915769, -0.76619554, -0.81082888, -0.8016069 , -0.74288195, -0.74114266, -0.77975517, -0.82456633, -0.89402408, -0.94717892, -0.95445163, -0.95940838, -0.97469257, -0.96941408, -0.93974322, -0.93018428, -0.92024815, -0.89179439, -0.83913408, -0.77374786, -0.70000863, -0.64301712, -0.6174817 , -0.64951989, -0.66351388, -0.71525883, -0.71833912, -0.71705934, -0.71618067, -0.70760155, -0.66680464, -0.6427875 , -0.57211126, -0.48185691};
float SS_servo6[] = {0.39637456, 0.3750873 , 0.34085903, 0.31301203, 0.28022847, 0.24770319, 0.21349209, 0.17570226, 0.13536022, 0.09141507, 0.05299275, 0.02322693, 0.01012581, 0.0118908 , 0.06232823, 0.14128975, 0.22059547, 0.2946141 , 0.36682581, 0.43185303, 0.49370794, 0.50243465, 0.50354482, 0.49332592, 0.49664443, 0.49964892, 0.51219335, 0.51993787, 0.5109044 , 0.48793405, 0.45667412, 0.42362053, 0.41125978, 0.43071854, 0.4455272 , 0.44057017, 0.40700419, 0.40529537, 0.40889341, 0.41894743, 0.43234067, 0.4404529 , 0.44387834, 0.42857499, 0.40930705};      
float SS_servo7[] = {-0.91939709, -0.92309786, -0.92443574, -0.93644601, -0.92244558, -0.8873075 , -0.82940375, -0.74863974, -0.65878334, -0.56299108, -0.44862333, -0.34854998, -0.30229941, -0.29250923, -0.35803725, -0.38150194, -0.38009264, -0.37095842, -0.3602275 , -0.34314796, -0.41402092, -0.37337596, -0.30736583, -0.21577285, -0.24057102, -0.3016745 , -0.38511725, -0.45418758, -0.47514266, -0.45248311, -0.39885008, -0.33853501, -0.32350204, -0.37662007, -0.44032852, -0.47523977, -0.43917942, -0.4775959 , -0.53051017, -0.59975064, -0.68203266, -0.7696871 , -0.84658984, -0.87437742, -0.89265592};

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
