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
float SS_servo0[] = {-0.52438375, -0.50485096, -0.44560243, -0.38006224, -0.31240807, -0.24434817, -0.15003197, -0.05557821,  0.02861238,  0.09712866,  0.2036953 ,  0.327435  ,  0.43204604,  0.49730141,  0.47682622,  0.41494624,  0.34846164,  0.28969797,  0.22312645,  0.1508989 ,  0.08023856,  0.04569339, -0.01565434, -0.07952571, -0.11443329, -0.1308297 , -0.1473554 , -0.16624292, -0.19159023, -0.2314006 , -0.26509001, -0.29680254, -0.33978533, -0.39014585, -0.43588736, -0.4787335 , -0.52034749, -0.52070726, -0.51500371, -0.50715701, -0.50948071, -0.52019691, -0.51550219, -0.49670343, -0.4842644 , -0.49980594};
float SS_servo1[] = {0.23529086, 0.25363553, 0.32998707, 0.4115669 , 0.49269842, 0.58239142, 0.58525761, 0.5165474 , 0.43730337, 0.40943556, 0.39021933, 0.32739998, 0.2525642 , 0.17725917, 0.19410107, 0.27143499, 0.35736618, 0.42566354, 0.50625839, 0.59482283, 0.67278351, 0.66627532, 0.70568239, 0.73442454, 0.70098345, 0.65201312, 0.61492799, 0.58838652, 0.57867009, 0.58907431, 0.58005525, 0.56251676, 0.57683159, 0.6492142 , 0.72408895, 0.8005617 , 0.8558682 , 0.78066658, 0.69530803, 0.61203286, 0.54582066, 0.49554744, 0.4301508 , 0.34588658, 0.26427616, 0.23495878};
float SS_servo2[] = {-0.20757225, -0.22805673, -0.24244835, -0.26156889, -0.27741755, -0.29100154, -0.31606563, -0.34513945, -0.36871398, -0.41666837, -0.46412694, -0.49379422, -0.50915749, -0.52441851, -0.5212836 , -0.51838446, -0.51464033, -0.50745815, -0.50294598, -0.5062326 , -0.506662  , -0.50577745, -0.51867517, -0.52264732, -0.49193951, -0.42207844, -0.33319465, -0.25664029, -0.17977427, -0.0962894 , -0.0256951 ,  0.04471108,  0.11481372,  0.21981496,  0.30940259,  0.37257619,  0.39039702,  0.31030672,  0.23620132,  0.18802662,  0.11917265,  0.04782821, -0.00599744, -0.0441866 , -0.08344742, -0.13862081};
float SS_servo3[] = {0.74462655, 0.72104197, 0.66205356, 0.62216416, 0.58384095, 0.54991909, 0.55144071, 0.56669849, 0.57387142, 0.62678647, 0.70553533, 0.78876256, 0.85181746, 0.90863759, 0.87352345, 0.81747814, 0.75632238, 0.68634571, 0.61738279, 0.55962303, 0.49141181, 0.41675894, 0.38338634, 0.32254941, 0.34795968, 0.42542866, 0.51300622, 0.59974373, 0.65889204, 0.59735864, 0.53125408, 0.46495668, 0.43090004, 0.36480402, 0.28651562, 0.2062569 , 0.15950662, 0.24351757, 0.31883129, 0.3514034 , 0.42221775, 0.49562927, 0.54831152, 0.57553799, 0.59916866, 0.6586136 };
float SS_servo4[] = { 0.5067426 ,  0.52257945,  0.52028182,  0.52032974,  0.51808497,  0.52104627,  0.52250622,  0.52223208,  0.52201623,  0.52402173,  0.52134647,  0.51099819,  0.48971542,  0.46228307,  0.41814579,  0.36968777,  0.34187392,  0.32202268,  0.30420919,  0.28799121,  0.27490435,  0.27255681,  0.25835549,  0.20392461,  0.12443157,  0.04990911, -0.01849394, -0.08467868, -0.15132373, -0.2263599 , -0.30611116, -0.3913962 , -0.44998446, -0.40199164, -0.30744063, -0.17069104, -0.02492334,  0.06208417,  0.14402859,  0.22349553,  0.29286593,  0.34527934,  0.42737491,  0.51048255,  0.52928022,  0.50886522};
float SS_servo5[] = {-0.35174914, -0.44031643, -0.51741494, -0.59362454, -0.66070864, -0.73026783, -0.78616262, -0.83622525, -0.88505656, -0.93688027, -0.94333765, -0.89558029, -0.82388058, -0.74746506, -0.68781064, -0.63824965, -0.63025352, -0.64025411, -0.65980091, -0.68816966, -0.73078203, -0.80002529, -0.86109286, -0.85876381, -0.7965231 , -0.72156166, -0.64916109, -0.57687526, -0.50083795, -0.41836217, -0.33548866, -0.24846554, -0.20941527, -0.29891881, -0.38379503, -0.44751685, -0.50192235, -0.58491722, -0.66325391, -0.74352585, -0.80543632, -0.74635452, -0.65153782, -0.53101718, -0.43675788, -0.3581716 };
float SS_servo6[] = {-0.07746931, -0.11245628, -0.16230484, -0.19088745, -0.22303218, -0.26505034, -0.31830903, -0.3767188 , -0.43472517, -0.50013681, -0.4901588 , -0.39048775, -0.26187756, -0.10454766, -0.0017724 ,  0.07493597,  0.14821703,  0.21538022,  0.27373049,  0.3404228 ,  0.4113752 ,  0.47852138,  0.51710851,  0.52114046,  0.50106346,  0.49233877,  0.50106643,  0.51083555,  0.51935182,  0.51755289,  0.51407588,  0.5174819 ,  0.51658915,  0.4733459 ,  0.43188097,  0.39002269,  0.34344745,  0.29049303,  0.24763019,  0.216604  ,  0.18085768,  0.14087648,  0.11982782,  0.10192125,  0.05382476, -0.01134232};
float SS_servo7[] = {-0.4238384 , -0.41403769, -0.40023505, -0.42480532, -0.43522079, -0.4165013 , -0.36576257, -0.29124911, -0.21602092, -0.12859955, -0.15193963, -0.24444362, -0.32581044, -0.3844512 , -0.45495253, -0.53526917, -0.59308771, -0.60901217, -0.56005127, -0.48349779, -0.40339005, -0.31476939, -0.20857667, -0.23183587, -0.28429144, -0.34341469, -0.42408534, -0.50489738, -0.58244779, -0.6529245 , -0.72891447, -0.82297749, -0.90317225, -0.8522545 , -0.78814127, -0.71878132, -0.65006495, -0.61502155, -0.600401  , -0.60077618, -0.59495831, -0.58315233, -0.60001778, -0.61944569, -0.58132223, -0.50066723};

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
