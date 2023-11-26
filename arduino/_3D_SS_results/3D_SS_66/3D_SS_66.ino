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
float SS_servo0[] = {-0.6553775 , -0.63381508, -0.57083755, -0.5078698 , -0.44450007, -0.38113034, -0.31603745, -0.24982825, -0.17695997, -0.09766893, -0.01695766,  0.06376772,  0.14464093,  0.22577298,  0.30468285,  0.31675042,  0.28329103,  0.27027417,  0.26352385,  0.2517933 ,  0.22203415,  0.17912455,  0.13102407,  0.08236002,  0.03362962, -0.00905534, -0.04730767, -0.08556   , -0.12361376, -0.15071626, -0.17781876, -0.20101563, -0.2236159 , -0.25221547, -0.28211492, -0.31504341, -0.35281609, -0.3913427 , -0.43577786, -0.47847643, -0.52734798, -0.56677071, -0.60312237, -0.63217366, -0.64473747, -0.64819701, -0.65266687, -0.6579743 , -0.66188621, -0.663303  , -0.66421441, -0.66423286, -0.66041278, -0.65689149, -0.65393506, -0.65426092};       
float SS_servo1[] = {0.63895139, 0.63602   , 0.64969253, 0.66696792, 0.69232808, 0.71768823, 0.73949724, 0.75798851, 0.76562572, 0.76682089, 0.75656509, 0.71573593, 0.70470694, 0.6906076 , 0.62635886, 0.67927143, 0.84795803, 0.96516028, 1.05744508, 1.13185576, 1.16324072, 1.16871291, 1.16579149, 1.16017414, 1.14579492, 1.12008855, 1.08876294, 1.05743732, 1.0260582 , 0.99172828, 0.95739835, 0.93093374, 0.92534892, 0.93442057, 0.94848351, 0.98286822, 1.05638134, 1.1312495 , 1.20952441, 1.26945731, 1.33833793, 1.38206722, 1.41795999, 1.43283591, 1.39380696, 1.3441914 , 1.30241528, 1.26136442, 1.20572959, 1.13173294, 1.05285786, 0.97331396, 0.88819841, 0.80319   , 0.71838417, 0.65040339};
float SS_servo2[] = {-0.08977472, -0.10204909, -0.11299457, -0.13362749, -0.15126357, -0.16889965, -0.19845105, -0.23395678, -0.28588584, -0.34636653, -0.41088242, -0.47493729, -0.53546963, -0.59194897, -0.64107413, -0.66269541, -0.66169124, -0.65605098, -0.65454799, -0.65565803, -0.65439703, -0.65168781, -0.65039062, -0.64449615, -0.64385002, -0.62820638, -0.59557766, -0.56294895, -0.52969826, -0.462144  , -0.39458975, -0.32468442, -0.27369782, -0.20053009, -0.12130441, -0.04104436,  0.03945627,  0.11999275,  0.20063808,  0.28102221,  0.35939746,  0.42190044,  0.47518797,  0.49324306,  0.41412544,  0.3576568 ,  0.31242797,  0.26983757,  0.22641953,  0.18366052,  0.13867248,  0.09492519,  0.05349813,  0.0106242 , -0.03498499, -0.07769809};       
float SS_servo3[] = {0.73977638, 0.69743302, 0.62812107, 0.57847433, 0.53017748, 0.48188064, 0.46108762, 0.45424028, 0.49525765, 0.57075421, 0.6670392 , 0.77369475, 0.88514769, 0.99119309, 1.08201855, 1.15077269, 1.19838496, 1.23240427, 1.2647032 , 1.27918377, 1.24036168, 1.17013957, 1.08898564, 0.99753838, 0.92090936, 0.85962965, 0.8073983 , 0.75516695, 0.70353481, 0.68495042, 0.66636602, 0.66457728, 0.69523858, 0.70266985, 0.70413986, 0.70249905, 0.68840959, 0.665529  , 0.64723977, 0.58304306, 0.54269297, 0.4900913 , 0.43174213, 0.39422064, 0.50856994, 0.5806193 , 0.63286143, 0.67739778, 0.70801048, 0.71600301, 0.7203952 , 0.72363868, 0.72251456, 0.72344399, 0.72825562, 0.7431462 };
float SS_servo4[] = { 6.51387616e-01,  6.62044933e-01,  6.61768428e-01,  6.59305588e-01,  6.58010336e-01,  6.56715083e-01,  6.52250000e-01,  6.46595179e-01,  6.40202697e-01,  6.35013716e-01,  6.28804975e-01,  6.28676687e-01,  6.31324567e-01,  6.33753861e-01,  6.36789927e-01,  6.36961535e-01,  6.26030921e-01,  6.06059949e-01,  5.81295991e-01,  5.54025260e-01,  5.27815560e-01,  5.00162057e-01,  4.79607164e-01,  4.50176715e-01,  4.05450179e-01,  3.58733749e-01,  3.12086826e-01,  2.65439903e-01,  2.18821799e-01,  1.73793103e-01,  1.28764407e-01,  8.45874074e-02,  4.00948145e-02, -2.16812259e-04, -4.07590914e-02, -7.91911638e-02, -1.15935528e-01, -1.56829190e-01, -2.03674156e-01, -2.59285197e-01, -3.05273533e-01, -3.01216865e-01, -2.75157814e-01, -2.12075148e-01, -1.32931559e-01, -5.19485953e-02,  2.92111735e-02,  1.10127474e-01,  1.87665096e-01,  2.59260914e-01,  3.25617913e-01,  3.91308297e-01,  4.59054607e-01,  5.28350949e-01,  6.00577584e-01,  6.45432388e-01};
float SS_servo5[] = {-0.69407511, -0.77098958, -0.85080316, -0.93008429, -1.00825652, -1.08642876, -1.16122567, -1.23416051, -1.29630703, -1.34517036, -1.37658014, -1.40628911, -1.42672715, -1.44247693, -1.45954242, -1.43189248, -1.35832877, -1.27171266, -1.18526592, -1.1142074 , -1.0940232 , -1.10476037, -1.14867802, -1.18008216, -1.18457856, -1.18521983, -1.18852888, -1.19183792, -1.19499345, -1.18968151, -1.18436957, -1.16234888, -1.11349626, -1.07675735, -1.03658311, -0.98546968, -0.90121614, -0.80702788, -0.71129363, -0.61210469, -0.5344591 , -0.52550836, -0.54329284, -0.60919123, -0.69977987, -0.74079268, -0.79341841, -0.81635396, -0.81339245, -0.80438639, -0.78929931, -0.77049   , -0.75309726, -0.73802409, -0.72733599, -0.68361997};       
float SS_servo6[] = { 0.22109166,  0.20058581,  0.16438502,  0.13111533,  0.09379893,  0.05648252,  0.01690836, -0.02247291, -0.05862143, -0.09180143, -0.12479023, -0.15300521, -0.18343794, -0.2154344 , -0.25018954, -0.25437686, -0.18734269, -0.10643529, -0.02557896,  0.0548245 ,  0.13298483,  0.2005801 ,  0.26194215,  0.31774988,  0.38618885,  0.44023822,  0.46781617,  0.49539413,  0.52256329,  0.52718647,  0.53180965,  0.55270309,  0.57813426,  0.59977967,  0.62148437,  0.63832054,  0.63378535,  0.63002139,  0.63605934,  0.65666804,  0.66942925,  0.65788693,  0.63432937,  0.5900219 ,  0.52865488,  0.4673627 ,  0.40617373,  0.34564527,  0.29980734,  0.28091416,  0.2741867 ,  0.27162094,  0.26851917,  0.2632214 ,  0.25377214,  0.23439309};       
float SS_servo7[] = {-0.84328656, -0.86891841, -0.88663976, -0.91790931, -0.93591218, -0.95391506, -0.9687112 , -0.98370474, -0.99032056, -0.97900014, -0.94361165, -0.90109037, -0.83164424, -0.75254101, -0.65954829, -0.58565835, -0.60698987, -0.61328105, -0.61376599, -0.61351394, -0.61099636, -0.59801383, -0.57863101, -0.55313012, -0.58162387, -0.58621926, -0.50388826, -0.42155726, -0.34017731, -0.31125008, -0.28232286, -0.33314449, -0.42621175, -0.51051377, -0.59273506, -0.65621767, -0.65770315, -0.66058602, -0.68836925, -0.75330682, -0.80750388, -0.81750985, -0.80494533, -0.75616002, -0.68946714, -0.61486697, -0.53519991, -0.45604451, -0.41701525, -0.44767933, -0.51165461, -0.58657488, -0.66109242, -0.73401156, -0.80390895, -0.84145772}; 

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
