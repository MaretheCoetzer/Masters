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
float SS_servo0[] = {-0.44712459, -0.45271511, -0.43990077, -0.37687859, -0.29982476, -0.22151308, -0.15238036, -0.11362582, -0.08395613, -0.0590196 , -0.05884561, -0.09176707, -0.12484843, -0.14995616, -0.17012292, -0.18777002, -0.20192335, -0.21289319, -0.22270221, -0.23272015, -0.24112881, -0.2474658 , -0.25675821, -0.26748549, -0.28198002, -0.29605859, -0.30969699, -0.32466185, -0.34145789, -0.35870059, -0.37602773, -0.38991799, -0.40048528, -0.40930064, -0.41610862, -0.42409823, -0.42923778, -0.43645213, -0.44245514, -0.4451758 };
float SS_servo1[] = {0.79205686, 0.7756421 , 0.78513399, 0.85299174, 0.93546096, 0.97632007, 0.91812817, 0.84483413, 0.76908653, 0.69775435, 0.66473661, 0.69591814, 0.73444948, 0.76294075, 0.7836815 , 0.80186263, 0.82064474, 0.83516132, 0.84205572, 0.8343278 , 0.81078065, 0.78772482, 0.7841614 , 0.79159044, 0.81088283, 0.82430626, 0.82644656, 0.83228625, 0.84069453, 0.84726855, 0.85018465, 0.84564366, 0.83966983, 0.83400158, 0.82385163, 0.81830407, 0.81765892, 0.82258292, 0.82122325, 0.80114793};
float SS_servo2[] = {-0.28171111, -0.29033983, -0.29993595, -0.30423079, -0.31172777, -0.32087903, -0.33101658, -0.34525464, -0.36263486, -0.38159031, -0.40296372, -0.42388681, -0.4392079 , -0.4507401 , -0.45966762, -0.46371151, -0.46563799, -0.46231483, -0.45999149, -0.46063757, -0.45853417, -0.44179538, -0.38993701, -0.31840134, -0.24616558, -0.19204672, -0.16035615, -0.12865876, -0.10280095, -0.09643197, -0.12287957, -0.16004667, -0.18839279, -0.21007645, -0.22826337, -0.24444208, -0.25378333, -0.26180154, -0.26925346, -0.27685834};
float SS_servo3[] = {0.772218  , 0.76105686, 0.7437662 , 0.732441  , 0.73531385, 0.74267586, 0.74608245, 0.75187164, 0.76089647, 0.77020782, 0.77828891, 0.78293129, 0.78358027, 0.78258115, 0.77884357, 0.76908214, 0.76323417, 0.75049757, 0.73456098, 0.70948483, 0.66811238, 0.65960203, 0.71851467, 0.78773012, 0.85781893, 0.8802925 , 0.80793808, 0.73442989, 0.66579304, 0.62957536, 0.65378485, 0.69872557, 0.73028137, 0.75173799, 0.76605869, 0.77839809, 0.78533425, 0.78913791, 0.78763791, 0.77558371};
float SS_servo4[] = {0.48385318, 0.48673715, 0.48726809, 0.48406514, 0.48508212, 0.48091895, 0.47681717, 0.46833241, 0.45894816, 0.44739596, 0.43174389, 0.41251023, 0.39393784, 0.37662027, 0.36194446, 0.34821983, 0.33607223, 0.32749082, 0.32342972, 0.31575945, 0.30607733, 0.29734238, 0.29127494, 0.28646155, 0.28214034, 0.27138763, 0.25749552, 0.23995415, 0.21964266, 0.19460551, 0.15998054, 0.1251494 , 0.1274868 , 0.15632884, 0.19368396, 0.24156875, 0.30349743, 0.37977067, 0.44722659, 0.48088731};
float SS_servo5[] = {-0.67788621, -0.71053078, -0.747416  , -0.75929237, -0.7709664 , -0.77247613, -0.78061479, -0.7867024 , -0.79318642, -0.79740312, -0.79973686, -0.79900867, -0.79363232, -0.78430995, -0.77696377, -0.76752387, -0.75373854, -0.74504816, -0.75001093, -0.76135738, -0.78103721, -0.79786627, -0.80827821, -0.81393046, -0.81631421, -0.80954451, -0.80617941, -0.79392743, -0.77654317, -0.75185184, -0.71138457, -0.67120379, -0.70384551, -0.77944799, -0.86736236, -0.91917547, -0.86729788, -0.79374426, -0.71217053, -0.66688805};
float SS_servo6[] = {0.25005908, 0.24810479, 0.24214569, 0.22974955, 0.2169628 , 0.20103505, 0.18504159, 0.16647476, 0.14407478, 0.11839704, 0.08535585, 0.06127465, 0.07560347, 0.10456466, 0.14211622, 0.18858928, 0.24744258, 0.3245344 , 0.39006082, 0.41874737, 0.42086375, 0.42208639, 0.41830263, 0.41336045, 0.40860607, 0.40299654, 0.40189232, 0.39685102, 0.39155917, 0.38479999, 0.37480586, 0.35987124, 0.34192158, 0.32570816, 0.31304579, 0.29772557, 0.28405609, 0.26966283, 0.26137658, 0.25167363};
float SS_servo7[] = {-0.69057867, -0.71534449, -0.74038509, -0.73556016, -0.72161122, -0.70012115, -0.68442737, -0.6686389 , -0.64735163, -0.62173518, -0.58685866, -0.57330753, -0.63186364, -0.7073123 , -0.79382297, -0.84297206, -0.78879663, -0.71175753, -0.62317871, -0.57024849, -0.60468604, -0.63937057, -0.65151341, -0.65413019, -0.65341489, -0.65676956, -0.67943035, -0.6925813 , -0.70556897, -0.7182872 , -0.72925313, -0.73188248, -0.72450435, -0.71661404, -0.71506714, -0.70532571, -0.68984272, -0.67261773, -0.67198904, -0.68012642};

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
