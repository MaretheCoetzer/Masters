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
float SS_servo0[] = {-0.52090986, -0.50515027, -0.44804184, -0.35086698, -0.25511105, -0.14288936, -0.03600569,  0.05511505,  0.16217765,  0.2786273 ,  0.37192921,  0.38003579,  0.34657974,  0.33670322,  0.33537185,  0.33145133,  0.33951616,  0.34749301,  0.35192255,  0.35275503,  0.34501511,  0.32453703,  0.29302599,  0.26187   ,  0.23162875,  0.20056416,  0.15142184,  0.07911622,  0.01734218, -0.03217298, -0.1008027 , -0.16760276, -0.21599356, -0.24906886, -0.27226297, -0.31911863, -0.35822261, -0.4100341 , -0.45406274, -0.48426033, -0.51563363, -0.52360494};
float SS_servo1[] = {0.8634198 , 0.85687548, 0.90162232, 0.95258145, 1.01795477, 1.0215846 , 0.96276625, 0.93025336, 0.89733021, 0.84883168, 0.78583485, 0.7855873 , 0.90411948, 0.95946923, 0.98000295, 0.96976059, 0.90398066, 0.83205742, 0.76113896, 0.69355099, 0.64611492, 0.64642399, 0.67929292, 0.70624103, 0.72758966, 0.74018333, 0.76703492, 0.84570572, 0.89255784, 0.9073253 , 0.95827421, 0.97089628, 0.95890186, 0.92475715, 0.87444897, 0.87713895, 0.87503386, 0.90077951, 0.9192938 , 0.9196148 , 0.92076127, 0.87480449};
float SS_servo2[] = {-0.44338656, -0.43514865, -0.42889046, -0.43089076, -0.43818888, -0.4441114 , -0.45144002, -0.46883099, -0.48247989, -0.49454337, -0.51377381, -0.52206935, -0.51840498, -0.49841755, -0.48049335, -0.45293685, -0.42639188, -0.40503834, -0.38670296, -0.3700789 , -0.35126898, -0.30450227, -0.22446075, -0.15685957, -0.08727423, -0.02011864,  0.0369148 ,  0.09273958,  0.14463569,  0.19165457,  0.21488035,  0.17963765,  0.10843802,  0.03039539, -0.0453973 , -0.12129265, -0.19211573, -0.26247734, -0.32777582, -0.38777496, -0.43147176, -0.44558728};
float SS_servo3[] = {0.82604941, 0.79619622, 0.79511261, 0.81983244, 0.85332218, 0.88123827, 0.90321644, 0.9424247 , 0.97760239, 1.02020989, 1.08014056, 1.11008012, 1.12153663, 1.0947755 , 1.0610026 , 1.00203145, 0.92813358, 0.85219576, 0.77563266, 0.6959964 , 0.65388177, 0.6677426 , 0.65739077, 0.65972947, 0.65320199, 0.60856845, 0.54265931, 0.46304914, 0.38320184, 0.30960808, 0.25731459, 0.24203838, 0.29927429, 0.37635097, 0.45379786, 0.53171232, 0.6099492 , 0.68838339, 0.7643149 , 0.83822831, 0.87389212, 0.83872451};
float SS_servo4[] = { 2.63980186e-01,  2.48819955e-01,  2.40947770e-01,  2.42070240e-01,  2.39816300e-01,  2.34474687e-01,  2.25168408e-01,  2.02810567e-01,  1.87693208e-01,  1.76280479e-01,  1.65155662e-01,  1.43624060e-01,  1.33203306e-01,  1.20859655e-01,  1.14087661e-01,  1.03241123e-01,  8.61029743e-02,  8.23072315e-02,  5.63383732e-02,  4.26959276e-02,  1.18353478e-02, -1.88528775e-02, -5.04650200e-02, -8.11322430e-02, -1.07972556e-01, -1.31821628e-01, -1.86187071e-01, -2.56242036e-01, -3.32719254e-01, -3.93328877e-01, -4.11592870e-01, -4.08692413e-01, -3.42815446e-01, -2.70091783e-01, -2.01870115e-01, -1.42089217e-01, -6.71384199e-02,  3.45005651e-04,  8.12327297e-02,  1.80947461e-01,  2.64946803e-01,  2.68089868e-01};
float SS_servo5[] = {-1.7555086 , -1.75159083, -1.71638608, -1.67870408, -1.64196434, -1.60721109, -1.58666878, -1.52728741, -1.47930444, -1.43619745, -1.36996209, -1.28714123, -1.23566029, -1.17309534, -1.14067109, -1.10952895, -1.08260147, -1.10817774, -1.08317702, -1.10823714, -1.10678437, -1.08424531, -1.04798715, -1.02145092, -1.01107873, -1.02493309, -0.99014288, -0.91648701, -0.83280479, -0.80005146, -0.89247058, -1.00484859, -1.09013739, -1.16955714, -1.24503729, -1.33054464, -1.40749951, -1.48925965, -1.56791949, -1.63937648, -1.70886238, -1.74946514};
float SS_servo6[] = {0.23282928, 0.21541283, 0.19723067, 0.17954583, 0.15948283, 0.13728459, 0.11187335, 0.07957588, 0.04151608, 0.0188197 , 0.03961995, 0.12859553, 0.24726735, 0.33079782, 0.40575344, 0.46857227, 0.49445568, 0.4986668 , 0.50086205, 0.49211341, 0.48163444, 0.48315336, 0.47340021, 0.48325538, 0.50037231, 0.51328341, 0.5228134 , 0.52357266, 0.52322963, 0.5210387 , 0.51695577, 0.49061316, 0.46925231, 0.447132  , 0.42512227, 0.39120908, 0.37221682, 0.34397771, 0.31715088, 0.29617403, 0.2753233 , 0.23729016};
float SS_servo7[] = {-1.73956115, -1.72131719, -1.64687513, -1.5574896 , -1.4650628 , -1.37533388, -1.29436233, -1.19857874, -1.09021156, -1.00801305, -0.9553031 , -0.96262901, -0.99025046, -0.92818162, -0.90699616, -0.86388206, -0.78855688, -0.71409221, -0.64091624, -0.55898765, -0.52138093, -0.55906793, -0.57640306, -0.6341456 , -0.70812345, -0.7833403 , -0.86563016, -0.94177373, -1.02344265, -1.1101254 , -1.19496769, -1.25428849, -1.31356066, -1.37022751, -1.42676686, -1.46151375, -1.52224986, -1.56481077, -1.60369945, -1.64805271, -1.69673251, -1.73323837};

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
