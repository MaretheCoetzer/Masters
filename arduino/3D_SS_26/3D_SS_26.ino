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
float SS_servo0[] = {-4.25626846e-01, -4.23178395e-01, -4.22895340e-01, -4.27502949e-01, -4.16434979e-01, -3.64078185e-01, -2.83403827e-01, -2.01192995e-01, -1.21258401e-01, -4.15916075e-02,  1.35140971e-02,  3.11913023e-04, -3.79133188e-02, -7.45144941e-02, -1.08258574e-01, -1.35385843e-01, -1.49083939e-01, -1.61728424e-01, -1.73201895e-01, -1.82177899e-01, -1.89671739e-01, -1.95462760e-01, -2.02328829e-01, -2.08969787e-01, -2.15482408e-01, -2.22140694e-01, -2.31698510e-01, -2.44579330e-01, -2.61784734e-01, -2.81910947e-01, -3.04578387e-01, -3.27689837e-01, -3.49920053e-01, -3.72094758e-01, -3.96054939e-01, -4.17332246e-01, -4.26348803e-01, -4.31067648e-01, -4.31575275e-01, -4.26444017e-01};
float SS_servo1[] = {0.8471466 , 0.81124389, 0.76423013, 0.75017519, 0.78709524, 0.8186485 , 0.80483994, 0.77977752, 0.74767161, 0.70739123, 0.67632109, 0.69863951, 0.77154791, 0.84487524, 0.90498056, 0.94881451, 0.96642529, 0.9585046 , 0.93415994, 0.90596854, 0.87783212, 0.84915259, 0.82698042, 0.81296257, 0.8097273 , 0.81626493, 0.82630724, 0.84004821, 0.85904588, 0.87948659, 0.9036269 , 0.92887431, 0.95178725, 0.97216559, 0.9898421 , 1.0056459 , 1.00414069, 0.97182115, 0.9226964 , 0.86493795};
float SS_servo2[] = {-0.32962506, -0.32959948, -0.32687139, -0.32378244, -0.32291966, -0.32103141, -0.31965947, -0.32460546, -0.33586566, -0.35093036, -0.37150594, -0.39765401, -0.42270597, -0.44480566, -0.46685042, -0.48063201, -0.48148765, -0.47976723, -0.47769126, -0.47466743, -0.47114171, -0.47164173, -0.46843593, -0.44148569, -0.3899241 , -0.30908059, -0.22315731, -0.15914826, -0.12482612, -0.11058023, -0.10768122, -0.12571329, -0.16483578, -0.20984059, -0.24941665, -0.27861579, -0.29410537, -0.31103323, -0.32385104, -0.3274721 };
float SS_servo3[] = {0.83051324, 0.7974535 , 0.7439029 , 0.69419614, 0.66017106, 0.63664468, 0.62192549, 0.62422443, 0.64176619, 0.67064436, 0.70938815, 0.74726837, 0.78214234, 0.81265538, 0.83708019, 0.84938548, 0.84382979, 0.8132115 , 0.7695613 , 0.72583395, 0.68399009, 0.65056061, 0.6412556 , 0.66782977, 0.73475489, 0.79013861, 0.77929022, 0.74640802, 0.70694532, 0.6669381 , 0.6490912 , 0.67047249, 0.73385805, 0.80721271, 0.86438736, 0.90051278, 0.9104097 , 0.90477342, 0.8829623 , 0.84274462};
float SS_servo4[] = {0.49274766, 0.48850199, 0.4874647 , 0.48540626, 0.4860339 , 0.49001622, 0.49509556, 0.50029276, 0.50616281, 0.51109041, 0.50686561, 0.49443798, 0.47519307, 0.45617248, 0.43490647, 0.41860223, 0.41433347, 0.40294543, 0.38978759, 0.38264636, 0.37888421, 0.3748339 , 0.37463155, 0.37577852, 0.38126671, 0.39149205, 0.39759814, 0.39859525, 0.39439486, 0.38282842, 0.35997851, 0.330426  , 0.30583879, 0.30561174, 0.33550621, 0.39401752, 0.47785353, 0.51905711, 0.51514618, 0.49635952};
float SS_servo5[] = {-0.78244111, -0.80347973, -0.84676133, -0.88500181, -0.91729601, -0.94269603, -0.9628209 , -0.97951015, -0.99439348, -1.00170442, -0.99576261, -0.98352754, -0.96200936, -0.93346382, -0.90091573, -0.87393113, -0.86892985, -0.86997343, -0.88247418, -0.90603084, -0.93370925, -0.96121441, -0.99310481, -1.02028647, -1.0451351 , -1.0707144 , -1.09089952, -1.10493683, -1.11150805, -1.10609988, -1.08348951, -1.04198082, -1.00724126, -1.00035543, -0.99437156, -0.94359131, -0.8526949 , -0.79399827, -0.78497865, -0.77681312};
float SS_servo6[] = {0.29739686, 0.29164631, 0.28573378, 0.276021  , 0.26556066, 0.2555885 , 0.24531196, 0.23405701, 0.22107042, 0.20588386, 0.18702157, 0.16642621, 0.16446723, 0.19597849, 0.24568803, 0.30356031, 0.37890513, 0.41008553, 0.40937997, 0.41683342, 0.42883107, 0.44049257, 0.45469381, 0.4662952 , 0.47597875, 0.48637063, 0.49412207, 0.49998498, 0.5040872 , 0.50458455, 0.49901471, 0.48351711, 0.45877947, 0.43017463, 0.40088895, 0.37532527, 0.35881524, 0.33866486, 0.31945158, 0.3014556 };
float SS_servo7[] = {-0.77459708, -0.79399373, -0.82930678, -0.85351072, -0.86549804, -0.86470088, -0.85468737, -0.83810627, -0.81421091, -0.77879005, -0.73284594, -0.69317963, -0.69227505, -0.72580095, -0.73111614, -0.66660062, -0.57130676, -0.51188357, -0.53147969, -0.57338446, -0.62442292, -0.67903838, -0.73642675, -0.78158495, -0.81168765, -0.83278437, -0.85321178, -0.87513761, -0.8981638 , -0.91741905, -0.92560982, -0.91623305, -0.89239631, -0.86030905, -0.82308982, -0.78587283, -0.76470228, -0.75836541, -0.76150105, -0.76663188};

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
