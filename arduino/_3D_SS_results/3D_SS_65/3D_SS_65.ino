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
float SS_servo0[] = {-0.66708129, -0.65979969, -0.60703626, -0.54898944, -0.48751115, -0.41680734, -0.33673792, -0.25618525, -0.17478987, -0.09321371, -0.01212182,  0.01002646, -0.01371804, -0.0278076 , -0.04296482, -0.06238599, -0.09689712, -0.13287238, -0.1718756 , -0.21010291, -0.24015569, -0.26161624, -0.27703525, -0.29519934, -0.31138478, -0.33345962, -0.36288545, -0.39814624, -0.43688025, -0.47369336, -0.51392494, -0.54799627, -0.56500846, -0.58322845, -0.60028051, -0.62042902, -0.62927927, -0.63585358, -0.64275344, -0.64632093, -0.65162774};
float SS_servo1[] = {0.86440528, 0.87520113, 0.90110695, 0.92425058, 0.9434081 , 0.95354456, 0.95403704, 0.95408651, 0.94057938, 0.91740159, 0.8959439 , 0.94425013, 1.07265531, 1.16615489, 1.24733044, 1.29713545, 1.29100293, 1.25650683, 1.22388846, 1.18478638, 1.13508868, 1.08496719, 1.0232999 , 0.96728355, 0.91193269, 0.88583713, 0.93083   , 0.99824054, 1.0634    , 1.11432564, 1.16855567, 1.21792818, 1.22465597, 1.23193424, 1.23759674, 1.24104136, 1.18313716, 1.10882354, 1.03473434, 0.9574034 , 0.88617117};
float SS_servo2[] = {-0.26741444, -0.2779131 , -0.29285214, -0.3074453 , -0.32168734, -0.35108901, -0.39990233, -0.45096089, -0.50414003, -0.55546071, -0.59488285, -0.61081498, -0.61817325, -0.62038331, -0.62814483, -0.63819789, -0.64580402, -0.65110241, -0.65731608, -0.65997293, -0.65752673, -0.60674969, -0.54577469, -0.48531682, -0.42502556, -0.35872458, -0.28102164, -0.20033824, -0.11966001, -0.03903746,  0.04134888,  0.07169522,  0.02498987, -0.01342137, -0.04538005, -0.0798496 , -0.11588782, -0.14720906, -0.17985498, -0.21376102, -0.24501316};
float SS_servo3[] = {0.86800659, 0.79828385, 0.72093126, 0.65502237, 0.5990145 , 0.60034056, 0.67257994, 0.76389745, 0.8675896 , 0.97421885, 1.07817645, 1.16355268, 1.22362772, 1.26249153, 1.2974303 , 1.30971395, 1.26058539, 1.18730546, 1.11087505, 1.02486951, 0.9597502 , 0.959487  , 0.97949434, 1.00045928, 1.02132354, 1.02903574, 0.9895032 , 0.95308854, 0.91634787, 0.85089938, 0.79952356, 0.79371887, 0.87471707, 0.93828079, 0.98848833, 1.03280505, 1.02605137, 0.98955994, 0.95529796, 0.92653981, 0.8971001 };
float SS_servo4[] = { 0.66130829,  0.66212808,  0.6420201 ,  0.62183307,  0.60186198,  0.57925404,  0.55843247,  0.54201384,  0.528558  ,  0.5179537 ,  0.50800557,  0.49441547,  0.47069699,  0.4414439 ,  0.41149568,  0.38297329,  0.37238229,  0.36658337,  0.35953139,  0.34323134,  0.31401423,  0.27887095,  0.2462755 ,  0.20953036,  0.1765517 ,  0.13997622,  0.10571537,  0.07009351,  0.0277507 , -0.02381599, -0.05916392, -0.00360913,  0.07638652,  0.15757772,  0.23860768,  0.31764157,  0.38605916,  0.4481083 ,  0.50736827,  0.57433045,  0.64233241};
float SS_servo5[] = {-0.99203659, -1.075372  , -1.13981279, -1.20014077, -1.25320533, -1.27695151, -1.27085053, -1.25547362, -1.23260964, -1.20536112, -1.15598918, -1.07131549, -0.97789799, -0.88396532, -0.80143073, -0.74557798, -0.78378801, -0.8555267 , -0.93620652, -1.00811091, -1.0489301 , -1.06065524, -1.08153133, -1.09951747, -1.12707069, -1.13113895, -1.06911718, -0.98854574, -0.90452392, -0.80951146, -0.74236943, -0.7887616 , -0.81809602, -0.88382311, -0.96077673, -1.00636211, -1.00262411, -0.98308407, -0.96009209, -0.94270759, -0.94483923};
float SS_servo6[] = { 0.23969469,  0.22459845,  0.18808318,  0.15070433,  0.11270041,  0.07339781,  0.03967124,  0.01040803, -0.02158381, -0.05527767, -0.05178534,  0.02003902,  0.10094941,  0.18218816,  0.26299404,  0.34114432,  0.40737108,  0.46479933,  0.51836061,  0.56588953,  0.62837709,  0.64350311,  0.6492083 ,  0.65440706,  0.6599493 ,  0.66199742,  0.64316471,  0.62119456,  0.60148793,  0.58690647,  0.56761381,  0.52768744,  0.4736426 ,  0.41978506,  0.36841312,  0.32233707,  0.29697738,  0.28650891,  0.27631106,  0.26483024,  0.2507803 };
float SS_servo7[] = {-0.97107404, -1.04218043, -1.08510265, -1.12237125, -1.14999574, -1.14000986, -1.09324924, -1.03291266, -0.95141002, -0.85710487, -0.80737307, -0.83333211, -0.83927613, -0.84061995, -0.84082249, -0.83762218, -0.82291346, -0.79936699, -0.7709042 , -0.73542162, -0.72309951, -0.74656892, -0.83039635, -0.91630921, -1.00265875, -1.0734554 , -1.05766131, -1.02709303, -1.00802625, -1.00589907, -0.9973964 , -0.93982751, -0.85681425, -0.77222492, -0.68891399, -0.62332171, -0.63950461, -0.70445461, -0.77585762, -0.84726894, -0.91381743};

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
