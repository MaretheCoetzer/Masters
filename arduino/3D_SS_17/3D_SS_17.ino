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
float SS_servo0[] = {-0.5140605 , -0.49899253, -0.4422871 , -0.36491255, -0.28982521, -0.21051197, -0.10581787, -0.01889986,  0.06389405,  0.16244264,  0.26383482,  0.31144248,  0.26958358,  0.21557883,  0.17933763,  0.13958255,  0.09175488,  0.051571  , -0.00114696, -0.03750612, -0.08205886, -0.10815798, -0.13430549, -0.16783126, -0.20061822, -0.22960628, -0.27540054, -0.31737552, -0.36129338, -0.41274149, -0.46401993, -0.50593367, -0.52351675, -0.52119422, -0.52307308, -0.52353655, -0.5228806 , -0.52059595, -0.52043348, -0.51470564, -0.51055802};
float SS_servo1[] = {0.44269744, 0.44758316, 0.51590876, 0.59887219, 0.68136691, 0.75876094, 0.73040504, 0.67179001, 0.62662738, 0.56257872, 0.50726929, 0.44943912, 0.51647711, 0.60893494, 0.66021018, 0.71265925, 0.78213326, 0.83030982, 0.88499093, 0.85905716, 0.84568199, 0.79385122, 0.76960322, 0.78976444, 0.8220929 , 0.85025926, 0.91095   , 0.95761135, 1.00071547, 1.05255125, 1.12072283, 1.15361721, 1.12588086, 1.04356   , 0.96567417, 0.88567091, 0.80901485, 0.72582396, 0.64093915, 0.55465138, 0.47395524};
float SS_servo2[] = {-7.72870917e-02, -1.12620078e-01, -1.67120261e-01, -1.86646287e-01, -2.10695878e-01, -2.31710427e-01, -2.64876887e-01, -3.04592793e-01, -3.58352374e-01, -4.19696898e-01, -4.81676228e-01, -5.18280188e-01, -5.20890784e-01, -5.16596773e-01, -5.04472541e-01, -4.89165218e-01, -4.85309294e-01, -4.90110225e-01, -4.83495303e-01, -4.81666746e-01, -4.95976258e-01, -4.78216132e-01, -4.27935955e-01, -3.51220989e-01, -2.62751820e-01, -1.79253042e-01, -7.76509838e-02,  2.25634962e-02,  1.08420051e-01,  1.94483848e-01,  2.85206532e-01,  3.35179187e-01,  3.30541057e-01,  2.66692650e-01,  2.21291182e-01,  1.88232730e-01,  1.37488559e-01,  6.76261261e-02, -2.11877465e-04, -3.43418810e-02, -5.55035829e-02};
float SS_servo3[] = {0.49324985, 0.50257489, 0.52877309, 0.4947496 , 0.47482971, 0.45991795, 0.47640009, 0.50566988, 0.56684681, 0.65229186, 0.76653005, 0.83276442, 0.81372753, 0.78047661, 0.72632991, 0.66025202, 0.61340537, 0.57959591, 0.5153283 , 0.43755265, 0.39884382, 0.43838544, 0.50864087, 0.5847278 , 0.66689209, 0.75621582, 0.74979094, 0.67471853, 0.59350417, 0.51031359, 0.44787309, 0.3801104 , 0.32992331, 0.38326983, 0.40123787, 0.3936062 , 0.42706527, 0.48596822, 0.53805592, 0.52713637, 0.49344769};
float SS_servo4[] = { 0.52037247,  0.5238717 ,  0.52106575,  0.52246623,  0.52054285,  0.5229334 ,  0.52296846,  0.52077873,  0.52103876,  0.52505223,  0.51442181,  0.48362205,  0.45077186,  0.42510752,  0.38947554,  0.35172062,  0.32972296,  0.32145931,  0.30062764,  0.28887232,  0.27684258,  0.25232144,  0.19937783,  0.14973115,  0.12325103,  0.08246019,  0.02926036, -0.02595159, -0.08564489, -0.14997833, -0.16062929, -0.09715553, -0.00535624,  0.07804754,  0.15638228,  0.21885303,  0.27762717,  0.34672849,  0.4162711 ,  0.4905536 ,  0.52591578};
float SS_servo5[] = {-0.46382454, -0.53019031, -0.60358053, -0.67773175, -0.74477803, -0.81175841, -0.86833287, -0.92326915, -0.97755685, -1.02983919, -1.02505135, -0.96669047, -0.92091864, -0.89153655, -0.84567355, -0.80136698, -0.79119394, -0.81228627, -0.82286399, -0.88476418, -0.9646459 , -1.02134898, -1.00005369, -0.94517799, -0.92790009, -0.87541167, -0.79935675, -0.72113745, -0.63973256, -0.55505655, -0.56964507, -0.65342817, -0.73416716, -0.81824891, -0.90356269, -0.93637916, -0.88151868, -0.80116071, -0.72216341, -0.63280739, -0.49151817};
float SS_servo6[] = { 1.34372584e-01,  9.62087486e-02,  4.78157073e-02,  3.83587414e-04, -4.98073218e-02, -9.51745779e-02, -1.45871937e-01, -2.04755666e-01, -2.63353191e-01, -3.08667514e-01, -2.77131883e-01, -1.86775829e-01, -9.70833205e-02,  9.03783449e-04,  8.49097683e-02,  1.50714586e-01,  2.18657704e-01,  2.95290912e-01,  3.71616216e-01,  4.42229369e-01,  4.82876521e-01,  4.94631817e-01,  4.78750800e-01,  4.74876214e-01,  4.91203495e-01,  5.12353250e-01,  5.13923115e-01,  5.13553512e-01,  5.12053506e-01,  5.26228644e-01,  5.11695178e-01,  4.58384347e-01,  3.97027379e-01,  3.51796787e-01,  3.20000255e-01,  2.79749057e-01,  2.52145407e-01,  2.34913342e-01,  2.09878110e-01,  1.93802304e-01,  1.68839027e-01};
float SS_servo7[] = {-0.66137207, -0.64429869, -0.63180722, -0.61186354, -0.58134666, -0.55147948, -0.5038202 , -0.43493053, -0.3612348 , -0.30408795, -0.3597215 , -0.4450979 , -0.51157551, -0.58065379, -0.63728798, -0.63066846, -0.56894903, -0.49001546, -0.41280787, -0.32378133, -0.22014313, -0.22918742, -0.2727132 , -0.314107  , -0.38073832, -0.45262283, -0.49264213, -0.53423287, -0.58097198, -0.66089238, -0.67498722, -0.61495926, -0.54761019, -0.52449651, -0.52926391, -0.51774566, -0.52861391, -0.56905106, -0.60314407, -0.64962321, -0.68100731};

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
