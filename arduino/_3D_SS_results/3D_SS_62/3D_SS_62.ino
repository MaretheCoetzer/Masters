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
float SS_servo0[] = {-0.66842695, -0.66404284, -0.61204597, -0.5554219 , -0.49709773, -0.42651984, -0.34683129, -0.26627672, -0.18564472, -0.10423551, -0.02890578, -0.06419089, -0.07030877, -0.06919686, -0.07008247, -0.07246398, -0.0812356 , -0.0971114 , -0.11582169, -0.14078262, -0.16727729, -0.19005997, -0.21421954, -0.23255685, -0.25521301, -0.27428594, -0.30182334, -0.34260105, -0.38569911, -0.43109894, -0.4781136 , -0.51834258, -0.53749371, -0.55555679, -0.575017  , -0.59480231, -0.6168808 , -0.62626018, -0.63444266, -0.6364401 , -0.64431528, -0.65128369};
float SS_servo1[] = {0.82857549, 0.84761852, 0.87547593, 0.90181225, 0.92481507, 0.93486614, 0.93191914, 0.92834543, 0.89841048, 0.83664579, 0.79865306, 0.92320954, 1.01109095, 1.07833674, 1.13528781, 1.17417031, 1.1892613 , 1.18431216, 1.15805929, 1.12372622, 1.07792762, 1.01649063, 0.95913825, 0.90012419, 0.85342678, 0.80426604, 0.79112706, 0.85747869, 0.94389385, 1.02932464, 1.10694723, 1.15732367, 1.1592348 , 1.16181498, 1.17043815, 1.18092809, 1.18023298, 1.11991902, 1.04667097, 0.96088906, 0.89004576, 0.82756264};
float SS_servo2[] = {-0.33226688, -0.34451984, -0.35858068, -0.36953075, -0.3821724 , -0.40152257, -0.44318912, -0.49643922, -0.54981824, -0.60430171, -0.65424729, -0.66168054, -0.64708025, -0.63226057, -0.62388595, -0.61500843, -0.60773767, -0.60255602, -0.59527585, -0.58923245, -0.58407228, -0.56949461, -0.52690129, -0.46720042, -0.40414269, -0.34086577, -0.26945047, -0.19014248, -0.10917683, -0.02835526,  0.05242525,  0.07874388,  0.01799026, -0.02153494, -0.06085046, -0.09633371, -0.13441321, -0.1730188 , -0.2074373 , -0.24274319, -0.2788824 , -0.31412213};
float SS_servo3[] = {0.88229968, 0.821624  , 0.74574561, 0.66841501, 0.60350882, 0.57738796, 0.62176223, 0.70493374, 0.80007745, 0.90276763, 1.00397445, 1.04633764, 1.0714108 , 1.09204615, 1.11150785, 1.11458177, 1.09207043, 1.05010112, 0.9860414 , 0.90808444, 0.82063009, 0.78906568, 0.81315101, 0.83421463, 0.85241495, 0.87037312, 0.87342236, 0.83632292, 0.7934308 , 0.75608871, 0.69127454, 0.68208802, 0.77612896, 0.8361418 , 0.90128295, 0.95813265, 1.0001543 , 0.99596821, 0.96583938, 0.93534948, 0.9101672 , 0.89489297};
float SS_servo4[] = { 0.64034333,  0.63708372,  0.61815098,  0.59589285,  0.57337953,  0.5506877 ,  0.52935701,  0.5136376 ,  0.50035397,  0.4927532 ,  0.48508343,  0.47271253,  0.45866155,  0.43975583,  0.41414077,  0.3871681 ,  0.36739778,  0.35191586,  0.34725188,  0.33919874,  0.33351574,  0.3237975 ,  0.30106968,  0.26276935,  0.21885414,  0.17563826,  0.13072042,  0.09264366,  0.05487742,  0.01252018, -0.03795574, -0.07049798, -0.01213381,  0.06880736,  0.14967751,  0.23061542,  0.30954403,  0.37881985,  0.44176926,  0.50041206,  0.56609849,  0.63009404};
float SS_servo5[] = {-1.03498122, -1.1098156 , -1.1812357 , -1.24468947, -1.30254322, -1.33725645, -1.34743058, -1.34763248, -1.33645038, -1.32653294, -1.30314928, -1.24189138, -1.1509673 , -1.05693522, -0.9639792 , -0.88429525, -0.84775217, -0.84644412, -0.88940127, -0.94640707, -1.02693623, -1.11347178, -1.17871693, -1.2010935 , -1.21083133, -1.22050733, -1.20664049, -1.13729284, -1.04550742, -0.95038886, -0.8500483 , -0.7884653 , -0.84023836, -0.89052309, -0.96201984, -1.02804358, -1.06421937, -1.05881094, -1.03968196, -1.01525467, -1.00000663, -1.00005945};
float SS_servo6[] = { 0.19842314,  0.1795123 ,  0.14175012,  0.10280336,  0.05981227,  0.01764847, -0.01741142, -0.04842252, -0.07743684, -0.11088965, -0.14255224, -0.10535076, -0.02464262,  0.05658566,  0.13735377,  0.21796504,  0.29829596,  0.37404798,  0.44140139,  0.49980755,  0.55369731,  0.59549912,  0.63145376,  0.64833973,  0.65670933,  0.66134707,  0.66190738,  0.64595464,  0.62485727,  0.60618622,  0.59351101,  0.57216379,  0.52171691,  0.46695081,  0.41351455,  0.36312295,  0.31626593,  0.2901004 ,  0.27359659,  0.2560382 ,  0.23575048,  0.21294363};
float SS_servo7[] = {-0.96295271, -1.02230785, -1.0659901 , -1.10591753, -1.13282722, -1.12740776, -1.09693215, -1.04802717, -0.98288391, -0.89653628, -0.80253195, -0.80247045, -0.82135203, -0.8224501 , -0.82329658, -0.82328405, -0.82293184, -0.81810537, -0.80485828, -0.78259476, -0.75400694, -0.71376154, -0.77130652, -0.88199612, -0.9778413 , -1.06587806, -1.13553331, -1.12771923, -1.09547752, -1.07256523, -1.07019582, -1.06218095, -0.99607791, -0.91490751, -0.83052403, -0.74946059, -0.68995551, -0.7106129 , -0.76898956, -0.83155039, -0.89046518, -0.93742878};

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
