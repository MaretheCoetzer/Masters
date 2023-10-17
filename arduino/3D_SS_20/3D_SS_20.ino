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
float SS_servo0[] = {-0.46931766, -0.48990777, -0.5108599 , -0.46848536, -0.39566528, -0.31013734, -0.21258116, -0.12989107, -0.05523128, -0.00163028,  0.0356655 ,  0.04214622,  0.00272013, -0.03454987, -0.06035436, -0.08211667, -0.1007078 , -0.1145487 , -0.12872533, -0.14290305, -0.1536056 , -0.16352619, -0.17436797, -0.1902975 , -0.2085527 , -0.2287807 , -0.24966293, -0.2718663 , -0.29688065, -0.32342184, -0.35033035, -0.37636455, -0.39930666, -0.41748394, -0.43126067, -0.4431767 , -0.45164563, -0.45982777, -0.46514684, -0.46617563};
float SS_servo1[] = {0.79381327, 0.79343793, 0.83295548, 0.89747138, 0.98085722, 1.02359878, 0.96955954, 0.89746248, 0.8298962 , 0.75978176, 0.6878199 , 0.67027967, 0.74178846, 0.80748198, 0.84347183, 0.86225846, 0.86800953, 0.85836275, 0.83396397, 0.79476567, 0.75317148, 0.72096775, 0.70312604, 0.70457024, 0.71966681, 0.73940168, 0.75359221, 0.76647036, 0.7874483 , 0.81606676, 0.8492634 , 0.87886619, 0.90155803, 0.91395949, 0.91295625, 0.90526794, 0.8949018 , 0.88486553, 0.85746991, 0.80660211};
float SS_servo2[] = {-0.22861874, -0.24355236, -0.26362447, -0.27682582, -0.28735379, -0.29748624, -0.31084437, -0.32939522, -0.35317151, -0.37987265, -0.40617467, -0.43345343, -0.4582964 , -0.47576854, -0.48674214, -0.49359388, -0.49663266, -0.49558234, -0.49633198, -0.4970246 , -0.49548992, -0.4677699 , -0.41466498, -0.35058883, -0.27535958, -0.19829035, -0.13507773, -0.10254632, -0.07294736, -0.04268643, -0.02128896, -0.03044957, -0.06968764, -0.10559899, -0.13156724, -0.15562528, -0.17380944, -0.18803015, -0.20382933, -0.22075935};
float SS_servo3[] = {0.72547101, 0.71110709, 0.69000413, 0.67260764, 0.66612995, 0.66748237, 0.67807405, 0.70070195, 0.73741561, 0.78256684, 0.83121304, 0.876774  , 0.91015337, 0.92784126, 0.9294184 , 0.91702467, 0.89390397, 0.85947818, 0.81411855, 0.75478001, 0.71532907, 0.71583946, 0.76221356, 0.82960151, 0.90469698, 0.93401247, 0.87862489, 0.80413171, 0.72879023, 0.65569232, 0.59946467, 0.59909009, 0.65918673, 0.71031713, 0.73574905, 0.75365076, 0.76155724, 0.76010511, 0.75126718, 0.72997362};
float SS_servo4[] = { 0.50338104,  0.49697533,  0.48718986,  0.47733756,  0.47176823,  0.46427305,  0.45362865,  0.44134024,  0.42390629,  0.40263791,  0.38173211,  0.35918042,  0.33503266,  0.31481543,  0.29764357,  0.27850098,  0.26189944,  0.24837456,  0.22893766,  0.20480938,  0.18129572,  0.16231669,  0.14647944,  0.13128936,  0.11784867,  0.09985212,  0.07389202,  0.03892294, -0.00555723, -0.02983272, -0.01006883,  0.03401687,  0.10359975,  0.18655749,  0.26617532,  0.33518437,  0.40438505,  0.47756343,  0.51264363,  0.50660514};
float SS_servo5[] = {-0.95716987, -0.98817667, -1.0325646 , -1.05896428, -1.07667372, -1.08361925, -1.08226673, -1.07561709, -1.05444926, -1.02262355, -0.98458071, -0.94618201, -0.91189256, -0.88638874, -0.86988177, -0.85469475, -0.85024606, -0.85791046, -0.86923685, -0.88720031, -0.90319714, -0.91867936, -0.9297396 , -0.93344973, -0.9311483 , -0.91793316, -0.89436971, -0.85354896, -0.7874138 , -0.75967642, -0.82234197, -0.91180702, -0.99603756, -1.07219854, -1.1403885 , -1.15430503, -1.08622987, -0.99283845, -0.92446333, -0.94622041};
float SS_servo6[] = { 0.14560046,  0.13681847,  0.12063627,  0.10082721,  0.08353256,  0.06288473,  0.04067447,  0.0128775 , -0.02250565, -0.04028578, -0.01527635,  0.0291561 ,  0.08950534,  0.16931674,  0.24967273,  0.31953462,  0.38836215,  0.45580342,  0.48587931,  0.4827245 ,  0.47595935,  0.4724784 ,  0.46804253,  0.46162964,  0.45584514,  0.44610615,  0.43095595,  0.4114667 ,  0.38537362,  0.35606693,  0.32796794,  0.30029159,  0.27375871,  0.25248933,  0.23518701,  0.21625339,  0.20115788,  0.18783292,  0.1697435 ,  0.15001955};
float SS_servo7[] = {-0.82675707, -0.85609879, -0.89179494, -0.90056015, -0.89560166, -0.87332142, -0.84400848, -0.79884581, -0.7312022 , -0.69786859, -0.74914633, -0.83448607, -0.92208468, -0.99814226, -1.04840604, -1.03935582, -0.96858973, -0.87091911, -0.80168305, -0.8440492 , -0.88589315, -0.92653586, -0.95563888, -0.97368699, -0.98388419, -0.98689454, -0.98785344, -0.98398516, -0.96332403, -0.93072149, -0.89568335, -0.86157073, -0.83047106, -0.81069529, -0.8027001 , -0.79373777, -0.79035316, -0.79196407, -0.79744842, -0.81391529};

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
