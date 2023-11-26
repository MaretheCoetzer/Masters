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
float SS_servo0[] = {-0.66890019, -0.66549029, -0.61313975, -0.55510135, -0.4926264 , -0.42113261, -0.3410453 , -0.26046873, -0.17897006, -0.09741784, -0.01629057,  0.01088644, -0.00920298, -0.02111734, -0.03264114, -0.04868879, -0.07706101, -0.11175084, -0.14880951, -0.18427053, -0.21644856, -0.24684957, -0.26778507, -0.28417747, -0.29835548, -0.31357055, -0.33388549, -0.36442296, -0.40193927, -0.4398199 , -0.47850598, -0.51720948, -0.54375819, -0.56089141, -0.58003874, -0.59939552, -0.61912214, -0.6266456 , -0.63316253, -0.63794832, -0.64355068, -0.65760759};
float SS_servo1[] = {0.85594661, 0.87628806, 0.90315197, 0.92612519, 0.94425629, 0.95350375, 0.95393744, 0.95398823, 0.94623799, 0.93289087, 0.91304852, 0.95193913, 1.07243811, 1.16112029, 1.23358856, 1.28656977, 1.29324646, 1.26291063, 1.22719606, 1.18857915, 1.15406835, 1.12918238, 1.06675365, 1.00226153, 0.93561242, 0.87455473, 0.85501921, 0.91023256, 0.98654779, 1.05407633, 1.11248629, 1.16754177, 1.19518662, 1.1985783 , 1.20551155, 1.21260151, 1.20032724, 1.13012904, 1.05301725, 0.97148115, 0.89797883, 0.85183927};
float SS_servo2[] = {-0.31476735, -0.32539797, -0.33733714, -0.34950284, -0.36091807, -0.38928677, -0.43573869, -0.48586621, -0.53738133, -0.58831839, -0.62751229, -0.64269065, -0.64687388, -0.6456702 , -0.64862809, -0.65542456, -0.66019013, -0.66141125, -0.66523979, -0.66184993, -0.66383646, -0.64626509, -0.59836722, -0.53915721, -0.47960386, -0.41983456, -0.35065309, -0.27054634, -0.18978153, -0.1090455 , -0.02839954,  0.04534453,  0.02717502, -0.01627925, -0.05533786, -0.09050955, -0.12881758, -0.1660071 , -0.1998664 , -0.23519453, -0.27015459, -0.30342217};
float SS_servo3[] = {0.88528227, 0.81741371, 0.73827749, 0.66981706, 0.61282965, 0.61594062, 0.68561622, 0.77691396, 0.87809628, 0.9831063 , 1.0844762 , 1.16729006, 1.22164406, 1.25488052, 1.28326276, 1.29588273, 1.25893827, 1.18598882, 1.10852868, 1.01767919, 0.94312234, 0.90410847, 0.93063965, 0.95221646, 0.97412082, 0.99575883, 0.98780518, 0.93757523, 0.91149821, 0.87164223, 0.80788088, 0.76253279, 0.80119494, 0.87088474, 0.93389733, 0.98660948, 1.01947344, 1.00336012, 0.97018573, 0.93877705, 0.91347143, 0.89928103};
float SS_servo4[] = { 0.64908535,  0.64704088,  0.62604231,  0.60554098,  0.58541186,  0.56367712,  0.54423776,  0.52945813,  0.51780549,  0.50867795,  0.49966508,  0.48739395,  0.46444539,  0.43461138,  0.40340471,  0.37409271,  0.35848191,  0.3543542 ,  0.3483198 ,  0.34274077,  0.331118  ,  0.31241733,  0.28555126,  0.24755358,  0.20682081,  0.16586513,  0.12422295,  0.08710085,  0.04902131,  0.00522521, -0.04540956, -0.05378166,  0.02054147,  0.10144947,  0.18238845,  0.26352833,  0.34005086,  0.40622481,  0.46658226,  0.52334616,  0.59127339,  0.64379775};
float SS_servo5[] = {-1.00823555, -1.08825575, -1.15147337, -1.209762  , -1.25862576, -1.28000078, -1.27335748, -1.25796597, -1.23596329, -1.21077071, -1.16357967, -1.08024921, -0.98658154, -0.89004927, -0.80398546, -0.73937531, -0.74847618, -0.81668993, -0.89618064, -0.98192716, -1.05304512, -1.10057919, -1.14652062, -1.16406181, -1.17871616, -1.19578526, -1.1765135 , -1.0927624 , -0.99887363, -0.90353427, -0.80526162, -0.77334128, -0.83363703, -0.8720257 , -0.95001703, -1.01847677, -1.04291065, -1.02941197, -1.00704471, -0.98124449, -0.96770893, -0.97630595};
float SS_servo6[] = { 0.19102416,  0.17492175,  0.13913602,  0.102324  ,  0.06343817,  0.02645679, -0.00613505, -0.03430221, -0.06528398, -0.09879951, -0.09629626, -0.02497581,  0.05560626,  0.13682094,  0.21777604,  0.29785013,  0.37237919,  0.43229475,  0.4862774 ,  0.54713911,  0.59751072,  0.63168271,  0.66235397,  0.66265107,  0.66377544,  0.66197463,  0.65386332,  0.62828671,  0.60031491,  0.57735952,  0.55840864,  0.53059227,  0.47960798,  0.42616571,  0.37414985,  0.32261661,  0.28121673,  0.26438041,  0.25174869,  0.23497268,  0.21906871,  0.19791312};
float SS_servo7[] = {-0.94474298, -1.01407192, -1.05856342, -1.09570757, -1.1152518 , -1.10511776, -1.0559488 , -0.99339573, -0.9109147 , -0.81578005, -0.76375442, -0.78604978, -0.79188223, -0.79328604, -0.79354088, -0.79273114, -0.78627294, -0.76495501, -0.73627126, -0.71296173, -0.71558422, -0.76476727, -0.8598098 , -0.94146793, -1.0236444 , -1.10300102, -1.14809591, -1.111672  , -1.06445385, -1.03421803, -1.0202356 , -0.99068035, -0.91466483, -0.83313457, -0.7514115 , -0.67019458, -0.62816487, -0.6747789 , -0.74231911, -0.80710082, -0.8732159 , -0.91786231};

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
