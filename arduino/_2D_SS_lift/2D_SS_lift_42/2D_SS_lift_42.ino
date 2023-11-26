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
float SS_servo0[] = {-0.21742343, -0.21885456, -0.23033289, -0.24196254, -0.25359219, -0.25644568, -0.2531929 , -0.24994012, -0.24668734, -0.24343456, -0.24018178, -0.236929  , -0.23367622, -0.23042344, -0.22181792, -0.20742518, -0.19303244, -0.17863969, -0.16424695, -0.14985421, -0.13546147, -0.11864878, -0.09937738, -0.07790221, -0.05523977, -0.0322245 , -0.00971486,  0.01097305,  0.02965104,  0.03707984,  0.03374539,  0.03110202,  0.02710286,  0.02023653,  0.01260479,  0.00742105,  0.00410105,  0.00433831,  0.00728699,  0.00793259,  0.00590717,  0.00333806,  0.00076896, -0.00180015, -0.00536014, -0.01545671, -0.02555327, -0.03564984, -0.04481955, -0.04762888, -0.05157617, -0.05268183, -0.05691502, -0.05993624, -0.07062603, -0.08069445, -0.08611977, -0.08659615, -0.08423773, -0.08195455, -0.08584946, -0.08953853, -0.090284  , -0.08766454, -0.08538875, -0.08640157, -0.08741439, -0.08842721, -0.08985463, -0.09822001, -0.1065854 , -0.11495079, -0.13033915, -0.15422391, -0.1831794 , -0.22024346};
float SS_servo1[] = {0.27772808, 0.27763893, 0.28613649, 0.28338213, 0.28062778, 0.29091755, 0.31028315, 0.32964875, 0.34901434, 0.36837994, 0.38774554, 0.40711114, 0.42647674, 0.44584233, 0.45760925, 0.46116072, 0.46471219, 0.46826366, 0.47181513, 0.4753666 , 0.47891807, 0.46066944, 0.42659528, 0.37757756, 0.32320818, 0.26907929, 0.21649025, 0.16619568, 0.12318267, 0.10097079, 0.10067627, 0.10059448, 0.10289586, 0.11069429, 0.11794937, 0.12222951, 0.12129683, 0.11311263, 0.09908806, 0.0873536 , 0.08176483, 0.07742698, 0.07308914, 0.0687513 , 0.06619141, 0.07536002, 0.08452864, 0.09369725, 0.10130593, 0.0992027 , 0.09907661, 0.09529137, 0.09650134, 0.09774378, 0.11249312, 0.12859159, 0.13406903, 0.12577812, 0.11250246, 0.09984121, 0.1018073 , 0.10422095, 0.10254752, 0.09011441, 0.08001112, 0.07669152, 0.07337193, 0.07005233, 0.06756051, 0.07892068, 0.09028084, 0.10164101, 0.12489615, 0.16559306, 0.21943018, 0.28480809};
float SS_servo2[] = {-0.17709777, -0.12984746, -0.07713708, -0.05713048, -0.03712388, -0.02629417, -0.02184956, -0.01740495, -0.01296033, -0.00851572, -0.00407111,  0.00037351,  0.00481812,  0.00926273,  0.00634731, -0.00452557, -0.01539845, -0.02627132, -0.0371442 , -0.04801708, -0.05888995, -0.06628666, -0.06659778, -0.06844553, -0.07115582, -0.07690876, -0.08673003, -0.10460489, -0.12235064, -0.13341628, -0.13669474, -0.135328  , -0.1309748 , -0.12464209, -0.11780083, -0.11404096, -0.11398586, -0.11518034, -0.12157342, -0.13128215, -0.1431896 , -0.15554458, -0.16789956, -0.18025455, -0.19242584, -0.2033854 , -0.21434497, -0.22530453, -0.23218289, -0.22910938, -0.21734492, -0.19874347, -0.17221908, -0.13892107, -0.10304579, -0.06584256, -0.03893742, -0.03322119, -0.03386729, -0.03693724, -0.04098736, -0.04314728, -0.04516218, -0.0510293 , -0.05689288, -0.06229157, -0.06769025, -0.07308894, -0.07862759, -0.08650845, -0.09438931, -0.10227016, -0.11539222, -0.1338495 , -0.15824639, -0.19250996};
float SS_servo3[] = { 3.32389309e-01,  3.38146541e-01,  2.89345314e-01,  2.32761101e-01,  1.76176887e-01,  1.42547462e-01,  1.24889508e-01,  1.07231554e-01,  8.95735995e-02,  7.19156453e-02,  5.42576911e-02,  3.65997369e-02,  1.89417827e-02,  1.28382857e-03, -1.05871856e-04,  1.60931560e-02,  3.22921839e-02,  4.84912117e-02,  6.46902396e-02,  8.08892674e-02,  9.70882953e-02,  1.06754793e-01,  1.01918323e-01,  9.90602427e-02,  9.80774518e-02,  1.03252967e-01,  1.16672869e-01,  1.44928425e-01,  1.75898619e-01,  1.91244610e-01,  1.90830497e-01,  1.82667502e-01,  1.68110234e-01,  1.49221282e-01,  1.27182551e-01,  1.13338771e-01,  1.05561826e-01,  1.00273263e-01,  1.06929117e-01,  1.21608634e-01,  1.57246274e-01,  1.97149869e-01,  2.37053464e-01,  2.76957059e-01,  3.18455247e-01,  3.70472381e-01,  4.22489516e-01,  4.74506651e-01,  5.19319086e-01,  5.38560684e-01,  5.28817122e-01,  4.98077599e-01,  4.34723366e-01,  3.51836657e-01,  2.66371792e-01,  1.86075193e-01,  1.25853550e-01,  1.05015881e-01,  9.78687810e-02,  9.60733618e-02,  9.83749122e-02,  9.77012990e-02,  9.85900574e-02,  1.03353763e-01,  1.09779988e-01,  1.15442030e-01,  1.21104072e-01,  1.26766114e-01,  1.32698511e-01,  1.43155035e-01,  1.53611560e-01,  1.64068085e-01,  1.82807987e-01,  2.12666219e-01,  2.57519132e-01, 3.17606848e-01};
float SS_servo4[] = { 2.00473629e-01,  1.78909508e-01,  1.57484886e-01,  1.46565617e-01,  1.35646347e-01,  1.26173973e-01,  1.17708318e-01,  1.09242663e-01,  1.00777008e-01,  9.23113530e-02,  8.38456980e-02,  7.53800430e-02,  6.69143880e-02,  5.84487331e-02,  5.60704675e-02,  6.02736974e-02,  6.44769273e-02,  6.86801572e-02,  7.28833871e-02,  7.70866170e-02,  8.12898470e-02,  9.19757690e-02,  9.99013965e-02,  1.04511122e-01,  1.07644616e-01,  1.09876038e-01,  1.11502043e-01,  1.11509272e-01,  1.09780872e-01,  1.00705624e-01,  8.11713523e-02,  6.34214149e-02,  5.06863592e-02,  4.37795940e-02,  3.65044905e-02,  3.36133105e-02,  3.05522902e-02,  2.66092599e-02,  2.18032562e-02,  1.48984552e-02,  1.01815881e-02,  5.91006744e-03,  1.63854683e-03, -2.63297378e-03, -5.60996647e-03, -4.74276836e-05,  5.51511110e-03,  1.10776499e-02,  2.01369491e-02,  2.15409311e-02,  1.36953121e-02,  3.25962087e-03, -8.28613054e-03, -1.88252362e-02, -2.98060476e-02, -3.98750707e-02, -4.95920166e-02, -5.52338289e-02, -5.99520271e-02, -6.28327501e-02, -4.92819737e-02, -2.69144852e-02,  4.04871489e-03,  3.82386923e-02,  7.01389547e-02,  9.62067395e-02,  1.22274524e-01,  1.48342309e-01,  1.73613409e-01,  1.85552716e-01,  1.97492022e-01,  2.09431328e-01,  2.13054252e-01,  2.13106176e-01,  2.11994434e-01, 2.10951822e-01};
float SS_servo5[] = {-0.17326074, -0.1328827 , -0.09936185, -0.0931129 , -0.08686395, -0.07925384, -0.07069667, -0.0621395 , -0.05358233, -0.04502516, -0.03646799, -0.02791082, -0.01935365, -0.01079648, -0.01323642, -0.0275661 , -0.04189577, -0.05622545, -0.07055512, -0.0848848 , -0.09921447, -0.12606396, -0.14751552, -0.16340923, -0.17619686, -0.18716866, -0.1969101 , -0.20485114, -0.20632176, -0.19500053, -0.16247358, -0.1319312 , -0.11191895, -0.10406278, -0.097573  , -0.09798337, -0.09946403, -0.0991849 , -0.09742551, -0.0937633 , -0.09386518, -0.09473323, -0.09560127, -0.09646932, -0.10021899, -0.12297772, -0.14573645, -0.16849517, -0.19803541, -0.20875784, -0.20089883, -0.18565221, -0.16948861, -0.15288362, -0.13742129, -0.12121384, -0.10695051, -0.10475054, -0.10369172, -0.10593847, -0.13970968, -0.19114757, -0.25933082, -0.33320046, -0.39145252, -0.42176   , -0.45206748, -0.48237497, -0.50950838, -0.48352654, -0.45754471, -0.43156287, -0.38013797, -0.31869776, -0.25200345, -0.19466487};
float SS_servo6[] = { 0.15803858,  0.14541627,  0.12530748,  0.10672442,  0.08814135,  0.07566303,  0.06743227,  0.05920151,  0.05097074,  0.04273998,  0.03450922,  0.02627845,  0.01804769,  0.00981693,  0.0045164 ,  0.00238394,  0.00025148, -0.00188097, -0.00401343, -0.00614589, -0.00827834, -0.01092076, -0.01395148, -0.01659521, -0.02001569, -0.02316067, -0.02638635, -0.0302066 , -0.03300618, -0.03083852, -0.01121305,  0.01770023,  0.05225814,  0.08814468,  0.12123154,  0.14925755,  0.17173129,  0.18890156,  0.19970004,  0.20908948,  0.20783621,  0.20441664,  0.20099708,  0.19757751,  0.19343742,  0.18454428,  0.17565113,  0.16675799,  0.15999439,  0.15767091,  0.15460513,  0.15010484,  0.14702804,  0.14623211,  0.14962181,  0.15645252,  0.16327445,  0.16720255,  0.16838999,  0.1640685 ,  0.14854887,  0.12953886,  0.11749408,  0.10509175,  0.09674564,  0.09159461,  0.08644357,  0.08129253,  0.07662555,  0.0800588 ,  0.08349205,  0.0869253 ,  0.09376626,  0.10786552,  0.13393899,  0.16154   };
float SS_servo7[] = {-0.27215227, -0.24958059, -0.21852346, -0.19670069, -0.17487792, -0.16111643, -0.15296382, -0.14481122, -0.13665861, -0.128506  , -0.1203534 , -0.11220079, -0.10404819, -0.09589558, -0.09237452, -0.09386096, -0.09534739, -0.09683382, -0.09832025, -0.09980669, -0.10129312, -0.10118163, -0.10049985, -0.10172327, -0.10126724, -0.10137591, -0.10131112, -0.10149843, -0.10075859, -0.11224438, -0.15952302, -0.22515608, -0.30312829, -0.38289589, -0.45330206, -0.50099593, -0.53014035, -0.53994921, -0.53097943, -0.52655347, -0.47825924, -0.42103577, -0.3638123 , -0.30658883, -0.25106685, -0.20676898, -0.16247111, -0.11817324, -0.10513829, -0.10822323, -0.11010691, -0.10702724, -0.10815658, -0.11140067, -0.12511175, -0.14312336, -0.16230295, -0.1793671 , -0.19017741, -0.18945675, -0.16408569, -0.13079566, -0.1096348 , -0.09161436, -0.0801512 , -0.07506466, -0.06997813, -0.06489159, -0.06079637, -0.07328975, -0.08578314, -0.09827653, -0.11991823, -0.15589462, -0.21299837, -0.27777609};

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
