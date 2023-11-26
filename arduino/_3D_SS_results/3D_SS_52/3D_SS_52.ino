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
float SS_servo0[] = {-0.66011998, -0.66308055, -0.65791981, -0.64757645, -0.61551786, -0.54545236, -0.46590157, -0.38515282, -0.30413092, -0.22321732, -0.14264841, -0.16081363, -0.19982074, -0.22939259, -0.25031251, -0.26882942, -0.28458871, -0.30094721, -0.31854924, -0.33689732, -0.35327187, -0.37090086, -0.38658358, -0.39620337, -0.39398509, -0.38937821, -0.3847785 , -0.37797393, -0.38329184, -0.41187654, -0.45153634, -0.48855532, -0.50846159, -0.53749081, -0.55019631, -0.57073934, -0.58944309, -0.6145445 , -0.63841058, -0.65378795, -0.65158037, -0.65209862, -0.65359847, -0.65664274, -0.65900818};
float SS_servo1[] = {0.5822607 , 0.61666936, 0.69169397, 0.76202029, 0.8104571 , 0.82102146, 0.82209439, 0.82238311, 0.8225025 , 0.82211034, 0.8221413 , 0.86711142, 0.93104285, 1.03363179, 1.1274717 , 1.18373608, 1.18196327, 1.1644892 , 1.1335136 , 1.08783258, 1.03263843, 1.00554772, 0.98177969, 0.94378984, 0.87943446, 0.80393136, 0.72781305, 0.65163919, 0.62196859, 0.66837208, 0.74572631, 0.80572615, 0.81233136, 0.84441586, 0.83725013, 0.85178823, 0.88490446, 0.93035302, 0.96231752, 0.94909689, 0.8791345 , 0.81158883, 0.74519667, 0.68121036, 0.63512815};      
float SS_servo2[] = {-0.25725498, -0.27082354, -0.27642585, -0.28008275, -0.2785448 , -0.27630673, -0.28634694, -0.30830255, -0.34539015, -0.39163197, -0.43574941, -0.46782508, -0.48692897, -0.50358349, -0.52287284, -0.53568176, -0.5408229 , -0.54647766, -0.55247652, -0.55852774, -0.56224894, -0.56449138, -0.55892611, -0.52866836, -0.47894858, -0.40939641, -0.34186085, -0.27049723, -0.19282809, -0.11513648, -0.03983549,  0.03895384,  0.04234815,  0.12170092,  0.10909941,  0.05902629,  0.01908498, -0.01829419, -0.05506206, -0.08638429, -0.11141008, -0.13704721, -0.16427327, -0.19468921, -0.22565206};
float SS_servo3[] = {0.84648634, 0.84659051, 0.77880356, 0.7023828 , 0.62235652, 0.557638  , 0.536479  , 0.55369545, 0.61126798, 0.69234935, 0.77441932, 0.81745739, 0.83646974, 0.89860652, 0.97311998, 1.00455472, 0.98128628, 0.94547129, 0.89731663, 0.83583138, 0.76576466, 0.71446745, 0.67306725, 0.70140253, 0.6848212 , 0.69624367, 0.70938265, 0.71864089, 0.68260931, 0.60119021, 0.53453732, 0.53158363, 0.56597182, 0.53073497, 0.56800479, 0.65215558, 0.74103731, 0.82895551, 0.90154976, 0.92124694, 0.89313267, 0.86859726, 0.84678568, 0.83136708, 0.84000221};      
float SS_servo4[] = {0.64355167, 0.64726468, 0.65241927, 0.65779906, 0.66254826, 0.66278249, 0.65810558, 0.65408947, 0.6480252 , 0.6452659 , 0.64683264, 0.64471222, 0.63834573, 0.61144569, 0.57314941, 0.54090885, 0.52680888, 0.52890149, 0.53497702, 0.54630571, 0.55086325, 0.53675607, 0.51312195, 0.49100309, 0.45801341, 0.42849217, 0.40153346, 0.37447443, 0.34221185, 0.30243987, 0.26350787, 0.22171903, 0.18485859, 0.16317783, 0.15254318, 0.18262681, 0.26016417, 0.34113259, 0.42167224, 0.49662438, 0.54865587, 0.58403546, 0.61389759, 0.64146073, 0.66690654};      
float SS_servo5[] = {-0.39438064, -0.42614639, -0.50500716, -0.58967316, -0.66955666, -0.72927356, -0.76442801, -0.78681207, -0.79587469, -0.80482162, -0.81478835, -0.83314252, -0.84240493, -0.76823419, -0.66464931, -0.59467606, -0.59630795, -0.64088061, -0.70622164, -0.79625292, -0.88063149, -0.9096631 , -0.91418707, -0.92341001, -0.91468335, -0.92072478, -0.93496004, -0.9463793 , -0.92581102, -0.85349653, -0.77154373, -0.69501459, -0.65056363, -0.61146886, -0.58881432, -0.64974821, -0.71707708, -0.72349742, -0.72733897, -0.72185536, -0.69083014, -0.63975318, -0.58414287, -0.56172638, -0.55775433};
float SS_servo6[] = { 0.40707496,  0.37975334,  0.33449999,  0.29374769,  0.25418513,  0.21681808,  0.17629032,  0.13741734,  0.09318276,  0.04631259,  0.01285925, -0.01433935, -0.04454191, -0.00280993,  0.07651321,  0.1569719 ,  0.23546299,  0.31134639,  0.38602189,  0.45333663,  0.48992826,  0.51262949,  0.51283188,  0.5010825 ,  0.51497039,  0.52883504,  0.54170928,  0.55578543,  0.55276535,  0.53015831,  0.50359766,  0.48466818,  0.48777893,  0.47918006,  0.47544871,  0.45947964,  0.41681357,  0.37078882,  0.33878815,  0.35481883,  0.37467257,  0.39612534,  0.41282571,  0.42098557,  0.41904158};
float SS_servo7[] = {-0.99254347, -0.96401821, -0.94863792, -0.95023335, -0.94954658, -0.93653044, -0.89522335, -0.84040868, -0.76138574, -0.67009756, -0.57809643, -0.52331692, -0.46031806, -0.469519  , -0.47225549, -0.47180667, -0.46706687, -0.45777597, -0.44843453, -0.43442337, -0.44961981, -0.44001774, -0.3837466 , -0.30316866, -0.38656203, -0.47369422, -0.55999569, -0.64602387, -0.68067161, -0.6508688 , -0.60343822, -0.57831442, -0.61617701, -0.62192415, -0.64631122, -0.64221276, -0.56145856, -0.4714162 , -0.41918473, -0.49043883, -0.5978212 , -0.70992135, -0.81478553, -0.90625728, -0.95858305};

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
