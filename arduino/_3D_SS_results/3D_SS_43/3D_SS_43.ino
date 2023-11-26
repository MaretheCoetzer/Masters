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
float SS_servo0[] = {-0.50473103, -0.52196053, -0.52361156, -0.52360316, -0.51621094, -0.47595969, -0.39808436, -0.31737102, -0.2364395 , -0.15506137, -0.09457036,  0.06153331,  0.18885749,  0.23016162,  0.18887523,  0.15220256,  0.11980011,  0.08832162,  0.06278442,  0.03479003,  0.00322055, -0.02914607, -0.0619704 , -0.09552735, -0.13366275, -0.17629515, -0.21036049, -0.2239995 , -0.23020771, -0.23731684, -0.24432398, -0.25275466, -0.26973859, -0.29420489, -0.32360055, -0.35389893, -0.38621085, -0.42892938, -0.47164791, -0.5020983 , -0.52458977, -0.52151221, -0.51997797, -0.52051394, -0.52424753, -0.51859617, -0.50966846, -0.50080816, -0.49132737, -0.48166064, -0.48074638, -0.47437251};
float SS_servo1[] = {0.30597673, 0.36075175, 0.44141377, 0.52233526, 0.59567344, 0.63597181, 0.61225522, 0.54936027, 0.51702874, 0.51362123, 0.50091197, 0.45805094, 0.39890028, 0.38631065, 0.46836555, 0.55903689, 0.64717611, 0.72688926, 0.77526601, 0.8103951 , 0.83775416, 0.86157665, 0.87912281, 0.88662312, 0.89925649, 0.92890058, 0.92852109, 0.88042219, 0.83056096, 0.77712423, 0.72097832, 0.67987646, 0.67247844, 0.68831504, 0.70693699, 0.72526185, 0.74410245, 0.81257268, 0.88104291, 0.92246816, 0.94161895, 0.88950027, 0.84295392, 0.80541219, 0.77146796, 0.70407854, 0.62377351, 0.54422271, 0.46219163, 0.37885649, 0.31769917, 0.27423524};
float SS_servo2[] = {-0.1419221 , -0.14954977, -0.15554976, -0.16051941, -0.163576  , -0.16835339, -0.19022179, -0.23477805, -0.29643047, -0.35305439, -0.39445136, -0.45799776, -0.50357674, -0.51820754, -0.5047291 , -0.49860931, -0.49805227, -0.50077927, -0.50156447, -0.50193519, -0.50191033, -0.50197502, -0.50090124, -0.49800522, -0.49330698, -0.4964512 , -0.51886957, -0.52351325, -0.52321576, -0.52256892, -0.51388494, -0.47263206, -0.39457241, -0.3139438 , -0.23321914, -0.15227064, -0.11545953,  0.01966865,  0.15479684,  0.2453745 ,  0.29588382,  0.24939005,  0.21565392,  0.17510065,  0.13879431,  0.10806241,  0.08004471,  0.05580572,  0.02757452, -0.00481761, -0.04606687, -0.09879879};
float SS_servo3[] = {0.6629632 , 0.61732114, 0.54321322, 0.46782605, 0.39061875, 0.32774345, 0.31647134, 0.3597319 , 0.43732639, 0.50273577, 0.53305133, 0.62742185, 0.68658905, 0.68140155, 0.64762428, 0.64123266, 0.64901254, 0.65633505, 0.64574331, 0.61947159, 0.58047112, 0.53795786, 0.48740891, 0.42399805, 0.35210284, 0.30258692, 0.312112  , 0.38609766, 0.46621468, 0.54618969, 0.61788293, 0.65699491, 0.63554356, 0.55845033, 0.49407617, 0.46805199, 0.46207163, 0.43588753, 0.40970344, 0.38454018, 0.37993368, 0.42964005, 0.45752493, 0.50714842, 0.54356233, 0.54664193, 0.53783094, 0.52223163, 0.51277604, 0.51041981, 0.53385267, 0.61320321};
float SS_servo4[] = { 0.49612878,  0.51973101,  0.52343886,  0.52329269,  0.52199645,  0.51960704,  0.5130667 ,  0.50656909,  0.5009113 ,  0.49484836,  0.49097857,  0.48687294,  0.48564225,  0.47917064,  0.44603542,  0.40624274,  0.36673948,  0.32931138,  0.29399281,  0.27354298,  0.26429873,  0.25739413,  0.25387756,  0.25578386,  0.25826952,  0.25099314,  0.22205858,  0.17629241,  0.11806974,  0.06166675,  0.00569061, -0.04885472, -0.10170993, -0.15665427, -0.21606718, -0.2753249 , -0.31750002, -0.30442997, -0.29135992, -0.23408541, -0.1651129 , -0.08446876, -0.00379785,  0.0768176 ,  0.15683711,  0.22978536,  0.30139195,  0.37392105,  0.44960968,  0.52515354,  0.52608062,  0.50660781};
float SS_servo5[] = {-0.38596294, -0.48500063, -0.5702675 , -0.65051239, -0.7296611 , -0.8000429 , -0.84785474, -0.88747255, -0.92851325, -0.96923007, -1.01860909, -1.04157896, -1.07195811, -1.09936834, -1.04272915, -0.95300799, -0.86007548, -0.77819426, -0.71458713, -0.69807321, -0.71890666, -0.74905362, -0.79394162, -0.86242404, -0.94038478, -0.99297601, -1.00485345, -0.97940146, -0.90983155, -0.85292645, -0.8020634 , -0.74216973, -0.6686528 , -0.58213625, -0.49541005, -0.41341763, -0.37308708, -0.36286428, -0.35264149, -0.36880885, -0.3936677 , -0.42298453, -0.48113724, -0.54460262, -0.58785861, -0.59266347, -0.58223589, -0.57122282, -0.56379001, -0.55926738, -0.54998246, -0.43541219};
float SS_servo6[] = { 0.09840125,  0.07771069,  0.03171288, -0.00877024, -0.05227127, -0.09457375, -0.14042232, -0.19297296, -0.24823602, -0.31137351, -0.35458464, -0.37649081, -0.38069517, -0.36328738, -0.29480838, -0.21426678, -0.13341534, -0.05251111,  0.02799404,  0.10839778,  0.18902554,  0.26984816,  0.35066641,  0.43147769,  0.491111  ,  0.52211384,  0.5195212 ,  0.51945007,  0.51026364,  0.51441702,  0.52173483,  0.52478601,  0.5182824 ,  0.51263776,  0.51634086,  0.52578968,  0.52028867,  0.50338941,  0.48649016,  0.44672771,  0.39875701,  0.33886324,  0.28062129,  0.22263785,  0.16738275,  0.15663858,  0.16479745,  0.17216911,  0.17883796,  0.18466754,  0.17679749,  0.14051378};
float SS_servo7[] = {-0.59338403, -0.6123598 , -0.60333581, -0.60828688, -0.6070766 , -0.59815811, -0.56202416, -0.50059452, -0.43343729, -0.35126115, -0.31552249, -0.2353037 , -0.20005758, -0.23787087, -0.31835562, -0.36297718, -0.38882764, -0.3941281 , -0.39405608, -0.39366554, -0.39291272, -0.39221399, -0.39168159, -0.39141543, -0.41056672, -0.40600033, -0.3050987 , -0.32089942, -0.35556744, -0.42205607, -0.49813809, -0.55738995, -0.58590426, -0.61033491, -0.65877367, -0.71962957, -0.75387604, -0.74016694, -0.72645785, -0.66900471, -0.6003462 , -0.52494526, -0.44713095, -0.36401599, -0.28886397, -0.32103468, -0.39954741, -0.47603244, -0.55527575, -0.63642049, -0.68539442, -0.64065735};

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
