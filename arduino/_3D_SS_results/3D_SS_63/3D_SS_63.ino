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
float SS_servo0[] = {-0.66996134, -0.6659619 , -0.60929693, -0.55113923, -0.49026204, -0.4226251 , -0.34505991, -0.26455233, -0.18369711, -0.10241274, -0.02137477,  0.02579089,  0.00258387, -0.00468777, -0.00342476, -0.00431072, -0.03473512, -0.07383531, -0.11285161, -0.14956549, -0.17793579, -0.19885701, -0.22503451, -0.25291704, -0.27482014, -0.29342835, -0.31128439, -0.32668548, -0.34276896, -0.36140639, -0.38161272, -0.40739495, -0.43275557, -0.46536958, -0.49000038, -0.51379621, -0.53566238, -0.56110063, -0.58704747, -0.60358873, -0.60860344, -0.61379303, -0.61851137, -0.62229806, -0.6318172 };
float SS_servo1[] = {0.94115406, 0.97658639, 0.99945592, 1.02403331, 1.04441272, 1.05753457, 1.06059533, 1.06060286, 1.06063665, 1.05319563, 0.99534933, 0.98944851, 1.14150207, 1.25815775, 1.35668684, 1.43320505, 1.43041964, 1.40300511, 1.37497725, 1.33777792, 1.3255283 , 1.3452433 , 1.34769097, 1.32368859, 1.27126689, 1.21191623, 1.15441772, 1.09593904, 1.05313509, 1.04334008, 1.04601336, 1.05548931, 1.05950123, 1.09544818, 1.13275638, 1.16147788, 1.1892798 , 1.22503333, 1.24967505, 1.21870769, 1.14656506, 1.07444545, 1.00139227, 0.92670883, 0.89188734};      
float SS_servo2[] = {-0.28292033, -0.29216216, -0.30203968, -0.31156791, -0.32158963, -0.34178755, -0.37466534, -0.4170116 , -0.46096327, -0.5069179 , -0.55200831, -0.58611123, -0.60139313, -0.60922134, -0.6155483 , -0.62296439, -0.63104565, -0.63575436, -0.63942948, -0.64254084, -0.64982279, -0.65376332, -0.66102339, -0.64939851, -0.58311099, -0.51531591, -0.44719502, -0.37974379, -0.30667633, -0.2263038 , -0.14528132, -0.06450722,  0.016139  ,  0.08745476,  0.06938138,  0.01817741, -0.02481896, -0.06532168, -0.10553893, -0.14208463, -0.16874212, -0.19168635, -0.21351565, -0.23312751, -0.2580044 };
float SS_servo3[] = {0.94881393, 0.90325991, 0.8327246 , 0.76123087, 0.69887618, 0.67632595, 0.71066796, 0.78522097, 0.87520514, 0.97084777, 1.06293343, 1.1652368 , 1.25411576, 1.32759629, 1.39542492, 1.44418272, 1.40345655, 1.32926292, 1.25350459, 1.17603866, 1.13276541, 1.11548108, 1.08684241, 1.08688599, 1.10006162, 1.11374189, 1.12656563, 1.14004937, 1.12891015, 1.05405562, 0.97322307, 0.89216815, 0.81287291, 0.73708682, 0.78037782, 0.88229206, 0.97159101, 1.05740308, 1.12663673, 1.13421264, 1.09202226, 1.04444198, 0.99420935, 0.93899923, 0.9333666 };      
float SS_servo4[] = {0.64662341, 0.6601357 , 0.65989572, 0.65902871, 0.65762018, 0.65583696, 0.64827009, 0.6423021 , 0.64152714, 0.6395739 , 0.6393784 , 0.63183772, 0.61473468, 0.59194911, 0.5673785 , 0.54894072, 0.54752404, 0.55606496, 0.56482352, 0.56702828, 0.55860371, 0.53894986, 0.51858995, 0.48321922, 0.43158033, 0.38093429, 0.32764273, 0.28060201, 0.22844943, 0.17473587, 0.12319306, 0.06921035, 0.01232787, 0.00333692, 0.07682983, 0.15755247, 0.23863415, 0.31978892, 0.40072303, 0.47587284, 0.53029547, 0.56782817, 0.59921312, 0.62005158, 0.6370534 };      
float SS_servo5[] = {-0.74506517, -0.82349186, -0.90147788, -0.9818629 , -1.0575113 , -1.11741089, -1.13972215, -1.14543987, -1.14721223, -1.14311113, -1.14446052, -1.09819944, -1.01340949, -0.91939918, -0.82396702, -0.75780325, -0.79964243, -0.89066551, -0.98403681, -1.07148931, -1.11410573, -1.09900835, -1.10330955, -1.09599814, -1.06289203, -1.03462794, -1.00140473, -0.98184728, -0.94108646, -0.86908537, -0.79233706, -0.71518233, -0.63761013, -0.62807047, -0.68942107, -0.71043687, -0.76556094, -0.82108171, -0.87422706, -0.87242001, -0.84408   , -0.79623524, -0.74298572, -0.691442  , -0.7507103 };
float SS_servo6[] = { 0.23120275,  0.21887187,  0.19179916,  0.16576884,  0.13653744,  0.10863737,  0.07776335,  0.04848495,  0.01886355, -0.01453327, -0.04853157, -0.06006801, -0.00165229,  0.07922955,  0.16004973,  0.23850341,  0.29829822,  0.35795877,  0.42024275,  0.49249316,  0.52187831,  0.49745069,  0.49645894,  0.48989266,  0.49962731,  0.51150115,  0.52117899,  0.53018015,  0.53347026,  0.52035524,  0.50671508,  0.49489437,  0.4956182 ,  0.4764564 ,  0.43044061,  0.38087703,  0.3326064 ,  0.28554211,  0.25441185,  0.26325712,  0.27226277,  0.28297444,  0.28936212,  0.28725787,  0.26600626};
float SS_servo7[] = {-0.8261966 , -0.8661629 , -0.90335414, -0.94812304, -0.9814774 , -0.9986316 , -0.96883335, -0.91293995, -0.83899023, -0.75431994, -0.66693743, -0.57407738, -0.58111009, -0.62068349, -0.63233114, -0.62967247, -0.60768726, -0.58161816, -0.55775236, -0.54594582, -0.4959283 , -0.40000815, -0.34668763, -0.27612738, -0.34258536, -0.43472204, -0.52233564, -0.60937053, -0.67831896, -0.69542558, -0.70502507, -0.72352308, -0.77071108, -0.76494066, -0.68750896, -0.60521896, -0.52159272, -0.43861059, -0.39733864, -0.47011199, -0.5658446 , -0.6686484 , -0.76505315, -0.84890997, -0.86358284};

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
