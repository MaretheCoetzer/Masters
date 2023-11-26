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
float SS_servo0[] = {-0.51724752, -0.49687805, -0.43311711, -0.3679471 , -0.29696905, -0.21822921, -0.13777268, -0.05665656,  0.02428989,  0.10550167,  0.178439  ,  0.14804504,  0.10243057,  0.06541169,  0.04411493,  0.02639827,  0.01051893, -0.00769156, -0.02640844, -0.04778439, -0.06785836, -0.08958958, -0.1085497 , -0.12907227, -0.13615785, -0.137096  , -0.14466933, -0.15055744, -0.16766946, -0.20082972, -0.24164432, -0.28331055, -0.32574943, -0.36678117, -0.40429016, -0.4325453 , -0.4523745 , -0.4667741 , -0.4806219 , -0.4948746 , -0.50441348, -0.51031086, -0.5139467 , -0.51830486, -0.521673  , -0.52184917, -0.52445411, -0.51583723};
float SS_servo1[] = {0.65384959, 0.65155319, 0.66507429, 0.68082255, 0.69052494, 0.69237177, 0.69245125, 0.69207568, 0.66471973, 0.61451191, 0.58434146, 0.65234647, 0.73997661, 0.84237047, 0.94590277, 1.00692429, 1.00419391, 0.98600705, 0.96400385, 0.92196047, 0.88392102, 0.87164801, 0.85206977, 0.82855775, 0.78180176, 0.72916663, 0.68846045, 0.64869065, 0.64921339, 0.70092999, 0.78356372, 0.87173007, 0.96006568, 1.04809123, 1.11017688, 1.13076594, 1.12206911, 1.11694221, 1.11094423, 1.10284084, 1.07131146, 1.02127298, 0.95713487, 0.8945025 , 0.83169307, 0.77327121, 0.74108307, 0.68245163};
float SS_servo2[] = {-0.16905272, -0.16953382, -0.16914312, -0.16706919, -0.1718121 , -0.19975201, -0.25074857, -0.3034333 , -0.35634371, -0.41118705, -0.45846651, -0.482303  , -0.49273542, -0.50402723, -0.5108869 , -0.51798813, -0.5174277 , -0.51871124, -0.52020504, -0.52280299, -0.52110615, -0.52243052, -0.51850836, -0.51723448, -0.45185037, -0.37773759, -0.30519714, -0.23148674, -0.15148395, -0.07097439,  0.01007393,  0.09130668,  0.17235637,  0.24346899,  0.22120261,  0.16918807,  0.12341131,  0.08742274,  0.04970196,  0.01326169, -0.01576319, -0.03911516, -0.05525973, -0.07169892, -0.08975433, -0.10771323, -0.1335707 , -0.16041779};
float SS_servo3[] = {0.67698208, 0.6293607 , 0.55266443, 0.47547571, 0.42660721, 0.4445915 , 0.52666477, 0.62146627, 0.71741381, 0.81549214, 0.9043305 , 0.9385146 , 0.94436731, 0.97680914, 1.02820463, 1.05169299, 1.01868672, 0.97213334, 0.92218369, 0.85439972, 0.78490728, 0.73782805, 0.68084309, 0.67269331, 0.68348157, 0.69069601, 0.69865968, 0.70552012, 0.70188304, 0.68944833, 0.68026812, 0.64181107, 0.58294076, 0.53685249, 0.58257226, 0.6615756 , 0.71416801, 0.76005452, 0.81120996, 0.85713862, 0.86295461, 0.84061654, 0.79212157, 0.74639514, 0.70657344, 0.677398  , 0.69082873, 0.70016768};
float SS_servo4[] = {0.47735068, 0.49668815, 0.50638003, 0.5157082 , 0.52180376, 0.51745154, 0.51148903, 0.50808232, 0.50743846, 0.51066178, 0.51422354, 0.51583218, 0.52393068, 0.51761262, 0.48729692, 0.45235324, 0.42631876, 0.41767393, 0.41038896, 0.41024479, 0.40479314, 0.38320231, 0.36304026, 0.34042601, 0.31313673, 0.27383943, 0.24294669, 0.21502584, 0.18938189, 0.15671917, 0.12688952, 0.0951714 , 0.06033466, 0.04649424, 0.06355045, 0.08064633, 0.10942693, 0.1870149 , 0.26839611, 0.34935716, 0.42715371, 0.5024621 , 0.55797201, 0.57185759, 0.54905498, 0.52604051, 0.50782959, 0.482644  };
float SS_servo5[] = {-0.35850778, -0.43928468, -0.52617597, -0.61256028, -0.68171432, -0.71488286, -0.72893744, -0.73703517, -0.74963647, -0.76861982, -0.78049185, -0.79815238, -0.83005968, -0.81210157, -0.72067461, -0.64349779, -0.62154602, -0.64798297, -0.68124448, -0.75034269, -0.80593951, -0.81301453, -0.82495414, -0.83776455, -0.83641024, -0.8041395 , -0.79384055, -0.78643267, -0.76823985, -0.71702053, -0.65708252, -0.58956503, -0.51694698, -0.46597217, -0.49001537, -0.53064433, -0.60672513, -0.67815943, -0.72036336, -0.7286951 , -0.72441116, -0.71827348, -0.70387435, -0.66179232, -0.60252878, -0.56289732, -0.4773581 , -0.39176376};
float SS_servo6[] = { 0.17793079,  0.15791647,  0.13273126,  0.11150396,  0.08560667,  0.05652987,  0.0215502 , -0.0200086 , -0.06168861, -0.10785129, -0.15152329, -0.1850926 , -0.18452448, -0.13354993, -0.05350143,  0.02700494,  0.1041699 ,  0.17455889,  0.24380808,  0.30707199,  0.37480153,  0.38523144,  0.38542742,  0.37948796,  0.38690122,  0.4043453 ,  0.42232989,  0.44024897,  0.4512656 ,  0.44128099,  0.43053144,  0.42092532,  0.42296782,  0.4250468 ,  0.45004523,  0.46454602,  0.45459335,  0.40032215,  0.34584768,  0.2965083 ,  0.2688287 ,  0.2584945 ,  0.26258999,  0.26568592,  0.26637748,  0.25971588,  0.22369448,  0.19410352};
float SS_servo7[] = {-0.63176666, -0.63841804, -0.66283978, -0.6954209 , -0.70435493, -0.68553228, -0.63341758, -0.55496748, -0.47530262, -0.38701365, -0.26940744, -0.17751786, -0.15038032, -0.22886443, -0.29082004, -0.30176582, -0.2964719 , -0.28217723, -0.26703939, -0.24747388, -0.31574403, -0.28311336, -0.22046564, -0.14843172, -0.17245536, -0.252284  , -0.33689061, -0.41919695, -0.47501471, -0.47521286, -0.45935068, -0.44164821, -0.44778168, -0.4519022 , -0.51672927, -0.57897259, -0.60442994, -0.52826614, -0.44914962, -0.38039042, -0.3694405 , -0.40544228, -0.48140859, -0.55714517, -0.62853597, -0.67703748, -0.64147709, -0.6253295 };

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
