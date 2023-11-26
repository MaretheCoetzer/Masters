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
float SS_servo0[] = {-0.60060463, -0.57910246, -0.51431042, -0.44839037, -0.37670993, -0.29766726, -0.21722865, -0.13624688, -0.05503367,  0.02547131,  0.07298109,  0.02686758, -0.02726283, -0.07412646, -0.11035171, -0.14707561, -0.18427846, -0.2215694 , -0.25753295, -0.29538397, -0.32875053, -0.35946394, -0.38213311, -0.38172751, -0.3661292 , -0.34913329, -0.33581951, -0.33514708, -0.35631659, -0.39011608, -0.4273813 , -0.46654562, -0.50650799, -0.53710479, -0.55871104, -0.56471562, -0.56809796, -0.57226752, -0.58093464, -0.59197785, -0.60320078, -0.60841168, -0.61263018, -0.61468988, -0.60442433};
float SS_servo1[] = {0.61718695, 0.61510974, 0.62798967, 0.64455179, 0.6538951 , 0.65550378, 0.65564835, 0.65549321, 0.65498155, 0.6548632 , 0.67042127, 0.75638207, 0.83967796, 0.90431062, 0.9270415 , 0.94207247, 0.95117974, 0.95176314, 0.93014391, 0.91114923, 0.91320133, 0.91594551, 0.89821943, 0.82604263, 0.74331369, 0.66226021, 0.59480364, 0.56803366, 0.60219381, 0.67521902, 0.75809996, 0.83966747, 0.91577467, 0.96135044, 0.97347876, 0.94908118, 0.9207038 , 0.89463961, 0.8757941 , 0.86037496, 0.84357578, 0.80109395, 0.7536008 , 0.72060637, 0.66358868};      
float SS_servo2[] = {-0.26384764, -0.26667815, -0.26017547, -0.24738761, -0.23209544, -0.22165584, -0.23684239, -0.26883075, -0.30857522, -0.34995872, -0.39065321, -0.42434047, -0.45023519, -0.47432315, -0.49015899, -0.50586346, -0.52425622, -0.54454619, -0.56015757, -0.57752469, -0.58955304, -0.58978942, -0.57851328, -0.5683399 , -0.52722109, -0.46122891, -0.38775682, -0.30897638, -0.22853824, -0.14799983, -0.06728851,  0.01284236,  0.08547719,  0.0914862 ,  0.03993546,  0.00874988, -0.02007491, -0.04933001, -0.08033214, -0.11150077, -0.13772877, -0.16093188, -0.18492875, -0.21081873, -0.2390792 };
float SS_servo3[] = {0.78834757, 0.75180046, 0.65735579, 0.55510366, 0.46880764, 0.41390102, 0.42774275, 0.48855765, 0.57063318, 0.65579976, 0.73289985, 0.77432906, 0.78931747, 0.79804437, 0.77965759, 0.75534125, 0.72912647, 0.69794558, 0.64245192, 0.5897933 , 0.55210911, 0.49768073, 0.4310352 , 0.46449234, 0.5053523 , 0.5204132 , 0.52764186, 0.52838502, 0.52087427, 0.51802226, 0.51457932, 0.4966256 , 0.48307378, 0.54289466, 0.62760845, 0.6571738 , 0.6808977 , 0.70623465, 0.73607791, 0.76554961, 0.77726321, 0.76515523, 0.75397729, 0.76838794, 0.78600839};      
float SS_servo4[] = {0.5894241 , 0.60417241, 0.60907355, 0.61352143, 0.61498955, 0.60614909, 0.59578233, 0.58500562, 0.57973792, 0.57557241, 0.57348574, 0.58658062, 0.59779243, 0.59517486, 0.57985498, 0.56962123, 0.56509301, 0.56435757, 0.55768302, 0.54250031, 0.51877362, 0.48859197, 0.45937158, 0.42626434, 0.38750505, 0.35464269, 0.32345141, 0.29561004, 0.27024584, 0.2430474 , 0.21729673, 0.18999455, 0.16868911, 0.15821429, 0.1670975 , 0.23243094, 0.31483362, 0.39741374, 0.4792831 , 0.56050327, 0.63627273, 0.66115686, 0.62753394, 0.62534599, 0.60969637};      
float SS_servo5[] = {-0.3906226 , -0.45409523, -0.53008646, -0.60374366, -0.65911015, -0.67960603, -0.681947  , -0.67022799, -0.66229376, -0.65484136, -0.65518132, -0.70491512, -0.7616768 , -0.79567665, -0.81533721, -0.85094607, -0.90384618, -0.9721352 , -1.04843499, -1.11333968, -1.13402303, -1.13560606, -1.14180685, -1.14642759, -1.11874536, -1.10034269, -1.07966118, -1.04869099, -1.00291936, -0.93677113, -0.87020754, -0.80584422, -0.75730971, -0.72099069, -0.72025702, -0.73164935, -0.72691307, -0.72161123, -0.71809649, -0.7157448 , -0.70999163, -0.65917862, -0.60800542, -0.57826875, -0.49636372};
float SS_servo6[] = {0.54608131, 0.52965805, 0.50259439, 0.47685551, 0.44996231, 0.41970441, 0.38306744, 0.34348554, 0.30042254, 0.25454244, 0.22194706, 0.21750105, 0.23158153, 0.27652151, 0.35596092, 0.43738446, 0.51858715, 0.59366661, 0.62340246, 0.63317869, 0.65734024, 0.65430549, 0.6368127 , 0.61020611, 0.61096927, 0.60948216, 0.60795354, 0.59653529, 0.57304098, 0.53495505, 0.49680885, 0.47248045, 0.46106728, 0.47640666, 0.49193223, 0.48695245, 0.48501732, 0.48595348, 0.49114287, 0.4990294 , 0.51824092, 0.53660301, 0.55261786, 0.55030881, 0.54967465};      
float SS_servo7[] = {-0.86173428, -0.86849569, -0.88625476, -0.90366141, -0.9050434 , -0.88002352, -0.82462709, -0.74903616, -0.65996061, -0.56439969, -0.46821327, -0.44177439, -0.45753673, -0.50117255, -0.5029745 , -0.5014648 , -0.50040356, -0.49417006, -0.45816974, -0.50812394, -0.53591009, -0.48183968, -0.40663667, -0.33760193, -0.38921101, -0.43434525, -0.47449108, -0.48231775, -0.45118363, -0.3774908 , -0.2978577 , -0.24706257, -0.2268907 , -0.27022877, -0.3269503 , -0.35233538, -0.383356  , -0.41900624, -0.46165421, -0.50937206, -0.58297252, -0.66997665, -0.75479685, -0.78697204, -0.82420224};

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
