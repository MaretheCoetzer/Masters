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

// Travelled 18 cm forward, turned 12deg to the right

// Insert servo angles here:
// For 3D gaits, these angles are obtained from linearisation.py
// For 2D gaits, these angles are obtained from 2D_trajectory_reader.py
float SS_servo0[] = {-0.48236425, -0.48774284, -0.47488233, -0.41315501, -0.33647658, -0.25533113, -0.16727848, -0.09905704, -0.06956389, -0.04204525, -0.01980366, -0.03090918, -0.07373602, -0.10698131, -0.13098437, -0.1513393 , -0.1706055 , -0.19090065, -0.21204543, -0.23053116, -0.24791628, -0.26253795, -0.27528029, -0.28369679, -0.29476843, -0.30596245, -0.32045116, -0.33393858, -0.34986493, -0.36580642, -0.3857588 , -0.40674348, -0.42602284, -0.44122222, -0.45154541, -0.45979183, -0.46703756, -0.47621123, -0.48246693, -0.48072759};
float SS_servo1[] = {0.73662829, 0.71436711, 0.717496  , 0.78502018, 0.86213943, 0.92601361, 0.94065067, 0.89761657, 0.82566106, 0.75525222, 0.69979565, 0.70642761, 0.7794645 , 0.84114057, 0.88387967, 0.91190332, 0.92598312, 0.92272845, 0.90374644, 0.87856407, 0.85618016, 0.83451651, 0.81695142, 0.80642414, 0.80694371, 0.81025513, 0.81815957, 0.81752463, 0.81354126, 0.81360125, 0.82554318, 0.83810406, 0.84119485, 0.83819697, 0.83561534, 0.83074398, 0.82470747, 0.8199536 , 0.79899137, 0.74977503};
float SS_servo2[] = {-0.23006858, -0.24468289, -0.26074877, -0.26627694, -0.27288134, -0.28287149, -0.29583786, -0.31116834, -0.33025037, -0.35000318, -0.37059089, -0.39446475, -0.41577157, -0.42859094, -0.43365835, -0.43507013, -0.43880921, -0.44583917, -0.44869678, -0.45697416, -0.46907455, -0.4808435 , -0.46287861, -0.39918025, -0.32524176, -0.24556484, -0.1608834 , -0.09974194, -0.0726053 , -0.04514961, -0.02433892, -0.0326804 , -0.07645375, -0.11373943, -0.1373988 , -0.15620181, -0.17079989, -0.18577373, -0.2041589 , -0.22223886};
float SS_servo3[] = {0.7200893 , 0.71623538, 0.70478051, 0.68930358, 0.68301404, 0.68707944, 0.69771655, 0.70666378, 0.71332504, 0.72648271, 0.75057354, 0.77925586, 0.8026755 , 0.81835132, 0.82050711, 0.81103853, 0.79694346, 0.77137861, 0.72181625, 0.68189433, 0.65435191, 0.63215248, 0.63663861, 0.70656878, 0.78571362, 0.84287502, 0.83821224, 0.79130795, 0.71630803, 0.64433467, 0.58215763, 0.57314944, 0.63096474, 0.67658135, 0.70159701, 0.71708536, 0.72337325, 0.72745621, 0.7299826 , 0.72059476};
float SS_servo4[] = {0.49085905, 0.48992916, 0.4908276 , 0.48749805, 0.48816241, 0.48892166, 0.48539689, 0.47435034, 0.45588305, 0.43151716, 0.41274332, 0.39319692, 0.37521815, 0.35981027, 0.35040655, 0.34576902, 0.34069209, 0.32746374, 0.30510823, 0.28087079, 0.26174182, 0.24472155, 0.22845045, 0.21683717, 0.20441394, 0.18972883, 0.16966281, 0.14044009, 0.09571252, 0.06374162, 0.07231265, 0.09992717, 0.13218409, 0.1816209 , 0.25033115, 0.32113267, 0.39590929, 0.46263225, 0.49377868, 0.4931051 };
float SS_servo5[] = {-0.67144885, -0.70317483, -0.74880031, -0.76733496, -0.7859642 , -0.80164502, -0.80864905, -0.80906508, -0.80546191, -0.78519025, -0.7664647 , -0.7468892 , -0.73007582, -0.70973443, -0.69971081, -0.7039065 , -0.71647372, -0.72933296, -0.74046108, -0.75048264, -0.76834223, -0.78519184, -0.79421991, -0.79893801, -0.79677259, -0.78709596, -0.76765165, -0.7346266 , -0.67713851, -0.64115185, -0.68319329, -0.75786973, -0.84338248, -0.91393408, -0.91275537, -0.86415698, -0.79292552, -0.7020068 , -0.6276034 , -0.65975246};
float SS_servo6[] = {0.24779642, 0.24351495, 0.23195044, 0.21732222, 0.19959513, 0.18054751, 0.157336  , 0.12725897, 0.08473258, 0.06440069, 0.09249871, 0.13240186, 0.18617347, 0.2580522 , 0.33814525, 0.41720071, 0.47623517, 0.49380123, 0.47061838, 0.43848476, 0.42907004, 0.43745947, 0.44358696, 0.44335564, 0.4389935 , 0.43442708, 0.42511958, 0.41503967, 0.40141084, 0.38016526, 0.35911728, 0.34134147, 0.32864246, 0.31429814, 0.29961491, 0.28443241, 0.27410619, 0.26678881, 0.25960858, 0.24888699};
float SS_servo7[] = {-0.69444085, -0.71965463, -0.74116878, -0.73899842, -0.72296157, -0.70036441, -0.66830758, -0.62910652, -0.57359835, -0.55667647, -0.62479546, -0.71033295, -0.77432575, -0.76022899, -0.71323571, -0.65362003, -0.56127418, -0.48657387, -0.47936639, -0.46362323, -0.49271162, -0.55474936, -0.60451314, -0.62701793, -0.63698437, -0.64434217, -0.64483339, -0.65144366, -0.66033146, -0.64980636, -0.63562194, -0.62847932, -0.63616432, -0.6386436 , -0.63242503, -0.6239701 , -0.62518611, -0.63544138, -0.65615897, -0.68005515};

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
        servo2=map(servo2_deg,-37,90,1500,2670);
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
        servo2=map(servo2_deg,-37,90,1500,2670);
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
