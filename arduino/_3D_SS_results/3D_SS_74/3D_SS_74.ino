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
float SS_servo0[] = {-0.63241745, -0.61092276, -0.54561527, -0.48034552, -0.41207198, -0.3357333 , -0.25542985, -0.17475845, -0.0935252 , -0.01245275,  0.03191143, -0.00109466, -0.05078076, -0.09430202, -0.13471369, -0.16229948, -0.18656623, -0.21063747, -0.23164438, -0.25160445, -0.27704943, -0.30313916, -0.31690105, -0.31862547, -0.31597734, -0.3155752 , -0.31779931, -0.32960853, -0.35728067, -0.3868807 , -0.4219595 , -0.45701984, -0.49919389, -0.53928958, -0.57558332, -0.61006986, -0.63926969, -0.66059611, -0.6643008 , -0.66584875, -0.6670406 , -0.66412691, -0.66012864, -0.65288701, -0.63798113};
float SS_servo1[] = {0.66531781, 0.65344049, 0.66459928, 0.67991645, 0.69229541, 0.69658645, 0.69686381, 0.6969109 , 0.69667877, 0.6960259 , 0.72629231, 0.79555528, 0.87475943, 0.94240955, 1.0286813 , 1.07782788, 1.05507403, 1.02617892, 0.98605343, 0.94698983, 0.94626965, 0.94328354, 0.87912982, 0.81191725, 0.76795333, 0.72834265, 0.69858581, 0.70017143, 0.74433378, 0.79484253, 0.85577078, 0.91625063, 0.99707798, 1.06552496, 1.1178647 , 1.15839875, 1.18937102, 1.20120843, 1.14727911, 1.08568021, 1.02298276, 0.94761758, 0.86804302, 0.79033352, 0.70582759};      
float SS_servo2[] = {-0.2554736 , -0.26320136, -0.26527168, -0.26423483, -0.26299919, -0.2727921 , -0.29734814, -0.33424895, -0.37840429, -0.42113572, -0.45868063, -0.48080217, -0.49922621, -0.52230276, -0.5427036 , -0.55650581, -0.55713133, -0.55099951, -0.54446436, -0.54261885, -0.54836966, -0.54711198, -0.53349269, -0.48214405, -0.4576482 , -0.38494185, -0.30467316, -0.22412006, -0.14333642, -0.06250298,  0.01818096,  0.09868245,  0.16417904,  0.13519456,  0.0782168 ,  0.02737187, -0.02059773, -0.06387816, -0.10215976, -0.1306614 , -0.15883625, -0.18329776, -0.20594794, -0.22426731, -0.24544151};
float SS_servo3[] = {0.85879179, 0.83721906, 0.76645264, 0.68738091, 0.61937768, 0.59447287, 0.6179022 , 0.68206224, 0.76936512, 0.85664574, 0.92974685, 0.95444844, 0.95874106, 0.97417113, 1.00509068, 1.01472638, 0.95396788, 0.8775868 , 0.79443277, 0.72108736, 0.68284015, 0.62822064, 0.56506565, 0.58482119, 0.64282845, 0.65074988, 0.65111544, 0.65120554, 0.65105086, 0.65075881, 0.64332882, 0.61627021, 0.61405832, 0.68820887, 0.80262869, 0.89837304, 0.98591991, 1.0532297 , 1.06493433, 1.04620591, 1.02574936, 0.98552022, 0.94257925, 0.90019899, 0.87151229};      
float SS_servo4[] = {0.52883308, 0.52816409, 0.534896  , 0.54327424, 0.54831854, 0.54493753, 0.53662384, 0.52312025, 0.51349078, 0.50590339, 0.49572921, 0.50408814, 0.52810289, 0.54842403, 0.54464832, 0.51283779, 0.49865356, 0.49207311, 0.48675727, 0.48020179, 0.4645738 , 0.44611797, 0.43639916, 0.41411705, 0.36973955, 0.32805615, 0.28965692, 0.25058088, 0.20751216, 0.16508284, 0.12247328, 0.0799356 , 0.04481192, 0.03524256, 0.05410607, 0.06808339, 0.10166347, 0.17672596, 0.24966767, 0.32254848, 0.39540009, 0.46790894, 0.52640914, 0.55237583, 0.54193361};      
float SS_servo5[] = {-0.26886001, -0.2992541 , -0.37799046, -0.4630397 , -0.53406376, -0.57306304, -0.58653663, -0.57502342, -0.56093732, -0.54701451, -0.52706055, -0.55900252, -0.63430446, -0.70108571, -0.70582418, -0.65907563, -0.69095781, -0.74251133, -0.80201537, -0.85982132, -0.8791765 , -0.89468256, -0.96087031, -0.98393367, -0.93021979, -0.88262099, -0.83737095, -0.77814156, -0.69918601, -0.6185052 , -0.5376008 , -0.45672612, -0.36507963, -0.31564146, -0.33642589, -0.35328935, -0.41525094, -0.48486488, -0.48211901, -0.46931558, -0.45667184, -0.44587908, -0.42314286, -0.41849745, -0.32273054};
float SS_servo6[] = {0.38506396, 0.36251587, 0.33183304, 0.30818477, 0.28014733, 0.25293483, 0.22227138, 0.18577717, 0.14717443, 0.10582634, 0.06830515, 0.04433032, 0.04575797, 0.05834637, 0.10487897, 0.17952993, 0.25202252, 0.32743882, 0.40582717, 0.46789157, 0.47821594, 0.49412673, 0.5060715 , 0.50389609, 0.47726735, 0.46144994, 0.44810512, 0.42437612, 0.38660421, 0.35426162, 0.3365519 , 0.34887105, 0.33340997, 0.36502516, 0.39873059, 0.42442656, 0.41563153, 0.37885309, 0.37778025, 0.38564589, 0.39337594, 0.39952499, 0.40446281, 0.40500381, 0.39291429};      
float SS_servo7[] = {-0.80379072, -0.79270156, -0.80110837, -0.82883469, -0.838163  , -0.82886138, -0.79244723, -0.72560445, -0.64393708, -0.55300479, -0.44765029, -0.3783026 , -0.37690817, -0.39242864, -0.46133896, -0.48955735, -0.48097147, -0.47231618, -0.4677463 , -0.43954118, -0.40831335, -0.40866241, -0.30332429, -0.27576884, -0.26237414, -0.2708573 , -0.28069138, -0.25773312, -0.19530107, -0.14009041, -0.11407642, -0.14872266, -0.12202273, -0.19424063, -0.27700559, -0.35034968, -0.35534485, -0.30895293, -0.36121244, -0.43796595, -0.51521583, -0.59813552, -0.68218678, -0.75239005, -0.7904086 };

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
