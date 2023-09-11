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

float SS_servo0[] = {-0.36108032, -0.43371597, -0.49152997, -0.49628849, -0.42342595, -0.33425499, -0.24450695, -0.1551067 , -0.06392036,  0.01987016,  0.07378966,  0.07317854,  0.04090186,  0.01505388, -0.00256748, -0.01324909, -0.02157262, -0.02871532, -0.03890835, -0.05447955, -0.07272626, -0.08750311, -0.10372093, -0.12105385, -0.1406819 , -0.16056486, -0.18292561, -0.20796383, -0.23378954, -0.25918218, -0.28464469, -0.3102795 , -0.33202724, -0.34804041, -0.35898887, -0.36551867, -0.36688967, -0.36400899, -0.35609794, -0.34026512};
float SS_servo1[] = {0.48957515, 0.54101513, 0.55088278, 0.59201711, 0.74047621, 0.88241354, 0.95837059, 1.00633205, 1.03221041, 1.04612993, 1.03140886, 1.0118923 , 1.04290001, 1.06490564, 1.07870885, 1.0949188 , 1.10563995, 1.10652364, 1.09983251, 1.0744566 , 1.03679759, 1.01474597, 0.9941421 , 0.9708187 , 0.95266167, 0.93979597, 0.91802465, 0.88887178, 0.85873791, 0.83519598, 0.81748128, 0.79931013, 0.78282647, 0.7663722 , 0.7381985 , 0.70808025, 0.67698638, 0.63048292, 0.56045631, 0.48615228};
float SS_servo2[] = {-0.13317275, -0.20649409, -0.27143913, -0.31311442, -0.33546872, -0.35073907, -0.36528802, -0.37715661, -0.38801178, -0.39864956, -0.41317803, -0.43118329, -0.44950785, -0.46278568, -0.46831599, -0.46851845, -0.46371138, -0.45784167, -0.46006078, -0.46515432, -0.434972  , -0.36577259, -0.29215808, -0.22041343, -0.15289265, -0.07539289, -0.01795502,  0.0083796 ,  0.02190558,  0.03713195,  0.04259307,  0.02178594, -0.01671389, -0.04619473, -0.06950405, -0.08754656, -0.09760328, -0.10559567, -0.11437866, -0.11151186};
float SS_servo3[] = {0.63614478, 0.69364387, 0.71698757, 0.72057955, 0.73278159, 0.74764246, 0.76098037, 0.77396241, 0.78745952, 0.80054801, 0.80942998, 0.81061376, 0.8138682 , 0.81185067, 0.80495854, 0.80321581, 0.79458155, 0.77920722, 0.76384298, 0.73117594, 0.72287443, 0.81880012, 0.9015284 , 0.9527867 , 1.00839915, 1.06659892, 1.0604381 , 1.01496625, 0.9530985 , 0.89665612, 0.84925424, 0.82917841, 0.83726456, 0.83917098, 0.82738423, 0.81051967, 0.78607066, 0.74810312, 0.69346761, 0.62893489};
float SS_servo4[] = {0.38010566, 0.45344557, 0.48005478, 0.48481289, 0.49017046, 0.49202846, 0.48902629, 0.48190913, 0.47362428, 0.46136586, 0.44003016, 0.41390749, 0.38908578, 0.36895154, 0.35148354, 0.33466823, 0.32067675, 0.30737011, 0.29537145, 0.27882567, 0.25784572, 0.2388834 , 0.22193103, 0.20205507, 0.18093068, 0.15778458, 0.12946808, 0.08913934, 0.03953746, 0.00686917, 0.01788404, 0.04581037, 0.1061896 , 0.18928379, 0.26112558, 0.33216058, 0.4047221 , 0.44362255, 0.41538549, 0.36209521};
float SS_servo5[] = {-0.48520842, -0.58778066, -0.66932675, -0.72179962, -0.74363669, -0.75028877, -0.75186314, -0.74552124, -0.73522326, -0.71979705, -0.70300125, -0.69245658, -0.68210012, -0.67515798, -0.66655194, -0.64796456, -0.63363632, -0.62519321, -0.62708904, -0.64391083, -0.66806209, -0.67896486, -0.69345535, -0.70816588, -0.72023492, -0.72550492, -0.73493178, -0.73910618, -0.73395753, -0.74359902, -0.80201244, -0.8619827 , -0.88085966, -0.84921002, -0.80157413, -0.69863217, -0.54543876, -0.43810584, -0.45229246, -0.45071064};
float SS_servo6[] = {0.11286503, 0.17903537, 0.19854087, 0.19705806, 0.19345411, 0.18669476, 0.17380024, 0.15813466, 0.13613686, 0.10784564, 0.07362154, 0.05138452, 0.06539531, 0.09953502, 0.15170436, 0.22718645, 0.30598548, 0.38511403, 0.46111434, 0.50809615, 0.50433781, 0.49192382, 0.48425991, 0.47834397, 0.47183569, 0.46409627, 0.4548501 , 0.43946897, 0.42005504, 0.39482518, 0.37124401, 0.34695093, 0.32163775, 0.29648241, 0.27182913, 0.24617543, 0.21901309, 0.18848268, 0.14732301, 0.09652594};
float SS_servo7[] = {-0.64939695, -0.76349067, -0.85233076, -0.9063088 , -0.920559  , -0.91640071, -0.90299342, -0.8812224 , -0.84802675, -0.8061668 , -0.76509283, -0.75295583, -0.77957417, -0.821452  , -0.82491041, -0.72972641, -0.5927626 , -0.44555104, -0.31033513, -0.25293861, -0.28438222, -0.300736  , -0.32342451, -0.35169643, -0.37863878, -0.40012723, -0.43030503, -0.46439762, -0.49680429, -0.51789473, -0.53671339, -0.5548457 , -0.56629317, -0.57164283, -0.58124166, -0.5873366 , -0.58879731, -0.59843693, -0.61315118, -0.61182356};

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=70000;
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);

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
  delay (3000);

  servo0_deg=SS_servo0[0]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[0]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[0]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[0]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[0]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[0]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[0]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[0]/3.14159265*180-servo6_deg;
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

        delay(1000);
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
}

void loop() 
{
  if(Serial.available()>0)
  {
    first=Serial.read();
    if(first=='1')
    {
      input=1;
    }
    if(first=='0')
    {
      input=0;
    }
  }
  if(input==1)
  {
      if(esp_timer_get_time()>=end_time)
      {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
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
        if(node>=steps)
           {
              node=0;
            }
         end_time=esp_timer_get_time()+time_step;
      }
   }

  if(input == 0)
  {
   servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
        servo7=map(servo7_deg,-90,90,920,2620);
        
        pca9685.setPWM(SER0,0,servo0);
        pca9685.setPWM(SER1,0,servo1);
        pca9685.setPWM(SER2,0,servo2);
        pca9685.setPWM(SER3,0,servo3);
        pca9685.setPWM(SER4,0,servo4);
        pca9685.setPWM(SER5,0,servo5);
        pca9685.setPWM(SER6,0,servo6);
        pca9685.setPWM(SER7,0,servo7);

        // Serial.print("Node:");
        // Serial.println(node);
        // Serial.print("servo0 -> rads:");
        // Serial.print(SS_servo0[node]);
        // Serial.print("        deg:");
        // Serial.print(servo0_deg);
        // Serial.print("        us:");
        // Serial.println(servo0);
        // Serial.print("servo1 -> rads:");
        // Serial.print(SS_servo1[node]);
        // Serial.print("        deg:");
        // Serial.print(servo1_deg);
        // Serial.print("        us:");
        // Serial.println(servo1);
        // Serial.print("servo2 -> rads:");
        // Serial.print(SS_servo2[node]);
        // Serial.print("        deg:");
        // Serial.print(servo2_deg);
        // Serial.print("        us:");
        // Serial.println(servo2);
        // Serial.print("servo3 -> rads:");
        // Serial.print(SS_servo3[node]);
        // Serial.print("        deg:");
        // Serial.print(servo3_deg);
        // Serial.print("        us:");
        // Serial.println(servo3);
        // Serial.print("servo4 -> rads:");
        // Serial.print(SS_servo4[node]);
        // Serial.print("        deg:");
        // Serial.print(servo4_deg);
        // Serial.print("        us:");
        // Serial.println(servo4);
        // Serial.print("servo5 -> rads:");
        // Serial.print(SS_servo5[node]);
        // Serial.print("        deg:");
        // Serial.print(servo5_deg);
        // Serial.print("        us:");
        // Serial.println(servo5);
        // Serial.print("servo6 -> rads:");
        // Serial.print(SS_servo6[node]);
        // Serial.print("        deg:");
        // Serial.print(servo6_deg);
        // Serial.print("        us:");
        // Serial.println(servo6);
        // Serial.print("servo7 -> rads:");
        // Serial.print(SS_servo7[node]);
        // Serial.print("        deg:");
        // Serial.print(servo7_deg);
        // Serial.print("        us:");
        // Serial.println(servo7);
  
  }
}
