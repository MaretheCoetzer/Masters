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

float SS_servo0[] = {-0.41347653, -0.46944298, -0.50186613, -0.51475744, -0.48888297, -0.40945349, -0.32370954, -0.23126615, -0.13691282, -0.07202001, -0.04766415, -0.04654356, -0.07757753, -0.1064425 , -0.12770423, -0.14530311, -0.16106698, -0.17346226, -0.18494472, -0.1937142 , -0.20248287, -0.21232807, -0.22509522, -0.24126434, -0.25680135, -0.27145916, -0.28670481, -0.30625197, -0.32754872, -0.34878483, -0.37310403, -0.39421679, -0.41014709, -0.42441872, -0.43629101, -0.4403788 , -0.44190503, -0.43591973, -0.42128449, -0.39853231};
float SS_servo1[] = {0.24194708, 0.26062956, 0.24591185, 0.2147798 , 0.24226641, 0.38839548, 0.53385924, 0.65351401, 0.72135399, 0.72328449, 0.67522948, 0.6415268 , 0.63839489, 0.63583259, 0.62962449, 0.62052779, 0.61692368, 0.62442598, 0.62906602, 0.62475467, 0.61492719, 0.60539081, 0.60102838, 0.60271579, 0.5953018 , 0.57213182, 0.5511334 , 0.5345408 , 0.52062138, 0.50912828, 0.51082645, 0.52449177, 0.52855723, 0.52466482, 0.50471558, 0.46107731, 0.41377733, 0.36007786, 0.30226093, 0.24349154};
float SS_servo2[] = {-0.2663569 , -0.32358592, -0.36727636, -0.39347127, -0.40441725, -0.41117815, -0.41534615, -0.42075007, -0.42707919, -0.43499976, -0.44506458, -0.45840284, -0.47062454, -0.48005806, -0.48786568, -0.48931449, -0.4861044 , -0.48755175, -0.48134421, -0.47580121, -0.44949769, -0.37677934, -0.29925064, -0.21966536, -0.15944065, -0.13415144, -0.10974786, -0.09035144, -0.07584827, -0.07551129, -0.09771948, -0.12780296, -0.15457669, -0.17799537, -0.20151284, -0.22285655, -0.24388982, -0.25684943, -0.259415  , -0.2500292 };
float SS_servo3[] = {0.4288544 , 0.45306145, 0.44992946, 0.43065389, 0.41006886, 0.40911526, 0.4116965 , 0.41774002, 0.42209374, 0.41781154, 0.40285305, 0.38547632, 0.36186579, 0.33828599, 0.31730039, 0.29117759, 0.26815003, 0.26364365, 0.25083113, 0.23296146, 0.27081853, 0.4105864 , 0.55605147, 0.69796109, 0.76715724, 0.72418288, 0.68202588, 0.63909685, 0.6000315 , 0.57469129, 0.57956494, 0.60321542, 0.61745057, 0.62052179, 0.61198652, 0.58710264, 0.55964056, 0.5229924 , 0.47862392, 0.42830226};
float SS_servo4[] = {0.4286976 , 0.49024273, 0.50899415, 0.51428189, 0.51104422, 0.50545013, 0.50512248, 0.49940823, 0.48913322, 0.47392885, 0.45536824, 0.42989005, 0.40521612, 0.38210515, 0.36294785, 0.34709682, 0.33372286, 0.32132139, 0.31431835, 0.3101994 , 0.30557975, 0.29736294, 0.28929834, 0.27988027, 0.26652384, 0.24892226, 0.22506185, 0.19545953, 0.15682375, 0.14527413, 0.19465495, 0.27834368, 0.36536462, 0.44466458, 0.48893737, 0.49124339, 0.47022864, 0.44429285, 0.42371278, 0.40764818};
float SS_servo5[] = {-0.20519644, -0.29114857, -0.35234571, -0.4010672 , -0.43055093, -0.43132061, -0.43051511, -0.42239944, -0.41215149, -0.40831975, -0.41462923, -0.4194155 , -0.43074772, -0.441125  , -0.45092996, -0.46252462, -0.46904716, -0.46255853, -0.46293517, -0.47133926, -0.4841518 , -0.49407869, -0.50235573, -0.50691162, -0.51491885, -0.53292212, -0.54286454, -0.54691521, -0.54035365, -0.5604739 , -0.56467515, -0.4548476 , -0.32971592, -0.19930844, -0.11304538, -0.14705172, -0.17047952, -0.19012878, -0.19185903, -0.17955236};
float SS_servo6[] = {0.25460109, 0.30321779, 0.31699628, 0.31685852, 0.30530601, 0.2938755 , 0.27757472, 0.25880268, 0.23504308, 0.20712417, 0.17447711, 0.13808555, 0.10364037, 0.09512518, 0.11342596, 0.13951031, 0.18339285, 0.25924651, 0.34062238, 0.41407562, 0.45206945, 0.44706132, 0.43825837, 0.43186748, 0.42980874, 0.43463736, 0.43399666, 0.42877628, 0.41950404, 0.40774998, 0.38951201, 0.37059953, 0.35041347, 0.3347913 , 0.32693928, 0.31919151, 0.31079739, 0.29589771, 0.27464964, 0.2420893 };
float SS_servo7[] = {-0.43055063, -0.51512547, -0.57962711, -0.62876367, -0.65175021, -0.64756598, -0.63035456, -0.60749151, -0.58170895, -0.56236866, -0.54983519, -0.53876226, -0.53641826, -0.55994533, -0.60369306, -0.64893854, -0.62830333, -0.48771054, -0.33207456, -0.17155576, -0.05966835, -0.06445579, -0.07245009, -0.07955582, -0.09888815, -0.1392928 , -0.17278226, -0.20202498, -0.22633939, -0.24626726, -0.25202812, -0.24425077, -0.23879373, -0.2434075 , -0.26675948, -0.30229355, -0.3392595 , -0.37137571, -0.39532769, -0.40180985};

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
