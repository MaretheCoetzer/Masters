//Gait template
//FIRST STEP OF LEG 1 MODIFIED
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

float SS_servo0[] = { 0.29778889,  0.28089719,  0.15526608,  0.08754312,  0.04024765, -0.00456085, -0.0489753 , -0.09691065, -0.15028241, -0.20229226, -0.24286404, -0.25743709, -0.26414067, -0.26952727, -0.28092138, -0.29448415, -0.31139468, -0.33053658, -0.34992864, -0.35983137, -0.34420833, -0.32066769, -0.30164234, -0.29294322, -0.28009987, -0.25308055, -0.22980691, -0.23080927, -0.24782866, -0.25771182, -0.27312702, -0.29429537, -0.3134938 , -0.32945106, -0.34196733, -0.35284329, -0.36002267, -0.34753359, -0.31690035, -0.28259253, -0.23806763};
float SS_servo1[] = {0.59048526, 0.57282171, 0.64284644, 0.63777346, 0.6095475 , 0.58078081, 0.54962123, 0.52114436, 0.50218547, 0.49408249, 0.49253833, 0.4912533 , 0.47934109, 0.46520925, 0.4549925 , 0.44915056, 0.44535654, 0.44284561, 0.44348641, 0.42642039, 0.36842257, 0.31694923, 0.27393442, 0.24597598, 0.21764531, 0.17170381, 0.13047617, 0.12166113, 0.14515366, 0.16529926, 0.20970201, 0.2827278 , 0.35341848, 0.42883824, 0.50645326, 0.5837327 , 0.64332787, 0.64545419, 0.59180795, 0.53246757, 0.48117464};
float SS_servo2[] = {-0.02587535, -0.14846561, -0.2293279 , -0.26602961, -0.29407059, -0.32105047, -0.35124316, -0.38247092, -0.41736854, -0.45445525, -0.47393377, -0.46416631, -0.45119243, -0.43828667, -0.4298085 , -0.4273938 , -0.428895  , -0.43224624, -0.43959425, -0.4443601 , -0.44295771, -0.43582098, -0.42828585, -0.42056065, -0.41536445, -0.40941513, -0.4013414 , -0.39449344, -0.37426807, -0.34751642, -0.33788604, -0.35315599, -0.36722993, -0.37712591, -0.3835502 , -0.39148833, -0.39681374, -0.38637696, -0.35443315, -0.3190176 , -0.27269995};
float SS_servo3[] = {0.53323376, 0.66523887, 0.69478399, 0.65314434, 0.59875798, 0.54309969, 0.49027275, 0.43803473, 0.39252853, 0.35866204, 0.33619559, 0.31933191, 0.29854097, 0.2769695 , 0.25913537, 0.24824512, 0.24052532, 0.23346209, 0.23126585, 0.21202894, 0.1819786 , 0.17607135, 0.1816685 , 0.18821254, 0.19538533, 0.20316962, 0.21301259, 0.2170339 , 0.20473313, 0.16293484, 0.15506715, 0.21590057, 0.28672705, 0.35689305, 0.43079508, 0.505837  , 0.55899652, 0.56326647, 0.51077212, 0.44988028, 0.40073001};
float SS_servo4[] = {0.44163784, 0.48256302, 0.49581669, 0.49998251, 0.50450203, 0.51047659, 0.51588583, 0.52066202, 0.52315474, 0.52278954, 0.50386199, 0.45185074, 0.3957859 , 0.34225374, 0.2884584 , 0.24258951, 0.20756704, 0.17864072, 0.17547779, 0.18513066, 0.17912913, 0.17338812, 0.17400258, 0.18310661, 0.19933629, 0.22061447, 0.24436476, 0.25472298, 0.23693299, 0.21954411, 0.22187273, 0.20922693, 0.21569471, 0.2382413 , 0.27340422, 0.31847968, 0.36549219, 0.41773944, 0.47671992, 0.52162159, 0.52458125};
float SS_servo5[] = { 0.30386583,  0.25812712,  0.19505053,  0.1264789 ,  0.05365102, -0.02090549, -0.09678815, -0.17387665, -0.24764923, -0.31110494, -0.33633665, -0.30105125, -0.26239995, -0.22665649, -0.19072196, -0.1615315 , -0.14101722, -0.12715128, -0.13449295, -0.16501753, -0.19891331, -0.21877126, -0.24371835, -0.27196705, -0.30208893, -0.34064989, -0.3785958 , -0.39550778, -0.37006241, -0.33769982, -0.37656178, -0.47638519, -0.55615958, -0.61790563, -0.66701482, -0.70536651, -0.74280263, -0.71694841, -0.57655027, -0.4446905 , -0.44147767};
float SS_servo6[] = {0.51179013, 0.52386378, 0.51919713, 0.51867158, 0.51609757, 0.51815247, 0.52025775, 0.52270574, 0.52358954, 0.52214472, 0.51772815, 0.50164645, 0.47762874, 0.45396262, 0.43904834, 0.44369802, 0.45821317, 0.46969339, 0.51296212, 0.53090241, 0.47941114, 0.45354979, 0.45169109, 0.45181238, 0.45726292, 0.46610246, 0.47937851, 0.48694631, 0.48438548, 0.48508102, 0.48349924, 0.46923686, 0.45588267, 0.4430509 , 0.43093691, 0.42421282, 0.43122513, 0.45170711, 0.48455642, 0.52166226, 0.52473455};
float SS_servo7[] = { 0.31090507,  0.29383265,  0.24570559,  0.17717233,  0.10707267,  0.03311181, -0.04256256, -0.12004909, -0.19458551, -0.25890508, -0.33146088, -0.4261505 , -0.53357286, -0.64065881, -0.73994417, -0.82468633, -0.8609857 , -0.81267506, -0.68770726, -0.56758223, -0.54676191, -0.55130086, -0.57198047, -0.58975244, -0.61226873, -0.64067395, -0.67095399, -0.687179  , -0.67536905, -0.66300542, -0.62864633, -0.5593852 , -0.4894596 , -0.42073593, -0.35089815, -0.28864117, -0.2542967 , -0.2633825 , -0.30686403, -0.35726833, -0.36224317};

int node=0;
int first=0;
int input=2;
int end_time=0;
int time_step=80000; // WAS 200 000
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);
int first_print=1;

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
  delay (500);

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
     
        Serial.print("Node:");
        Serial.print(node);
        Serial.print("       SS_servo0:");
        Serial.println(SS_servo3[node]);
        node+=1;
        if(node>=steps)
           {
              input=0;
              node=steps-1;
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

        if(first_print==1)
        {
        first_print=0;
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
        }
  
  }
}
