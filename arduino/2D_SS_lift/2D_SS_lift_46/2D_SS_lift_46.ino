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
float SS_servo0[] = {0.5703814 , 0.43664764, 0.5133834 , 0.59011915, 0.66685491, 0.74359066, 0.82032642, 0.8746577 , 0.85233143, 0.83000515, 0.79008631, 0.72141299, 0.66582874, 0.60914654, 0.54921742, 0.49036658, 0.44161721, 0.41302433, 0.39945841, 0.39617563, 0.40110653, 0.41342773, 0.4124395 , 0.39702305, 0.38924105, 0.38356665, 0.37761772, 0.36822278, 0.3587108 , 0.34919883, 0.33968685, 0.33017487, 0.31977248, 0.3056731 , 0.29157371, 0.27747433, 0.26336406, 0.251393  , 0.24445702, 0.24100157, 0.23494715, 0.22677573, 0.21720071, 0.20878302, 0.20216304, 0.19643261, 0.19398968, 0.19216669, 0.19860726, 0.19840356, 0.19819987, 0.1979343 , 0.19174473, 0.18555516, 0.18380046, 0.1862372 , 0.18674486, 0.18988462, 0.18999319, 0.187113  , 0.18820624, 0.1936934 , 0.20213017, 0.21499258, 0.22812898, 0.24126538, 0.25503077, 0.26899131, 0.28340827, 0.29979523, 0.31374638, 0.32695405, 0.33699289, 0.35220877, 0.37024269, 0.38830652, 0.40710644};
float SS_servo1[] = {0.24747556, 0.2655897 , 0.2551959 , 0.2448021 , 0.2344083 , 0.2240145 , 0.2136207 , 0.20959264, 0.22734519, 0.24509774, 0.27136678, 0.30206019, 0.31303061, 0.36084108, 0.43102857, 0.49791806, 0.53773229, 0.51113071, 0.4389788 , 0.34684858, 0.27714746, 0.20992138, 0.18147881, 0.17125622, 0.1518309 , 0.13023485, 0.10967868, 0.10339116, 0.09758821, 0.09178526, 0.08598231, 0.08017936, 0.07735124, 0.08687455, 0.09639786, 0.10592116, 0.11548397, 0.12195867, 0.11966998, 0.10994947, 0.10553367, 0.10508643, 0.10652059, 0.10659149, 0.10264344, 0.10359692, 0.10070188, 0.09662179, 0.0801544 , 0.07710314, 0.07405188, 0.07114666, 0.08222265, 0.09329865, 0.09796754, 0.09510805, 0.09719271, 0.09699201, 0.10203009, 0.11793866, 0.12623117, 0.12652589, 0.12206164, 0.11269189, 0.10687295, 0.10105402, 0.09989541, 0.09972303, 0.09853902, 0.100614  , 0.1101046 , 0.12453908, 0.14494531, 0.16396113, 0.1815443 , 0.19937464, 0.22328705};
float SS_servo2[] = {0.56291875, 0.43795801, 0.50965985, 0.58136169, 0.65306353, 0.72476537, 0.79646721, 0.84910227, 0.83649977, 0.82389726, 0.79715198, 0.74902447, 0.70442185, 0.66811215, 0.63376708, 0.59613072, 0.56304986, 0.53599317, 0.50881519, 0.48599901, 0.46420351, 0.44136159, 0.42409417, 0.40661336, 0.39009612, 0.3742557 , 0.35804535, 0.34821021, 0.33859158, 0.32897294, 0.3193543 , 0.30973567, 0.30018429, 0.29091217, 0.28164004, 0.27236792, 0.26307612, 0.25534036, 0.24688928, 0.23838439, 0.23125297, 0.22525391, 0.21965177, 0.20853287, 0.20190331, 0.1985069 , 0.19450939, 0.19060423, 0.17990133, 0.16277027, 0.14563921, 0.12845148, 0.10583913, 0.08322677, 0.06760294, 0.07126635, 0.088058  , 0.1112685 , 0.13802034, 0.1670396 , 0.19022032, 0.20423112, 0.21354945, 0.22380469, 0.2343482 , 0.24489171, 0.25697024, 0.26957275, 0.28330628, 0.30136074, 0.31976093, 0.33301059, 0.3429099 , 0.3579261 , 0.3757435 , 0.39358498, 0.41201884};
float SS_servo3[] = {0.26264174, 0.26499561, 0.26364497, 0.26229433, 0.2609437 , 0.25959306, 0.25824242, 0.25721604, 0.2572991 , 0.25738216, 0.25774377, 0.24454343, 0.23094295, 0.21311968, 0.19920584, 0.19599777, 0.1917381 , 0.1819968 , 0.17834397, 0.16974484, 0.1642428 , 0.16428376, 0.15671188, 0.15010536, 0.14877757, 0.14793357, 0.14825892, 0.14295453, 0.13745895, 0.13196336, 0.12646778, 0.1209722 , 0.1164879 , 0.1162024 , 0.1159169 , 0.11563139, 0.11540231, 0.1132207 , 0.11406287, 0.11475793, 0.11260541, 0.10776674, 0.10106869, 0.10681509, 0.10318778, 0.0993682 , 0.09969181, 0.10022428, 0.12976354, 0.18045138, 0.23113921, 0.28206912, 0.3561726 , 0.43027609, 0.48298442, 0.46062004, 0.38828484, 0.30364766, 0.21650428, 0.16082219, 0.12350008, 0.10627318, 0.10002279, 0.09603143, 0.09559967, 0.0951679 , 0.09751577, 0.10016206, 0.10046719, 0.09917203, 0.09946266, 0.11386713, 0.134554  , 0.15395061, 0.17196384, 0.19023649, 0.21489266};
float SS_servo4[] = { 0.57306518,  0.6345157 ,  0.5992557 ,  0.5639957 ,  0.5287357 ,  0.4934757 ,  0.4582157 ,  0.42406136,  0.39369006,  0.36331876,  0.33421659,  0.30315953,  0.26612223,  0.23008162,  0.19575901,  0.1643621 ,  0.13675323,  0.11174993,  0.08913479,  0.06758705,  0.05225158,  0.04041175,  0.02644958,  0.00754406, -0.01599784, -0.03644387, -0.05495694, -0.06815992, -0.08118257, -0.09420521, -0.10722786, -0.12025051, -0.13121297, -0.13362156, -0.13603016, -0.13843876, -0.14079563, -0.14586773, -0.15311225, -0.1629315 , -0.17094568, -0.17763156, -0.18342796, -0.18193898, -0.18613615, -0.1924997 , -0.20216922, -0.21192203, -0.21990197, -0.22219272, -0.22448347, -0.22671577, -0.22335226, -0.21998876, -0.21807859, -0.2180951 , -0.21328703, -0.20278617, -0.1849904 , -0.14644659, -0.10173614, -0.0523935 ,  0.00157889,  0.06289887,  0.1283266 ,  0.19375433,  0.26291674,  0.33283009,  0.40138448,  0.46433821,  0.52014863,  0.5660538 ,  0.59231919,  0.60004386,  0.60581474,  0.61140043,  0.61242918};
float SS_servo5[] = {-0.06376926, -0.13143124, -0.09260713, -0.05378303, -0.01495893,  0.02386518,  0.06268928,  0.0900003 ,  0.07791897,  0.06583763,  0.04457196,  0.017948  ,  0.0070897 ,  0.00348705,  0.00176002, -0.00351956, -0.01009823, -0.01654918, -0.02336707, -0.02916351, -0.04334709, -0.06199194, -0.07302298, -0.07377304, -0.05846346, -0.04798173, -0.04110064, -0.03845114, -0.03594533, -0.03343953, -0.03093372, -0.02842792, -0.02911063, -0.04303197, -0.05695331, -0.07087466, -0.08488675, -0.09219991, -0.09357308, -0.08994684, -0.09013021, -0.0934646 , -0.09966745, -0.11989147, -0.12888422, -0.1265785 , -0.11456253, -0.10234623, -0.08926612, -0.08802411, -0.08678209, -0.08564104, -0.09416568, -0.10269033, -0.10550785, -0.10331277, -0.11007119, -0.12539377, -0.156836  , -0.22690941, -0.3068747 , -0.38755296, -0.4704772 , -0.56072325, -0.64647032, -0.73221738, -0.80625889, -0.87597523, -0.92954751, -0.94616915, -0.93061635, -0.87604166, -0.74571011, -0.57866859, -0.43032213, -0.28447497, -0.20012836};
float SS_servo6[] = {0.58415083, 0.64866812, 0.61164843, 0.57462873, 0.53760904, 0.50058934, 0.46356965, 0.42745927, 0.39446015, 0.36146102, 0.33112799, 0.30358059, 0.27194213, 0.23770681, 0.20320516, 0.1693011 , 0.13697839, 0.10821371, 0.08598225, 0.06473339, 0.04454119, 0.02604545, 0.00967461, 0.00200676, 0.01824156, 0.0457173 , 0.07433575, 0.10544829, 0.13664553, 0.16784277, 0.19904   , 0.23023724, 0.26016179, 0.28480217, 0.30944255, 0.33408293, 0.3586534 , 0.37375934, 0.37920538, 0.37819662, 0.37097641, 0.36046662, 0.34774075, 0.32926964, 0.30639829, 0.30579856, 0.31238724, 0.3213788 , 0.333041  , 0.33601703, 0.33899305, 0.34194444, 0.34253716, 0.34312987, 0.34630261, 0.34900463, 0.34886259, 0.34659047, 0.34056323, 0.32828427, 0.31766844, 0.31504413, 0.31672506, 0.3217078 , 0.33131997, 0.34093213, 0.35652555, 0.37322497, 0.38686587, 0.4040922 , 0.4226028 , 0.44389133, 0.46306232, 0.48935604, 0.51856253, 0.54801465, 0.58351112};
float SS_servo7[] = {-0.0941159 , -0.16820074, -0.12569123, -0.08318172, -0.04067222,  0.00183729,  0.0443468 ,  0.07570879,  0.06892924,  0.06214969,  0.04338015,  0.00969594, -0.01205302, -0.01928735, -0.02065569, -0.02085614, -0.01789213, -0.01673793, -0.024346  , -0.03075935, -0.03505554, -0.04014863, -0.0462524 , -0.06991051, -0.13628942, -0.22390008, -0.3134873 , -0.37922225, -0.44414717, -0.50907209, -0.57399701, -0.63892193, -0.69404392, -0.70846432, -0.72288473, -0.73730513, -0.75141449, -0.72524289, -0.66454859, -0.58429394, -0.48900796, -0.38729631, -0.29207088, -0.20033014, -0.09974758, -0.10910391, -0.12993486, -0.15545861, -0.18176041, -0.19096537, -0.20017032, -0.20930799, -0.21200368, -0.21469937, -0.21987098, -0.22307604, -0.21974122, -0.20914859, -0.19208979, -0.15803302, -0.12670224, -0.11020725, -0.10104277, -0.09428686, -0.09289265, -0.09149844, -0.09628435, -0.10191383, -0.10141972, -0.10102269, -0.10090886, -0.10301587, -0.10177412, -0.10583732, -0.11145574, -0.11729306, -0.1285169 };

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
