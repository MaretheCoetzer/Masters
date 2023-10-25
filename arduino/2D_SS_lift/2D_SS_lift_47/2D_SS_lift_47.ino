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
float SS_servo0[] = {-0.23372393, -0.23476926, -0.2338489 , -0.23523061, -0.24556731, -0.25466796, -0.2586194 , -0.26257085, -0.26652229, -0.27047373, -0.2712658, -0.26752767, -0.26378953, -0.2600514 , -0.2480994 , -0.22573972, -0.19563403, -0.16260444, -0.12957486, -0.09490356, -0.05951553, -0.02570208, -0.00615659, -0.00348165, -0.00617112, -0.00929418, -0.01480156, -0.01703492, -0.01403572, -0.01147019, -0.01369617, -0.01592215, -0.01814813, -0.02037411, -0.02350257, -0.03230913, -0.04111569, -0.04992225, -0.05942686, -0.0691152 , -0.0780542 , -0.08856887, -0.10103889, -0.1136951 , -0.12685501, -0.13949724, -0.14059713, -0.13545245, -0.12812442, -0.12660562, -0.12729986, -0.12799409, -0.13311081, -0.14002035, -0.14554412, -0.14833973, -0.1524106 , -0.15705374, -0.16196272, -0.16323555, -0.16347228, -0.16310344, -0.16523736, -0.1678999 , -0.17056244, -0.17322498, -0.17588752, -0.17855007, -0.18732669, -0.1981112 , -0.20889571, -0.21968022, -0.23205274, -0.23901501, -0.23828721, -0.23126561, -0.22179471, -0.21241182, -0.20603399, -0.20404823, -0.20480329, -0.20566848, -0.20792194, -0.20973786, -0.21440584, -0.2261038 };
float SS_servo1[] = {0.14302422, 0.12836575, 0.10619636, 0.09839879, 0.12681139, 0.16377414, 0.216795  , 0.26981586, 0.32283672, 0.37585758, 0.42892309, 0.48205263, 0.53518216, 0.5883117 , 0.61401462, 0.59767401, 0.54423207, 0.47271626, 0.40120046, 0.32310925, 0.24214737, 0.16618053, 0.1206079 , 0.10494991, 0.10101346, 0.0982074 , 0.10277123, 0.10106872, 0.08860519, 0.07672566, 0.07403214, 0.07133862, 0.0686451 , 0.06595159, 0.06496798, 0.07474252, 0.08451706, 0.0942916 , 0.10503718, 0.11424506, 0.12147682, 0.13067357, 0.1406005 , 0.14839771, 0.15378278, 0.15529196, 0.14967857, 0.12893923, 0.10654459, 0.09521141, 0.08888952, 0.08256763, 0.0847654 , 0.09041697, 0.09472072, 0.09439584, 0.09567471, 0.09803593, 0.10118768, 0.09723211, 0.09205273, 0.085636  , 0.08474614, 0.08502364, 0.08530114, 0.08557863, 0.08585613, 0.08613363, 0.10063808, 0.11981472, 0.13899136, 0.158168  , 0.1791881 , 0.18777435, 0.18223223, 0.16435172, 0.14239614, 0.12085703, 0.10793493, 0.1011762 , 0.10090431, 0.10087943, 0.10396851, 0.10692828, 0.11546097, 0.13430828};
float SS_servo2[] = { 0.07984241,  0.08364614,  0.08467588,  0.08058583,  0.07653794,  0.07146521,  0.06561598,  0.05976674,  0.05391751,  0.04806827,  0.04069467,  0.0311353 ,  0.02157592,  0.01201655,  0.00633376,  0.00180706,  0.00240364, -0.00173837, -0.00588039, -0.01614171, -0.02907459, -0.04353482, -0.05228356, -0.05627309, -0.05645154, -0.05436313, -0.04937419, -0.04503028, -0.04961201, -0.05451904, -0.05798048, -0.06144191, -0.06490335, -0.06836478, -0.0718153 , -0.07519712, -0.07857893, -0.08196075, -0.08556572, -0.08942344, -0.09567525, -0.10097413, -0.10699121, -0.11462526, -0.12335394, -0.1330936 , -0.14401482, -0.16589547, -0.18912964, -0.21385451, -0.2391136 , -0.26437269, -0.28835455, -0.31181863, -0.33214075, -0.34378658, -0.34569778, -0.33912805, -0.3241269 , -0.30153325, -0.26619977, -0.22917251, -0.19470719, -0.16078299, -0.12685878, -0.09293458, -0.05901038, -0.02508618,  0.0042167 ,  0.03200191,  0.05978712,  0.08757233,  0.1122606 ,  0.12082029,  0.12287253,  0.12127843,  0.11992119,  0.1179957 ,  0.11815448,  0.11901353,  0.11772534,  0.11616544,  0.11118033,  0.10511169,  0.09612762,  0.08322938};
float SS_servo3[] = {0.16419662, 0.13937434, 0.11663749, 0.10429268, 0.0938743 , 0.08656864, 0.08363989, 0.08071114, 0.07778239, 0.07485364, 0.07633562, 0.08414208, 0.09194854, 0.09975501, 0.10219619, 0.10102705, 0.09072545, 0.09096478, 0.0912041 , 0.10406143, 0.1224275 , 0.14494805, 0.15695455, 0.15484861, 0.14562462, 0.13220977, 0.11549334, 0.10044899, 0.10335155, 0.10662274, 0.10645753, 0.10629233, 0.10612712, 0.10596192, 0.10565416, 0.10444953, 0.10324489, 0.10204025, 0.10085085, 0.09826268, 0.10005456, 0.0986975 , 0.09556464, 0.09319828, 0.08961997, 0.08522993, 0.09942349, 0.13321316, 0.17413553, 0.22876181, 0.29459355, 0.36042529, 0.43584102, 0.51514201, 0.59393173, 0.65268165, 0.68279685, 0.68910502, 0.67224256, 0.63447086, 0.5395928 , 0.43248879, 0.39064824, 0.36259226, 0.33453628, 0.3064803 , 0.27842432, 0.25036834, 0.22810018, 0.20773276, 0.18736534, 0.16699793, 0.13788108, 0.11485942, 0.10646107, 0.10579027, 0.10550197, 0.10669705, 0.10628041, 0.10171853, 0.10256545, 0.10399383, 0.11275214, 0.12449752, 0.1420885 , 0.16390962};
float SS_servo4[] = { 0.2128481 ,  0.19604599,  0.18040858,  0.16699332,  0.15116858,  0.13743367,  0.12977779,  0.1221219 ,  0.11446601,  0.10681012,  0.10144471,  0.09936357,  0.09728244,  0.0952013 ,  0.09240304,  0.08652308,  0.07729404,  0.07312795,  0.06896185,  0.06895243,  0.07075772,  0.07433941,  0.07896416,  0.08089669,  0.07959474,  0.07276116,  0.05803778,  0.0409823 ,  0.03149121,  0.02312891,  0.01748481,  0.01184071,  0.00619661,  0.00055251, -0.00411506, -0.00263869, -0.00116231,  0.00031406,  0.00255437,  0.00312943,  0.00895624,  0.01956815,  0.03460831,  0.05742454,  0.08412603,  0.09834796,  0.0982703 ,  0.08121724,  0.06336476,  0.04766804,  0.03341219,  0.01915634,  0.00466222, -0.0099285 , -0.02488216, -0.04021944, -0.05650241, -0.07311438, -0.08926134, -0.10473676, -0.11479806, -0.12351333, -0.11762876, -0.10866049, -0.09969222, -0.09072395, -0.08175568, -0.07278741, -0.04902995, -0.02041566,  0.00819862,  0.03681291,  0.06745912,  0.09898469,  0.12800064,  0.15397356,  0.17654097,  0.19453409,  0.20655404,  0.21411341,  0.21496693,  0.21550832,  0.212115  ,  0.20906101,  0.20654433,  0.2131221 };
float SS_servo5[] = {-0.13567703, -0.11856852, -0.10734793, -0.10072105, -0.087162  , -0.07681676, -0.07605387, -0.07529099, -0.0745281 , -0.07376521, -0.07634405, -0.08371455, -0.09108506, -0.09845556, -0.10181255, -0.10017861, -0.09053266, -0.09022948, -0.08992631, -0.09785485, -0.10937718, -0.12348167, -0.13858892, -0.15266753, -0.1596022 , -0.15493367, -0.13169169, -0.10335141, -0.09042323, -0.08008441, -0.07587429, -0.07166417, -0.06745406, -0.06324394, -0.06113906, -0.07227955, -0.08342004, -0.09456053, -0.10766745, -0.11929362, -0.14209195, -0.17591156, -0.22210986, -0.28694116, -0.36353382, -0.41773851, -0.42573916, -0.40122979, -0.37230212, -0.34849549, -0.32706258, -0.30562967, -0.28412964, -0.26260241, -0.23884402, -0.21342805, -0.18707403, -0.16014604, -0.13394932, -0.1089513 , -0.094211  , -0.08224747, -0.09971694, -0.12340308, -0.14708922, -0.17077536, -0.1944615 , -0.21814764, -0.27118076, -0.33385154, -0.39652231, -0.45919309, -0.52798834, -0.58956755, -0.63363731, -0.65905363, -0.66425674, -0.6475357 , -0.60241973, -0.54743167, -0.46347439, -0.37896389, -0.2874795 , -0.20153287, -0.12723794, -0.12965883};
float SS_servo6[] = {0.21680864, 0.20711163, 0.19130989, 0.1728439 , 0.15695284, 0.14222991, 0.1289389 , 0.11564788, 0.10235687, 0.08906585, 0.07713285, 0.0671471 , 0.05716134, 0.04717558, 0.04001945, 0.03658774, 0.03583753, 0.03340272, 0.03096791, 0.02658674, 0.02135583, 0.01442503, 0.01030644, 0.00556903, 0.00443305, 0.00988481, 0.03481943, 0.06784474, 0.10648285, 0.14399769, 0.17422469, 0.2044517 , 0.2346787 , 0.26490571, 0.29311105, 0.30859685, 0.32408264, 0.33956843, 0.35074052, 0.35139089, 0.34311913, 0.33115519, 0.31602912, 0.29697112, 0.27351124, 0.25156911, 0.24820796, 0.24352709, 0.23888191, 0.23356525, 0.22893003, 0.22429481, 0.2204853 , 0.21701052, 0.21364857, 0.21042737, 0.20794065, 0.20542566, 0.20246542, 0.19523172, 0.18973089, 0.18395474, 0.17967789, 0.17571772, 0.17175755, 0.16779737, 0.1638372 , 0.15987703, 0.15967603, 0.16070955, 0.16174307, 0.1627766 , 0.16276179, 0.16202479, 0.160312  , 0.15652067, 0.154423  , 0.15363606, 0.15348261, 0.14765278, 0.14710544, 0.1469696 , 0.15202125, 0.16029427, 0.17971772, 0.2059116 };
float SS_servo7[] = {-0.24297316, -0.24012499, -0.22855801, -0.21176989, -0.19815649, -0.1858698 , -0.17368409, -0.16149838, -0.14931266, -0.13712695, -0.12638512, -0.11771366, -0.10904221, -0.10037075, -0.09491278, -0.09828586, -0.10581647, -0.10900142, -0.11218637, -0.11124411, -0.10849998, -0.10128781, -0.09869892, -0.09927847, -0.10656424, -0.12686702, -0.18441438, -0.25871532, -0.34470785, -0.42475226, -0.47404039, -0.52332853, -0.57261667, -0.6219048 , -0.66371174, -0.65844971, -0.65318767, -0.64792564, -0.6281882 , -0.57486044, -0.49509173, -0.40934239, -0.32301898, -0.23743643, -0.15543594, -0.1106396 , -0.11186896, -0.11294133, -0.11132061, -0.10900778, -0.10747686, -0.10594594, -0.10647045, -0.10782822, -0.10792202, -0.107422  , -0.10937006, -0.11132856, -0.11212494, -0.1041111 , -0.09859877, -0.09250832, -0.08906746, -0.08618624, -0.08330502, -0.0804238 , -0.07754258, -0.07466136, -0.0773816 , -0.08194139, -0.08650118, -0.09106097, -0.09478865, -0.09866955, -0.09934141, -0.09555647, -0.094338  , -0.09547569, -0.09525903, -0.08621878, -0.08691875, -0.0884236 , -0.10007483, -0.11745584, -0.157384  , -0.21457698};

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
