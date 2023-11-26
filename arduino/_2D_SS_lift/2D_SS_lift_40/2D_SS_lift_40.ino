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
float SS_servo0[] = {-0.18160258, -0.16336896, -0.15228846, -0.14815912, -0.15488968, -0.16667048, -0.17805836, -0.1865724 , -0.18584099, -0.17364615, -0.15246972, -0.12371407, -0.09054528, -0.05513506, -0.02137147, -0.00412852, -0.00100327, -0.00349274, -0.00790118, -0.01159637, -0.0138152 , -0.01544755, -0.02094055, -0.03062243, -0.04356745, -0.05639576, -0.06823504, -0.07952678, -0.08770502, -0.08835602, -0.08148807, -0.06931293, -0.0608375 , -0.05806813, -0.05632488, -0.057133, -0.05805432, -0.05897564, -0.05989695, -0.06107532, -0.06884572, -0.07661613, -0.08438654, -0.09254415, -0.09588006, -0.10172606, -0.10767531, -0.12209132, -0.13492443, -0.14107251, -0.14045789, -0.13542155, -0.12824627, -0.12187673, -0.12041023, -0.12248279, -0.12572501, -0.12555663, -0.12800129, -0.13090096, -0.1376827 , -0.14552061, -0.15620261, -0.17341767};
float SS_servo1[] = {0.1508715 , 0.10778025, 0.08387534, 0.09256769, 0.14845017, 0.22944068, 0.31349508, 0.39020357, 0.44339306, 0.45245745, 0.43513638, 0.38365979, 0.31103284, 0.23204018, 0.15833055, 0.117556  , 0.1041514 , 0.09930862, 0.10239819, 0.10411303, 0.10358068, 0.10034586, 0.10418547, 0.11776537, 0.13710111, 0.1559923 , 0.17298075, 0.18612364, 0.1921884 , 0.18764174, 0.16453727, 0.13017972, 0.10667522, 0.09301792, 0.08327924, 0.07913214, 0.07523312, 0.0713341 , 0.06743507, 0.06406412, 0.07423551, 0.0844069 , 0.09457829, 0.10554348, 0.10685491, 0.11303812, 0.12000808, 0.14435928, 0.16679884, 0.17267818, 0.16271981, 0.14757089, 0.12818153, 0.11084482, 0.10248843, 0.10162255, 0.10183578, 0.09499622, 0.09338261, 0.09256072, 0.09849381, 0.10489672, 0.11355433, 0.14033708};
float SS_servo2[] = {-0.09868057, -0.08681157, -0.06986857, -0.05360028, -0.04637416, -0.04501946, -0.0478496 , -0.05415157, -0.05555057, -0.0617537 , -0.06291793, -0.06926151, -0.07402779, -0.08847305, -0.10058664, -0.10691541, -0.10737264, -0.1053871 , -0.09970789, -0.09677077, -0.09764169, -0.10128336, -0.10508316, -0.1076013 , -0.10905361, -0.11212426, -0.11790012, -0.12317654, -0.12734844, -0.12932864, -0.13814597, -0.14516174, -0.14564339, -0.14238631, -0.13869759, -0.13796158, -0.13735658, -0.13675158, -0.13614658, -0.13561295, -0.13690987, -0.13820679, -0.1395037 , -0.13917024, -0.12878049, -0.10509817, -0.07371907, -0.03598336,  0.00154526,  0.01994243,  0.02363736,  0.02305381,  0.0187129 ,  0.01487274,  0.01124438,  0.00783052,  0.00200701, -0.0087524 , -0.01921076, -0.0297634 , -0.0411203 , -0.05392488, -0.07037793, -0.08798307};
float SS_servo3[] = {0.2585462 , 0.22797628, 0.179856  , 0.13670963, 0.11237675, 0.10040055, 0.09680034, 0.09906293, 0.09611341, 0.0994877 , 0.09649192, 0.10090689, 0.10351665, 0.12629194, 0.14604813, 0.15322673, 0.14713383, 0.13327523, 0.11595084, 0.10421466, 0.10093116, 0.10175515, 0.10217428, 0.101233  , 0.09725289, 0.09629831, 0.10092569, 0.10183939, 0.09975473, 0.0978722 , 0.10647826, 0.1109338 , 0.10559391, 0.09454725, 0.08774897, 0.10939426, 0.13230149, 0.15520872, 0.17811595, 0.20206613, 0.25276305, 0.30345998, 0.3541569 , 0.40273658, 0.41953688, 0.38759646, 0.328182  , 0.2458248 , 0.16535706, 0.12122838, 0.10489902, 0.10106336, 0.10492309, 0.10824295, 0.11021868, 0.11209647, 0.11754039, 0.1329563 , 0.14765185, 0.16240879, 0.17768947, 0.19426809, 0.2148025 , 0.24270431};
float SS_servo4[] = { 0.20167577,  0.18825539,  0.17046753,  0.15591928,  0.14597503,  0.13879611,  0.13359228,  0.12863174,  0.12580789,  0.12560148,  0.12752709,  0.1267323 ,  0.1236949 ,  0.11492586,  0.1051314 ,  0.10105535,  0.10324935,  0.1014246 ,  0.09384766,  0.08749483,  0.08436103,  0.080061  ,  0.07594534,  0.07407928,  0.0720189 ,  0.07422657,  0.08467118,  0.0956641 ,  0.10138161,  0.10148495,  0.09566661,  0.08761562,  0.07825905,  0.0694589 ,  0.06299143,  0.0556308 ,  0.04823054,  0.04083028,  0.03343003,  0.02612003,  0.02112484,  0.01612965,  0.01113447,  0.00722993,  0.00481596,  0.00270624, -0.00086406, -0.00237578, -0.00539335, -0.0060073 ,  0.00072562,  0.02538384,  0.05859532,  0.09647374,  0.12959852,  0.15394117,  0.17022546,  0.18009159,  0.18459889,  0.18821001,  0.18417499,  0.17751316,  0.17331222,  0.1909904 };
float SS_servo5[] = {-0.1809814 , -0.16042854, -0.13823808, -0.1189695 , -0.10854065, -0.10326087, -0.10206789, -0.10255355, -0.10261813, -0.11137096, -0.12056756, -0.12734286, -0.12819601, -0.11692661, -0.10191935, -0.09931683, -0.11074033, -0.11687059, -0.10737127, -0.10034167, -0.09907365, -0.09691349, -0.09588968, -0.09818486, -0.1009373 , -0.11258032, -0.14079031, -0.17279725, -0.19489337, -0.20095214, -0.19830412, -0.19164239, -0.17897637, -0.16918543, -0.16230634, -0.15314388, -0.1438801 , -0.13461633, -0.12535256, -0.11626407, -0.11167054, -0.10707702, -0.1024835 , -0.10007066, -0.10057882, -0.1019346 , -0.09975479, -0.10146241, -0.09880254, -0.10412828, -0.12662296, -0.18189477, -0.25516298, -0.33732441, -0.39966698, -0.4262916 , -0.42329273, -0.39568596, -0.35042412, -0.30247719, -0.23162119, -0.15481402, -0.11019218, -0.15361988};
float SS_servo6[] = { 0.12267966,  0.10810825,  0.0899602 ,  0.07398454,  0.05927363,  0.04560891,  0.03259643,  0.01972784,  0.00837437, -0.00449621, -0.01480087, -0.01998747, -0.02028905, -0.0067295 ,  0.00335169,  0.0067443 ,  0.00761352,  0.01910904,  0.04951479,  0.0822663 ,  0.1169156 ,  0.14956269,  0.17133641,  0.18183375,  0.18193084,  0.17471834,  0.16206278,  0.14399676,  0.13329935,  0.13330038,  0.13515557,  0.12932879,  0.12763648,  0.11999466,  0.1108916 ,  0.10780751,  0.10499046,  0.10217341,  0.09935636,  0.09662924,  0.09620823,  0.09578722,  0.09536621,  0.09418863,  0.09168438,  0.09372966,  0.09761136,  0.10329876,  0.10838885,  0.10919391,  0.10258602,  0.08831668,  0.0770975 ,  0.06907843,  0.06395109,  0.06116538,  0.05772593,  0.05478522,  0.05859074,  0.06369808,  0.07991248,  0.09766429,  0.11254118,  0.1220774 };
float SS_servo7[] = {-0.29348119, -0.27033217, -0.24712818, -0.22473885, -0.20450783, -0.18597215, -0.16886162, -0.15324516, -0.1359536 , -0.11899726, -0.10341269, -0.10132875, -0.10779257, -0.13779566, -0.14689368, -0.12941528, -0.10173452, -0.13113024, -0.19989482, -0.27340476, -0.3495538 , -0.41856656, -0.44799441, -0.43437192, -0.385717  , -0.30989682, -0.22390744, -0.13155633, -0.09923413, -0.10511648, -0.11123024, -0.10021644, -0.10310541, -0.09564705, -0.08335559, -0.08290457, -0.08297886, -0.08305316, -0.08312745, -0.08337635, -0.088103  , -0.09282965, -0.0975563 , -0.10069174, -0.10101638, -0.1107794 , -0.1236723 , -0.13992279, -0.15367543, -0.1618299 , -0.15718898, -0.13334183, -0.11558614, -0.10384414, -0.09887196, -0.098304  , -0.09770983, -0.09821868, -0.11250925, -0.12956986, -0.17026444, -0.21575846, -0.25895847, -0.28624036};

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
