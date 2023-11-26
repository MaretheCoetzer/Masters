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
float SS_servo0[] = {-0.16672424, -0.16471029, -0.16188004, -0.15857585, -0.15101719, -0.13680595, -0.11306839, -0.07961766, -0.04491132, -0.00857943,  0.00797108,  0.01051962,  0.00826277,  0.00429887, -0.00198474, -0.00653788, -0.00963059, -0.01167475, -0.01370323, -0.01349201, -0.0166706 , -0.0205239 , -0.02852381, -0.03893674, -0.05146822, -0.0630552 , -0.07439199, -0.08315525, -0.08742375, -0.08441568, -0.06985611, -0.05303658, -0.05405936, -0.05508215, -0.05563184, -0.05534393, -0.05505602, -0.05476812, -0.05448021, -0.0541923 , -0.05390439, -0.05361648, -0.05332858, -0.05736641, -0.06429703, -0.07122766, -0.07815829, -0.08508892, -0.09201954, -0.10024753, -0.11296232, -0.13154253, -0.14507441, -0.1492375 , -0.14718768, -0.1410984 , -0.13352535, -0.12567883, -0.12172877, -0.12177244, -0.12532163, -0.12918072, -0.13030928, -0.13249907, -0.136993  , -0.14384591, -0.15591866};
float SS_servo1[] = {0.13493751, 0.18448693, 0.26742476, 0.34544832, 0.39622737, 0.4168578 , 0.39553583, 0.32158605, 0.24213803, 0.16227068, 0.11946713, 0.10480521, 0.09885821, 0.09621886, 0.10202682, 0.10473969, 0.10461352, 0.10224764, 0.09926069, 0.09290732, 0.0932748 , 0.09529486, 0.10747134, 0.12297016, 0.14080434, 0.15823546, 0.17380869, 0.18554907, 0.18951551, 0.17798877, 0.14052935, 0.09922543, 0.09521427, 0.09120311, 0.08664054, 0.08110171, 0.07556289, 0.07002406, 0.06448523, 0.0589464 , 0.05340758, 0.04786875, 0.04232992, 0.04580189, 0.05529974, 0.06479759, 0.07429544, 0.0837933 , 0.09329115, 0.1056298 , 0.12906924, 0.1620136 , 0.1865044 , 0.18778196, 0.17602421, 0.15808319, 0.13819565, 0.11859592, 0.10585269, 0.10198653, 0.10293132, 0.10308125, 0.09978013, 0.0973607 , 0.0977736 , 0.10261221, 0.11323393};
float SS_servo2[] = {-0.05526176, -0.04174446, -0.0298386 , -0.02536648, -0.02738025, -0.03418245, -0.04235825, -0.0563602 , -0.074218  , -0.09244287, -0.10368084, -0.10416505, -0.1006667 , -0.09577147, -0.08911577, -0.0834513 , -0.08240307, -0.0838518 , -0.08740102, -0.09121763, -0.09532491, -0.09911203, -0.10093148, -0.10254753, -0.10478133, -0.10889955, -0.11155228, -0.11476937, -0.11805482, -0.12168525, -0.13790622, -0.1631117 , -0.19153204, -0.21995238, -0.23875714, -0.24053759, -0.24231805, -0.2440985 , -0.24587895, -0.24765941, -0.24943986, -0.25122031, -0.25300077, -0.23835292, -0.21271879, -0.18708466, -0.16145054, -0.13581641, -0.11018228, -0.08209756, -0.04544413, -0.00882329,  0.02376866,  0.033664  ,  0.03570733,  0.0336332 ,  0.03171802,  0.02958407,  0.02696767,  0.02491282,  0.02238293,  0.01614548,  0.00502486, -0.00725455, -0.01997805, -0.03293295, -0.04881918};
float SS_servo3[] = {0.19303027, 0.15499748, 0.11809602, 0.0978441 , 0.09066766, 0.09424399, 0.10210349, 0.12245198, 0.15044375, 0.18213369, 0.1959284 , 0.1873996 , 0.16977236, 0.14920252, 0.12878177, 0.11077471, 0.10223624, 0.09866371, 0.09874029, 0.10055005, 0.10279197, 0.1046687 , 0.104301  , 0.1019418 , 0.09885098, 0.10108914, 0.09901464, 0.09946823, 0.10139582, 0.10327437, 0.13359284, 0.20021639, 0.28622818, 0.37223997, 0.43754772, 0.46619913, 0.49485054, 0.52350195, 0.55215336, 0.58080477, 0.60945617, 0.63810758, 0.66675899, 0.65108067, 0.60575724, 0.56043381, 0.51511038, 0.46978695, 0.42446352, 0.3706616 , 0.29255531, 0.21243386, 0.14282332, 0.11545373, 0.10360171, 0.10208208, 0.10131916, 0.10187021, 0.10241393, 0.10263487, 0.1015434 , 0.10650948, 0.123523  , 0.14165353, 0.15883206, 0.17614525, 0.19469147};
float SS_servo4[] = {0.21569989, 0.20213421, 0.188801  , 0.18003834, 0.1732063 , 0.16928315, 0.1638839 , 0.15966271, 0.15685731, 0.15398089, 0.15452001, 0.160519 , 0.16353196, 0.16014408, 0.14801107, 0.13419492, 0.12540884, 0.11937602, 0.11504398, 0.10961992, 0.10604824, 0.10298183, 0.10302071, 0.10188668, 0.10083029, 0.10792454, 0.1117444 , 0.11185151, 0.1085315 , 0.1016086 , 0.08878224, 0.07480036, 0.06711587, 0.05943139, 0.05301821, 0.04885587, 0.04469353, 0.04053119, 0.03636885, 0.03220651, 0.02804416, 0.02388182, 0.01971948, 0.01932571, 0.02145215, 0.02357859, 0.02570502, 0.02783146, 0.02995789, 0.03268486, 0.03439385, 0.03031702, 0.02484357, 0.02345204, 0.02698035, 0.04667498, 0.08100108, 0.12004627, 0.15535895, 0.18260061, 0.19986769, 0.20774009, 0.20729212, 0.20451486, 0.20177711, 0.20046124, 0.2096159 };
float SS_servo5[] = {-0.14477367, -0.12801167, -0.11387105, -0.10736242, -0.10484749, -0.10715992, -0.10496551, -0.10449216, -0.10703195, -0.10646473, -0.11646625, -0.13793511, -0.1544505 , -0.15821813, -0.14069123, -0.11936024, -0.10800088, -0.10232096, -0.10068554, -0.09559942, -0.09447342, -0.0940874 , -0.09824917, -0.10157427, -0.10702792, -0.1274393 , -0.14255727, -0.14878091, -0.14673678, -0.13814601, -0.1200914 , -0.09896073, -0.08938518, -0.07980963, -0.07243794, -0.06896814, -0.06549834, -0.06202854, -0.05855874, -0.05508895, -0.05161915, -0.04814935, -0.04467955, -0.0486014 , -0.05746635, -0.0663313 , -0.07519624, -0.08406119, -0.09292613, -0.10277893, -0.10846833, -0.10476111, -0.09649798, -0.10081478, -0.115648  , -0.16151946, -0.23612118, -0.31937767, -0.38652344, -0.42023492, -0.41792637, -0.38196832, -0.31311786, -0.23955885, -0.17039366, -0.11015936, -0.11781715};
float SS_servo6[] = { 0.1099996 ,  0.10085124,  0.08640452,  0.07015569,  0.05346752,  0.03740111,  0.02246682,  0.00858917, -0.00509891, -0.01811323, -0.03237295, -0.04583208, -0.05447351, -0.05426216, -0.03511713, -0.0078283 ,  0.0252855 ,  0.05901277,  0.08812147,  0.1119171 ,  0.12817653,  0.14286639,  0.14790993,  0.14831004,  0.14095205,  0.12853317,  0.11932796,  0.11633257,  0.11437952,  0.11033107,  0.10827088,  0.10421688,  0.10237577,  0.10053466,  0.09795882,  0.09408214,  0.09020546,  0.08632878,  0.0824521 ,  0.07857542,  0.07469874,  0.07082206,  0.06694538,  0.06492249,  0.06413932,  0.06335615,  0.06257298,  0.0617898 ,  0.06100663,  0.06109782,  0.06544778,  0.06982675,  0.07299875,  0.07314549,  0.07055234,  0.06299825,  0.05006042,  0.04183046,  0.03719984,  0.03644953,  0.03235291,  0.02776204,  0.02892377,  0.03416132,  0.04375517,  0.06526407,  0.09453143};
float SS_servo7[] = {-0.32193737, -0.31393234, -0.29725708, -0.27536045, -0.25266508, -0.23017136, -0.20848582, -0.18828635, -0.16861874, -0.14735397, -0.12725826, -0.10931553, -0.10230564, -0.11349759, -0.15978433, -0.22275233, -0.2982335 , -0.36829924, -0.41838841, -0.44352245, -0.44004338, -0.43025775, -0.38171317, -0.31877752, -0.23267902, -0.14003737, -0.09986186, -0.09986154, -0.10060223, -0.09785452, -0.10159146, -0.1005593 , -0.1029311 , -0.1053029 , -0.10576131, -0.10283207, -0.09990284, -0.0969736 , -0.09404437, -0.09111514, -0.0881859 , -0.08525667, -0.08232743, -0.08289259, -0.08579459, -0.08869659, -0.09159859, -0.09450059, -0.09740259, -0.10186371, -0.11292859, -0.12635238, -0.13560259, -0.14300031, -0.14541347, -0.13578178, -0.1142117 , -0.10130872, -0.09670626, -0.09915525, -0.09705804, -0.09545756, -0.10344441, -0.12095648, -0.1491633 , -0.20194269, -0.27542038};

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
