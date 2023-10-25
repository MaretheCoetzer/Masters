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
float SS_servo0[] = {-0.18863216, -0.17197551, -0.16405712, -0.16774819, -0.18262122, -0.19750152, -0.21231095, -0.22412218, -0.23369385, -0.23375715, -0.2216064, -0.20105831, -0.17358295, -0.14268537, -0.10893134, -0.07725822, -0.04723377, -0.02060731, -0.00485303, -0.00255246, -0.00570874, -0.01047433, -0.01923361, -0.02775958, -0.0364359 , -0.04658624, -0.05666201, -0.06665908, -0.07754625, -0.08702282, -0.091238  , -0.08767185, -0.07354253, -0.05803823, -0.05521843, -0.05530931, -0.05535822, -0.05538686, -0.05541549, -0.05544413, -0.05547276, -0.05550139, -0.05553003, -0.06311981, -0.07075025, -0.0783807 , -0.08601114, -0.09398444, -0.10154098, -0.10857226, -0.11994081, -0.13195335, -0.14742779, -0.15999793, -0.16211011, -0.15669047, -0.14741038, -0.13673191, -0.13062598, -0.126457  , -0.12499523, -0.12579377, -0.13034726, -0.13360042, -0.13448617, -0.13444603, -0.13682346, -0.14080404, -0.14838543, -0.15798376, -0.171832  , -0.19307491};
float SS_servo1[] = {0.14715464, 0.1061716 , 0.09063997, 0.12097091, 0.19613546, 0.28764123, 0.37939699, 0.46706867, 0.54769539, 0.59671601, 0.59498432, 0.56471089, 0.50190575, 0.42768913, 0.34909597, 0.27845144, 0.21176162, 0.1533337 , 0.1130063 , 0.10229149, 0.09972333, 0.10433496, 0.11464444, 0.12537872, 0.13663309, 0.14810726, 0.16164139, 0.17465691, 0.1862332 , 0.19495813, 0.19718852, 0.18392268, 0.14677072, 0.10645739, 0.09257748, 0.08539942, 0.07942052, 0.0740211 , 0.06862169, 0.06322227, 0.05782286, 0.05242344, 0.04702403, 0.05797297, 0.06900981, 0.08004666, 0.09108351, 0.10318269, 0.11311797, 0.12334057, 0.14216097, 0.16252236, 0.1911677 , 0.21221811, 0.2095341 , 0.19045753, 0.1648726 , 0.13932089, 0.12345532, 0.11021055, 0.10305841, 0.10185064, 0.10253438, 0.10440684, 0.10070718, 0.09404013, 0.09211968, 0.09275308, 0.09912264, 0.10679739, 0.1211503 , 0.15865898};
float SS_servo2[] = {-0.08290686, -0.07169258, -0.05466002, -0.04012579, -0.03389526, -0.03384349, -0.0339473 , -0.04032231, -0.04723367, -0.04927003, -0.05544562, -0.05735127, -0.06358403, -0.06547744, -0.07010133, -0.07589327, -0.08320968, -0.09323599, -0.10383275, -0.10727881, -0.10785708, -0.10510061, -0.10155903, -0.10024066, -0.10336606, -0.10833938, -0.11043646, -0.11560588, -0.12004399, -0.12437436, -0.12766911, -0.12953128, -0.14290172, -0.16321162, -0.1875662 , -0.21205627, -0.22425121, -0.23050447, -0.23675774, -0.243011  , -0.24926427, -0.25551753, -0.2617708 , -0.24615584, -0.23042329, -0.21469074, -0.19895819, -0.18179084, -0.15031874, -0.11069242, -0.07042049, -0.03022807,  0.00953701,  0.03892198,  0.0474475 ,  0.04865609,  0.04414622,  0.04191769,  0.04070548,  0.03796869,  0.03567224,  0.03416395,  0.03098623,  0.02798412,  0.01946838,  0.00787747, -0.00375264, -0.01604148, -0.02980992, -0.04625535, -0.06604549, -0.08448213};
float SS_servo3[] = {0.26017872, 0.22977518, 0.18164622, 0.13913718, 0.11346295, 0.10267032, 0.0922327 , 0.09450868, 0.0982707 , 0.09554705, 0.09810363, 0.09548143, 0.09879355, 0.09474222, 0.09662501, 0.10276863, 0.11255556, 0.1289155 , 0.14208538, 0.14306548, 0.13529384, 0.12471073, 0.11011125, 0.10086113, 0.10081124, 0.1017572 , 0.09908877, 0.10226725, 0.10075239, 0.0990181 , 0.09935411, 0.09707567, 0.1226112 , 0.17563711, 0.25244432, 0.33241743, 0.38801864, 0.43184201, 0.47566539, 0.51948876, 0.56331214, 0.60713551, 0.65095889, 0.64338605, 0.63553685, 0.62768765, 0.61983845, 0.60857577, 0.54958064, 0.46243513, 0.37311567, 0.28455285, 0.20005746, 0.13533687, 0.11093436, 0.10021818, 0.10240352, 0.10310558, 0.10199868, 0.10270697, 0.10317937, 0.10342322, 0.10134331, 0.10271256, 0.11448235, 0.13146046, 0.14840784, 0.16601276, 0.18509459, 0.20686764, 0.23358105, 0.26597126};
float SS_servo4[] = { 0.21524002,  0.20296993,  0.18891013,  0.17511792,  0.16372995,  0.15671796,  0.14979472,  0.14631071,  0.14359746,  0.14214104,  0.14361575,  0.14750403,  0.14885069,  0.14712058,  0.13622798,  0.12556223,  0.11708301,  0.1125125 ,  0.10469369,  0.10294267,  0.09754258,  0.09616385,  0.09084575,  0.08627628,  0.08531793,  0.08400772,  0.0880408 ,  0.10158547,  0.11409661,  0.12041953,  0.11918434,  0.11298594,  0.10212795,  0.08954071,  0.08193637,  0.0760774 ,  0.06892455,  0.06114642,  0.0533683 ,  0.04559017,  0.03781205,  0.03003392,  0.02225579,  0.02006783,  0.01790993,  0.01575203,  0.01359413,  0.01235812,  0.0141754 ,  0.01403825,  0.01201231,  0.00932165,  0.00305768, -0.00216704, -0.00452456, -0.00575582,  0.00675212,  0.03582718,  0.07144046,  0.10976806,  0.14956355,  0.18664238,  0.21530742,  0.23203615,  0.23730883,  0.23541634,  0.22882915,  0.22052763,  0.20837557,  0.19661695,  0.19595582,  0.21920996};
float SS_servo5[] = {-0.16145841, -0.14432093, -0.12938554, -0.11445763, -0.10448632, -0.10102277, -0.09769884, -0.1013175 , -0.10607721, -0.10996199, -0.12287084, -0.13715845, -0.14913594, -0.15348794, -0.13895116, -0.12301076, -0.11092379, -0.10562756, -0.09806496, -0.1005337 , -0.09853972, -0.10079612, -0.09746272, -0.09481254, -0.09925489, -0.10572423, -0.12080706, -0.15546548, -0.19129663, -0.21457978, -0.21838137, -0.21184965, -0.19832265, -0.18166247, -0.17442682, -0.16993445, -0.16150801, -0.15118042, -0.14085283, -0.13052524, -0.12019764, -0.10987005, -0.09954246, -0.09950637, -0.09952562, -0.09954486, -0.09956411, -0.10108518, -0.11009099, -0.11379607, -0.11380268, -0.11221302, -0.10204218, -0.09574726, -0.09792248, -0.10363541, -0.13583588, -0.21119495, -0.29316718, -0.37712998, -0.46376021, -0.53631907, -0.58430745, -0.58069114, -0.5359927 , -0.46899271, -0.38332098, -0.29590841, -0.20458571, -0.12798435, -0.11494099, -0.16683306};
float SS_servo6[] = { 0.12938793,  0.11264999,  0.09339159,  0.07613262,  0.06060895,  0.04665838,  0.03273873,  0.01852468,  0.00429509, -0.00868175, -0.02244663, -0.03395579, -0.04072578, -0.0427878 , -0.02953709, -0.00421978,  0.02435207,  0.05876031,  0.09428897,  0.12785838,  0.15843531,  0.18283366,  0.19893437,  0.20602292,  0.2050881 ,  0.19735585,  0.18760633,  0.17407534,  0.15382209,  0.14327047,  0.14128712,  0.13844716,  0.13666181,  0.13000208,  0.11913057,  0.10804419,  0.10261921,  0.09993013,  0.09724104,  0.09455196,  0.09186288,  0.0891738 ,  0.08648472,  0.0873077 ,  0.08814957,  0.08899143,  0.0898333 ,  0.09057349,  0.08787703,  0.08829537,  0.09109354,  0.0942215 ,  0.09912235,  0.10290059,  0.10453431,  0.10270314,  0.09371301,  0.07823997,  0.06821008,  0.06100956,  0.05792409,  0.05842352,  0.05272598,  0.04983304,  0.04762003,  0.04749781,  0.05360975,  0.06425072,  0.08506436,  0.10775852,  0.12521602,  0.13542093};
float SS_servo7[] = {-0.32655957, -0.30006695, -0.27431609, -0.25215607, -0.23365447, -0.21595905, -0.19828462, -0.18002422, -0.16132804, -0.14176995, -0.1237468 , -0.10684521, -0.10244849, -0.10619551, -0.1419343 , -0.20025449, -0.26438527, -0.33939534, -0.42123857, -0.48898051, -0.54814419, -0.57653539, -0.57618914, -0.54138843, -0.47820256, -0.39627709, -0.31106689, -0.22732917, -0.12804931, -0.10091319, -0.10322948, -0.10352876, -0.10836661, -0.10373375, -0.08983207, -0.07467764, -0.06975826, -0.06978503, -0.06981179, -0.06983855, -0.06986532, -0.06989208, -0.06991884, -0.07601438, -0.08214256, -0.08827074, -0.09439891, -0.09994597, -0.09977184, -0.10457794, -0.11437331, -0.12459101, -0.13707896, -0.14902932, -0.15923852, -0.1637207 , -0.15242024, -0.12515601, -0.10840494, -0.09866414, -0.09663866, -0.10046175, -0.09742156, -0.09628749, -0.09732303, -0.10364834, -0.12279045, -0.15182329, -0.20307163, -0.26097009, -0.31020149, -0.33622252};

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
