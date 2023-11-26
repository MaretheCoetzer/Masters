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
float SS_servo0[] = {-0.21230247, -0.22462075, -0.24303192, -0.25978846, -0.2764681 , -0.28441922, -0.29199463, -0.29718348, -0.29456503, -0.29044515, -0.27667967, -0.25265735, -0.2209057 , -0.18539185, -0.1492243 , -0.1123873 , -0.076393  , -0.04095127, -0.01557154, -0.00974599, -0.01010969, -0.0149696 , -0.01895513, -0.02281714, -0.02506474, -0.02584432, -0.02739735, -0.03024439, -0.03818981, -0.04693394, -0.0534937 , -0.06034313, -0.06535246, -0.07258198, -0.08343057, -0.09748124, -0.11234639, -0.11689279, -0.1164799 , -0.11024761, -0.10323918, -0.10163556, -0.10063905, -0.10179602, -0.102953  , -0.10410997, -0.10526694, -0.10642391, -0.11329024, -0.1206369 , -0.12798356, -0.13497746, -0.13955416, -0.14160773, -0.14136473, -0.14303069, -0.14557997, -0.1513327 , -0.16298967, -0.18357359, -0.20119318, -0.21017934, -0.20760305, -0.20076903, -0.18817448, -0.1808745 , -0.17368487, -0.17059361, -0.16954243, -0.17075483, -0.17116684, -0.16860962, -0.16544904, -0.16531588, -0.16518272, -0.16504956, -0.16764137, -0.17194144, -0.17624151, -0.17973363, -0.19012691, -0.20746155};
float SS_servo1[] = {0.1780872 , 0.22646492, 0.30590997, 0.38984414, 0.48106229, 0.54731574, 0.61239134, 0.66941215, 0.70008274, 0.72603008, 0.72159472, 0.67578791, 0.60085726, 0.51628382, 0.43154849, 0.34811709, 0.26481668, 0.18526787, 0.12659629, 0.10735539, 0.09917525, 0.10219478, 0.10322111, 0.10407664, 0.10199927, 0.09634644, 0.09184333, 0.09024658, 0.09854369, 0.10778161, 0.11220911, 0.11731002, 0.11527832, 0.11707038, 0.12365925, 0.13379131, 0.14143498, 0.14168561, 0.13237967, 0.11267712, 0.09357576, 0.08364326, 0.0762991 , 0.07276615, 0.0692332 , 0.06570025, 0.0621673 , 0.05863434, 0.06713383, 0.07664561, 0.08615738, 0.09570652, 0.10220716, 0.10216861, 0.09735634, 0.0956651 , 0.09685477, 0.10504733, 0.12433272, 0.16148865, 0.19448631, 0.20748651, 0.19785589, 0.17900857, 0.15002856, 0.1325191 , 0.11527068, 0.107189  , 0.1036703 , 0.10305586, 0.10215657, 0.09609628, 0.08885463, 0.08798998, 0.08712533, 0.08626068, 0.08910102, 0.09426398, 0.09942694, 0.10651901, 0.12745215, 0.16384283};
float SS_servo2[] = { 0.02721448,  0.03643514,  0.04161395,  0.04173772,  0.03813982,  0.03227369,  0.02626553,  0.01929228,  0.00916185, -0.00064836, -0.0045117, -0.00966775, -0.01337712, -0.01662067, -0.02162483, -0.02833265, -0.04260279, -0.05699798, -0.06683777, -0.0697201 , -0.06881494, -0.06241876, -0.05970906, -0.05744751, -0.06016918, -0.06478059, -0.06989125, -0.07449379, -0.07757906, -0.07986769, -0.08365989, -0.088285  , -0.09681944, -0.10465033, -0.1118172 , -0.11772102, -0.12192523, -0.12409114, -0.13000472, -0.13618096, -0.13904481, -0.14327472, -0.148615  , -0.15810714, -0.16759929, -0.17709143, -0.18658357, -0.19607571, -0.20433629, -0.21249326, -0.22065023, -0.22790534, -0.22196931, -0.20289024, -0.17182668, -0.13429004, -0.09528204, -0.05574391, -0.01773895,  0.01984524,  0.05742997,  0.07935034,  0.08263688,  0.08159192,  0.07703518,  0.07435652,  0.07172558,  0.07108237,  0.07142861,  0.07268534,  0.07275407,  0.07149028,  0.06978942,  0.06780453,  0.06581964,  0.06383475,  0.06020367,  0.05554062,  0.05087757,  0.04534417,  0.03599673,  0.02299515};
float SS_servo3[] = {0.17887365, 0.1459004 , 0.11576825, 0.10017431, 0.09030204, 0.08793812, 0.08595166, 0.08619081, 0.09371074, 0.10104999, 0.10047209, 0.10032168, 0.09746209, 0.09530586, 0.09667324, 0.10228902, 0.12139431, 0.1437215 , 0.15672178, 0.15517726, 0.14445312, 0.124667  , 0.1121154 , 0.10055092, 0.09943097, 0.10154026, 0.10427769, 0.10627949, 0.1047546 , 0.10094164, 0.09977399, 0.10035954, 0.10545105, 0.10849803, 0.10767264, 0.1013466 , 0.08745799, 0.08286123, 0.08633245, 0.09171188, 0.09256853, 0.10300649, 0.12277478, 0.17573563, 0.22869648, 0.28165733, 0.33461818, 0.38757903, 0.46275746, 0.53980504, 0.61685262, 0.69386907, 0.73542967, 0.72511065, 0.66735402, 0.58234786, 0.49580458, 0.40927377, 0.32638427, 0.24446057, 0.16434259, 0.1142762 , 0.10303177, 0.09993633, 0.10547004, 0.10811068, 0.11069551, 0.11016188, 0.10807595, 0.10243002, 0.1005613 , 0.10221728, 0.10480269, 0.108299  , 0.11179531, 0.11529162, 0.12034685, 0.12637934, 0.13241184, 0.14383369, 0.16304104, 0.19133664};
float SS_servo4[] = { 0.19886355,  0.18331904,  0.1639145 ,  0.15229499,  0.13916425,  0.13119352,  0.12345316,  0.11681357,  0.11377497,  0.11094507,  0.10759096,  0.10154863,  0.09440082,  0.08979067,  0.08737591,  0.08802432,  0.08796005,  0.08682622,  0.0832016 ,  0.07404717,  0.06287412,  0.05703101,  0.05259591,  0.04848071,  0.04743571,  0.04528753,  0.04127609,  0.03784839,  0.03450952,  0.03018511,  0.02817134,  0.03314857,  0.0405264 ,  0.05541793,  0.07823902,  0.10595382,  0.12594714,  0.13126814,  0.12502919,  0.10993331,  0.097041  ,  0.08621327,  0.07638964,  0.06882667,  0.0612637 ,  0.05370072,  0.04613775,  0.03857478,  0.02968755,  0.02068891,  0.01169027,  0.00254999, -0.00734811, -0.01728033, -0.02677913, -0.03607338, -0.04475798, -0.05328874, -0.06243942, -0.07198667, -0.07959448, -0.07955417, -0.05904959, -0.02963808,  0.0088208 ,  0.04929332,  0.08978518,  0.13013026,  0.17040227,  0.20173843,  0.22360038,  0.23431483,  0.23794379,  0.23248583,  0.22702787,  0.22156991,  0.20995443,  0.19447887,  0.17900331,  0.17673236,  0.18476755,  0.20273342};
float SS_servo5[] = {-0.14655597, -0.12918737, -0.10945246, -0.10119473, -0.09173892, -0.08984265, -0.08832436, -0.08876934, -0.09563692, -0.1024534 , -0.10402306, -0.10234155, -0.09821877, -0.09759278, -0.10145686, -0.11073009, -0.12038432, -0.12495039, -0.12455485, -0.11337087, -0.099635  , -0.09458366, -0.0926743 , -0.09134241, -0.09589756, -0.09878357, -0.09829337, -0.09872065, -0.09978586, -0.09950488, -0.10428733, -0.12319282, -0.15040954, -0.19360311, -0.25561469, -0.33073055, -0.39461669, -0.41474048, -0.41062219, -0.3869416 , -0.36557081, -0.35025642, -0.33560944, -0.32607368, -0.31653793, -0.30700217, -0.29746642, -0.28793066, -0.27515918, -0.26211549, -0.24907179, -0.2349803 , -0.21741942, -0.20128475, -0.18618326, -0.17225574, -0.15845044, -0.14443856, -0.1299839 , -0.11479532, -0.10171495, -0.10690208, -0.1535701 , -0.21921628, -0.30905455, -0.40225012, -0.49540043, -0.58354879, -0.66920731, -0.72019707, -0.73165978, -0.6975509 , -0.63933424, -0.55395378, -0.46857332, -0.38319285, -0.29360891, -0.20138985, -0.10917078, -0.09993811, -0.11607813, -0.15057412};
float SS_servo6[] = {0.20582382, 0.19136961, 0.17447791, 0.15958754, 0.14618169, 0.13473794, 0.12336002, 0.11215406, 0.10151062, 0.09111697, 0.08340059, 0.07758737, 0.07348719, 0.06899209, 0.06344856, 0.05688227, 0.05216984, 0.05304561, 0.05927289, 0.08548009, 0.11953494, 0.15810959, 0.19762506, 0.23709934, 0.27521396, 0.31252501, 0.3381981 , 0.35972341, 0.36506392, 0.36318607, 0.355458  , 0.3439599 , 0.32919521, 0.31048298, 0.28938167, 0.26565451, 0.24104421, 0.23539024, 0.23236007, 0.22902118, 0.22655006, 0.22141005, 0.2168267 , 0.21277532, 0.20872394, 0.20467255, 0.20062117, 0.19656979, 0.19544602, 0.19456854, 0.19369106, 0.19313957, 0.19261527, 0.18995628, 0.18641639, 0.18350077, 0.18148511, 0.18105676, 0.18229071, 0.18930214, 0.1955244 , 0.19558402, 0.18441572, 0.17032619, 0.1624038 , 0.15898357, 0.15566982, 0.15621601, 0.15868356, 0.15773818, 0.15692958, 0.15328003, 0.14957689, 0.15066865, 0.15176041, 0.15285217, 0.15934762, 0.16923059, 0.17911356, 0.1914685 , 0.20041286, 0.21042078};
float SS_servo7[] = {-0.21310587, -0.1979539 , -0.18327081, -0.16843861, -0.15846422, -0.14952603, -0.14063069, -0.13181912, -0.1232816 , -0.11478391, -0.10756518, -0.10638174, -0.10844175, -0.10803298, -0.10555461, -0.10022707, -0.10050555, -0.10916555, -0.12877212, -0.18944744, -0.26789253, -0.35774141, -0.4510923 , -0.544102  , -0.63057321, -0.71388005, -0.75311668, -0.77671187, -0.741713  , -0.6821338 , -0.60563374, -0.52153221, -0.43721988, -0.34496664, -0.25615025, -0.17027938, -0.11080417, -0.10852105, -0.11099755, -0.11146289, -0.1114405 , -0.10785152, -0.10402574, -0.1017793 , -0.09953286, -0.09728642, -0.09503998, -0.09279354, -0.09591706, -0.09949234, -0.10306763, -0.10654533, -0.10817803, -0.10698704, -0.10415158, -0.10331353, -0.10317487, -0.10570158, -0.11237749, -0.1307226 , -0.14556757, -0.15065234, -0.13263837, -0.10943617, -0.09711676, -0.09307226, -0.08920414, -0.09219351, -0.09859638, -0.09973741, -0.09985336, -0.09336012, -0.08670182, -0.08952799, -0.09235416, -0.09518033, -0.110668  , -0.13409303, -0.15751805, -0.18224343, -0.20012005, -0.21857549};

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