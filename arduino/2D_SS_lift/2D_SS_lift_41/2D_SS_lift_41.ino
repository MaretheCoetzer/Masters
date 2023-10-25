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
float SS_servo0[] = {-0.15946678, -0.14064852, -0.09464535, -0.06965721, -0.05504942, -0.04742563, -0.04002514, -0.0363187 , -0.03341811, -0.03051751, -0.02761692, -0.02471632, -0.02181573, -0.02050655, -0.0229051 , -0.02530365, -0.02770219, -0.03010074, -0.03182579, -0.02585055, -0.01154452,  0.00868167,  0.03005029,  0.0501013 ,  0.06263688,  0.05903053,  0.05348089,  0.05022585,  0.04586825,  0.04095284,  0.03153513,  0.02390621,  0.01657161,  0.01225281,  0.0118805 ,  0.01296827,  0.01120841,  0.00741085,  0.00361328, -0.00018428, -0.01005193, -0.02089995, -0.03151326, -0.03721085, -0.04110885, -0.04396958, -0.04803617, -0.05097944, -0.05439401, -0.05806237, -0.06093945, -0.06936357, -0.07607978, -0.08011241, -0.08226701, -0.08352526, -0.08404999, -0.08474821, -0.08453095, -0.0804905 , -0.081587  , -0.08268349, -0.08377999, -0.08989107, -0.09688983, -0.10524388, -0.11692837, -0.13122755, -0.14972984};
float SS_servo1[] = {0.26264114, 0.2824912 , 0.28351963, 0.22057547, 0.17431887, 0.14924039, 0.12558922, 0.12617471, 0.13204734, 0.13791997, 0.1437926 , 0.14966522, 0.15553785, 0.17018201, 0.20526226, 0.24034251, 0.27542275, 0.310503  , 0.34343903, 0.34233871, 0.30289048, 0.24133713, 0.18809117, 0.13901663, 0.10672306, 0.10054489, 0.10320848, 0.10246521, 0.10295457, 0.10623098, 0.1136914 , 0.12270807, 0.12740991, 0.12708324, 0.11845279, 0.10413876, 0.09489277, 0.09055957, 0.08622637, 0.08189317, 0.08498999, 0.08928683, 0.09341856, 0.0954838 , 0.09496612, 0.09270676, 0.09304084, 0.09388865, 0.0942095 , 0.09452528, 0.09577659, 0.10680625, 0.11612356, 0.11700001, 0.11271378, 0.10742064, 0.10361115, 0.10156254, 0.09462387, 0.08229966, 0.07993886, 0.07757805, 0.07521725, 0.08337451, 0.09339366, 0.10613893, 0.1249884 , 0.14965696, 0.18310608};
float SS_servo2[] = {-0.14550237, -0.11739266, -0.0524906 , -0.01885254,  0.00086806,  0.004331  ,  0.00598247,  0.00528477,  0.0040746 ,  0.00286444,  0.00165427,  0.00044411, -0.00076606, -0.0052778 , -0.01748161, -0.02968541, -0.04188922, -0.05409303, -0.067779  , -0.07564571, -0.08466367, -0.09667779, -0.11609446, -0.13880739, -0.15305908, -0.15747377, -0.15482465, -0.15030118, -0.14510623, -0.13872343, -0.13296365, -0.12689263, -0.12533813, -0.1261188 , -0.1312073 , -0.14249122, -0.15641802, -0.17145258, -0.18648714, -0.2015217 , -0.21754368, -0.23372513, -0.24940277, -0.25740411, -0.25901821, -0.25438159, -0.24356128, -0.22614073, -0.20319897, -0.17541529, -0.14401734, -0.11100654, -0.08027913, -0.06713925, -0.0639226 , -0.06701122, -0.07016138, -0.07170973, -0.07577565, -0.08057529, -0.08381728, -0.08705927, -0.09030126, -0.09248416, -0.09447959, -0.09696185, -0.10423367, -0.12045539, -0.14239221};
float SS_servo3[] = {0.25337002, 0.26173951, 0.25613362, 0.18044863, 0.12373373, 0.10316386, 0.08674775, 0.07685183, 0.06837827, 0.05990471, 0.05143114, 0.04295758, 0.03448401, 0.03308643, 0.04817461, 0.06326278, 0.07835096, 0.09343914, 0.11170865, 0.11832001, 0.12734323, 0.14272511, 0.1741934 , 0.21209823, 0.23423637, 0.22972121, 0.21582331, 0.199403  , 0.18067438, 0.16117337, 0.1379499 , 0.11922085, 0.1059039 , 0.09838407, 0.09931666, 0.11281886, 0.14063974, 0.17892829, 0.21721684, 0.25550539, 0.30447047, 0.35515991, 0.40617401, 0.45135654, 0.47980713, 0.49090619, 0.48415993, 0.46145875, 0.41595672, 0.35267479, 0.28466239, 0.21186408, 0.14533601, 0.11142712, 0.09627264, 0.09469253, 0.09619938, 0.09586348, 0.09758635, 0.10317141, 0.1052021 , 0.10723279, 0.10926348, 0.10950417, 0.109428  , 0.11031576, 0.12025626, 0.14883426, 0.1892547 };
float SS_servo4[] = { 0.18370133,  0.17910107,  0.16953148,  0.16172086,  0.15380679,  0.14093061,  0.12783939,  0.12078311,  0.11504332,  0.10930353,  0.10356374,  0.09782396,  0.09208417,  0.08759158,  0.08600475,  0.08441792,  0.08283109,  0.08124426,  0.07921234,  0.07590938,  0.07617957,  0.08002784,  0.08402296,  0.08718463,  0.08864856,  0.08273416,  0.06757396,  0.05041534,  0.03760244,  0.03147924,  0.02374175,  0.02205613,  0.01616742,  0.01079676,  0.00713267,  0.00039907, -0.00363133, -0.00282618, -0.00202102, -0.00121587,  0.00918473,  0.02113507,  0.03362417,  0.03930656,  0.02910653,  0.01046834, -0.00745283, -0.02255002, -0.03618889, -0.04811914, -0.0577541 , -0.06715786, -0.07611651, -0.08221082, -0.08656895, -0.08199575, -0.06347926, -0.03568841, -0.00291894,  0.03171189,  0.06080324,  0.08989459,  0.11898594,  0.13945812,  0.15840455,  0.17391775,  0.18080914,  0.18334084,  0.18541039};
float SS_servo5[] = {-0.11623217, -0.11091497, -0.09975635, -0.10223083, -0.10269779, -0.09007581, -0.07654109, -0.07355453, -0.07286902, -0.07218351, -0.071498, -0.07081248, -0.07012697, -0.07158548, -0.07803919, -0.08449289, -0.09094659, -0.09740029, -0.10270921, -0.10530983, -0.11508087, -0.13179317, -0.14767798, -0.16209226, -0.17173414, -0.17321018, -0.15104577, -0.12357954, -0.10591865, -0.10017836, -0.0961344 , -0.09922391, -0.09748197, -0.09570569, -0.09762904, -0.09601092, -0.10061819, -0.11428873, -0.12795927, -0.14162981, -0.18004092, -0.22244783, -0.26568223, -0.28696556, -0.27458019, -0.24453452, -0.21582894, -0.19009883, -0.1688717 , -0.15167609, -0.13660933, -0.12349693, -0.10955113, -0.10448951, -0.10434873, -0.12165614, -0.16462359, -0.22985593, -0.30444706, -0.37160396, -0.40679392, -0.44198389, -0.47717385, -0.46880516, -0.45272577, -0.42182654, -0.35685472, -0.27511986, -0.19138535};
float SS_servo6[] = { 0.17825632,  0.17678741,  0.172946  ,  0.16221851,  0.14704293,  0.13568789,  0.12505104,  0.11467268,  0.10435072,  0.09402875,  0.08370679,  0.07338482,  0.06306286,  0.05374824,  0.04678057,  0.0398129 ,  0.03284523,  0.02587756,  0.02028083,  0.01604599,  0.01197512,  0.00646472,  0.00359516, -0.00123752, -0.00382795, -0.00262851,  0.0199246 ,  0.05337035,  0.09090832,  0.128967  ,  0.16430735,  0.19601282,  0.22324669,  0.24511948,  0.26136489,  0.27204669,  0.27335612,  0.26718924,  0.26102236,  0.25485549,  0.2365313 ,  0.21624361,  0.1977019 ,  0.19265617,  0.18961418,  0.1856304 ,  0.18106627,  0.17841413,  0.17467068,  0.17193669,  0.17346606,  0.17952514,  0.18723447,  0.19225048,  0.19364156,  0.18614334,  0.167996  ,  0.15450191,  0.14289115,  0.13493789,  0.13080218,  0.12666648,  0.12253077,  0.12441589,  0.12736682,  0.13095531,  0.1337007 ,  0.14247347,  0.1574253 };
float SS_servo7[] = {-0.22248366, -0.22342571, -0.22371365, -0.22026124, -0.2060362 , -0.19659296, -0.18814029, -0.17841477, -0.16841159, -0.1584084 , -0.14840522, -0.13840204, -0.12839885, -0.12004935, -0.11555261, -0.11105587, -0.10655913, -0.10206239, -0.10013146, -0.10083911, -0.10180342, -0.09955386, -0.10153834, -0.09971937, -0.10110801, -0.11700665, -0.17192595, -0.24802354, -0.33409217, -0.4158306 , -0.49190716, -0.54485826, -0.58557554, -0.60488263, -0.60511348, -0.59263136, -0.55090741, -0.4832782 , -0.41564899, -0.34801978, -0.26376698, -0.17682935, -0.10586983, -0.10520811, -0.10751269, -0.10756035, -0.10627574, -0.10605003, -0.10506166, -0.10666063, -0.11431488, -0.13250783, -0.15224952, -0.16954027, -0.18091216, -0.17362519, -0.14200635, -0.11829086, -0.10137155, -0.0893859 , -0.08560974, -0.08183357, -0.07805741, -0.08608358, -0.09619897, -0.10758411, -0.11785654, -0.13974521, -0.17373868};

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
