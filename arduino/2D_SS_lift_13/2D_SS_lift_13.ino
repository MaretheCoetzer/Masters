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
float SS_servo0[] = {-0.25776949, -0.26592189, -0.2765882 , -0.28509465, -0.28965187, -0.28601244, -0.28109265, -0.27617286, -0.26236567, -0.24681628, -0.22774412, -0.20209297, -0.17055591, -0.1349981 , -0.09776847, -0.05831478, -0.01924138,  0.00774912,  0.02742426,  0.02905735,  0.03069043,  0.03232352,  0.03395661,  0.033841  ,  0.02179467,  0.00974833, -0.00229801, -0.01591036, -0.03018844, -0.0421769 , -0.05318865, -0.06387071, -0.07388605, -0.08185099, -0.08551544, -0.0855269 , -0.08683284, -0.09284627, -0.09765057, -0.1028251 , -0.11174218, -0.11827755, -0.12208354, -0.1282821 , -0.1328198 , -0.13945164, -0.14658315, -0.15331388, -0.15456076, -0.16028969, -0.16729133, -0.17254655, -0.17446385, -0.17638116, -0.17829846, -0.18021577, -0.18213307, -0.18405037, -0.18596768, -0.1936689 , -0.20512988, -0.21659085, -0.22805183, -0.2395128 , -0.25097378, -0.26770219, -0.26953036, -0.26344935, -0.25549609, -0.24687681, -0.2402202 , -0.23324049, -0.2270776 , -0.22520602, -0.22481141, -0.22715253, -0.23290379, -0.24213413, -0.25399386};
float SS_servo1[] = {0.15269774, 0.19553354, 0.26939499, 0.34566069, 0.42341902, 0.47548272, 0.52310693, 0.57073114, 0.58676283, 0.59660144, 0.59587722, 0.57503556, 0.53006451, 0.46929699, 0.40155169, 0.31815482, 0.23337456, 0.17092799, 0.12175726, 0.1074443 , 0.09313134, 0.07881838, 0.06450541, 0.05374652, 0.06723578, 0.08072503, 0.09421429, 0.109463  , 0.12429558, 0.13604222, 0.14559484, 0.15361201, 0.1591985 , 0.15940458, 0.14936164, 0.1256358 , 0.10453589, 0.08759996, 0.07710986, 0.07563121, 0.08127599, 0.08546315, 0.08628305, 0.08811397, 0.08886239, 0.09263716, 0.09747045, 0.10137579, 0.09506622, 0.09822674, 0.1039771 , 0.10676935, 0.10390786, 0.10104636, 0.09818486, 0.09532337, 0.09246187, 0.08960037, 0.08673888, 0.09654025, 0.11457294, 0.13260562, 0.15063831, 0.16867099, 0.18670368, 0.21350908, 0.21077887, 0.19402886, 0.17305355, 0.15308059, 0.13574205, 0.12036207, 0.1078263 , 0.10088111, 0.09787736, 0.09991381, 0.10823298, 0.12093114, 0.13965724};
float SS_servo2[] = { 8.88485140e-02,  9.50406250e-02,  9.88504496e-02,  9.56736856e-02,  8.92105579e-02,  8.09859546e-02,  7.23534609e-02,  6.37209672e-02,  5.19266818e-02,  3.95125906e-02,  2.87162721e-02,  2.32403384e-02,  1.58785209e-02,  7.83177361e-03,  8.78090110e-05, -1.60328381e-02, -2.99906207e-02, -4.31899506e-02, -5.54713276e-02, -6.00950486e-02, -6.47187696e-02, -6.93424906e-02, -7.39662117e-02, -7.80563006e-02, -7.85056060e-02, -7.89549115e-02, -7.94042170e-02, -8.05180404e-02, -8.22353064e-02, -8.51603349e-02, -8.93769117e-02, -9.48422691e-02, -1.01378533e-01, -1.08534557e-01, -1.18764622e-01, -1.39050746e-01, -1.63808051e-01, -1.91457240e-01, -2.18646874e-01, -2.43860702e-01, -2.70342201e-01, -2.97195181e-01, -3.20403135e-01, -3.39725106e-01, -3.54795782e-01, -3.64325532e-01, -3.68517486e-01, -3.66519484e-01, -3.58354872e-01, -3.37721665e-01, -3.15785094e-01, -2.89561308e-01, -2.55143472e-01, -2.20725635e-01, -1.86307798e-01, -1.51889962e-01, -1.17472125e-01, -8.30542884e-02, -4.86364517e-02, -1.58437877e-02,  1.58924566e-02,  4.76287009e-02,  7.93649452e-02,  1.11101189e-01,  1.42837434e-01,  1.56574934e-01,  1.53269301e-01,  1.50578982e-01,  1.47904227e-01,  1.46140618e-01,  1.43983379e-01,  1.41617866e-01,  1.38048844e-01,  1.32538937e-01,  1.27182196e-01,  1.20971466e-01,  1.13696219e-01,  1.01567846e-01,  8.58113164e-02};
float SS_servo3[] = {0.19071313, 0.15454042, 0.11969017, 0.10203671, 0.09357716, 0.09034213, 0.08805214, 0.08576216, 0.09107226, 0.0978722 , 0.10337145, 0.10102738, 0.1001253 , 0.10063474, 0.10222963, 0.1215008 , 0.13945517, 0.15569323, 0.16947475, 0.16749744, 0.16552013, 0.16354282, 0.16156551, 0.15865421, 0.14937063, 0.14008705, 0.13080347, 0.12153518, 0.11175349, 0.10573981, 0.10197719, 0.09976851, 0.09853074, 0.09713773, 0.09991065, 0.11660426, 0.14754611, 0.18343611, 0.23439356, 0.29462485, 0.36233216, 0.43863373, 0.51139634, 0.57211518, 0.62670477, 0.66733757, 0.69569332, 0.70876681, 0.7068434 , 0.67683562, 0.64456205, 0.60642841, 0.55709457, 0.50776073, 0.45842688, 0.40909304, 0.3597592 , 0.31042536, 0.26109152, 0.22799634, 0.20545688, 0.18291741, 0.16037794, 0.13783848, 0.11529901, 0.11051993, 0.11063672, 0.11079014, 0.11029951, 0.11037348, 0.1100983 , 0.11288287, 0.1193426 , 0.12690212, 0.13527874, 0.14507959, 0.15665576, 0.17547637, 0.20257265};
float SS_servo4[] = { 0.21670508,  0.20719453,  0.19518341,  0.18255566,  0.16973687,  0.15903783,  0.14870735,  0.13837688,  0.1309403 ,  0.12407101,  0.11853413,  0.11553137,  0.11266382,  0.10957939,  0.1044414 ,  0.09357466,  0.079573  ,  0.0679139 ,  0.05843907,  0.05125668,  0.04407429,  0.03689189,  0.0297095 ,  0.023318  ,  0.02232247,  0.02132694,  0.02033141,  0.01878817,  0.01264243,  0.00706834,  0.00324413,  0.00268464,  0.00695624,  0.01643765,  0.03238451,  0.05104055,  0.07287458,  0.08844537,  0.09300707,  0.08722806,  0.06656249,  0.04044988,  0.01841295, -0.00212993, -0.02156249, -0.04234296, -0.06486134, -0.09027808, -0.1204039 , -0.15595639, -0.19223076, -0.21946324, -0.22941418, -0.23936513, -0.24931607, -0.25926701, -0.26921795, -0.27916889, -0.28911983, -0.2810738 , -0.26132911, -0.24158441, -0.22183972, -0.20209502, -0.18235032, -0.14894056, -0.10893936, -0.06718226, -0.02693575,  0.01136767,  0.04629893,  0.07947165,  0.10932737,  0.13466093,  0.15792833,  0.17816439,  0.19462182,  0.20722673,  0.21757201};
float SS_servo5[] = {-0.12226447, -0.09928259, -0.10000093, -0.09895452, -0.09489696, -0.09322594, -0.09213158, -0.09103722, -0.09423512, -0.09827443, -0.10303152, -0.11013486, -0.11976069, -0.12890948, -0.13237657, -0.12344359, -0.1054645 , -0.0922697 , -0.08399827, -0.08096534, -0.0779324 , -0.07489947, -0.07186653, -0.07027305, -0.07850047, -0.0867279 , -0.09495532, -0.10339003, -0.10450301, -0.1053461 , -0.10990492, -0.12177616, -0.14429285, -0.17848189, -0.22736683, -0.28836856, -0.35600105, -0.41666542, -0.44651375, -0.44709649, -0.41805061, -0.3748164 , -0.33770555, -0.30736504, -0.27702011, -0.24508964, -0.20964697, -0.16865997, -0.11798821, -0.05635424,  0.00672164,  0.05253414,  0.06535154,  0.07816894,  0.09098634,  0.10380374,  0.11662114,  0.12943854,  0.14225594,  0.12159586,  0.07917423,  0.03675259, -0.00566904, -0.04809067, -0.0905123 , -0.16256086, -0.24338315, -0.31850857, -0.38108853, -0.42449811, -0.44985948, -0.46088285, -0.45177401, -0.42661652, -0.39193243, -0.34644631, -0.28749519, -0.21845551, -0.14101636};
float SS_servo6[] = {0.21480266, 0.21123307, 0.20254494, 0.18383709, 0.16225377, 0.14286482, 0.12379908, 0.10473334, 0.08535247, 0.06590982, 0.04623733, 0.02804438, 0.01334391, 0.00373786, 0.00101231, 0.01730551, 0.04446629, 0.0785421 , 0.11293671, 0.14545542, 0.17797414, 0.21049286, 0.24301157, 0.27432311, 0.2973985 , 0.32047388, 0.34354927, 0.36274764, 0.37429677, 0.37983647, 0.379916  , 0.37440607, 0.36388093, 0.35039734, 0.33479613, 0.31448343, 0.29453686, 0.27694769, 0.27174883, 0.27219523, 0.26588363, 0.2569203 , 0.24993834, 0.24248694, 0.23733922, 0.23334595, 0.2291958 , 0.22345406, 0.2128581 , 0.20569192, 0.19897719, 0.19273716, 0.18740441, 0.18207165, 0.1767389 , 0.17140614, 0.16607339, 0.16074064, 0.15540788, 0.15348058, 0.15376695, 0.15405332, 0.15433968, 0.15462605, 0.15491242, 0.1518308 , 0.14938462, 0.14672066, 0.14305318, 0.14129057, 0.13743539, 0.13645307, 0.13729116, 0.13850225, 0.14318181, 0.1519131 , 0.16569494, 0.18541521, 0.21465974};
float SS_servo7[] = {-0.2653022 , -0.28162701, -0.29145154, -0.27857578, -0.25734375, -0.23869673, -0.22055112, -0.20240551, -0.18225884, -0.16171989, -0.13881522, -0.11618263, -0.10266223, -0.09907551, -0.10733387, -0.15256203, -0.21594325, -0.29101941, -0.36480654, -0.42333558, -0.48186462, -0.54039366, -0.59892271, -0.65235864, -0.67104607, -0.6897335 , -0.70842093, -0.7157816 , -0.70075694, -0.66631389, -0.61667748, -0.55278019, -0.47746031, -0.39782633, -0.31758002, -0.23156798, -0.16272381, -0.12452047, -0.12821742, -0.14085521, -0.1406068 , -0.13197389, -0.12513678, -0.12105529, -0.11924855, -0.12073819, -0.12182592, -0.11993858, -0.10815847, -0.10213519, -0.09683577, -0.09206731, -0.08831364, -0.08455998, -0.08080632, -0.07705266, -0.073299  , -0.06954534, -0.06579168, -0.06714793, -0.07182581, -0.07650369, -0.08118157, -0.08585945, -0.09053733, -0.08990148, -0.09139218, -0.09121581, -0.08971362, -0.0896081 , -0.0865205 , -0.08647601, -0.08869365, -0.09432924, -0.10568959, -0.1252052 , -0.15488653, -0.19841529, -0.25937845};

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
        servo2=map(servo2_deg,-37,90,1500,2670);
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
        servo2=map(servo2_deg,-37,90,1500,2670);
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
