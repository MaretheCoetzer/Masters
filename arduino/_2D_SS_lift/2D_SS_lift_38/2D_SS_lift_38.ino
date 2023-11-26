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
float SS_servo0[] = {-0.20193226, -0.18659271, -0.17490129, -0.16861754, -0.16818138, -0.17836474, -0.19102837, -0.20154076, -0.20683   , -0.20770593, -0.19787344, -0.17785093, -0.15053834, -0.11806042, -0.08068574, -0.04139863, -0.01977413, -0.01203613, -0.00768659, -0.01016901, -0.01441599, -0.01756397, -0.01672255, -0.01866773, -0.02075454, -0.02689397, -0.03442317, -0.04481344, -0.05347602, -0.06257934, -0.07099283, -0.08071107, -0.08956907, -0.0969089 , -0.10234365, -0.10431321, -0.09773237, -0.08030669, -0.06497931, -0.06615938, -0.06733946, -0.06764717, -0.06701161, -0.06637605, -0.06574049, -0.06510494, -0.06446938, -0.06383382, -0.06319826, -0.0625627 , -0.0679282 , -0.07474588, -0.08156355, -0.08838123, -0.0951989 , -0.10201658, -0.11117589, -0.12633891, -0.14530523, -0.15765997, -0.16055629, -0.15791864, -0.15128637, -0.14349016, -0.13497442, -0.13021014, -0.12934883, -0.13233882, -0.1358973 , -0.13849374, -0.140267  , -0.14639596, -0.1592465 , -0.17755158, -0.20505798};
float SS_servo1[] = {0.18309215, 0.14118454, 0.10152252, 0.08850125, 0.10285235, 0.16536878, 0.24632334, 0.3215102 , 0.38503941, 0.43869196, 0.46189531, 0.44646768, 0.41145074, 0.35336957, 0.27129071, 0.18980122, 0.13831998, 0.11545977, 0.10007794, 0.09977437, 0.10047764, 0.09894692, 0.09154778, 0.09011911, 0.08904164, 0.09801227, 0.10931523, 0.12353671, 0.13625107, 0.14912106, 0.16113166, 0.17222014, 0.18289886, 0.19309745, 0.20010495, 0.1980739 , 0.17969121, 0.13649863, 0.09863035, 0.09465835, 0.09068634, 0.0858618 , 0.08011544, 0.07436907, 0.0686227 , 0.06287634, 0.05712997, 0.05138361, 0.04563724, 0.03989087, 0.04672634, 0.05660645, 0.06648656, 0.07636667, 0.08624678, 0.09612689, 0.11101538, 0.13868224, 0.1727763 , 0.19467326, 0.19338225, 0.18044233, 0.16207269, 0.14259274, 0.12165909, 0.10766948, 0.10133235, 0.10263869, 0.10372386, 0.10205752, 0.10158196, 0.10681   , 0.12449613, 0.15219906, 0.19305819};
float SS_servo2[] = {-0.07648021, -0.06517784, -0.04761   , -0.03178783, -0.02270306, -0.0235229 , -0.02530862, -0.02986078, -0.03866158, -0.04616944, -0.04790458, -0.05413646, -0.0563343 , -0.06136698, -0.07393267, -0.08253424, -0.0894685 , -0.09089564, -0.09092066, -0.08811863, -0.08684005, -0.08745495, -0.09242942, -0.09610782, -0.09969507, -0.10067437, -0.10131329, -0.10260992, -0.10547644, -0.10830384, -0.11236447, -0.11541298, -0.11828815, -0.12104505, -0.12327869, -0.12626401, -0.13027294, -0.14765289, -0.17232108, -0.19924197, -0.22616286, -0.24036253, -0.24080699, -0.24125146, -0.24169592, -0.24214039, -0.24258486, -0.24302932, -0.24347379, -0.24391825, -0.22316459, -0.19728124, -0.17139789, -0.14551454, -0.11963119, -0.09374783, -0.06334172, -0.02669817,  0.00909619,  0.03801637,  0.04511971,  0.0461759 ,  0.04358345,  0.04225679,  0.03971433,  0.03709217,  0.03517393,  0.03333038,  0.02895036,  0.0191764 ,  0.0059187 , -0.00950994, -0.02771219, -0.04954002, -0.07684068};
float SS_servo3[] = {0.26600654, 0.23169353, 0.17956334, 0.13436623, 0.10370301, 0.09376884, 0.08745199, 0.08582415, 0.09249373, 0.09808077, 0.09509124, 0.09784411, 0.09633152, 0.09747857, 0.11472732, 0.13079223, 0.1373737 , 0.13318627, 0.12673996, 0.1157392 , 0.10526196, 0.09858471, 0.10296601, 0.10506964, 0.10705241, 0.10557691, 0.10292074, 0.09869994, 0.09963519, 0.09974918, 0.10290642, 0.10046347, 0.09898561, 0.09987915, 0.10037306, 0.1003659 , 0.10339014, 0.13966095, 0.21002574, 0.29674068, 0.38345561, 0.44087877, 0.46662934, 0.49237991, 0.51813048, 0.54388105, 0.56963162, 0.59538219, 0.62113276, 0.64688333, 0.61808978, 0.57609722, 0.53410466, 0.4921121 , 0.45011954, 0.40812698, 0.35190887, 0.27419496, 0.19627265, 0.13402233, 0.11233011, 0.10247333, 0.10267262, 0.10156479, 0.1029455 , 0.10390219, 0.10319073, 0.10220931, 0.10496526, 0.11787481, 0.14079874, 0.16507617, 0.19392164, 0.2292336 , 0.27045754};
float SS_servo4[] = {0.2257368 , 0.21032927, 0.19668597, 0.18645551, 0.17880132, 0.17417356, 0.16807573, 0.16242449, 0.15779768, 0.15359533, 0.15154103, 0.15339748, 0.15558049, 0.15275365, 0.14130149, 0.13216065, 0.12182878, 0.11713537, 0.11487291, 0.11751189, 0.11936979, 0.11605073, 0.10852997, 0.10350266, 0.09865566, 0.09896782, 0.09892512, 0.09445391, 0.0938147 , 0.09480342, 0.10450042, 0.1125625 , 0.11662046, 0.11626701, 0.11249527, 0.10564486, 0.09659571, 0.08405731, 0.07539151, 0.07406362, 0.07273573, 0.06955983, 0.06438573, 0.05921162, 0.05403751, 0.0488634 , 0.04368929, 0.03851518, 0.03334107, 0.02816696, 0.02662156, 0.02595427, 0.02528697, 0.02461967, 0.02395237, 0.02328507, 0.024831  , 0.02516167, 0.02083493, 0.01553983, 0.0147284 , 0.01896298, 0.04193431, 0.07757965, 0.11682992, 0.15139797, 0.17867508, 0.19683358, 0.20643091, 0.21025269, 0.2098778 , 0.208721  , 0.20724997, 0.21284526, 0.22920115};
float SS_servo5[] = {-0.14458504, -0.1248184 , -0.11356649, -0.10593211, -0.1026745 , -0.10493735, -0.10258516, -0.10204465, -0.10388854, -0.1050262 , -0.10736669, -0.12096254, -0.13127831, -0.13456735, -0.1196625 , -0.10260038, -0.08918421, -0.08679   , -0.08872746, -0.09939865, -0.11101134, -0.11222249, -0.10266702, -0.0978967 , -0.09342745, -0.09757249, -0.10145997, -0.0992435 , -0.10281991, -0.11042028, -0.13506837, -0.15995471, -0.17540668, -0.17934584, -0.17570561, -0.16785276, -0.15457029, -0.13705715, -0.12631845, -0.12997996, -0.13364147, -0.13263887, -0.12659305, -0.12054724, -0.11450142, -0.10845561, -0.10240979, -0.09636398, -0.09031816, -0.08427235, -0.08514564, -0.08769328, -0.09024091, -0.09278855, -0.09533619, -0.09788382, -0.10462863, -0.10823034, -0.10361189, -0.09588837, -0.10139558, -0.11767603, -0.1696412 , -0.24613805, -0.32882916, -0.39115111, -0.42352692, -0.42049588, -0.38619751, -0.33214484, -0.26051238, -0.19191979, -0.12650318, -0.10265934, -0.14780862};
float SS_servo6[] = { 0.12969466,  0.1233174 ,  0.1103221 ,  0.09503057,  0.07912481,  0.06166889,  0.04526556,  0.02926828,  0.01391968, -0.00080708, -0.01459428, -0.02963842, -0.03867214, -0.04218731, -0.02830904, -0.01277574, -0.00171813,  0.00046605,  0.00175643,  0.0012447 ,  0.00437995,  0.01935153,  0.05073942,  0.08079034,  0.11071583,  0.13705291,  0.16089976,  0.17326362,  0.1751772 ,  0.16823172,  0.15506183,  0.13539311,  0.1278893 ,  0.12543112,  0.12355285,  0.12030971,  0.11864499,  0.11651877,  0.10919568,  0.09931584,  0.089436  ,  0.08304739,  0.08043376,  0.07782013,  0.0752065 ,  0.07259288,  0.06997925,  0.06736562,  0.06475199,  0.06213836,  0.06281871,  0.06429616,  0.06577361,  0.06725106,  0.0687285 ,  0.07020595,  0.07132643,  0.07579964,  0.08109395,  0.08547306,  0.08647102,  0.08366304,  0.0745233 ,  0.06164917,  0.05338863,  0.04857705,  0.04604499,  0.04397272,  0.03986638,  0.03765519,  0.04326907,  0.05033633,  0.06967989,  0.09887854,  0.12850023};
float SS_servo7[] = {-0.33204868, -0.33039447, -0.32027586, -0.30219468, -0.2820198 , -0.25810057, -0.23466399, -0.21297121, -0.19293215, -0.17258994, -0.15101492, -0.13031281, -0.11791564, -0.11987524, -0.15435172, -0.17823397, -0.20198853, -0.16758205, -0.13417638, -0.10278857, -0.11420049, -0.15261725, -0.22235782, -0.28463343, -0.34628533, -0.39009213, -0.42508197, -0.42150398, -0.37761192, -0.30406239, -0.21829465, -0.11757297, -0.10076034, -0.10047888, -0.10070302, -0.10015555, -0.10184948, -0.1053981 , -0.09736658, -0.0836284 , -0.06989022, -0.06235719, -0.06153366, -0.06071014, -0.05988661, -0.05906309, -0.05823956, -0.05741603, -0.05659251, -0.05576898, -0.0611797 , -0.06809903, -0.07501835, -0.08193768, -0.08885701, -0.09577634, -0.1016793 , -0.11367814, -0.12855086, -0.14042828, -0.14956505, -0.151545  , -0.13805551, -0.11577201, -0.10279556, -0.09744837, -0.09693803, -0.09748791, -0.09529375, -0.09777213, -0.11323338, -0.13474965, -0.18228409, -0.25089325, -0.32581475};

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
