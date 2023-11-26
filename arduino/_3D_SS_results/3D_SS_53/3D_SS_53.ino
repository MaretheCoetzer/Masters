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
float SS_servo0[] = {-0.66833245, -0.66123589, -0.60709395, -0.54918223, -0.48820854, -0.41938722, -0.33967404, -0.25915333, -0.17805596, -0.09687149, -0.01591244, -0.01908049, -0.04523194, -0.05626553, -0.0604168 , -0.07492285, -0.11111692, -0.14630484, -0.18129837, -0.21538243, -0.24772152, -0.27565928, -0.30276369, -0.331617  , -0.34860775, -0.36050669, -0.37617783, -0.38959528, -0.40779136, -0.42948514, -0.45382704, -0.48281179, -0.51080517, -0.54224053, -0.56361176, -0.58334308, -0.60144049, -0.62584562, -0.6426728 , -0.64584   , -0.64791502, -0.64756378, -0.64728918, -0.64891258, -0.65284018};
float SS_servo1[] = {0.86330166, 0.89014087, 0.91589712, 0.93941034, 0.9591598 , 0.97124527, 0.97209234, 0.97209552, 0.96920305, 0.96157094, 0.94779757, 1.01557234, 1.15441546, 1.26648917, 1.36108258, 1.41660968, 1.3926445 , 1.35945981, 1.32470682, 1.28261653, 1.25790629, 1.26137567, 1.26458337, 1.25426997, 1.20543059, 1.15246936, 1.10997123, 1.0656108 , 1.05124357, 1.0740724 , 1.0964341 , 1.11879698, 1.13251944, 1.16449298, 1.19044635, 1.20418111, 1.21658146, 1.24369472, 1.23743174, 1.16413217, 1.08805343, 1.00896429, 0.93206705, 0.87428636, 0.84577207};      
float SS_servo2[] = {-2.91295405e-01, -2.97168224e-01, -3.02780747e-01, -3.07698540e-01, -3.10809300e-01, -3.21643799e-01, -3.52467379e-01, -3.90626672e-01, -4.33590598e-01, -4.78101095e-01, -5.21309036e-01, -5.58123781e-01, -5.75076357e-01, -5.87347552e-01, -5.98796818e-01, -6.11475642e-01, -6.18144236e-01, -6.24559881e-01, -6.31155046e-01, -6.38590100e-01, -6.42605681e-01, -6.50993911e-01, -6.57593284e-01, -6.58091258e-01, -5.95411555e-01, -5.25966577e-01, -4.56766666e-01, -3.87158320e-01, -3.11370796e-01, -2.30948765e-01, -1.50287097e-01, -6.94925514e-02,  1.12933299e-02,  7.96319490e-02,  5.15084062e-02,  6.48369892e-04, -4.33488744e-02, -8.40479091e-02, -1.21790444e-01, -1.49312336e-01, -1.73814470e-01, -1.96796140e-01, -2.19167881e-01, -2.44362263e-01, -2.78599422e-01};
float SS_servo3[] = {0.93755743, 0.88123448, 0.79689908, 0.71815989, 0.64660183, 0.61392144, 0.65370531, 0.72554522, 0.81790151, 0.91720573, 1.01247238, 1.09574011, 1.17724692, 1.25290394, 1.32398806, 1.35159026, 1.28676969, 1.21526369, 1.14362804, 1.07138585, 1.00719941, 0.97755687, 0.94630867, 0.95691065, 0.97442971, 0.98643267, 0.99793998, 1.00897379, 0.99519289, 0.93981778, 0.88779563, 0.82589325, 0.76536106, 0.72427611, 0.78578569, 0.88034021, 0.96503791, 1.04755   , 1.09087448, 1.05518271, 1.01328611, 0.96874816, 0.92548332, 0.90771902, 0.93810477};      
float SS_servo4[] = { 0.57557334,  0.5846078 ,  0.58166687,  0.57847229,  0.57634193,  0.56988194,  0.5597078 ,  0.55208551,  0.54864145,  0.54908854,  0.54892347,  0.54398121,  0.52439335,  0.4997982 ,  0.47306587,  0.45450541,  0.4514564 ,  0.4522135 ,  0.45154246,  0.44418978,  0.43279959,  0.41358418,  0.39563475,  0.36987893,  0.33037893,  0.28747429,  0.24870062,  0.20961776,  0.1684922 ,  0.12683708,  0.08414674,  0.0350306 , -0.02522808, -0.03848027,  0.03543231,  0.11630968,  0.19742945,  0.27837954,  0.35242775,  0.4136878 ,  0.47883455,  0.55192825,  0.5979804 ,  0.58288428,  0.57404741};
float SS_servo5[] = {-0.74276857, -0.81752637, -0.89598859, -0.97342042, -1.04644469, -1.09100668, -1.09966211, -1.09614105, -1.08685449, -1.0794456 , -1.07042772, -1.05334405, -0.97135343, -0.87575742, -0.78049946, -0.73909993, -0.80139619, -0.8819631 , -0.96169114, -1.03749455, -1.09551253, -1.10786319, -1.12207859, -1.13450656, -1.12435483, -1.10144647, -1.09002986, -1.07688956, -1.04017757, -0.96548773, -0.89459725, -0.8213332 , -0.72731178, -0.69754558, -0.76273055, -0.79018195, -0.79253123, -0.81472933, -0.85134438, -0.83199382, -0.8121019 , -0.80174371, -0.78114568, -0.76489396, -0.73457925};
float SS_servo6[] = {0.34637055, 0.33877835, 0.31978883, 0.30266508, 0.28279904, 0.25717457, 0.22968745, 0.20438632, 0.17315667, 0.14243588, 0.11565569, 0.08334284, 0.09521391, 0.17186113, 0.25315809, 0.32725172, 0.38243717, 0.43004266, 0.47670661, 0.51896559, 0.56564915, 0.57572941, 0.5803565 , 0.57055218, 0.58357059, 0.59834222, 0.61295054, 0.62785783, 0.63276747, 0.61723353, 0.60780453, 0.60610794, 0.60803604, 0.58940102, 0.54441812, 0.49530456, 0.4486428 , 0.40454544, 0.37562518, 0.38357082, 0.39439485, 0.40035109, 0.40118249, 0.38968543, 0.35728414};      
float SS_servo7[] = {-0.81436414, -0.86506912, -0.91758686, -0.97403541, -1.01983863, -1.03002935, -0.99682267, -0.94640998, -0.86773597, -0.78648607, -0.70384871, -0.60051612, -0.55162832, -0.58125825, -0.58463001, -0.57742037, -0.54992443, -0.51195857, -0.47342189, -0.43221458, -0.49233163, -0.47516403, -0.42866156, -0.32971304, -0.37656021, -0.46326341, -0.54932308, -0.635428  , -0.688832  , -0.68121623, -0.69027457, -0.72063942, -0.76272071, -0.75863177, -0.68862276, -0.61416302, -0.54005709, -0.46633858, -0.44116984, -0.5292574 , -0.62517   , -0.71403795, -0.79460136, -0.83666778, -0.8104836 };

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
