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
float SS_servo0[] = {-0.50216139, -0.50665897, -0.50201219, -0.45043642, -0.37889016, -0.29752861, -0.20732651, -0.11947116, -0.02412937,  0.06804112,  0.13532218,  0.13593482,  0.08469283,  0.0370509 ,  0.00130611, -0.03056473, -0.06108365, -0.09037302, -0.12037756, -0.14849555, -0.17106848, -0.19282209, -0.21301272, -0.23608122, -0.261865  , -0.29211445, -0.32295418, -0.35240753, -0.37907805, -0.40369356, -0.42829163, -0.45068137, -0.46774861, -0.48139591, -0.48950168, -0.49802081, -0.50569451, -0.51203004, -0.51180509, -0.50370149};
float SS_servo1[] = {0.55042158, 0.51640223, 0.51459136, 0.58022922, 0.65942369, 0.71827405, 0.73375426, 0.72480004, 0.67294487, 0.61070192, 0.55319823, 0.53461108, 0.61394217, 0.68752251, 0.73414363, 0.76479129, 0.78328334, 0.78978426, 0.78261448, 0.75917135, 0.72732867, 0.71018426, 0.70502304, 0.71307073, 0.72983114, 0.75601032, 0.78170659, 0.79369615, 0.79118907, 0.79122494, 0.80100076, 0.81395977, 0.81183786, 0.79977524, 0.77666406, 0.75348929, 0.7277642 , 0.69791743, 0.64700004, 0.57355699};
float SS_servo2[] = {-0.25578142, -0.27313231, -0.29289807, -0.30313292, -0.31185812, -0.32537648, -0.34603195, -0.37117374, -0.39986274, -0.43159532, -0.4637115 , -0.48905049, -0.50726965, -0.5117881 , -0.51309006, -0.51138254, -0.50863465, -0.50152395, -0.4931066 , -0.48950546, -0.47384344, -0.42941389, -0.36071425, -0.28673629, -0.20826227, -0.12540424, -0.04528389,  0.00830492,  0.02358917,  0.04061646,  0.04905701,  0.02779038, -0.02330047, -0.06958976, -0.10518485, -0.13771257, -0.1659196 , -0.19332033, -0.22116319, -0.24557643};
float SS_servo3[] = {0.67224354, 0.66046846, 0.63390336, 0.60288108, 0.57840949, 0.57023295, 0.58093181, 0.60242675, 0.6323256 , 0.6739968 , 0.72198652, 0.74658575, 0.7536848 , 0.73645273, 0.71241619, 0.67731645, 0.63351317, 0.57453305, 0.49865228, 0.42005589, 0.38629309, 0.44602884, 0.52011659, 0.59417051, 0.65764944, 0.69645318, 0.70670163, 0.66184679, 0.58892234, 0.52303292, 0.4718922 , 0.48649029, 0.55618016, 0.61190422, 0.64388789, 0.66672137, 0.67798306, 0.68500022, 0.68614934, 0.67379211};
float SS_servo4[] = { 0.51148353,  0.51315777,  0.51269686,  0.50945108,  0.50195301,  0.4949797 ,  0.48850596,  0.47878426,  0.46621679,  0.45193787,  0.43122588,  0.40336323,  0.37016045,  0.34055377,  0.31583553,  0.29825946,  0.2868041 ,  0.27520517,  0.25632535,  0.22941769,  0.2000119 ,  0.17689788,  0.1574816 ,  0.13671884,  0.1133608 ,  0.08654909,  0.05632209,  0.01400018, -0.04651892, -0.0786091 , -0.0374646 ,  0.04085605,  0.12241048,  0.2014736 ,  0.27411541,  0.34808016,  0.42092009,  0.48531221,  0.51132633,  0.51125391};
float SS_servo5[] = {-0.56815011, -0.61524578, -0.67755953, -0.71865372, -0.74331449, -0.76336837, -0.78028127, -0.78950044, -0.79108879, -0.7834845 , -0.75851023, -0.72719859, -0.68956008, -0.65716882, -0.63493498, -0.63252053, -0.65009265, -0.67598631, -0.70145865, -0.72407718, -0.73958686, -0.75530569, -0.76529266, -0.76444084, -0.75408495, -0.73522063, -0.70987719, -0.66899705, -0.5992439 , -0.58221928, -0.66889842, -0.74169674, -0.79900102, -0.83151126, -0.81498814, -0.76785393, -0.69311026, -0.5941203 , -0.51631534, -0.55053027};
float SS_servo6[] = { 0.23581587,  0.22440766,  0.20124646,  0.17359411,  0.1440206 ,  0.11083991,  0.07660181,  0.03564552, -0.01142646, -0.0313269 ,  0.00156575,  0.05288593,  0.11935889,  0.1921211 ,  0.27328384,  0.35239322,  0.42416895,  0.47801561,  0.49700597,  0.48996569,  0.4697425 ,  0.45966682,  0.46960167,  0.48227245,  0.49085212,  0.49494677,  0.49771463,  0.49771606,  0.49192617,  0.47464936,  0.44845728,  0.41480868,  0.38472173,  0.35705124,  0.33338078,  0.31151569,  0.29536664,  0.28282384,  0.26558079,  0.24189589};
float SS_servo7[] = {-0.68620891, -0.71033742, -0.73149942, -0.73000673, -0.71483096, -0.68482896, -0.64640263, -0.59071365, -0.51940687, -0.4962905 , -0.57156362, -0.65132625, -0.66999413, -0.6236929 , -0.56091276, -0.49418325, -0.41724251, -0.31962421, -0.2627757 , -0.30515758, -0.32720309, -0.36097573, -0.42097451, -0.48008032, -0.52855658, -0.56789949, -0.60581276, -0.64969012, -0.69285581, -0.70753454, -0.69599637, -0.66206845, -0.6385153 , -0.62193928, -0.61348858, -0.61040666, -0.62078369, -0.6416139 , -0.66247252, -0.67618497};

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=100000;
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
