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
float SS_servo0[] = {-0.52438929, -0.51413563, -0.46514903, -0.3975036 , -0.31986514, -0.2546838 , -0.19386187, -0.12356024, -0.04734274,  0.0187333 ,  0.09937304,  0.22684081,  0.31838103,  0.34337898,  0.26951625,  0.20524568,  0.15070082,  0.09741462,  0.04466136, -0.00940902, -0.06539798, -0.12128714, -0.15992075, -0.18846892, -0.20665197, -0.22390968, -0.23405821, -0.25734474, -0.27920059, -0.32921893, -0.36979683, -0.40167763, -0.44863075, -0.49660624, -0.52254331, -0.52359686, -0.52359431, -0.52359541, -0.5226555 , -0.52360446, -0.52535601, -0.5231636 , -0.52244094, -0.52331869};
float SS_servo1[] = {0.26026108, 0.25962868, 0.32944697, 0.41171019, 0.49846944, 0.59222812, 0.63792637, 0.56413304, 0.48062933, 0.44367613, 0.45942546, 0.40249335, 0.33023296, 0.29152428, 0.40165029, 0.50030529, 0.58734656, 0.67614571, 0.76540268, 0.85460509, 0.94444313, 0.98848345, 0.95601593, 0.9058972 , 0.84334204, 0.81024695, 0.77765719, 0.77959581, 0.78560851, 0.83897095, 0.86592069, 0.86973077, 0.90457006, 0.95264356, 0.95975899, 0.91388766, 0.84243503, 0.77053344, 0.69628783, 0.62145043, 0.5509808 , 0.48294174, 0.41584645, 0.34432278};
float SS_servo2[] = {-0.29281476, -0.32700194, -0.34404531, -0.34961041, -0.34909035, -0.35271561, -0.3600485 , -0.36809973, -0.38398862, -0.41496652, -0.46694851, -0.49450003, -0.5140293 , -0.52960778, -0.51139848, -0.5184299 , -0.51553919, -0.5016925 , -0.49132078, -0.49575666, -0.50856567, -0.51352773, -0.52431752, -0.49427043, -0.4240811 , -0.35803026, -0.27869505, -0.19656834, -0.11253366, -0.01927293,  0.06349281,  0.14285455,  0.22491643,  0.30088794,  0.34864595,  0.32805048,  0.2551764 ,  0.18043263,  0.10521546,  0.03253686, -0.03845857, -0.11076313, -0.17550516, -0.23786973};
float SS_servo3[] = {0.7874015 , 0.77971422, 0.7089    , 0.62809267, 0.54730691, 0.48209584, 0.4281517 , 0.37714081, 0.34777857, 0.35279475, 0.41344273, 0.47021678, 0.51710964, 0.54624053, 0.48418796, 0.46585209, 0.43286854, 0.37994676, 0.33178869, 0.30778545, 0.28946359, 0.23053643, 0.21090073, 0.25143144, 0.33925452, 0.41374296, 0.49296133, 0.57671219, 0.6447374 , 0.58064787, 0.50012008, 0.41875843, 0.33827944, 0.26837371, 0.1854519 , 0.17591956, 0.2516839 , 0.33802065, 0.42465335, 0.50077251, 0.57333655, 0.65435712, 0.71667382, 0.76775163};
float SS_servo4[] = { 0.52467416,  0.52419452,  0.51681987,  0.51409601,  0.51604686,  0.51962608,  0.52361041,  0.52338395,  0.52346256,  0.52315027,  0.52150471,  0.51150679,  0.48480573,  0.44865117,  0.40131105,  0.36510998,  0.31870595,  0.28546119,  0.26454636,  0.23915497,  0.24645409,  0.22066384,  0.20646168,  0.19071004,  0.15492784,  0.09049434,  0.02913134, -0.03167424, -0.08652521, -0.1484567 , -0.21401538, -0.28902502, -0.34824609, -0.31570452, -0.21534661, -0.10246127, -0.02013539,  0.0592901 ,  0.14117964,  0.20568712,  0.26852816,  0.36832752,  0.46272755,  0.51176669};
float SS_servo5[] = {-0.33559543, -0.40013033, -0.4831736 , -0.56647003, -0.65016774, -0.7331806 , -0.81644996, -0.89485845, -0.96838517, -1.03631592, -1.08181691, -1.06334983, -1.01015957, -0.94821521, -0.89158769, -0.85454046, -0.79017426, -0.74907469, -0.73260081, -0.71011343, -0.76022718, -0.77181136, -0.84127425, -0.91431109, -0.9483683 , -0.88507626, -0.80436494, -0.72159   , -0.64229795, -0.56007372, -0.47789303, -0.38192116, -0.32345015, -0.38753018, -0.4529124 , -0.50975302, -0.5786685 , -0.63789859, -0.70333464, -0.71826345, -0.64666519, -0.55527268, -0.45879565, -0.34122455};
float SS_servo6[] = { 0.02175475, -0.01557278, -0.07495021, -0.13273916, -0.18898033, -0.2470115 , -0.304042  , -0.3560065 , -0.4142261 , -0.47856503, -0.50548   , -0.43491347, -0.32853941, -0.20615043, -0.1201101 , -0.03481031,  0.05439623,  0.13331112,  0.20393848,  0.28546275,  0.36988266,  0.442688  ,  0.48777542,  0.48093524,  0.44383205,  0.44426524,  0.44702241,  0.46351893,  0.48003743,  0.49601508,  0.50699004,  0.51840948,  0.53063968,  0.50280757,  0.45421677,  0.38868469,  0.33763272,  0.30504428,  0.27153688,  0.21681459,  0.15325345,  0.11216582,  0.08729945,  0.06053345};
float SS_servo7[] = {-0.48499402, -0.48253844, -0.46866852, -0.44734601, -0.41886974, -0.38000232, -0.34002811, -0.31045719, -0.26097081, -0.19326371, -0.17924053, -0.26794771, -0.34486879, -0.40350642, -0.48531827, -0.54013169, -0.58639769, -0.58862634, -0.52450249, -0.44689949, -0.36850231, -0.28925398, -0.18475357, -0.11380475, -0.11376223, -0.18215687, -0.24308892, -0.32212572, -0.39597177, -0.47788769, -0.55628894, -0.63966388, -0.72253   , -0.71739066, -0.66715218, -0.58889919, -0.5529601 , -0.55090801, -0.54869104, -0.50970962, -0.4527691 , -0.43521899, -0.45270012, -0.47343124};

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
