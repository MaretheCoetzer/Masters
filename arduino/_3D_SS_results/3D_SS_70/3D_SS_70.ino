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
float SS_servo0[] = {-0.6671533 , -0.6505556 , -0.5735466 , -0.49244599, -0.4092636 , -0.32305817, -0.23685275, -0.15251288, -0.06901079,  0.01156194,  0.0929525 ,  0.17391318,  0.23153016,  0.17844245,  0.15397405,  0.11038248,  0.06395014,  0.01473119, -0.03422593, -0.08171603, -0.12663024, -0.17035953, -0.21345383, -0.25498621, -0.30152883, -0.35545181, -0.41615481, -0.44981147, -0.46503683, -0.48026219, -0.4887119 , -0.49410865, -0.50801643, -0.529045  , -0.5469764 , -0.56289939, -0.57602556, -0.58552068, -0.59173622, -0.59579005, -0.60704136, -0.63990819, -0.67156075, -0.69590176, -0.69961318, -0.70332459, -0.70703601, -0.71074743, -0.71445884, -0.71817026, -0.72188168, -0.69726677, -0.67152696, -0.64578715, -0.62004733, -0.59430752, -0.5752087 , -0.58752133, -0.58969969, -0.58599741, -0.58424262, -0.59042664, -0.60457772, -0.62642886, -0.65225363};
float SS_servo1[] = {1.85673734, 1.84348988, 1.81961809, 1.75172798, 1.66989304, 1.59346529, 1.51703754, 1.454246  , 1.39630701, 1.32969386, 1.24885924, 1.16797416, 1.13940968, 1.21719108, 1.25429214, 1.29879088, 1.33414596, 1.37231714, 1.40961285, 1.44692036, 1.48500771, 1.5245677 , 1.56846243, 1.61195739, 1.654763  , 1.67297414, 1.66069285, 1.6474212 , 1.6402597 , 1.6330982 , 1.63826248, 1.64898048, 1.66124128, 1.66799707, 1.70169169, 1.75050916, 1.81323359, 1.88764786, 1.97260518, 2.05893639, 2.12420641, 2.110454  , 2.09061953, 2.07372895, 2.06514453, 2.05656012, 2.04797571, 2.03939129, 2.03080688, 2.02222247, 2.01363806, 2.02649054, 2.04019434, 2.05389813, 2.06760193, 2.08130573, 2.08381123, 2.04509334, 2.02579415, 2.01928818, 2.01016312, 1.99228532, 1.96335396, 1.9288806 , 1.87811347};
float SS_servo2[] = { 0.75274282,  0.71670601,  0.64943168,  0.57933603,  0.5087252 ,  0.43836973,  0.36801425,  0.29607582,  0.22355869,  0.15434665,  0.08908259,  0.02922631, -0.02341893, -0.06831831, -0.10771495, -0.14897867, -0.19409745, -0.24363968, -0.29029122, -0.3361557 , -0.38384892, -0.43122159, -0.47772761, -0.52226103, -0.57290253, -0.62750676, -1.01512134, -1.03419729, -0.63828358, -0.24236986,  0.16806449,  0.58504154,  0.72714457,  0.67934169,  0.68116338,  0.68549636,  0.6911986 ,  0.6983735 ,  0.71066602,  0.72440643,  0.70301837,  0.67325469,  0.64785366,  0.63259885,  0.64597167,  0.65934449,  0.67271731,  0.68609014,  0.69946296,  0.71283578,  0.7262086 ,  0.78066115,  0.83674507,  0.89282899,  0.94891292,  1.00499684,  1.04328189,  1.00818085,  0.96634558,  0.94214409,  0.92539144,  0.89882686,  0.86273042,  0.82041915,  0.77376125};
float SS_servo3[] = {0.34438066, 0.37351509, 0.45686014, 0.54433294, 0.63034245, 0.71293458, 0.79552672, 0.87871609, 0.96224032, 1.04615676, 1.13443558, 1.22087879, 1.30525651, 1.34855272, 1.38169301, 1.41114984, 1.43729042, 1.47030574, 1.4969989 , 1.52619464, 1.55775491, 1.59344791, 1.63138516, 1.6672474 , 1.70687691, 1.71993764, 5.22832061, 7.19473688, 6.24846251, 5.30218814, 3.41016266, 1.09200262, 0.15679108, 0.23170672, 0.28457062, 0.34057457, 0.40011957, 0.45664185, 0.4951392 , 0.52383327, 0.58903181, 0.60914259, 0.63227611, 0.64108948, 0.60949839, 0.57790729, 0.5463162 , 0.51472511, 0.48313401, 0.45154292, 0.41995183, 0.36510535, 0.30933535, 0.25356535, 0.19779534, 0.14202534, 0.09833514, 0.12781251, 0.20080941, 0.25156084, 0.27605949, 0.30197554, 0.32629891, 0.34192715, 0.34791896};
float SS_servo4[] = { 0.6577634 ,  0.62829584,  0.57862027,  0.52383307,  0.46792183,  0.4091209 ,  0.35031997,  0.28709264,  0.22232565,  0.16038263,  0.10368116,  0.05272894,  0.00826368, -0.04125842, -0.086384  , -0.13557616, -0.19236129, -0.25278895, -0.314145  , -0.37633807, -0.43643124, -0.49535025, -0.55536568, -0.61412243, -0.6772822 , -0.75451862, -0.84239404, -0.89025086, -0.90852221, -0.92679355, -0.93095911, -0.92876891, -0.94435108, -0.98738543, -1.018097  , -1.04323702, -1.06300235, -1.07977535, -1.09858195, -1.12566492, -1.13652128, -1.20873771, -1.32105935, -1.3609448 , -1.19645087, -1.03195694, -0.86746301, -0.70296909, -0.53847516, -0.37398123, -0.2094873 , -0.04569024,  0.11807915,  0.28184854,  0.44561793,  0.60938731,  0.67084224,  0.64497224,  0.633096  ,  0.62922285,  0.63875882,  0.6446588 ,  0.64649129,  0.6541538 ,  0.67029929};
float SS_servo5[] = {-1.9360616 , -1.92592404, -1.88737759, -1.842875  , -1.80103115, -1.75652689, -1.71202262, -1.66827468, -1.62492395, -1.5838102 , -1.53517827, -1.48745662, -1.43597623, -1.3734531 , -1.29848769, -1.21756957, -1.13036258, -1.04270996, -0.95555384, -0.86310893, -0.77595851, -0.69201971, -0.60736204, -0.52657561, -0.45017162, -0.37466636, -0.32074573, -0.3105095 , -0.3258781 , -0.34124671, -0.36303215, -0.38770888, -0.39197011, -0.36049187, -0.33008843, -0.29707824, -0.2623675 , -0.22362648, -0.17637474, -0.11448793, -0.11448274, -1.05809947, -2.66729735, -3.87149073, -3.93296002, -3.99442931, -4.05589861, -4.1173679 , -4.17883719, -4.24030649, -4.30177578, -3.91491497, -3.51024991, -3.10558484, -2.70091977, -2.29625471, -2.14805418, -2.10661559, -2.04323622, -2.00475612, -2.01720416, -2.00082711, -1.97552819, -1.94159336, -1.92612532};
float SS_servo6[] = { 0.48488411,  0.45314649,  0.40365424,  0.35100784,  0.29500452,  0.23619651,  0.17738851,  0.11462379,  0.05046973, -0.01192684, -0.07082635, -0.12614292, -0.17925596, -0.23450382, -0.21088342, -0.1305439 , -0.04934411,  0.031956  ,  0.11427367,  0.19695469,  0.27884481,  0.3599674 ,  0.44055039,  0.52105595,  0.60148693,  0.65550547,  0.60981945,  0.59695694,  0.61041882,  0.62388069,  0.63951042,  0.65611694,  0.65342984,  0.63360157,  0.6268713 ,  0.62532614,  0.63052543,  0.64011007,  0.65131868,  0.6609923 ,  0.65713404,  0.61642208,  0.56782749,  0.53083896,  0.52659708,  0.5223552 ,  0.51811332,  0.51387143,  0.50962955,  0.50538767,  0.50114579,  0.52972926,  0.5596163 ,  0.58950335,  0.61939039,  0.64927743,  0.66838741,  0.64776802,  0.6340228 ,  0.62524343,  0.61270213,  0.59453785,  0.57029499,  0.54073314,  0.50581046};
float SS_servo7[] = {-1.86579594, -1.83555387, -1.7718023 , -1.70842862, -1.6460779 , -1.58987113, -1.53366435, -1.4770777 , -1.42010125, -1.35897562, -1.28884192, -1.2084229 , -1.11441843, -1.02669958, -1.04517301, -1.12651015, -1.20046   , -1.26658616, -1.33466983, -1.40689996, -1.48458376, -1.56428632, -1.64402091, -1.72281511, -1.78792751, -1.80420988, -1.82955627, -1.89664647, -1.94874922, -2.00085198, -2.03215297, -2.05408115, -2.07269339, -2.08940197, -2.10323858, -2.10728376, -2.11212611, -2.11190857, -2.10760582, -2.11144202, -2.12412423, -2.1662986 , -2.20489158, -2.23522473, -2.24225265, -2.24928058, -2.2563085 , -2.26333642, -2.27036434, -2.27739227, -2.28442019, -2.26183406, -2.23807188, -2.2143097 , -2.19054753, -2.16678535, -2.14356486, -2.12123611, -2.05077265, -1.9795246 , -1.91668868, -1.87416577, -1.84908469, -1.84284294, -1.86113936};

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
