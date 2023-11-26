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
float SS_servo0[] = {-0.56331815, -0.55184007, -0.48492815, -0.41610588, -0.34637846, -0.26869364, -0.18838113, -0.10786119, -0.02692095,  0.05425531,  0.13351759,  0.12864344,  0.08276875,  0.04773389,  0.03026281,  0.01459646, -0.00415389, -0.02627083, -0.05136128, -0.08049069, -0.10899093, -0.13165468, -0.15212737, -0.16392712, -0.16515834, -0.16172143, -0.15500971, -0.15320472, -0.16637617, -0.19432584, -0.22940477, -0.26910092, -0.30991683, -0.34529964, -0.3738067 , -0.3950275 , -0.40761556, -0.41835347, -0.43027878, -0.44308531, -0.45925764, -0.47651402, -0.49478101, -0.51390232, -0.53141203, -0.55067556, -0.55794142};
float SS_servo1[] = {0.70451327, 0.72552803, 0.73858737, 0.75043926, 0.76149426, 0.76448224, 0.76472736, 0.76475797, 0.76430205, 0.75325097, 0.70596412, 0.7494705 , 0.83926534, 0.93172016, 1.02619727, 1.09946629, 1.11191134, 1.10350625, 1.08661918, 1.05711482, 1.01703345, 0.98791134, 0.95605437, 0.88882814, 0.82051325, 0.7411819 , 0.66166046, 0.60998792, 0.60818798, 0.65422755, 0.72955053, 0.81438795, 0.90038029, 0.96677306, 0.9951821 , 0.9956608 , 0.97812875, 0.95734857, 0.93772984, 0.91831903, 0.89970233, 0.87874369, 0.84899044, 0.8186486 , 0.79031098, 0.77451626, 0.73496574};
float SS_servo2[] = {-0.36653149, -0.37939237, -0.37860738, -0.37436172, -0.36971771, -0.3792196 , -0.40744175, -0.45191212, -0.49909622, -0.54616781, -0.5916856 , -0.62168344, -0.63272713, -0.64424372, -0.65302209, -0.66262047, -0.66335496, -0.66097263, -0.65832685, -0.6557013 , -0.65377818, -0.63737456, -0.61830685, -0.59799055, -0.58407429, -0.53726585, -0.47846957, -0.40722631, -0.32721025, -0.24665502, -0.16585968, -0.08492643, -0.00519039,  0.04009342, -0.00278602, -0.03806389, -0.06321149, -0.08679249, -0.11067013, -0.1348001 , -0.15989386, -0.18518273, -0.21231224, -0.24482558, -0.27974004, -0.31217978, -0.34649043};
float SS_servo3[] = {0.78779196, 0.78717334, 0.71986198, 0.64266763, 0.56989571, 0.54437304, 0.57465077, 0.653171  , 0.74495761, 0.83430672, 0.9208129 , 0.97226577, 0.97562437, 0.9975221 , 1.04762664, 1.08134659, 1.05497922, 1.00702071, 0.94830846, 0.8745096 , 0.79429054, 0.70468771, 0.61735752, 0.62869406, 0.70019237, 0.73393031, 0.7559084 , 0.76515688, 0.76532078, 0.76504524, 0.76160679, 0.71285612, 0.68636706, 0.65617556, 0.71740484, 0.75272417, 0.76277966, 0.76861867, 0.77302587, 0.77634433, 0.77550438, 0.77082587, 0.76046224, 0.75731153, 0.76241625, 0.77372808, 0.78608773};
float SS_servo4[] = {0.60875669, 0.61176054, 0.62197961, 0.63275207, 0.64338973, 0.65001305, 0.64682216, 0.64106396, 0.63869254, 0.63748608, 0.64166769, 0.63890001, 0.64842025, 0.6486495 , 0.61574465, 0.58348614, 0.56378587, 0.56754816, 0.57420977, 0.58560133, 0.59354596, 0.5822739 , 0.56038573, 0.53856846, 0.49655695, 0.46052449, 0.42780491, 0.39577824, 0.36185213, 0.32622114, 0.29017961, 0.25166485, 0.22227654, 0.21936836, 0.23101526, 0.22398705, 0.27591671, 0.357084  , 0.43774887, 0.51834531, 0.59868038, 0.66154497, 0.6483483 , 0.61800432, 0.6121228 , 0.63127664, 0.62688503};
float SS_servo5[] = {-0.35454345, -0.38418024, -0.46463206, -0.54964757, -0.63066644, -0.68862037, -0.71200417, -0.71516191, -0.71371208, -0.71390769, -0.72044203, -0.71980663, -0.75327033, -0.75560442, -0.66675579, -0.58981106, -0.57594859, -0.62391255, -0.68793567, -0.77699555, -0.86853782, -0.90780044, -0.92615305, -0.95795623, -0.93524844, -0.92738324, -0.92231642, -0.90371712, -0.86379889, -0.80345733, -0.72785541, -0.64726672, -0.58679801, -0.55523072, -0.56917674, -0.56833436, -0.59164211, -0.5872472 , -0.58087082, -0.57502116, -0.57118333, -0.55562121, -0.49738537, -0.47865192, -0.51655442, -0.52863278, -0.44537395};
float SS_servo6[] = { 0.35943392,  0.33448626,  0.29731794,  0.26573393,  0.23165843,  0.20191986,  0.16832433,  0.131337  ,  0.09290236,  0.04862885,  0.01068516, -0.02429039, -0.05536419, -0.01679472,  0.06167849,  0.14214602,  0.22117083,  0.29635246,  0.37007087,  0.4388457 ,  0.47814274,  0.50260163,  0.50092639,  0.48061569,  0.48739451,  0.49260658,  0.50365262,  0.49595369,  0.47882805,  0.45022114,  0.41406235,  0.38302464,  0.35085507,  0.35792906,  0.37545231,  0.38320046,  0.36108571,  0.34111332,  0.33123443,  0.32542835,  0.33517817,  0.34749363,  0.36031715,  0.3699025 ,  0.37545899,  0.3743342 ,  0.3696404 };
float SS_servo7[] = {-0.96589623, -0.94184052, -0.93422126, -0.94401746, -0.94419312, -0.93210309, -0.88809609, -0.81585381, -0.72796071, -0.62995271, -0.52317822, -0.42317313, -0.34641121, -0.40034176, -0.44405673, -0.44654713, -0.44284649, -0.43172607, -0.4201999 , -0.40610785, -0.42079905, -0.41812566, -0.36735838, -0.24644818, -0.31630374, -0.38758108, -0.46784138, -0.49920475, -0.4960434 , -0.45454325, -0.38392773, -0.32271368, -0.25937486, -0.28109594, -0.34450258, -0.3971593 , -0.39230409, -0.392192  , -0.41337673, -0.44413636, -0.51146693, -0.58791207, -0.67588365, -0.76329781, -0.84122966, -0.89578287, -0.94635896};

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
