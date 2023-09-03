//Gait template
//FIRST STEP OF LEG 1 MODIFIED
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

float SS_servo0[] = { 0.4702135 ,  0.42104206,  0.28097617,  0.18440994,  0.10817335,  0.04246229, -0.01523361, -0.06561217, -0.10961185, -0.14813779, -0.17967338, -0.2045979 , -0.22577559, -0.24330995, -0.25805276, -0.27049012, -0.28064796, -0.28911628, -0.2961357 , -0.30138136, -0.30469346, -0.30646536, -0.3077356 , -0.30865137, -0.30902329, -0.30891614, -0.30840952, -0.30760173, -0.30682063, -0.30627959, -0.30487568, -0.30415483, -0.30298113, -0.30017026, -0.29576267, -0.28979694, -0.27998751, -0.26299937, -0.23479855, -0.19866217};
float SS_servo1[] = {0.99754025, 0.95607548, 1.0431468 , 1.0717476 , 1.06026289, 1.02674122, 0.98053635, 0.92872763, 0.87884105, 0.83946755, 0.81594346, 0.80898791, 0.81276786, 0.81838539, 0.82116165, 0.81789648, 0.80779022, 0.79316609, 0.77409401, 0.74849097, 0.71812641, 0.6878042 , 0.66278318, 0.64250729, 0.62636079, 0.61431827, 0.60675025, 0.60460248, 0.60979378, 0.62569776, 0.65265423, 0.68549654, 0.71570548, 0.74114416, 0.76011612, 0.76840826, 0.7678804 , 0.76045963, 0.74244493, 0.71239101};
float SS_servo2[] = { 0.42720666,  0.24835892,  0.10947514,  0.02506855, -0.04637056, -0.11079143, -0.16896515, -0.22068377, -0.26562532, -0.30311145, -0.33216939, -0.35315833, -0.36898042, -0.38146185, -0.39206746, -0.40129739, -0.40913411, -0.41555717, -0.42066272, -0.42565052, -0.43077255, -0.43393459, -0.43662824, -0.4383874 , -0.43956794, -0.44015843, -0.44025235, -0.43974371, -0.43893995, -0.43760431, -0.43267847, -0.42659705, -0.42027194, -0.41151464, -0.40094364, -0.38881634, -0.37291413, -0.34945675, -0.31249872, -0.26529825};
float SS_servo3[] = {0.8317576 , 1.00569092, 1.1148182 , 1.12684719, 1.1073612 , 1.07087914, 1.0242335 , 0.97310679, 0.92318706, 0.88069194, 0.85277654, 0.83978711, 0.83551383, 0.83352793, 0.83017955, 0.8227121 , 0.81035441, 0.79391591, 0.77321317, 0.74872637, 0.72298786, 0.69691933, 0.67565731, 0.65807609, 0.64435578, 0.63440932, 0.62864214, 0.62776505, 0.63367711, 0.64936782, 0.67203124, 0.69759426, 0.72031511, 0.73718955, 0.74740636, 0.74750202, 0.73938745, 0.72481861, 0.69570423, 0.65240813};
float SS_servo4[] = {0.4120557 , 0.49321875, 0.51824282, 0.51877082, 0.50991841, 0.49325838, 0.46942638, 0.44019398, 0.40655471, 0.3708247 , 0.33970576, 0.31533019, 0.29643635, 0.28196874, 0.27093599, 0.26246614, 0.25566177, 0.24722818, 0.23635961, 0.2261303 , 0.2183447 , 0.21437479, 0.2109875 , 0.20840894, 0.2063854 , 0.20483228, 0.20370649, 0.20266518, 0.2018708 , 0.19999049, 0.20211533, 0.23414212, 0.27026315, 0.30098128, 0.32471268, 0.33700437, 0.34431531, 0.35189877, 0.39391291, 0.44614069};
float SS_servo5[] = {-0.18811793, -0.32316428, -0.42774518, -0.51853371, -0.60510359, -0.68863974, -0.76796502, -0.84109495, -0.90337867, -0.94908393, -0.97692514, -0.98656752, -0.98528763, -0.98134343, -0.98011845, -0.98495574, -0.99594079, -1.00700984, -1.01750063, -1.03482492, -1.060329  , -1.0909987 , -1.11619495, -1.13656124, -1.15169071, -1.16121858, -1.16430847, -1.1592764 , -1.1437455 , -1.11351193, -1.07031978, -1.08093367, -1.12010583, -1.16603745, -1.21706798, -1.28572783, -1.32741734, -1.28408288, -1.16554646, -1.01801406};
float SS_servo6[] = {0.44257116, 0.50701167, 0.52109204, 0.51722803, 0.50535553, 0.4869034 , 0.46179309, 0.43178293, 0.396426  , 0.3612623 , 0.35587363, 0.3713678 , 0.39579674, 0.42342147, 0.44986357, 0.47570977, 0.50243364, 0.51514265, 0.5060501 , 0.48319416, 0.46069823, 0.4507408 , 0.44143352, 0.43375714, 0.42741102, 0.42255671, 0.41920262, 0.41748425, 0.41755078, 0.41942977, 0.42317313, 0.42805637, 0.43077143, 0.43199448, 0.43185358, 0.42984982, 0.42728559, 0.4239901 , 0.42103804, 0.41444688};
float SS_servo7[] = {-0.27641563, -0.39951245, -0.49708789, -0.58535625, -0.67165028, -0.75697569, -0.8390574 , -0.91553303, -0.97998881, -1.02839037, -1.0936223 , -1.15401376, -1.2070132 , -1.25718513, -1.30875105, -1.36409412, -1.40201275, -1.38369696, -1.32316556, -1.27321885, -1.26533005, -1.29852876, -1.3257376 , -1.34815584, -1.36520157, -1.37675912, -1.3818987 , -1.37912628, -1.36607704, -1.33825213, -1.29466017, -1.24781256, -1.20403543, -1.16320963, -1.12740259, -1.10029184, -1.07634343, -1.04726709, -1.0139901 , -0.97693099};

int node=0;
int first=0;
int input=2;
int end_time=0;
int time_step=80000; // WAS 200 000
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);
int first_print=1;

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
  delay (500);

        servo0_deg=SS_servo0[0]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[0]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[0]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[0]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[0]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[0]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[0]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[0]/3.14159265*180-servo6_deg;
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
  
}

void loop() 
{
  if(Serial.available()>0)
  {
    first=Serial.read();
    if(first=='1')
    {
      input=1;
    }
    if(first=='0')
    {
      input=0;
    }
  }
  if(input==1)
  {
      if(esp_timer_get_time()>=end_time)
      {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
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
        Serial.print(node);
        Serial.print("       SS_servo0:");
        Serial.println(SS_servo3[node]);
        node+=1;
        if(node>=steps)
           {
              input=0;
              node=steps-1;
            }
         end_time=esp_timer_get_time()+time_step;
      }
   }

  if(input == 0)
  {
        servo0_deg=SS_servo0[node]/3.14159265*180;
        servo0=map(servo0_deg,-38,90,2200,1030);
        servo1_deg=SS_servo1[node]/3.14159265*180-servo0_deg;
        servo1=map(servo1_deg,-90,90,2600,950);
        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-37,90,1500,2670);
        servo3_deg=SS_servo3[node]/3.14159265*180-servo2_deg;
        servo3=map(servo3_deg,-90,90,1100,2700);
        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,-90,38,2700,1500);
        servo5_deg=SS_servo5[node]/3.14159265*180-servo4_deg;
        servo5=map(servo5_deg,-90,90,2600,930);
        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-90,38,850,2250);
        servo7_deg=SS_servo7[node]/3.14159265*180-servo6_deg;
        servo7=map(servo7_deg,-90,90,920,2620);
        
        pca9685.setPWM(SER0,0,servo0);
        pca9685.setPWM(SER1,0,servo1);
        pca9685.setPWM(SER2,0,servo2);
        pca9685.setPWM(SER3,0,servo3);
        pca9685.setPWM(SER4,0,servo4);
        pca9685.setPWM(SER5,0,servo5);
        pca9685.setPWM(SER6,0,servo6);
        pca9685.setPWM(SER7,0,servo7);

        if(first_print==1)
        {
        first_print=0;
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
        }
  
  }
}
