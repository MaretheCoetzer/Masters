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
float SS_servo0[] = {-0.51457378, -0.49854418, -0.44192018, -0.36845484, -0.29258197, -0.20612629, -0.10900451, -0.02709655,  0.05463138,  0.13423874,  0.19029395,  0.18333228,  0.122652  ,  0.06496453,  0.02236096, -0.01455472, -0.04632643, -0.07551721, -0.10363071, -0.13210015, -0.15871125, -0.18048053, -0.20472212, -0.23232034, -0.26358878, -0.29635253, -0.32912794, -0.36132703, -0.39283481, -0.42026594, -0.44617889, -0.46914427, -0.48588782, -0.4958166 , -0.50097896, -0.5040434 , -0.50763911, -0.51293836, -0.51580133, -0.51473942};
float SS_servo1[] = {0.5517031 , 0.55843172, 0.62543599, 0.70267476, 0.78424651, 0.82451041, 0.77526359, 0.70704798, 0.63296305, 0.56683919, 0.50575903, 0.48521953, 0.5595218 , 0.62944285, 0.67706731, 0.7145163 , 0.73922082, 0.75274399, 0.7558047 , 0.75016613, 0.73552492, 0.71677404, 0.71557651, 0.72906226, 0.75380834, 0.78262389, 0.80848237, 0.82666448, 0.83371728, 0.83236415, 0.83643848, 0.83594368, 0.82576764, 0.79980868, 0.76409395, 0.72606977, 0.69110007, 0.65843904, 0.6199171 , 0.56802979};
float SS_servo2[] = {-0.2195416 , -0.23539036, -0.25260292, -0.26752882, -0.28681789, -0.30980524, -0.3343579 , -0.36199397, -0.39403359, -0.4277471 , -0.45994637, -0.48693162, -0.50814818, -0.51636557, -0.51838172, -0.51402552, -0.5061653 , -0.49408003, -0.48401296, -0.48432239, -0.47829023, -0.43178354, -0.36104184, -0.28508088, -0.20731868, -0.12256868, -0.03139256,  0.04077579,  0.07703069,  0.09156402,  0.106443  ,  0.09311   ,  0.04085989, -0.01025693, -0.0510546 , -0.08656578, -0.11903283, -0.1498411 , -0.17912541, -0.20888278};
float SS_servo3[] = {0.67210337, 0.66137557, 0.64156107, 0.62344452, 0.61929354, 0.6262321 , 0.63628737, 0.65109779, 0.67619802, 0.71119236, 0.74783692, 0.76130411, 0.75201566, 0.72090578, 0.68717452, 0.64355043, 0.59251006, 0.52924412, 0.46325446, 0.40922532, 0.40389446, 0.46661493, 0.54260571, 0.61815141, 0.69127458, 0.73354591, 0.71022113, 0.65766082, 0.59357531, 0.51871118, 0.45119176, 0.43686173, 0.50129889, 0.55997083, 0.59612423, 0.62188706, 0.64186908, 0.65647578, 0.66595187, 0.67170246};
float SS_servo4[] = { 5.04420743e-01,  5.09890271e-01,  5.14683494e-01,  5.13915423e-01,  5.13571703e-01,  5.11545868e-01,  5.08005024e-01,  5.02802504e-01,  4.93623082e-01,  4.81601110e-01,  4.63079930e-01,  4.36633462e-01,  4.05907874e-01,  3.76390320e-01,  3.45678391e-01,  3.17278664e-01,  2.93312099e-01,  2.74175085e-01,  2.57874415e-01,  2.40030289e-01,  2.15832665e-01,  1.88801800e-01,  1.63722733e-01,  1.38024392e-01,  1.10062982e-01,  7.92383882e-02,  4.39155639e-02, -3.61405755e-04, -6.20653002e-02, -1.01079572e-01, -8.46910687e-02, -3.74847279e-02,  4.03112782e-02,  1.19619542e-01,  1.92556920e-01,  2.60957182e-01,  3.35309507e-01,  4.12578440e-01,  4.81091004e-01,  5.04217032e-01};
float SS_servo5[] = {-0.48332613, -0.53451299, -0.59303915, -0.63533079, -0.67430002, -0.70700806, -0.73761915, -0.76725981, -0.78698805, -0.79466341, -0.78599899, -0.77345794, -0.76428361, -0.75487701, -0.73404049, -0.71549737, -0.70667699, -0.71217186, -0.73057029, -0.7555821 , -0.77501641, -0.78475095, -0.78739994, -0.78030374, -0.76374094, -0.73923379, -0.70721878, -0.66174386, -0.58828386, -0.56353279, -0.64298058, -0.735449  , -0.80828349, -0.86493582, -0.88139239, -0.83909698, -0.76578997, -0.68785982, -0.59539841, -0.48106655};
float SS_servo6[] = { 0.21128223,  0.19394642,  0.16956909,  0.13564798,  0.10351187,  0.06924313,  0.03317974, -0.00894109, -0.05896938, -0.08471697, -0.06398782, -0.03893816, -0.0136385 ,  0.03108196,  0.09616929,  0.16542179,  0.24133829,  0.31739948,  0.38948595,  0.44213728,  0.4604154 ,  0.46600702,  0.4741943 ,  0.48715509,  0.49757748,  0.50304086,  0.50532775,  0.50606335,  0.50045603,  0.48507777,  0.46191828,  0.43157833,  0.39983008,  0.36916907,  0.3407549 ,  0.31236668,  0.28794547,  0.26738931,  0.24770123,  0.22112877};
float SS_servo7[] = {-0.65874062, -0.66521829, -0.6709714 , -0.65123056, -0.63004871, -0.60044622, -0.56664591, -0.52098389, -0.45596577, -0.43214244, -0.49388945, -0.56998076, -0.65703964, -0.72877615, -0.72545607, -0.66843703, -0.59418978, -0.51706647, -0.43764939, -0.33778545, -0.27892943, -0.3376784 , -0.39857462, -0.46181638, -0.51665248, -0.56094373, -0.60168953, -0.64577734, -0.68796384, -0.71361886, -0.71531037, -0.70018373, -0.68124969, -0.66648215, -0.65616206, -0.64465562, -0.64010801, -0.64516396, -0.65463363, -0.65724541};

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
