//Gait template
// Each input = 1, the driver sees increments the gait to the next step.
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

// SS_walk_2cm_clearance values:
float SS_servo0[] = {-0.35861913, -0.42556418, -0.49327415, -0.49615498, -0.42329659, -0.34596236, -0.25276189, -0.16891282, -0.06905559,  0.0270663 ,  0.09964363,  0.10252765,  0.07568157,  0.0550019 ,  0.04270968,  0.03160335,  0.02102372,  0.0103049 , -0.00151611, -0.01442857, -0.02787037, -0.04483047, -0.06369567, -0.08386644, -0.10488093, -0.12653518, -0.14960809, -0.17380274, -0.20227007, -0.23290129, -0.2626779 , -0.28928285, -0.31084545, -0.33238271, -0.35046746, -0.36406672, -0.3680002 , -0.36262201, -0.35878996, -0.34185264};
float SS_servo1[] = {0.64139388, 0.68287766, 0.68812939, 0.73174579, 0.89058449, 1.00666543, 1.03908472, 1.04855094, 1.07482748, 1.09777949, 1.10854744, 1.10373127, 1.14741465, 1.18122211, 1.20551443, 1.22171518, 1.229764  , 1.23091939, 1.22400031, 1.20752747, 1.18319968, 1.16215918, 1.13694622, 1.11755744, 1.09852266, 1.08198067, 1.06865142, 1.06233828, 1.0507595 , 1.03049549, 1.0154323 , 1.0069657 , 1.00198283, 0.97760724, 0.9442667 , 0.89707788, 0.85311284, 0.79535582, 0.71756269, 0.63897242};
float SS_servo2[] = {-0.06195786, -0.12776768, -0.20264544, -0.25823942, -0.29502451, -0.32205162, -0.3442807 , -0.3615568 , -0.37574569, -0.38646099, -0.39772787, -0.41325339, -0.42962651, -0.44009779, -0.44515599, -0.44845472, -0.45185685, -0.45513135, -0.45990687, -0.46513669, -0.43596005, -0.36169236, -0.29241719, -0.21846016, -0.14602798, -0.0720571 , -0.00149504,  0.06841836,  0.10796004,  0.11739685,  0.12280311,  0.10473412,  0.07012457,  0.0364979 ,  0.00686141, -0.01975857, -0.03481767, -0.0410775 , -0.0519799 , -0.04567433};
float SS_servo3[] = {0.78224473, 0.83521716, 0.85510944, 0.84882481, 0.85206813, 0.85888603, 0.87400012, 0.89025642, 0.91808211, 0.94620374, 0.97383558, 0.99047448, 1.0044554 , 1.01004065, 1.01325984, 1.00923718, 1.0001575 , 0.98604025, 0.96690289, 0.94051438, 0.95883551, 1.05415768, 1.11264369, 1.16613001, 1.19614475, 1.20804896, 1.20495174, 1.20564625, 1.17515141, 1.1074546 , 1.05165745, 1.03292052, 1.05138451, 1.04869695, 1.03341232, 1.00421047, 0.97108402, 0.92143697, 0.8549708 , 0.77894769};
float SS_servo4[] = { 0.36570298,  0.42574758,  0.43654457,  0.42354222,  0.41431264,  0.40420943,  0.39180841,  0.38031135,  0.372401  ,  0.36619577,  0.35443896,  0.33692128,  0.31843423,  0.30326809,  0.29082795,  0.2778308 ,  0.26293212,  0.24711983,  0.2304035 ,  0.21401631,  0.19904587,  0.18080118,  0.15858714,  0.13522057,  0.11011545,  0.08320387,  0.05357207,  0.02158009, -0.02299808, -0.05874631, -0.04924144,  0.00642192,  0.09938695,  0.17468164,  0.2546837 ,  0.31909645,  0.39004857,  0.43156879,  0.38624345,  0.34695259};
float SS_servo5[] = {-0.76926721, -0.88472501, -0.99347189, -1.07499019, -1.12508173, -1.1537258 , -1.15873883, -1.15232453, -1.13043694, -1.10357785, -1.07077319, -1.04936548, -1.02988268, -1.01370641, -0.99564435, -0.98096649, -0.96799161, -0.95849803, -0.95494746, -0.96080315, -0.97610179, -0.98975632, -1.00376582, -1.01365433, -1.0221108 , -1.026533  , -1.02575614, -1.01612128, -0.99670384, -1.00354245, -1.07062984, -1.10206148, -1.07914499, -1.07953527, -1.07495561, -1.046069  , -0.89393363, -0.75791802, -0.75075535, -0.73622853};
float SS_servo6[] = {-0.04079404,  0.01273893,  0.01385226, -0.00829437, -0.02693283, -0.04523097, -0.06161851, -0.07781863, -0.0919355 , -0.10701898, -0.11035699, -0.09227105, -0.06009858, -0.01071981,  0.06551583,  0.14583027,  0.22422391,  0.3028552 ,  0.37841466,  0.45480345,  0.50577197,  0.49942849,  0.48366139,  0.46831439,  0.45149655,  0.4335592 ,  0.4134298 ,  0.39135997,  0.36134984,  0.32612051,  0.29323929,  0.25933741,  0.22769575,  0.19356928,  0.16458276,  0.13195901,  0.09772657,  0.0607845 ,  0.00454215, -0.05353191};
float SS_servo7[] = {-0.92563026, -1.06500807, -1.19121582, -1.28043977, -1.32775975, -1.34561107, -1.33654742, -1.30853093, -1.25875193, -1.20062278, -1.16935365, -1.19076172, -1.23201739, -1.25958947, -1.25085111, -1.21286935, -1.13496918, -1.0255556 , -0.89967557, -0.77527484, -0.71474319, -0.73558999, -0.7597881 , -0.78246722, -0.8047938 , -0.82499322, -0.84204453, -0.85196037, -0.86195554, -0.87554361, -0.8863632 , -0.88435256, -0.87502281, -0.87719359, -0.89102347, -0.90604731, -0.90421846, -0.90552694, -0.90492607, -0.88584107};

int node=0;
int first=0;
int input=0;
int steps[] = {0,6,14,26,34,39};
int next = 0;
int step_count = sizeof(steps)/sizeof(steps[0])-1;


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
  delay (3000);

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

        Serial.println(step_count);
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
        Serial.println("0");
        delay(1000);

        
}

void loop() 
{
  if(Serial.available()>0)
  {
    first=Serial.read();
    if(first=='1')
    {
      input=1;
      next = next+1;
      Serial.print("next int: ");
      Serial.println(next);
      if (next>=step_count)
      {
        next = step_count;
      }
      first=0;
    }
  }
  if(input==1)
  {

        node = steps[next];
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

        input = 0;
      
   }
}
