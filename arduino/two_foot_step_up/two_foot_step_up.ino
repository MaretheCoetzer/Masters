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

float SS_servo0[] = {0.43809059125326477, 0.5347242448472446, 0.6275816082043255, 0.7237486645112898, 0.8273063363308486, 0.8829668071477682, 0.8796646941831716, 0.8584535600809755, 0.8205152846654942, 0.7701204770304665, 0.7338952226742036, 0.6760825219600267, 0.557729004849347, 0.4357865512319069, 0.325115635858324, 0.22278378997990522, 0.12312249397097916, 0.02705251483583742, -0.08124108191387236, -0.19713145769652893, -0.2985264187656244, -0.374528355563241, -0.4479195470349835};
float SS_servo1[] = {0.6094743287096183, 0.7882708688041302, 0.9648998771126043, 1.1501779261588363, 1.3528356027833655, 1.5025141461229197, 1.4970827364886727, 1.4151017242295576, 1.3163947604574662, 1.169630655569422, 1.13513577433638, 1.1333437295099782, 1.1264872381035258, 1.1452115686978126, 1.1596859872709206, 1.1520469207456834, 1.1291406530194665, 1.1150424919227078, 1.0792171603093623, 0.9839242128263129, 0.8624330772367313, 0.727788503691907, 0.602873242538732};
float SS_servo2[] = {0.08483429220105529, -0.030263150401022572, -0.10785471549575044, -0.1810818215259342, -0.25258696297099437, -0.3165637995779213, -0.3629601510924033, -0.39928491006116384, -0.4336133884340132, -0.46905937526491126, -0.5223213940355664, -0.5239858613573188, -0.4522665576073039, -0.3720184895383532, -0.27032644925687616, -0.18472389692157426, -0.13199681186560933, -0.05850999920408238, -0.07089607628938642, -0.1868648688957495, -0.2925473126175545, -0.40969661307597044, -0.5011304620342718};
float SS_servo3[] = {0.6442384304939833, 0.6892556254466854, 0.6978000415538773, 0.7036822222433607, 0.7108730224988672, 0.6923351953238808, 0.6304358456453627, 0.5572617202286514, 0.48219918372862924, 0.39727832664241464, 0.3375915732033557, 0.3893374043679023, 0.5389046387746612, 0.7008562208767495, 0.8965386871690039, 1.0715530779550824, 1.196110724690571, 1.3683858688781614, 1.4191009477264762, 1.2250898672890709, 1.03591826424522, 0.834655450399355, 0.6360330463929894};
float SS_servo4[] = {0.523681377279936, 0.522764915473755, 0.5219830833584557, 0.5189809472092484, 0.5137947965628771, 0.49289860334034574, 0.46729468331635104, 0.4250736439987501, 0.3664773085749946, 0.3129349055351487, 0.26257948713764606, 0.21516285531878293, 0.1665863902517049, 0.11672090440374254, 0.06268631439066931, 0.009449495141722126, -0.042064503263489345, -0.10108726438227796, -0.1774776965520216, -0.28995426107796884, -0.41284967321829485, -0.5404989756163611, -0.6599374749694913};
float SS_servo5[] = {0.3619010599232435, 0.2836317456108356, 0.20800966856309538, 0.13967486427802858, 0.0822351348955497, 0.024336959669974276, -0.05259838775253564, -0.11869423194791993, -0.17030208987133158, -0.23532959524185174, -0.2911154708594988, -0.3438863897370787, -0.4036040645946542, -0.4609344379329103, -0.5119483882732726, -0.5716426293188513, -0.6459450284346976, -0.7048397497083542, -0.7766322634994623, -0.862169898299299, -0.9418440242136862, -1.021043227562779, -1.1069239851985087};
float SS_servo6[] = {0.4995436437561915, 0.4523571208998278, 0.42876897942561865, 0.40027831758185894, 0.3595237913077274, 0.31388473317594223, 0.2691289703008852, 0.2262110893302273, 0.18318479085648595, 0.13768361100715595, 0.09572160835194347, 0.04872173620786225, -0.011948368762353254, -0.07317850695522597, -0.13199851487525296, -0.1877324596348191, -0.2436604878041832, -0.3021582358364221, -0.3646253725731844, -0.4480722281521532, -0.5523984209791404, -0.671747370851127, -0.7807298263116552};
float SS_servo7[] = {0.3878038208810067, 0.35680520447679287, 0.3083553522520003, 0.2679238103886896, 0.2445356542101665, 0.21390957144266048, 0.16298434556697872, 0.10823473252851915, 0.05201354120054563, -0.013488906271746561, -0.07456621159865616, -0.12130660649476667, -0.16402025174474652, -0.20779091671438016, -0.25307537784274337, -0.3085871376810193, -0.3767420726959458, -0.4335399295079434, -0.5184431315826269, -0.6387742893048906, -0.7445403540929146, -0.8438876827771781, -0.948531016710603};

int node=0;
int first=0;
int input=2;
int end_time=0;
int time_step=80000; // WAS 200 000
int first_print=1;

// ------------------------Centers:--------------------------
int centre0=312;
int centre1=293;
int centre2=305;
int centre3=315;
int centre4=315;
int centre5=300;
int centre6=310;
int centre7=315;

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
        if(node>=23)
           {
              input=0;
              node=22;
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
