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

float SS_servo0[] = {-0.15391849240503686, -0.045132645555257755, 0.03702369681611945, 0.12931700240462193, 0.19026695713848632, 0.21488014125112936, 0.23765936979958788, 0.2632235584200785, 0.28391392429534906, 0.2752826970188209, 0.24250352314698725, 0.19940151581120885, 0.14762121682979848, 0.08948163424387923, 0.012569346147484912, -0.07885530905725507, -0.16136251340684188, -0.23147373104175387, -0.2975609955438407, -0.3379941443262138, -0.36352864998747464, -0.3881830943931548, -0.4155472788391793, -0.4408502220177355, -0.4648298978003256, -0.48366632549355965, -0.49840471594907554, -0.5120681657285459, -0.5208913302555997, -0.5231763271575942, -0.5236607118656781, -0.5235467308760706, -0.5236433116552721, -0.5235262233399236, -0.5235030814173157, -0.5232440605446257, -0.5230233500515414, -0.5193937843912289, -0.524924246419523};
float SS_servo1[] = {0.863295048956339, 1.055246575466914, 1.2125675669235236, 1.3974477796187235, 1.500832865727326, 1.4608458034062033, 1.4465033610330278, 1.4363517466235254, 1.4324237664979502, 1.4188825612661167, 1.4004822277863536, 1.4219572289888014, 1.4322496979002477, 1.4048711698118437, 1.3506283299842814, 1.2824740822289635, 1.197816012940949, 1.1205623626446828, 1.0587301917518541, 0.9591135309236155, 0.8606833827622596, 0.7677143306633546, 0.682497452283365, 0.601176856894881, 0.5158887748447007, 0.43825432929499286, 0.3653137785890076, 0.2929775673402702, 0.2197791632140874, 0.15152001255477066, 0.10223804994544186, 0.03844609971114194, -0.02827912671415663, -0.09116000797159778, -0.1520747912834603, -0.21382379294599677, -0.2763637377920871, -0.34040960871021275, -0.3902711132204718};
float SS_servo2[] = {-0.04044659384374027, -0.0437926761901098, -0.04552055689240352, -0.05061997374748539, -0.0628653656886584, -0.08153707213548211, -0.10760624043422255, -0.1428332732728994, -0.17483925583835505, -0.19787997258981074, -0.23106602426141942, -0.25533137455938415, -0.26806831126023195, -0.3022674990035038, -0.3611782696897654, -0.39454020956369207, -0.4303911493249928, -0.45707016805293943, -0.4527987486787845, -0.453233604399591, -0.44648892041477584, -0.4460077948808635, -0.45479028984122655, -0.46894293343985544, -0.5074345128335144, -0.5252367159141393, -0.5126956676370761, -0.4915506153184831, -0.4669856596881972, -0.44016087196706116, -0.42089536782508075, -0.41833183573922916, -0.42896252810543917, -0.44872470799197617, -0.4717692132367776, -0.4911138322891486, -0.5073111926891718, -0.5195221851683438, -0.5241437515472537};
float SS_servo3[] = {0.9634586906770934, 0.9707367071036871, 0.959033554060201, 0.9599694453429605, 0.9571427906676179, 0.9416708321466449, 0.9301259236316413, 0.9220667164011228, 0.8966149772867819, 0.8591665121784524, 0.8598210621549296, 0.8510604403343728, 0.8119621769172429, 0.7639038038701302, 0.7001034943321017, 0.5921703131767657, 0.49012701884324333, 0.3935655349118707, 0.2793233383041302, 0.22457187129064798, 0.3064305752366443, 0.38955136845430627, 0.46672296061153096, 0.5444028558371433, 0.5162651502424763, 0.4249517610571912, 0.35293287887103936, 0.29099725388004233, 0.2315136927614774, 0.18340401855030747, 0.12088711697431814, 0.0424486222246232, -0.027505273150772204, -0.08558728042888548, -0.13894179720726402, -0.19638951170888036, -0.2574927600127284, -0.3204428048098183, -0.3839303821703297};
float SS_servo4[] = {0.49208106455721434, 0.48782528868134467, 0.5016073999833992, 0.5042178042239271, 0.518854109287286, 0.5216920356725789, 0.513795964875402, 0.5119358939073054, 0.521489126129591, 0.5211235745236058, 0.5009382464527665, 0.4887807874139114, 0.48149787231911534, 0.4731718592612405, 0.4632045337728958, 0.4449520881708277, 0.4250010511123611, 0.4081295081953049, 0.38646598939912763, 0.34736852543089336, 0.31104160324932006, 0.26308169939482834, 0.2136291302175001, 0.16199394931770808, 0.09744845662740974, 0.0161252997849941, -0.06435828914320456, -0.15600813398913324, -0.25005029096637765, -0.3062448883730086, -0.3102698461261339, -0.31975836160459437, -0.315505718506729, -0.32255645782477715, -0.33131233883571914, -0.3350568099317012, -0.33987879643888813, -0.34997691776745304, -0.3519906764922245};
float SS_servo5[] = {0.38394605787800634, 0.3949326073747182, 0.37386606451004345, 0.3678438880341703, 0.3450581045947467, 0.31508969366252065, 0.2882471755055162, 0.2511802971371692, 0.1988789930245533, 0.1559225622452568, 0.1375347437668606, 0.11204788682624285, 0.06830997214759943, 0.006496595318455716, -0.08332358138533197, -0.18805522995090018, -0.29464885444844857, -0.39729024287046993, -0.4922342027632997, -0.5710454025852852, -0.6437866198153768, -0.7009568732998079, -0.7590408914768926, -0.8177771521450998, -0.8651140556544648, -0.8764026862739873, -0.8742430510245227, -0.859598670456939, -0.8398646146544351, -0.830423101019095, -0.7526324498736002, -0.7397835538063168, -0.7738544206089397, -0.7346737988398976, -0.6727446669052247, -0.6271252073679494, -0.5858639322239377, -0.527150774956313, -0.45083845473850015};
float SS_servo6[] = {0.5295892898986133, 0.5049564036516806, 0.4702022324109373, 0.4472162355845861, 0.4088544375162985, 0.35936995284659035, 0.31073408546058107, 0.2660471018707348, 0.21489635054950848, 0.19910072254101202, 0.2648792499164172, 0.3595965488094393, 0.4432623250868892, 0.5135991153975104, 0.5277228910791651, 0.4859680995585866, 0.45485050585309195, 0.44936057231384113, 0.4875311856292607, 0.520839902364116, 0.5237117765555612, 0.5215610491237087, 0.5186073614585627, 0.5158648949287783, 0.5022771340079853, 0.47416439911295777, 0.42653431839270506, 0.38450579342243274, 0.3574832327296353, 0.29860966271091627, 0.2323119267521693, 0.1576747112798147, 0.08196189707740134, 0.009188812655194806, -0.06219056925874769, -0.13404309524176197, -0.20671085838974645, -0.2790556893388766, -0.3395657448065944};
float SS_servo7[] = {-0.5101856785712926, -0.4820538398121184, -0.4570558844272109, -0.43875292835136537, -0.4201521397235931, -0.40266292959293465, -0.38860566528805335, -0.3845194936579147, -0.3901769959883607, -0.37987789726160437, -0.24855699899441325, -0.07134288456372274, 0.09205076296185302, 0.2575200565178319, 0.37556863586097833, 0.36476224965404425, 0.3285239335473961, 0.35091731191839387, 0.3623152638471535, 0.24646482629358576, 0.15415471577660395, 0.06588805858224683, -0.023001275399986542, -0.11066172675435387, -0.19770387866185005, -0.2635368361555725, -0.3072987235676565, -0.3563925056996974, -0.4159613486342296, -0.43680031785145845, -0.4288519663377927, -0.4287521688073132, -0.43176723679885587, -0.4324220385466436, -0.431834878797394, -0.4315373461617717, -0.43134680424058147, -0.42947120782286274, -0.4319539129635038};

int node=0;
int first=0;
int input=2;
int end_time=0;
int time_step=80000; // WAS 200 000
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);
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
