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

float SS_servo0[] = {0.0132436711512529, -0.006283591824065026, 0.05774423424508032, 0.13072439324654456, 0.1584623106100793, 0.14055711040452723, 0.16263085697618118, 0.1800696666722373, 0.20869720421367963, 0.22174508312494445, 0.18052058309488167, 0.1345149017764196, 0.10214563530569852, 0.07382339765284435, 0.038759513443251944, -0.006104752831205856, -0.05453855815041706, -0.10552199440166174, -0.15683639302801494, -0.20567159717318534, -0.2396146271465254, -0.2679307076989511, -0.2990228490912862, -0.32595915733957587, -0.35072750833324107, -0.3754717548978256, -0.40003486100729746, -0.42448605294689934, -0.4483184125306589, -0.4646518372152781, -0.4766286576480962, -0.4894957346049762, -0.4978108499968027, -0.5071991890199561, -0.5165019581101552, -0.5229188113812099, -0.5237095075197417, -0.5229735353593326, -0.5219270935449807, -0.5193450969935659, -0.5156764369919946, -0.5125823912641474, -0.50971828057014, -0.5205672257849302};
float SS_servo1[] = {1.0890292292399724, 1.1253983581721627, 1.2654796736530332, 1.4330990182516623, 1.521809707345287, 1.4258861163102754, 1.3696369610359485, 1.303968851924315, 1.2513062593827688, 1.2094292377524198, 1.1817453996344789, 1.1830341605918535, 1.193766855021043, 1.2051904827169715, 1.2066760101501193, 1.1945908372827398, 1.179248051039685, 1.1642250547264168, 1.148815021490885, 1.1424462601017242, 1.1370935926570738, 1.1160417747407203, 1.0564256729917312, 0.9674537073012998, 0.8831062316903688, 0.7988481996477129, 0.7180225656682604, 0.63931757326315, 0.569193457151816, 0.5045582068635251, 0.438052290950276, 0.3696376956840613, 0.29247985339982585, 0.22359494566645943, 0.1683506853091808, 0.11176664247640038, 0.03906981596117837, -0.031844488194929646, -0.10168347198652597, -0.17007241870587075, -0.23528032937064083, -0.2987922705232575, -0.3633473905426971, -0.41825845685242924};
float SS_servo2[] = {-0.00218219930014338, -0.04917987936232512, -0.07960855722462847, -0.10793191096828962, -0.13679314729203468, -0.1535494056712138, -0.1702783393578584, -0.18433111422479814, -0.19973701689517168, -0.22315123273720738, -0.2462428947408221, -0.26539163470872346, -0.2821821967818595, -0.294753728031642, -0.31559741503580696, -0.33848940872834254, -0.36629612495758357, -0.3987899635453289, -0.43327108652242885, -0.4646550723531343, -0.4703033538883719, -0.4394559020074162, -0.4089894942923348, -0.39247994043971846, -0.40184042949966925, -0.4115994476309325, -0.44116569967508723, -0.4829694728152643, -0.5212365425858757, -0.5246060208964143, -0.5079397546086702, -0.4909343050887743, -0.4734454859002773, -0.4560833695458075, -0.439195360407969, -0.4432230021755264, -0.46595095586197804, -0.4783628199339608, -0.49285829780053936, -0.5016699086504898, -0.5087187614188188, -0.5156278469179603, -0.5225244033576991, -0.5243388657468699};
float SS_servo3[] = {1.0556829445562805, 1.016646660592805, 0.9564404368546165, 0.9068016443004879, 0.8493626582939486, 0.7652298813814215, 0.7003019912970962, 0.6418960732697149, 0.5951362380074016, 0.5613588663389074, 0.5214547923996945, 0.4916155728939987, 0.4781280535580666, 0.46654774824337913, 0.4479541212335077, 0.4138139172280513, 0.378934148606657, 0.3457799407873928, 0.31373448675829974, 0.2848842808113606, 0.25028105217712904, 0.183523753355749, 0.15239687742664265, 0.24581779460825692, 0.33283241005495445, 0.4195993555012576, 0.49020650701721136, 0.5508296385534085, 0.5324231681008824, 0.4441543262030325, 0.37904863222266494, 0.3134371181909826, 0.24906728015919782, 0.18495405786060987, 0.11744422423228765, 0.0564150591230324, 0.0011981819203668274, -0.059119232426786184, -0.11567811222176878, -0.1748086879672061, -0.23281612360276693, -0.2910537203619704, -0.3504549886296162, -0.41489397676669487};
float SS_servo4[] = {0.5240315381044919, 0.5195235886167432, 0.5214707043604205, 0.521661161064047, 0.5011329969787048, 0.47741459081579957, 0.4259197564273581, 0.3905170888090663, 0.342128980379005, 0.30426291939660133, 0.2896624299413186, 0.28165181071812606, 0.29044072991391734, 0.3157799296592592, 0.33979481755368995, 0.354723968338858, 0.3666717528515228, 0.3762573531676785, 0.3854674584412211, 0.3952523845696621, 0.40373395391687306, 0.3993518935265339, 0.3690952899255934, 0.3230292960210216, 0.27334962526074985, 0.22362471220461874, 0.17176650755214978, 0.11859028943909561, 0.05351749753188886, -0.022559805116073032, -0.09849266130729749, -0.17686027790644315, -0.2608609638856565, -0.319016567331442, -0.34146904547738616, -0.34565563854885906, -0.35819997506258777, -0.36092991869494356, -0.35806592604188525, -0.3534579142568874, -0.3551335662501295, -0.36352536781054273, -0.3818007663279158, -0.41059494916526373};
float SS_servo5[] = {0.396292498378225, 0.32991678025760757, 0.26342061812170786, 0.20780515443400463, 0.16094405268410805, 0.10402637189506607, 0.09055127935768198, 0.06998059386109447, 0.06970613959687298, 0.06040238731386811, 0.01951277435145691, -0.016919536722234754, -0.051888297468808575, -0.09897027476568725, -0.1569373268873167, -0.22360810623184157, -0.29390465761826856, -0.3659103862209894, -0.43995403462608446, -0.510133948609821, -0.5623170429072486, -0.6043428867480275, -0.6452047540629289, -0.689646410946636, -0.7344557045645551, -0.7792702283726318, -0.8248588658325231, -0.8709257746243156, -0.8971497316302537, -0.8907091966300901, -0.8830400168750385, -0.8788004872073776, -0.8777995297053682, -0.8654729197633628, -0.8237944476957233, -0.800532730996659, -0.8164372678469918, -0.808472254997465, -0.8049928777166908, -0.7859325294675283, -0.7218791287643228, -0.6523857164582856, -0.602928389495782, -0.584188689718434};
float SS_servo6[] = {0.5250859306833123, 0.5091565108371162, 0.49743140612598274, 0.4833596672306044, 0.4558608404237463, 0.43399440221927266, 0.383827223307459, 0.32996212803450337, 0.2675253386460054, 0.20786361941855291, 0.17822243484480416, 0.20874191200909448, 0.27581873467584217, 0.36425782417600755, 0.44037147453889536, 0.47278300133703016, 0.4842333450307153, 0.4805263161305415, 0.4841685240714582, 0.502938635807285, 0.521915730381708, 0.5197654354355747, 0.5044218168460937, 0.5026507835858071, 0.4984052368776133, 0.4941065666418338, 0.4889040416896591, 0.48314308736727213, 0.46381149471408123, 0.42192039271629905, 0.3799662749058622, 0.34927347746461906, 0.31900102019520343, 0.2761709922275039, 0.2208415945873446, 0.15781283707863958, 0.09764849529206415, 0.038591262026397836, -0.02157920726232971, -0.08237160969735882, -0.14792997178589426, -0.21733219872491072, -0.2886614580551886, -0.3604357492559508};
float SS_servo7[] = {0.28310978852791485, 0.22851734995599499, 0.16835819211132272, 0.11571750241973233, 0.06942386946111682, 0.011312714572120144, -0.00488407476796341, -0.010727333498312012, -0.0003707946531761356, 0.009155408436830131, -0.02246460663681044, -0.03326255126291863, 0.09938508506127429, 0.25495847103785835, 0.3432756425928965, 0.3588850780436869, 0.36688196625732306, 0.37551714791592405, 0.34947027978053313, 0.2908065974974694, 0.23961567523375812, 0.20158901259654338, 0.15093986183333247, 0.07103826233503999, -0.008526927900274303, -0.08806426082392081, -0.16896933345609785, -0.2507194379392955, -0.3167894335468052, -0.35368672473066914, -0.3906463577350816, -0.44334881097658446, -0.5040239536445423, -0.5483484399042267, -0.5649229352633275, -0.5706624837897939, -0.5914614951887207, -0.608599553247289, -0.6224348329424062, -0.632037092098487, -0.6318819905137611, -0.6268390017070513, -0.621089320117982, -0.6157929883400741};

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
