//Gait template
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50
#define SER0 0 //Hip 1
#define SER1 1 //Knee 1
#define SER2 2 //Hip 2
#define SER3 3 //Knee 2
#define SER4 4 //Hip 3
#define SER5 5 //Knee 3
#define SER6 6 //Hip 4
#define SER7 7 //Knee 4

float SS_servo0[] = {-0.5103085368575471, -0.5236323215084977, -0.45184199300129924, -0.3674883473535613, -0.29342440519521523, -0.24918288583984347, -0.26686876540231336, -0.280725910013489, -0.287817764897003, -0.29191364180676216, -0.29388958330338927, -0.2938199436572926, -0.2911425137746539, -0.28712901995080853, -0.2828171095046325, -0.2776478278740473, -0.2736600539485567, -0.2724640508442871, -0.27262706061362024, -0.27508356098896797, -0.27873239317619974, -0.2856376116682087, -0.2990139185644434, -0.31665941803127795, -0.3350445928532838, -0.3552628516116818, -0.3772536520990224, -0.4000918124141116, -0.42110344146167, -0.43691077463815725, -0.4483001513466326, -0.45583838526131365, -0.4611380176516683, -0.4642147664936251, -0.4641666829800851, -0.46004522393858416, -0.45301273151369026, -0.44282695841691266, -0.4275473101264145};
float SS_servo1[] = {0.31322968012539576, 0.3561939315025595, 0.503950688623998, 0.6725450823694853, 0.7823416343352779, 0.7586295551810793, 0.7551111151217255, 0.7278710937991761, 0.6925203485079814, 0.658968546852279, 0.6274096400122469, 0.5979130063602242, 0.5731770682230363, 0.5515042316540771, 0.5319380902762036, 0.5166474295655255, 0.4977491519333391, 0.46915554986130764, 0.4317227282990964, 0.39750897799935647, 0.3714536594939842, 0.3635310130041038, 0.3732746569089877, 0.3933734151966227, 0.4124870412354197, 0.4251257421649204, 0.431980661729257, 0.4317011590473476, 0.41914668366613744, 0.399511872512392, 0.3792803537667964, 0.3623273666067163, 0.3497399306514964, 0.33930955038970123, 0.3324438778383743, 0.33324744959070735, 0.32999763070682653, 0.31771977219448444, 0.2907934883759799};
float SS_servo2[] = {-0.22848601191518797, -0.2958274553183232, -0.34139628240823167, -0.3874726631799385, -0.4318340090991538, -0.47455430256487385, -0.5061784279598598, -0.51448031430192, -0.5158192373057021, -0.5150576810016607, -0.5127862703055793, -0.5088761347080131, -0.5033745061249355, -0.49711201018197615, -0.4905713666395802, -0.48391825428121693, -0.4785269782082243, -0.4744569703741435, -0.470162857288395, -0.4686093264652045, -0.45007544582173786, -0.39414774860983287, -0.3185271627991677, -0.22781562508249814, -0.13784195837805957, -0.07800277605885164, -0.043322650197973045, -0.032424554651394356, -0.05982058787172645, -0.08716398573200893, -0.10786252436905046, -0.12304284641491274, -0.13483899707574556, -0.14338098195070015, -0.14797687484991975, -0.14712528838169028, -0.14197676625408298, -0.13264841901106256, -0.11852396035933642};
float SS_servo3[] = {0.5040062288240029, 0.5076672001240445, 0.519649321783212, 0.5467678999927169, 0.582191483224382, 0.6170261379659832, 0.6249530205851981, 0.5925728059223778, 0.5529471883740928, 0.5158101080939033, 0.4818892026993276, 0.45047961159496397, 0.4244412840188524, 0.40176572597526644, 0.3810405301192162, 0.36489005602498936, 0.34530457250465457, 0.3151301871532042, 0.2751991064597979, 0.23904994756473796, 0.23885341398061796, 0.3238519074165607, 0.45860093105235716, 0.5828038120612572, 0.6418434659593375, 0.6327275875996216, 0.5901084544398947, 0.55924418858487, 0.5550949810058168, 0.5481173393609908, 0.53730853359609, 0.5275753919406735, 0.5211153088850646, 0.515586611911734, 0.512637379782686, 0.516142028004634, 0.5134039237638194, 0.49942554119325433, 0.4695510750381116};
float SS_servo4[] = {0.497421243174831, 0.5003781976752373, 0.49697734613362565, 0.49182363036793686, 0.47917138293126094, 0.4525884942413673, 0.4190485215529065, 0.3824179273032429, 0.343336549402443, 0.3053689512820889, 0.2746019594738695, 0.24879451299168792, 0.2277811403755049, 0.20943012713724282, 0.19088221784234044, 0.17540656146521205, 0.16481578203505504, 0.15174439295013015, 0.13377138616421946, 0.11677951950311538, 0.10402737317438938, 0.0972079330940064, 0.09438560940725628, 0.09284471315464185, 0.08882531493795989, 0.07876140290503725, 0.06344171578135659, 0.04345349556324787, 0.0165621987250096, -0.01433308593506423, -0.027011558572562624, -0.0052634723878254765, 0.029784274434078887, 0.07792677998971842, 0.13869296081513716, 0.22006236586604813, 0.3093184030390545, 0.39472341453746235, 0.4357154442897383};
float SS_servo5[] = {-0.6333988619103281, -0.7099675379800479, -0.7476334423709443, -0.7682636787459434, -0.7699584539377197, -0.7522420134792178, -0.7394364466634822, -0.7404248874703052, -0.7394233683381782, -0.7352731129774215, -0.7349519754799746, -0.7361637962148636, -0.7350934769547484, -0.7324935591739641, -0.7269464181348527, -0.7195576122763325, -0.7238169187792322, -0.7396810235946951, -0.761658432359247, -0.7849233073418799, -0.8054628104886405, -0.8160319927564473, -0.8181020643631519, -0.8144185106157533, -0.8094771582655811, -0.8063363746189101, -0.8049611674520541, -0.8066005572932057, -0.8125035747491763, -0.8157367730240371, -0.8359511526498518, -0.8859914194768849, -0.9309075318127719, -0.9603462805603256, -0.9277460125663498, -0.7897219864514168, -0.6518021092412778, -0.5494943736507175, -0.49306293166067616};
float SS_servo6[] = {0.29094794248471917, 0.28448695349678516, 0.27507272508288194, 0.26515729741912253, 0.24828485661729008, 0.21741481050685707, 0.17900118510052623, 0.1375995634437775, 0.09342336410977742, 0.06694145052726774, 0.07891292558566797, 0.1126655751226001, 0.17374078445417035, 0.24704074108869317, 0.3134906087384097, 0.3905768912593148, 0.46654229783902973, 0.5035089198805085, 0.4950572342459458, 0.4840581279960575, 0.47631484996599155, 0.47251618912162835, 0.4717061919851869, 0.4718059151349534, 0.4700777988456875, 0.463984402522996, 0.4542209939968247, 0.4412101470136504, 0.42299386674967143, 0.40104284766606474, 0.3789905927008591, 0.3603633728436056, 0.34246841866849725, 0.32589759795413126, 0.30832151292003906, 0.2887442246618403, 0.27255708860685934, 0.26063007995527354, 0.24369865418832834};
float SS_servo7[] = {-0.7733391773053795, -0.8473994542133901, -0.8818524753552228, -0.8983201956740983, -0.8930349102517989, -0.8639877441495776, -0.8391903964097797, -0.8302200297533012, -0.818768989311679, -0.8236580259770945, -0.8674725161548562, -0.915510141201025, -0.9324975828944417, -0.9079052827058358, -0.8255977287388928, -0.6852625385853829, -0.5703816602784789, -0.5276041996566803, -0.5516945504517252, -0.579104728542594, -0.6033545436913678, -0.6165365727500943, -0.6208451331080341, -0.6196884980422019, -0.6183813301139264, -0.6211519511941991, -0.6277518675171246, -0.6390555138204683, -0.6567494048038904, -0.6731615949332862, -0.6859297683859891, -0.6948931741248329, -0.6969679948189836, -0.6952882470781416, -0.6846169472583603, -0.6581205810727807, -0.6369542295075304, -0.6275387834748717, -0.6239281337951469};

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=100000;
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);

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
  servo0=map(servo0_deg,30,-30,265,355);
  pca9685.setPWM(SER0,0,servo0);

  servo1_deg=SS_servo1[0]/3.14159265*180;
  servo1=map(servo1_deg,30,-30,250,335);
  pca9685.setPWM(SER1,0,servo1);

  servo2_deg=SS_servo2[0]/3.14159265*180;
  servo2=map(servo2_deg,-30,30,265,350);
  pca9685.setPWM(SER2,0,servo2);

  servo3_deg=SS_servo3[0]/3.14159265*180;
  servo3=map(servo3_deg,-30,30,265,360);
  pca9685.setPWM(SER3,0,servo3);

  servo4_deg=SS_servo4[0]/3.14159265*180;
  servo4=map(servo4_deg,30,-30,265,360);
  pca9685.setPWM(SER4,0,servo4);

  servo5_deg=SS_servo5[0]/3.14159265*180;
  servo5=map(servo5_deg,30,-30,250,345);
  pca9685.setPWM(SER5,0,servo5);

  servo6_deg=SS_servo6[0]/3.14159265*180;
  servo6=map(servo6_deg,-30,30,260,355);
  pca9685.setPWM(SER6,0,servo6);

  servo7_deg=SS_servo7[0]/3.14159265*180;
  servo7=map(servo7_deg,-30,30,265,365);
  pca9685.setPWM(SER7,0,servo7);
  
  Serial.println("Ready Steady");
  end_time=esp_timer_get_time()+time_step;
  Serial.println("Hip last entry");
  Serial.println(SS_servo6[steps]);
  Serial.println("Knee last entry");
  Serial.println(SS_servo7[steps]);
  
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
        servo0=map(servo0_deg,30,-30,265,355);
        pca9685.setPWM(SER0,0,servo0);

        servo1_deg=SS_servo1[node]/3.14159265*180;
        servo1=map(servo1_deg,30,-30,250,335);
        pca9685.setPWM(SER1,0,servo1);

        servo2_deg=SS_servo2[node]/3.14159265*180;
        servo2=map(servo2_deg,-30,30,265,350);
        pca9685.setPWM(SER2,0,servo2);

        servo3_deg=SS_servo3[node]/3.14159265*180;
        servo3=map(servo3_deg,-30,30,265,360);
        pca9685.setPWM(SER3,0,servo3);

        servo4_deg=SS_servo4[node]/3.14159265*180;
        servo4=map(servo4_deg,30,-30,265,360);
        pca9685.setPWM(SER4,0,servo4);

        servo5_deg=SS_servo5[node]/3.14159265*180;
        servo5=map(servo5_deg,30,-30,250,345);
        pca9685.setPWM(SER5,0,servo5);

        servo6_deg=SS_servo6[node]/3.14159265*180;
        servo6=map(servo6_deg,-30,30,260,355);
        pca9685.setPWM(SER6,0,servo6);

        servo7_deg=SS_servo7[node]/3.14159265*180;
        servo7=map(servo7_deg,-30,30,265,365);
        pca9685.setPWM(SER7,0,servo7);
     
//        Serial.print("Node:");
//        Serial.print(node);
//        Serial.print("       SS_servo0:");
//        Serial.println(SS_servo3[node]);
        node+=1;
        if(node>=steps)
           {
              Serial.print("1 cycle");
              node=0;
            }
         end_time=esp_timer_get_time()+time_step;
      }
   }

  if(input == 0)
  {
   servo0_deg=SS_servo0[node]/3.14159265*180;
   servo0=map(servo0_deg,30,-30,265,355);
   pca9685.setPWM(SER0,0,servo0);

   servo1_deg=SS_servo1[node]/3.14159265*180;
   servo1=map(servo1_deg,30,-30,250,335);
   pca9685.setPWM(SER1,0,servo1);

   servo2_deg=SS_servo2[node]/3.14159265*180;
   servo2=map(servo2_deg,-30,30,265,350);
   pca9685.setPWM(SER2,0,servo2);

   servo3_deg=SS_servo3[node]/3.14159265*180;
   servo3=map(servo3_deg,-30,30,265,360);
   pca9685.setPWM(SER3,0,servo3);

   servo4_deg=SS_servo4[node]/3.14159265*180;
   servo4=map(servo4_deg,30,-30,265,360);
   pca9685.setPWM(SER4,0,servo4);
 
   servo5_deg=SS_servo5[node]/3.14159265*180;
   servo5=map(servo5_deg,30,-30,250,345);
   pca9685.setPWM(SER5,0,servo5);

   servo6_deg=SS_servo6[node]/3.14159265*180;
   servo6=map(servo6_deg,-30,30,260,355);
   pca9685.setPWM(SER6,0,servo6);

   servo7_deg=SS_servo7[node]/3.14159265*180;
   servo7=map(servo7_deg,-30,30,265,365);
   pca9685.setPWM(SER7,0,servo7);
  
  }
}