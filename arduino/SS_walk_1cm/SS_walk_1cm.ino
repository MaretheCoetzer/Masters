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

float SS_servo0[] = {-0.4337159725654013, -0.4915299743023592, -0.4962884855898935, -0.4234259455901433, -0.3342549910436192, -0.2445069517820943, -0.1551066950037357, -0.0639203582820421, 0.019870164354414806, 0.0737896602230666, 0.07317853761747148, 0.040901861373460406, 0.015053876290210195, -0.002567484808821609, -0.013249091138229262, -0.021572623968028535, -0.02871531581628869, -0.03890835351903482, -0.05447955233310575, -0.0727262614503133, -0.08750311072858037, -0.10372092879925217, -0.12105384861514623, -0.1406818980213626, -0.1605648634070189, -0.18292561021547957, -0.20796382988094106, -0.23378954306815894, -0.2591821830176663, -0.28464469189489505, -0.31027950450043257, -0.3320272441420761, -0.34804040801753006, -0.3589888700186532, -0.36551866887167395, -0.36688966899147873, -0.36400899004663007, -0.3560979377500404, -0.3402651227009295};
float SS_servo1[] = {0.5410151330784913, 0.550882775440808, 0.5920171139027041, 0.7404762100108861, 0.8824135379525796, 0.958370591348672, 1.006332045548276, 1.0322104140661519, 1.0461299319828454, 1.0314088590284607, 1.0118922963545085, 1.0429000052546236, 1.0649056356376132, 1.0787088499176953, 1.0949188023175198, 1.1056399481387937, 1.1065236413315935, 1.0998325108664873, 1.0744565978196412, 1.0367975863694687, 1.0147459658303815, 0.9941421012165921, 0.9708186959972284, 0.9526616662069023, 0.9397959717069554, 0.9180246464215416, 0.8888717764642992, 0.8587379050972784, 0.8351959785985437, 0.8174812766895361, 0.7993101317336404, 0.782826468020289, 0.7663721994572814, 0.73819849515719, 0.7080802543185548, 0.6769863758683505, 0.6304829230006189, 0.5604563057856348, 0.48615227780944587};
float SS_servo2[] = {-0.2064940925146868, -0.27143912875566073, -0.31311442225301395, -0.3354687195117963, -0.35073907271746274, -0.36528801895762236, -0.3771566128444948, -0.388011776475958, -0.3986495611849711, -0.4131780348393825, -0.4311832866623852, -0.44950785212629174, -0.4627856839386634, -0.46831599225385234, -0.46851844589941855, -0.4637113836995795, -0.4578416734898801, -0.4600607779276691, -0.465154316115695, -0.4349719980246524, -0.3657725897002907, -0.2921580785729106, -0.22041343170145178, -0.1528926459486733, -0.07539288795360403, -0.017955023160923612, 0.008379597549481888, 0.021905578188738728, 0.03713194696288954, 0.042593071172166784, 0.02178593994872655, -0.016713892880622312, -0.04619472980930381, -0.06950405197594005, -0.08754655595933332, -0.09760328169162533, -0.10559566748704063, -0.11437865864210205, -0.1115118552417738};
float SS_servo3[] = {0.6936438745228365, 0.716987571317055, 0.72057954883968, 0.7327815907599052, 0.7476424617217414, 0.7609803687525366, 0.7739624117736758, 0.7874595188528681, 0.8005480127862219, 0.8094299788117992, 0.8106137588218789, 0.8138681987449374, 0.8118506746919784, 0.8049585435926102, 0.8032158095416173, 0.7945815504740894, 0.779207224768195, 0.7638429782514268, 0.7311759351299114, 0.7228744326652609, 0.8188001208613208, 0.901528397290842, 0.9527867018985895, 1.0083991485990413, 1.0665989181203388, 1.0604381015946567, 1.0149662547685099, 0.9530985024552948, 0.8966561198844955, 0.8492542354545145, 0.8291784129965025, 0.8372645618784549, 0.8391709769468216, 0.8273842304092635, 0.8105196650386292, 0.7860706551662271, 0.7481031172209403, 0.6934676070273624, 0.6289348944352054};
float SS_servo4[] = {0.45344556622791093, 0.4800547803262073, 0.48481288998209354, 0.49017045802760645, 0.4920284590603736, 0.48902629143140824, 0.4819091279891053, 0.4736242837968569, 0.46136586188889434, 0.4400301597235902, 0.41390749074423916, 0.38908578141966665, 0.36895154336422337, 0.3514835408877329, 0.3346682262919553, 0.32067674918808436, 0.30737011474127973, 0.2953714505610177, 0.27882567216146553, 0.2578457189851021, 0.23888339651860338, 0.22193103090476696, 0.2020550702425443, 0.1809306820607609, 0.15778458182834965, 0.12946807641881247, 0.08913933757274259, 0.03953746235961351, 0.00686916842072439, 0.017884042209482972, 0.04581037492817591, 0.10618960285946906, 0.1892837870894394, 0.26112557588389, 0.33216057579822844, 0.4047220977696579, 0.44362254712635946, 0.41538548801427105, 0.36209521341403705};
float SS_servo5[] = {-0.5877806631658713, -0.6693267470720619, -0.721799623253903, -0.7436366949337925, -0.7502887678487078, -0.7518631380835883, -0.7455212436291923, -0.7352232572668727, -0.7197970489691203, -0.7030012540361551, -0.6924565807500648, -0.6821001243764672, -0.6751579777061848, -0.6665519412913431, -0.6479645643952857, -0.6336363234409181, -0.6251932121526803, -0.6270890385517192, -0.643910831568218, -0.6680620912486857, -0.6789648636849527, -0.6934553482476624, -0.7081658792952369, -0.7202349237138304, -0.7255049221487174, -0.7349317841708826, -0.739106182721595, -0.733957528374811, -0.7435990214562014, -0.8020124449943672, -0.8619826969406116, -0.8808596558443601, -0.8492100226350051, -0.8015741274989763, -0.6986321712061278, -0.5454387582716448, -0.4381058406676874, -0.45229245600916484, -0.45071064009484085};
float SS_servo6[] = {0.17903536763407116, 0.1985408723976905, 0.19705805774737012, 0.19345411427546924, 0.18669476370674165, 0.17380023745973516, 0.158134661727159, 0.13613686111318415, 0.10784563825201597, 0.07362153541167835, 0.05138452018727359, 0.06539530585734941, 0.09953501637736202, 0.1517043605488601, 0.22718644950620537, 0.3059854769404895, 0.385114033239463, 0.4611143409003108, 0.5080961471504696, 0.5043378093657179, 0.491923822930573, 0.48425991392866324, 0.478343966016298, 0.47183569375089035, 0.4640962675142108, 0.45485010032122425, 0.43946896785591866, 0.420055038331227, 0.3948251802696209, 0.3712440119136731, 0.3469509264612612, 0.3216377490525779, 0.29648241196084474, 0.2718291319011972, 0.24617543351879975, 0.21901308765816882, 0.1884826835134975, 0.14732301211600354, 0.09652594159783866};
float SS_servo7[] = {-0.7634906703590735, -0.8523307588615984, -0.9063087966790812, -0.920558999017915, -0.9164007115007454, -0.9029934181656656, -0.8812224008182366, -0.8480267473784902, -0.806166801333649, -0.7650928312440037, -0.752955829132407, -0.7795741652461609, -0.8214520013790558, -0.8249104051317854, -0.7297264141928315, -0.5927625968545164, -0.44555104310777527, -0.3103351336486239, -0.2529386086370492, -0.2843822150038454, -0.30073599706133936, -0.32342451254897625, -0.35169643086178565, -0.3786387827859861, -0.40012722981657606, -0.43030502982897695, -0.4643976188749431, -0.49680429223767963, -0.5178947289686899, -0.5367133887643721, -0.5548456961486755, -0.5662931709561959, -0.5716428304391128, -0.5812416601294058, -0.5873365999205206, -0.5887973115326285, -0.5984369273269571, -0.6131511799269506, -0.6118235631341271};

int node=0;
int first=0;
int input=0;
int end_time=0;
int time_step=100000;
int steps = sizeof(SS_servo0)/sizeof(SS_servo0[0]);

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
        
        node+=1;
        if(node>=steps)
           {
              node=0;
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
