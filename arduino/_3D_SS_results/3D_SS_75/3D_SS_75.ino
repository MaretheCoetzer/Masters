// This is a 2.5cm clearance height gait
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
float SS_servo0[] = {-0.66760492, -0.66151045, -0.61033406, -0.55449297, -0.49470425, -0.42577979, -0.34609655, -0.26524493, -0.18391197, -0.10233849, -0.02095062,  0.06006324,  0.04795127,  0.04028433,  0.0382002 ,  0.03787384,  0.01997038, -0.01708617, -0.05510495, -0.0930493 , -0.12710556, -0.1562282 , -0.18263897, -0.21169561, -0.2368099 , -0.25990255, -0.28146374, -0.30186806, -0.32571074, -0.35173785, -0.38493734, -0.41813809, -0.45388158, -0.49342951, -0.52515783, -0.54353827, -0.56442712, -0.58297945, -0.60600801, -0.61592138, -0.62573723, -0.63445888, -0.64204812, -0.64910306};
float SS_servo1[] = {0.9096463 , 0.92099119, 0.94866839, 0.97405074, 0.99487802, 1.006682  , 1.00752277, 1.00647742, 0.99451243, 0.96250268, 0.94550811, 0.89927224, 1.03090422, 1.14901789, 1.24538181, 1.32300031, 1.35280374, 1.32819132, 1.29326304, 1.26208459, 1.22520825, 1.20296835, 1.22596962, 1.2251485 , 1.17366173, 1.11574807, 1.05623421, 1.00190695, 0.98544815, 1.022497  , 1.07555836, 1.11935064, 1.15836205, 1.20854251, 1.25320539, 1.26001284, 1.27133185, 1.27953879, 1.28397668, 1.22052327, 1.14752893, 1.07425838, 1.00206551, 0.93202692};
float SS_servo2[] = {-0.27668297, -0.29052118, -0.30604032, -0.32249485, -0.33647666, -0.36732898, -0.4170814 , -0.4691707 , -0.52063504, -0.56804572, -0.60390158, -0.62582386, -0.63027889, -0.62936573, -0.62613969, -0.62785638, -0.63208865, -0.63608335, -0.63886155, -0.63582817, -0.63496972, -0.63703274, -0.64013773, -0.63871011, -0.58162062, -0.52234158, -0.46328838, -0.40298952, -0.33355874, -0.25327561, -0.17240068, -0.09162637, -0.01107371,  0.06940586,  0.07699539,  0.02954558, -0.00984004, -0.04225668, -0.07836693, -0.11616576, -0.14955555, -0.18511147, -0.22038025, -0.25241078};
float SS_servo3[] = {0.87655951, 0.81204464, 0.735739  , 0.67016947, 0.61283627, 0.61544621, 0.69442259, 0.79178076, 0.89826313, 1.00544807, 1.10861099, 1.20751485, 1.28410649, 1.34383632, 1.3903538 , 1.43219808, 1.42146279, 1.35223968, 1.27239527, 1.1832309 , 1.09751004, 1.0353347 , 1.0110088 , 1.02569488, 1.04579235, 1.0679739 , 1.09137589, 1.1129513 , 1.10268259, 1.04185051, 0.99368763, 0.93237471, 0.85283403, 0.77929028, 0.79809002, 0.87889535, 0.94446777, 0.99588846, 1.03865003, 1.02847947, 0.99325396, 0.96270556, 0.93567345, 0.90700214};
float SS_servo4[] = { 0.66491041,  0.66029783,  0.6372985 ,  0.61109149,  0.58819911,  0.56256296,  0.54145138,  0.52543604,  0.51315262,  0.5052235 ,  0.50064095,  0.49709283,  0.4877689 ,  0.47097681,  0.44689323,  0.42450623,  0.41150245,  0.40986275,  0.40865719,  0.40982696,  0.40493861,  0.39491026,  0.38050003,  0.35625137,  0.31591373,  0.27526508,  0.23096209,  0.18863229,  0.14399535,  0.1036103 ,  0.05928578,  0.00931426, -0.04760034, -0.06740283,  0.00205644,  0.08236296,  0.16345535,  0.24449547,  0.32260369,  0.3896584 ,  0.4507298 ,  0.5107664 ,  0.57734814,  0.64618583};
float SS_servo5[] = {-1.09334144, -1.17067134, -1.23637121, -1.29173196, -1.3437538 , -1.36647235, -1.35621001, -1.33616051, -1.30781443, -1.27648119, -1.23014145, -1.15922361, -1.07179432, -0.97850341, -0.88084321, -0.79865364, -0.7847385 , -0.85174609, -0.9360273 , -1.02744177, -1.10995527, -1.16742692, -1.17505668, -1.18395355, -1.1897398 , -1.20267628, -1.21147686, -1.22359857, -1.19821747, -1.11715833, -1.02634477, -0.93184131, -0.83273991, -0.79140484, -0.85815636, -0.90923147, -0.98555418, -1.0638173 , -1.11672366, -1.10881339, -1.08791094, -1.06498737, -1.04695103, -1.04753998};
float SS_servo6[] = { 0.17591231,  0.15773466,  0.11915842,  0.07832824,  0.03816883, -0.00165751, -0.03078251, -0.05627166, -0.08154856, -0.10900907, -0.10978168, -0.0640496 ,  0.01526229,  0.09695486,  0.17858885,  0.25950675,  0.33197554,  0.39018693,  0.44532957,  0.50387698,  0.57002598,  0.61341587,  0.61291566,  0.6242624 ,  0.63087086,  0.63513513,  0.64167685,  0.6430233 ,  0.63717055,  0.61423877,  0.58882145,  0.5687921 ,  0.54989142,  0.52582741,  0.48221722,  0.42908489,  0.37566833,  0.32546496,  0.280655  ,  0.25443158,  0.24096304,  0.22536191,  0.20923186,  0.19149696};
float SS_servo7[] = {-0.99422649, -1.06591849, -1.11371027, -1.15272523, -1.18230962, -1.17654557, -1.1342441 , -1.07327287, -0.99407518, -0.89991614, -0.8370534 , -0.83236799, -0.85312662, -0.85482182, -0.85627815, -0.85633359, -0.84735126, -0.82468631, -0.79419697, -0.76585249, -0.74781932, -0.72224648, -0.69430306, -0.7214571 , -0.79826761, -0.88502882, -0.97905743, -1.06294121, -1.11236207, -1.08779366, -1.05811848, -1.04613348, -1.04465163, -1.03133448, -0.96624596, -0.8843611 , -0.7996886 , -0.71727183, -0.65714083, -0.67851051, -0.74247709, -0.80825395, -0.87391376, -0.9365832 };

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
