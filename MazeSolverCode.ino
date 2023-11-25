#include <LinkedList.h>

LinkedList<char> Path; 
LinkedList<char> Shortest_Path;

#define R_ENCODER_A 3 /* Right encoder A */
#define L_ENCODER_A 2 /* Left encoder A */

/********************** does not used *************************/
#define L_ENCODER_B 13 /* Left encoder B */

#define R_ENCODER_B 11 /* Right encoder B */
/**************************************************************/


#define R_speed_PWM 11 /* Right motor speed connected to PWM pin */
#define L_speed_PWM 6 /* Left motor speed connected to PWM pin */

#define R_forward 8 /* Right motor forwarding pin */
#define R_backward 4 /* Right motor backwarding pin */
#define L_forward 9 /* Left motor forwarding pin */
#define L_backward 7 /* Left motor backwarding pin */



int distance_right = 100; /* Distance right for calibration */
int distance_left = 100; /* distance left for calibration */
int distance_front=100;

/* ------------ Ultrasonic sensors ------------------*/
/* these ulta=ra sonic sensors are used basically for calibration of the robot while moving in he maze */
const int trig_right = A3;
const int echo_right = A2;

const int trig_left = A1;
const int echo_left = A0;

const int trig_front = A4;
const int echo_front= A5;


/* ---------------- IR Sensors ----------------------*/
/* thses sensors are used for determining the existance of walls in the right left or front, we choose IR due it its high speed in responsing */

const int super_left = 13;
const int super_right = 12;


bool is_solved = false;

int prev_L = 0;
int prev_R = 0;

volatile int counter_L = 0;
volatile int counter_R = 0;

int counter_prev_L = 0;
int counter_prev_R = 0;

unsigned int direction_L = 0;
unsigned int direction_R = 0;

/* ------- PID controller configuration ---------*/
double RPM_L = 0.0;
double RPM_R = 0.0;

double set_RPM_L = 0.0;
double set_RPM_R = 0.0;
double PWM_L = 0.0;
double PWM_R = 0.0;

int timer_counter;
double Kp = 1; /* proportional constant gain */
double Ki =0; /* integral constant gain */
double Kd = 0; /* differential constant gain */

int index ;
void STOP() {
    digitalWrite(R_forward , LOW);
    digitalWrite(L_forward , LOW);
    digitalWrite(R_backward , LOW);
    digitalWrite(L_backward , LOW);
    set_RPM_L = 0;
    set_RPM_R = 0;
    digitalWrite(R_speed_PWM , RPM_R);
    digitalWrite(L_speed_PWM , RPM_L);
}

void FOWRARD() {
    digitalWrite(R_forward , HIGH);
    digitalWrite(L_forward , HIGH);
    digitalWrite(R_backward , LOW);
    digitalWrite(L_backward , LOW);
    set_RPM_L = 160;
    set_RPM_R = 160;// 0 255 0=low. 255=high

    analogWrite(R_speed_PWM , set_RPM_R);
    analogWrite(L_speed_PWM , set_RPM_L);
}

void ISR_countR() {
    counter_R++;

}
void ISR_countL() {
    counter_L++;

}

int Distance(int trig , int echo) {
    int distance, distanceCm , duration;
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    duration = pulseIn(echo, HIGH);
    distanceCm = duration * 0.034 / 2;
    //Serial.println(distanceCm);
    return distanceCm;
}
void LEFT(){
    // digitalWrite(R_speed_PWM , 100);
    // digitalWrite(L_speed_PWM , 100);
    digitalWrite(R_forward , HIGH);
    digitalWrite(L_forward , LOW);
    digitalWrite(R_backward , LOW);
    digitalWrite(L_backward , LOW);
    //set_RPM_L = 90;
    set_RPM_R = 110;

    analogWrite(R_speed_PWM , set_RPM_R);
    //nalogWrite(L_speed_PWM , set_RPM_L);

    // digitalWrite(R_speed_PWM , RPM_R);
    // digitalWrite(L_speed_PWM , RPM_L);
}

void RIGHT(){
    // digitalWrite(R_speed_PWM , 100);
    // digitalWrite(L_speed_PWM , 100);
    digitalWrite(R_forward , LOW);
    digitalWrite(L_forward , HIGH);
    digitalWrite(R_backward , LOW);
    digitalWrite(L_backward , LOW);
    set_RPM_L = 90;
    //set_RPM_R = 90;

    //analogWrite(R_speed_PWM , set_RPM_R);
    analogWrite(L_speed_PWM , set_RPM_L);

    // digitalWrite(R_speed_PWM , RPM_R);
    // digitalWrite(L_speed_PWM , RPM_L);
}
void RIGHT_CAL() {
    digitalWrite(R_forward , HIGH);
    digitalWrite(L_forward , HIGH);
    digitalWrite(R_backward , LOW);
    digitalWrite(L_backward , LOW);
    set_RPM_L = 100;
    set_RPM_R = 60;

    analogWrite(R_speed_PWM , set_RPM_R);
    analogWrite(L_speed_PWM , set_RPM_L);
}
void LEFT_CAL() {
    digitalWrite(R_forward , HIGH);
    digitalWrite(L_forward , HIGH);
    digitalWrite(R_backward , LOW);
    digitalWrite(L_backward , LOW);
    set_RPM_L = 60;
    set_RPM_R = 100;

    analogWrite(R_speed_PWM , set_RPM_R);
    analogWrite(L_speed_PWM , set_RPM_L);
}
void setup() {
    Path = LinkedList<char>();
    Shortest_Path = LinkedList<char>();
    pinMode(trig_front, INPUT);

    pinMode(trig_right, OUTPUT);
    pinMode(echo_right, INPUT);

    pinMode(trig_left, OUTPUT);
    pinMode(echo_left, INPUT);

    pinMode(R_forward , OUTPUT);
    pinMode(R_backward , OUTPUT);
    pinMode(L_forward , OUTPUT);
    pinMode(L_backward , OUTPUT);
    pinMode(R_speed_PWM , OUTPUT);
    pinMode(L_speed_PWM, OUTPUT);

    pinMode(super_left , INPUT);
    pinMode(super_right, INPUT);

    interrupts();
    attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), ISR_countR,RISING); // Increase counter 1 when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), ISR_countL,RISING); // Increase counter 2 when speed sensor pin goes High
    // put your setup code here, to run once:

    Serial.begin(9600);
}
void CENTER() {
    //FOWRARD();
    distance_right = Distance(trig_right , echo_right);
    distance_left = Distance(trig_left , echo_left);

    while (distance_right < 8) {
        LEFT_CAL();
        distance_right = Distance(trig_right , echo_right);
    }
    while (distance_left < 8) {
        RIGHT_CAL();
        distance_left  = Distance(trig_left  , echo_left);
    }
    FOWRARD();
}

void DistanceAltraFront()
{
   distance_front = Distance(trig_front , echo_front);
  return distance_front;
}
void ROTATE() {
    if (digitalRead(super_left) == 0 && digitalRead(super_right) == 0 && !digitalRead(trig_front)) {
        if (distance_right > distance_left) {
        while ( digitalRead(trig_front)) {
        delay(50);
        }
        } else {
          while (digitalRead(trig_front)) {
          delay(50);
          }
        }
    }
}

bool SOLVED(){
    if(Path.size() > 3){
        if(Path.get(Path.size() - 1) == 'R' && Path.get(Path.size() - 2) == 'R' && Path.get(Path.size() - 3) == 'R'){
            is_solved = true;
            return true;
        }
        if(Path.get(Path.size() - 1) == 'R' && Path.get(Path.size() - 2) == 'R' && Path.get(Path.size() - 3) == 'R' && Path.get(Path.size() - 3) == 'L'){
            is_solved = true;
            return true;
        }

        return false;
    }

    return false;
}

/* in this function we try to make an optemization for our code
and we neglect the dead ends paths to achieve a shortest path to the goal ....
*/
void check_for_optimization(){
    if(Path.size() >= 3){
        index = Path.size();
      //  / LBR -----> B /
        if(Path.get(index - 3) == 'L' && Path.get(index - 2) == 'B' && Path.get(index - 1) == 'R'){
            Shortest_Path.add('B');
        }
      //  / LBS -----> R /
        else if(Path.get(index - 3) == 'L' && Path.get(index - 2) == 'B' && Path.get(index - 1) == 'S'){
            Shortest_Path.add('R');
        }
       // / LBL -----> S /
        else if(Path.get(index - 3) == 'L' && Path.get(index - 2) == 'B' && Path.get(index - 1) == 'L'){
            Shortest_Path.add('S');
        }
      //  / SBL -----> R /
        else if(Path.get(index - 3) == 'S' && Path.get(index - 2) == 'B' && Path.get(index - 1) == 'L'){
           Shortest_Path.add('R');
        }
      //  / SBS -----> B /
        else if(Path.get(index - 3) == 'S' && Path.get(index - 2) == 'B' && Path.get(index - 1) == 'S'){
           Shortest_Path.add('B');
        }
       // / RBL ------> B/
        else if(Path.get(index - 3) == 'R' && Path.get(index - 2) == 'B' && Path.get(index - 1) == 'L'){
            Shortest_Path.add('B');
        }else{
         //   / no optemization can make */
            Shortest_Path.add(Path.get(index - 3));
            Shortest_Path.add(Path.get(index - 2));
            Shortest_Path.add(Path.get(index - 1));
        }
    }else{
        for(int i = Path.size() - 1 ; i >= 0 ; i--){
            Shortest_Path.add(Path.get(i));
        }
    }

}
void FORSTEP(){
    counter_L = 0 ;
    counter_R = 0;
    FOWRARD();
    while (counter_L < 280) {
        CENTER();
        if(distance_front<8){
            STOP();
            delay(200);
            break;
        }
    }
    STOP();
    delay(400);
}
void loop() {

    if(SOLVED() || is_solved){
      //  / using the sortest path by eleminating the dead ends using the left hand\
      //  follower algorithm /
        LEFT();
        delay(3000);
        STOP();
        // for(int i = 0 ; i < 10 ; i++){
        //     analogWrite(5 , 100);
        //     delay(500);
        //     analogWrite(5 , 0);
        //     delay(500);
        // }
       // / this is an indicator for arriving to the goal /

        for(int i = 0 ; i < Shortest_Path.size() ; i++){
          Serial.println("1");
          if(Shortest_Path.get(i) == 'S'){
            FORSTEP();
          }else if(Shortest_Path.get(i) == 'L'){
            STOP();
            delay(300);
            counter_L = 0;
            counter_R = 0;
            LEFT();
            while((counter_L + counter_R) / 2 <= 61){

            }
            STOP();
            delay(400);
            /* moving one step forward, I will not count it in the path */
            FORSTEP();
          }else if(Shortest_Path.get(i) == 'R'){
            STOP();
            delay(300);
            counter_L = 0;
            counter_R = 0;
            RIGHT();
            while((counter_L + counter_R) / 2 <= 61){

            }
            STOP();
            delay(400);
            //CENTER();
            FORSTEP();
          } else if(Shortest_Path.get(i) == 'B'){
            STOP();
            delay(300);
            counter_L = 0;
            counter_R = 0;
            LEFT();
            while((counter_L + counter_R) / 2 <= 61){

            }
            STOP();
            delay(500);
            counter_L = 0;
            counter_R = 0;
            LEFT();
            while((counter_L + counter_R) / 2 <= 61){

            }
            STOP();
            delay(500);
            //CENTER();
            FORSTEP();
        } 
        }
    }
 //   / The goal is not found yet/
    else if(!is_solved){
    //    / Searching for the goal and storing the path
      //  */
        if(digitalRead(super_left) == 1){
            Path.add('L');
            check_for_optimization();
            STOP();
            delay(300);
            counter_L = 0;
            counter_R = 0;
            LEFT();
            while((counter_L + counter_R) / 2 <= 61){

            }
            STOP();
            delay(400);
            /* moving one step forward, I will not count it in the path */
            FORSTEP();
        }else if(distance_front>10){
          Path.add('S');
          check_for_optimization();
          //CENTER();
          FORSTEP();
        }else if(digitalRead(super_right) == 1){
          Path.add('R');
          check_for_optimization();
          STOP();
          delay(300);
          counter_L = 0;
          counter_R = 0;
          RIGHT();
          while((counter_L + counter_R) / 2 <= 61){

          }
          STOP();
          delay(400);
          //CENTER();
          FORSTEP();
        }
        else if(!digitalRead(super_right) && !digitalRead(super_left) && distance_front<8){
          /*
          * here we reach a deadend ! 
          */
          Path.add('B');
          check_for_optimization();
          STOP();
          delay(300);
          counter_L = 0;
          counter_R = 0;
          LEFT();
          while((counter_L + counter_R) / 2 <= 61){

          }
          STOP();
          delay(500);
          counter_L = 0;
          counter_R = 0;
          LEFT();
          while((counter_L + counter_R) / 2 <= 61){

          }
          STOP();
          delay(500);
          //CENTER();
          FORSTEP();

          }
        // delay(2000);
        CENTER();
    }

}