/*
State machine for controlling arduino MEGA 2560 for ECEN BYU Robotics Competition Dec 2021
*/
#include <Vector.h> // offbrand vector library, not the std lib
#include <Servo.h>
#include <math.h>

// Pinout definitions:
#define SERVO1_PIN 22
#define SERVO2_PIN 24
#define SERVO3_PIN 26
#define SERVO4_PIN 28

#define LASERS 7

#define sensor1 A4
#define sensor2 A5
#define sensor3 A6
#define sensor4 A7

#define BTN1 30
#define BTN2 32
#define BTN3 34
#define BTN4 36

#define BTN_START 38

// Value definitions
#define SERVO_DELAY 250 // how long to make movement
#define FIRE_DELAY 0 // bad coding practice, but how long to tell robot to wait before firing again
// Min and Max values to write to servo motors, may need to be tweaked individually.
#define SERVO1_MAX 90
#define SERVO1_MIN 0
#define SERVO2_MAX 90
#define SERVO2_MIN 0
#define SERVO3_MAX 90
#define SERVO3_MIN 0
#define SERVO4_MAX 90
#define SERVO4_MIN 0

// Debouncing
int debounce_timer = 0;
#define DEBOUNCE_MIN_TIME 50

// Sensor values
#define SENSOR_STDDEVS 3 // calculate if value is greater than 3 std-devs from means.
Vector<float> sensor1_running_avg;
Vector<float> sensor2_running_avg;
Vector<float> sensor3_running_avg;
Vector<float> sensor4_running_avg;
float sensor1_reading = 0;
float sensor2_reading = 0;
float sensor3_reading = 0;
float sensor4_reading = 0;
float sensor_avg[4] = {0.0, 0.0, 0.0, 0.0};
float sensor_stddev[4] = {0.0, 0.0, 0.0, 0.0};

// Servo definitions
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

enum state{
    init_st,
    pre_game_st,
    pre_game_debounce_st,
    wait_st,
    c1_st,
    c1_debounce_st,
    c2_st,
    c2_debounce_st,
    c3_st,
    c3_debounce_st,
    c4_st,
    c4_debounce_st,
    cancel_debounce_st,
    exit_st
};
// Servo direction code
enum dir{
    fwd,
    bwd
};

dir s1_dir = fwd;
dir s2_dir = fwd;
dir s3_dir = fwd;
dir s4_dir = fwd;

state current_state = init_st;
state prev_state = init_st;

// the setup function runs once when you press reset or power the board
void setup() {
    // Attach servos to pins
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    // Initialie motors to position 0
    servo1.write(SERVO1_MIN);
    servo2.write(SERVO2_MIN);
    servo3.write(SERVO3_MIN);
    servo4.write(SERVO4_MIN);

    pinMode(LASERS, OUTPUT);
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
    pinMode(BTN3, INPUT);
    pinMode(BTN4, INPUT);
    pinMode(BTN_START, INPUT);

    Serial.begin(9600);

}

// Calculate the average and std-dev of sensor readings from vector
void calculate_sensor_stats() {
    int N = sensor1_running_avg.size();
    // First we'll sum up all the values in the array (wishing this was pythong)
    for (int i = 0; i < N; i++) {
        sensor_avg[0] += sensor1_running_avg[i];
        sensor_avg[1] += sensor2_running_avg[i];
        sensor_avg[2] += sensor3_running_avg[i];
        sensor_avg[3] += sensor4_running_avg[i];
    }
    // find the average of the values
    sensor_avg[0] = sensor_avg[0]/N;
    sensor_avg[1] = sensor_avg[1]/N;
    sensor_avg[2] = sensor_avg[2]/N;
    sensor_avg[3] = sensor_avg[3]/N;
    
    // finding std-dev
    for (int i = 0; i < N; i++) {
        sensor_stddev[0] += pow((sensor1_running_avg[i] - sensor_avg[0]), 2);
        sensor_stddev[1] += pow((sensor2_running_avg[i] - sensor_avg[1]), 2);
        sensor_stddev[2] += pow((sensor3_running_avg[i] - sensor_avg[2]), 2);
        sensor_stddev[3] += pow((sensor4_running_avg[i] - sensor_avg[3]), 2);
    }
    sensor_stddev[0] = sqrt(sensor_stddev[0]/(N-1));
    sensor_stddev[1] = sqrt(sensor_stddev[1]/(N-1));
    sensor_stddev[2] = sqrt(sensor_stddev[2]/(N-1));
    sensor_stddev[3] = sqrt(sensor_stddev[3]/(N-1));
}

// Insert state transition logger:
void print_states() {
        if (prev_state != current_state) {
            switch(current_state) {
                case pre_game_st:
                    Serial.println("ST_Pre_Game");
                break;
            
                case pre_game_debounce_st:
                    Serial.println("ST_Pre_Game_Debounce");
                break;
                
                case wait_st:
                    Serial.println("ST_Wait");
                break;
                
                case c1_debounce_st:
                    Serial.println("ST_Corner 1 debounce");
                break;

                case c1_st:
                    Serial.println("ST_Corner 1 Fire");
                break;
                
                case c2_debounce_st:
                    Serial.println("ST_Corner 2 debounce");
                break;

                case c2_st:
                    Serial.println("ST_Corner 2 Fire");    
                break;
                
                case c3_debounce_st:
                    Serial.println("ST_Corner 3 debounce");
                break;

                case c3_st:
                    Serial.println("ST_Corner 3 Fire");
                break;
                
                case c4_debounce_st:
                    Serial.println("ST_Corner 4 debounce");
                break;

                case c4_st:
                    Serial.println("ST_Corner 4 Fire");
                break;

                case cancel_debounce_st:
                    Serial.println("Cancel_Debounce_ST");
                break;

                case exit_st:
                  Serial.println("Exit_State, let go of the buttons");
            }
        }
}

void tick() {
    // State Transitions
    switch(current_state) {
        case init_st:
            current_state = pre_game_st;
            pinMode(LASERS, HIGH);
        break;

        case pre_game_st:

            if (digitalRead(BTN_START) == HIGH) {
                current_state = pre_game_debounce_st;
            } else {
                current_state = current_state;
            }

        break;
    
        case pre_game_debounce_st:
            if(digitalRead(BTN_START) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    pinMode(LASERS, LOW);
                    current_state = wait_st;
                    calculate_sensor_stats();
                    debounce_timer = 0;
                }
            } else {
                current_state = pre_game_st;
                debounce_timer = 0;
            }
        break;
        
        case wait_st:
            // Insert sensor value checks here first
            if (sensor1_reading >= (sensor_avg[0]+sensor_stddev[0]*SENSOR_STDDEVS)) {
                current_state = c1_st;
            } else if (sensor2_reading >= (sensor_avg[1]+sensor_stddev[1]*SENSOR_STDDEVS)) {
                current_state = c2_st;
            } else if (sensor3_reading >= (sensor_avg[2]+sensor_stddev[2]*SENSOR_STDDEVS)) {
                current_state = c3_st;
            } else if (sensor4_reading >= (sensor_avg[3]+sensor_stddev[3]*SENSOR_STDDEVS)) {
                current_state = c4_st;
            } else if (digitalRead(BTN_START) == HIGH && digitalRead(BTN4) == HIGH) {
                current_state = cancel_debounce_st;
            } else if (digitalRead(BTN1) == HIGH) {
                current_state = c1_debounce_st;
            } else if (digitalRead(BTN2) == HIGH) {
                current_state = c2_debounce_st;
            } else if (digitalRead(BTN3) == HIGH) {
                current_state = c3_debounce_st;
            } else if (digitalRead(BTN4) == HIGH) {
                current_state = c4_debounce_st;
            }

        break;
        
        case c1_debounce_st:
            if (digitalRead(BTN1)== HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    current_state = c1_st;
                    debounce_timer = 0;
                }
            } else {
                current_state = wait_st;
                debounce_timer = 0;
            }
        break;

        case c1_st:
            current_state = wait_st;
        break;
        
        case c2_debounce_st:
            if (digitalRead(BTN2) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    current_state = c2_st;
                    debounce_timer = 0;
                }
            } else {
                current_state = wait_st;
                debounce_timer = 0;
            }
        break;
        
        case c2_st:
            current_state = wait_st;
        break;
        
        case c3_debounce_st:
            if (digitalRead(BTN3) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    current_state = c3_st;
                    debounce_timer = 0;
                }
            } else {
                current_state = wait_st;
                debounce_timer = 0;
            }
        break;

        case c3_st:
            current_state = wait_st;
        break;
        
        case c4_debounce_st:
            if (digitalRead(BTN4) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    current_state = c4_st;
                    debounce_timer = 0;
                }
            } else {
                current_state = wait_st;
                debounce_timer = 0;
            }
        break;

        case c4_st:
            current_state = wait_st;
        break;

        case cancel_debounce_st:
            if (digitalRead(BTN_START) == HIGH && digitalRead(BTN4) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    current_state = exit_st;
                    debounce_timer = 0;
                }
            } else {
                current_state = wait_st;
                debounce_timer = 0;
            }
        break;

        case exit_st:
            if (digitalRead(BTN_START) == LOW && digitalRead(BTN4) == LOW) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    current_state = pre_game_st;
                    debounce_timer = 0;
                    sensor1_running_avg.clear();
                    sensor2_running_avg.clear();
                    sensor3_running_avg.clear();
                    sensor4_running_avg.clear();
                }
            } else {
                current_state = exit_st;
                debounce_timer = 0;
            }
        break;
    }

    // State actions
    switch(current_state) {
        case init_st:

        break;

        case pre_game_st:
            // create running average of sensor readings
            sensor1_running_avg.push_back(sensor1_reading);
            sensor2_running_avg.push_back(sensor2_reading);
            sensor3_running_avg.push_back(sensor3_reading);
            sensor4_running_avg.push_back(sensor4_reading);
        break;
    
        case pre_game_debounce_st:
            debounce_timer++;
        break;
        
        case wait_st:
            // do nothing, just chill and wait for a sensor reading
        break;
        
        case c1_debounce_st:
            debounce_timer++;
        break;

        case c1_st:
            if(true) { // s1_dir == fwd) {
                servo1.write(SERVO1_MAX);
                delay(SERVO_DELAY);
                servo1.write(SERVO1_MIN);
                delay(SERVO_DELAY);
                s1_dir = bwd;
            } else {
                servo1.write(SERVO1_MIN);
                delay(SERVO_DELAY);
                s1_dir = fwd;
            }
            delay(FIRE_DELAY);
        
        break;
        
        case c2_debounce_st:
            debounce_timer++;
        break;
        
        case c2_st:
            if(true) { //s2_dir == fwd) {
                servo2.write(SERVO2_MAX);
                delay(SERVO_DELAY);
                servo2.write(SERVO2_MIN);
                delay(SERVO_DELAY);
                s2_dir = bwd;
            } else {
                servo2.write(SERVO2_MIN);
                delay(SERVO_DELAY);
                s2_dir = fwd;
            }
            delay(FIRE_DELAY);
        
        break;
        
        case c3_debounce_st:
            debounce_timer++;
        break;

        case c3_st:
            if(true) { //s3_dir == fwd) {
                servo3.write(SERVO3_MAX);
                delay(SERVO_DELAY);
                servo3.write(SERVO3_MIN);
                delay(SERVO_DELAY);
                s3_dir = bwd;
            } else {
                servo3.write(SERVO3_MIN);
                delay(SERVO_DELAY);
                s3_dir = fwd;
            }
            delay(FIRE_DELAY);
        
        break;
        
        case c4_debounce_st:
            debounce_timer++;
        break;

        case c4_st:
            if(true) { // s4_dir == fwd) {
                servo4.write(SERVO4_MAX);
                delay(SERVO_DELAY);
                servo4.write(SERVO4_MIN);
                delay(SERVO_DELAY);
                s4_dir = bwd;
            } else {
                servo4.write(SERVO4_MIN);
                delay(SERVO_DELAY);
                s4_dir = fwd;
            }
            delay(FIRE_DELAY);
        break;

        case cancel_debounce_st:
            debounce_timer++;
        break;
        case exit_st:
            debounce_timer++;
        break;
    }
    print_states();
    prev_state = current_state;
}

void loop() {
    // can implement 3 value average here if needed, will need to change initialization in Pre-game
    sensor1_reading = analogRead(sensor1);
    sensor2_reading = analogRead(sensor2);
    sensor3_reading = analogRead(sensor3);
    sensor4_reading = analogRead(sensor4);
    tick();
}
