/*
State machine for controlling arduino MEGA 2560 for ECEN BYU Robotics Competition Dec 2021
*/
#include <Vector.h> // offbrand vector library, not the std lib
#include <Servo.h>
#include <math.h>

#define OP_AMP_THRESHOLD 1.5

// Pinout definitions:
#define SERVO1_PIN 22
#define SERVO2_PIN 24
#define SERVO3_PIN 26
#define SERVO4_PIN 28

// 40, 42, 44, 46
#define LASER1 40
#define LASER2 42
#define LASER3 44
#define LASER4 46

#define sensor1 A5
#define sensor2 A6
#define sensor3 A4
#define sensor4 A7

#define BTN1 30
#define BTN2 32
#define BTN3 34
#define BTN4 36

#define BTN_START 38

// Value definitions
#define SERVO_DELAY 200 // how long to make movement
#define FIRE_DELAY 80 // 150 // bad coding practice, but how long to tell robot to wait before firing again
int prev_corner = 0;
int prev_prev_corner = 0;
#define REPEAT_SHOT_DELAY 200

// Min and Max values to write to servo motors, may need to be tweaked individually.
#define SERVO1_MAX 90
#define SERVO1_MIN 0
#define SERVO2_MAX 90
#define SERVO2_MIN 0
#define SERVO3_MAX 110
#define SERVO3_MIN 5
#define SERVO4_MAX 90
#define SERVO4_MIN 0
#define SERVO_PRIME 30

// Debouncing
int debounce_timer = 0;
#define DEBOUNCE_MIN_TIME 50

// Sensor values
#define SENSOR_STDDEVS 6 // calculate if value is greater than 3 std-devs from means.
#define CONVERSION 5/1024
// Vector<float> sensor1_running_avg;
// Vector<float> sensor2_running_avg;
// Vector<float> sensor3_running_avg;
// Vector<float> sensor4_running_avg;

#define STARTING_ARRAY_SIZE 100
int starting_array_index = 0;
double sensor1_running_avg[STARTING_ARRAY_SIZE];
double sensor2_running_avg[STARTING_ARRAY_SIZE];
double sensor3_running_avg[STARTING_ARRAY_SIZE];
double sensor4_running_avg[STARTING_ARRAY_SIZE];

double sensor1_reading = 0;
double sensor2_reading = 0;
double sensor3_reading = 0;
double sensor4_reading = 0;
double sensor_avg[4] = {0.0, 0.0, 0.0, 0.0};
double sensor_stddev[4] = {0.0, 0.0, 0.0, 0.0};

// Servo definitions
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

bool safety = false;

enum state{
    init_st,
    pre_game_st,
    pre_game_debounce_st,
    pre_game_safety_st,
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
    servo1.write(SERVO_PRIME);
    servo2.write(SERVO_PRIME);
    servo3.write(SERVO_PRIME);
    servo4.write(SERVO_PRIME);

    pinMode(LASER1, OUTPUT);
    pinMode(LASER2, OUTPUT);
    pinMode(LASER3, OUTPUT);
    pinMode(LASER4, OUTPUT);
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
    pinMode(BTN3, INPUT);
    pinMode(BTN4, INPUT);
    pinMode(BTN_START, INPUT);

    Serial.begin(9600);

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
            digitalWrite(LASER1, HIGH);
            digitalWrite(LASER2, HIGH);
            digitalWrite(LASER3, HIGH);
            digitalWrite(LASER4, HIGH);
        break;

        case pre_game_st:
            if (digitalRead(BTN4) == HIGH) {
                current_state = pre_game_safety_st;
                safety = true;
            }
            if (digitalRead(BTN_START) == HIGH) {
                current_state = pre_game_debounce_st;
            } else {
                current_state = current_state;
            }

        break;
    
        case pre_game_debounce_st:
            if(digitalRead(BTN_START) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    digitalWrite(LASER1, LOW);
                    digitalWrite(LASER2, LOW);
                    digitalWrite(LASER3, LOW);
                    digitalWrite(LASER4, LOW);
                    current_state = wait_st;
                    // calculate_sensor_stats();
                    debounce_timer = 0;
                }
            } else {
                current_state = pre_game_st;
                debounce_timer = 0;
            }
        break;

        case pre_game_safety_st:
            if(digitalRead(BTN4) == HIGH) {
                if (debounce_timer >= DEBOUNCE_MIN_TIME) {
                    digitalWrite(LASER1, LOW);
                    digitalWrite(LASER2, LOW);
                    digitalWrite(LASER3, LOW);
                    digitalWrite(LASER4, LOW);
                    current_state = wait_st;
                    // calculate_sensor_stats();
                    debounce_timer = 0;
                    delay(1000);
                }
            } else {
                current_state = pre_game_st;
                debounce_timer = 0;
            }
        break;
        
        case wait_st:

            // Hard coded values
            if (safety == false) {
                if (sensor1_reading >= (OP_AMP_THRESHOLD)) {
                    if (prev_corner == 1) {
                        delay(REPEAT_SHOT_DELAY);
                    }
                    prev_prev_corner = prev_corner;
                    prev_corner = 1;
                    current_state = c1_st;
                } else if (sensor2_reading >= (OP_AMP_THRESHOLD)) {
                    if (prev_corner == 2) {
                        delay(REPEAT_SHOT_DELAY);
                    }
                    prev_corner = 2;
                    current_state = c2_st;
                } else if (sensor3_reading >= (OP_AMP_THRESHOLD)) {
                    if (prev_corner == 3) {
                        delay(REPEAT_SHOT_DELAY);
                    }
                    prev_corner = 3;
                    current_state = c3_st;
                } else if (sensor4_reading >= (OP_AMP_THRESHOLD)) {
                    if (prev_corner == 4) {
                        delay(REPEAT_SHOT_DELAY);
                    }
                    prev_corner = 4;
                    current_state = c4_st;
                }
            // leave below alone
            } if (digitalRead(BTN_START) == HIGH && digitalRead(BTN4) == HIGH) {
                current_state = cancel_debounce_st;
                safety = false;
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
                    starting_array_index = 0;
                    // sensor1_running_avg.clear();
                    // sensor2_running_avg.clear();
                    // sensor3_running_avg.clear();
                    // sensor4_running_avg.clear();
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
            // sensor1_running_avg.push_back(sensor1_reading);
            // sensor2_running_avg.push_back(sensor2_reading);
            // sensor3_running_avg.push_back(sensor3_reading);
            // sensor4_running_avg.push_back(sensor4_reading);
            if (starting_array_index < STARTING_ARRAY_SIZE) {
                sensor1_running_avg[starting_array_index] = sensor1_reading;
                sensor2_running_avg[starting_array_index] = sensor2_reading;
                sensor3_running_avg[starting_array_index] = sensor3_reading;
                sensor4_running_avg[starting_array_index] = sensor4_reading;
            }
            starting_array_index++;

        break;
    
        case pre_game_debounce_st:
            debounce_timer++;
        break;

        case pre_game_safety_st:
            debounce_timer++;
        break;
        
        case wait_st:
            // do nothing, just chill and wait for a sensor reading
        break;
        
        case c1_debounce_st:
            debounce_timer++;
        break;

        case c1_st:
            servo1.write(SERVO1_MAX);
            delay(SERVO_DELAY);
            servo1.write(SERVO1_MIN);
            //delay(SERVO_DELAY);
            
            delay(FIRE_DELAY);
        
        break;
        
        case c2_debounce_st:
            debounce_timer++;
        break;
        
        case c2_st:
            servo2.write(SERVO2_MAX);
            delay(SERVO_DELAY);
            servo2.write(SERVO2_MIN);
            // delay(SERVO_DELAY);
            
            delay(FIRE_DELAY);
        
        break;
        
        case c3_debounce_st:
            debounce_timer++;
        break;

        case c3_st:
            servo3.write(SERVO3_MAX);
            delay(SERVO_DELAY);
            servo3.write(SERVO3_MIN);
            // delay(SERVO_DELAY);
            
            delay(FIRE_DELAY);
        
        break;
        
        case c4_debounce_st:
            debounce_timer++;
        break;

        case c4_st:
            servo4.write(SERVO4_MAX);
            delay(SERVO_DELAY);
            servo4.write(SERVO4_MIN);
            // delay(SERVO_DELAY);
            
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
    sensor1_reading = float(analogRead(sensor1))*CONVERSION;
    sensor2_reading = float(analogRead(sensor2))*CONVERSION;
    sensor3_reading = float(analogRead(sensor3))*CONVERSION;
    sensor4_reading = float(analogRead(sensor4))*CONVERSION;
    tick();
}
