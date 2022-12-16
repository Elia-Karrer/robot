/*
    PS4-Controller Steuerung für HTL-EL Roboter
    Elia Karrer, 2022
*/

#include <Arduino.h>
#include <FastLED.h>
#include <PS4Controller.h>
#include <WiFi.h>


#define PIN_BATTERY 	            39
#define PIN_LED_DATA                23
#define PIN_LED_CLK                 18
#define PIN_MOTOR_DIR_L             33
#define PIN_MOTOR_SPEED_L           32
#define PIN_MOTOR_DIR_R             15
#define PIN_MOTOR_SPEED_R           2
#define PIN_LF_LR                   12
#define PIN_LF_ADC                  36

#define MOTOR_SPEED_R               0
#define MOTOR_DIR_R                 1
#define MOTOR_SPEED_L               2
#define MOTOR_DIR_L                 3
#define MOTOR_L                     0
#define MOTOR_R                     1

#define LF_BLACK_L                  0b10
#define LF_BLACK_R                  0b01
#define LF_00                       0
#define LF_01                       LF_BLACK_R
#define LF_10                       LF_BLACK_L
#define LF_11                       (LF_BLACK_L | LF_BLACK_R)
#define LF_TRESHOLD                 3000

#define CHARGING                    false
#define DEBUG_LOG


const uint8_t motor_pins[4] = {PIN_MOTOR_SPEED_L, PIN_MOTOR_DIR_L, PIN_MOTOR_SPEED_R, PIN_MOTOR_DIR_R};
const uint8_t motor_channels[4] = {MOTOR_SPEED_L, MOTOR_DIR_L, MOTOR_SPEED_R, MOTOR_DIR_R};

uint32_t last_time_check = 0;
uint32_t last_time_triangle = 0;
bool game_logic = true;
CRGB leds[4];



// Sets every all 4 LEDs to the value of the parameter "color"
// Format: 0xRRGGBB, where (R=red, G=green, B=blue)
void set_leds(uint32_t color)
{
    for(uint8_t i = 0; i < 4; i++)
        leds[i] = CRGB(color);

    FastLED.show();
}



// lr: 0=left, 1=right
// speed: [-1023 ... 1023] negative means backwards
void motor_single(uint8_t lr, int16_t speed)
{
    uint8_t ch_speed, ch_dir;


    if(speed > 1024 || speed < -1024)
        return;

    if(lr == MOTOR_L)
    {
        ch_dir = MOTOR_DIR_L;
        ch_speed = MOTOR_SPEED_L;
    }

    if(lr == MOTOR_R)
    {
        ch_dir = MOTOR_DIR_R;
        ch_speed = MOTOR_SPEED_R;
        speed *= -1;                                        // Flip direction
    }

    if(speed == 0)
    {
        ledcWrite(ch_dir, 0);
        ledcWrite(ch_speed, 0);
    }

    speed /= 2; 	                                        // Range from 512 to 1023: because <512 only beeping, no rotation

    if(speed > 0)
    {
        ledcWrite(ch_dir, 0);
        ledcWrite(ch_speed, speed+512);
    }

    if(speed < 0)
    {
        ledcWrite(ch_dir, -(speed-512));
        ledcWrite(ch_speed, 0);
    }
}


// Motor controlling functions
void motor_l(int16_t speed)
{
    motor_single(MOTOR_L, speed);
}
void motor_r(int16_t speed)
{
    motor_single(MOTOR_R, speed);
}
void motor(int16_t l, int16_t r)
{
    motor_l(l);
    motor_r(r);
}
void motor_both(int16_t speed)
{
    motor(speed, speed);
}



// Reads the line-following infrared sensors
// Format: xxxxxxRL where (0=White 1=Line)
uint8_t read_lf(void)
{
    uint8_t output = 0;
    uint16_t value;
    
    digitalWrite(PIN_LF_LR, 1);
    value = analogRead(PIN_LF_ADC);
    if(value > LF_TRESHOLD && value < 4050)
        output |= LF_BLACK_L;

    digitalWrite(PIN_LF_LR, 0);
    value = analogRead(PIN_LF_ADC);
    if(value > LF_TRESHOLD && value < 4050)
        output |= LF_BLACK_R;

    return output;
}



// Needs to be called continous
// Follows a black path/line with certain speed
void drive_follow_line(uint16_t speed)
{
    uint8_t lf;
    
    lf = read_lf();

    // On white ground (pause)
    if(lf == LF_00)
        motor_both(0);

    // On line
    if(lf == LF_11)
        motor_both(speed);

    // Left edge of line
    if(lf == LF_10)
        motor(0, speed);
    
    // Right edge of line
    if(lf == LF_01)
        motor(speed, 0);
}




// Press the direction where the car should go with (L2, R2)
// Rewinding with (L1, R1) but (L2, R2) still pressed
// L2   Gas Right
// R2   Gas Left
// L1   Flip Direction
// R1   Flip Direction
// U    Only Forward
// R    Only Backwards
void controller_drive_lr(void)
{
    if(PS4.Up())
    {
        motor_both(PS4.R2Value()*4);
        return;
    }

    if(PS4.Down())
    {
        motor_both(-(PS4.L2Value()*4));
        return;
    }
    
    motor_l(PS4.R1() ? -(PS4.R2Value()*4) : PS4.R2Value()*4);
    motor_r(PS4.L1() ? -(PS4.L2Value()*4) : PS4.L2Value()*4);
}



// Usual video-game control for cars
// Circle   Slow mode
// StickL   Steering
// L        Left Rotation 
// R        Right Rotation
// R2       Gas
// L2       Rerverse Gas
void controller_drive_game_logic(void)
{
    int8_t x;
    uint8_t l2, r2;
    uint16_t v_l, v_r;
    float f_l2, f_r2, f_sr, f_sl;
    

    x = PS4.LStickX();
    l2 = PS4.L2Value();
    r2 = PS4.R2Value();
    f_l2 = ((float) PS4.L2Value()) / 255.0f;
    f_r2 = ((float) PS4.R2Value()) / 255.0f;

    // No Gas
    if(r2 == 0 && l2 == 0)
    {
        motor_both(0);
        return;
    }

    // Fast rotation
    if(PS4.Right())
    {
        motor(r2*4, -(r2*4));
        return;
    }
    if(PS4.Left())
    {
        motor(-(r2*4), r2*4);
        return;
    }

    // Steering Factors
    if(x < 0)
    {
        f_sr = 1.0f;
        f_sl = ((float) (127-(-x))) / 127.0f;
    }
    if(x >= 0)
    {
        f_sr = ((float) (128-x)) / 128.0f;
        f_sl = 1.0f;
    }


    // Slow mode detection
    float max_speed_factor;
    if(PS4.Circle())
        max_speed_factor = 400.0f;
    else
        max_speed_factor = 1023.0f;

    // Control motors
    if(r2)
    {
        v_l = (uint32_t) (f_r2 * f_sl * max_speed_factor);
        v_r = (uint32_t) (f_r2 * f_sr * max_speed_factor);
        motor(v_l, v_r);
    }
    else
    {
        v_l = (uint32_t) (f_l2 * f_sl * max_speed_factor);
        v_r = (uint32_t) (f_l2 * f_sr * max_speed_factor);
        motor(-v_l, -v_r);
    }
}



void controller_drive(void)
{
    if(!PS4.isConnected())
    {
        set_leds(0x00FF00);
        return;
    }

    // Control mode check
    if(PS4.Triangle())
        if((micros() - last_time_triangle) > 500000)
        {
            game_logic = !game_logic;
            last_time_triangle = micros();
        }

    // Line following
    if(PS4.Cross())
    {
        set_leds(0xFF00FF);
        drive_follow_line(PS4.R2Value()*4);
        return;
    }

    // Game logic
    if(game_logic)
    {
        set_leds(0x0000FF);
        controller_drive_game_logic();
    }

    // Default mode
    else
    {
        set_leds(0x00FFFF);
        controller_drive_lr();
    }
}



// The sleep mode will send the robot to a state, where minimum power is used
// The user gets reminded by the LEDs warning-color, and the UART interface
// After 60 seconds the function will return, to check the sleep-conditions again
void sleep(void)
{
    set_leds(0xFF00);                                               // Set LED status to red (WARNING)
    WiFi.mode(WIFI_OFF);                                            // Turn off WiFi
    btStop();                                                       // Turn off Bluetooth                                             
    motor_both(0);                                                  // Turn off Motors

    #ifdef DEBUG_LOG
        Serial.println("LOG\tsleep: start");                        // Remind user by UART interface
    #endif

    esp_deep_sleep_start();                                         // Sleep for 60 seconds
}



// After 200 measurements (1 second) the average voltage gets calculated
// If it is lower than 3V, 3 times in a row, it will send the robot to sleep
// history: 1=undervoltage, 0=no_undervoltage
void check_undervoltage(void)
{
    static uint32_t sum = 0;                                        // (4095 * 200) < UINT32_MAX
    static uint8_t i = 0;                                           // 200 < (2^8 - 1)
    static uint8_t history = 0;

    // Every 5ms
    if(i < 200)
    {
        sum += analogRead(PIN_BATTERY);
        i++;
    }

    // Every 1000ms
    else
    {
        uint16_t avg = sum / 200;

        #ifdef DEBUG_LOG
            Serial.print("LOG\tundervoltage_protcetion: ADC=");
            Serial.print(avg);
            Serial.print(", U_ADC=");
            Serial.print(((float)(3*avg)) / 4096);
            Serial.print("V, Li_ion=");
            Serial.print(((((float)(3*avg)) / 4096)*3)/2);
            Serial.println("V");
        #endif

        history = (history << 1) | (avg < 2730);    	            // Insert new value to history

        if((history & 0b111) == 0b111 || CHARGING)
            sleep();                                                // Sleep, if Li-ion's voltage < 3V
        
        sum = i = 0;                                                // Reset counter and sum
    }
}



void setup(void)
{
    Serial.begin(115200);

    // Motors
    for(uint8_t i = 0; i < 4; i++)
    {
        pinMode(motor_pins[i], OUTPUT);
        ledcSetup(motor_channels[i], 16000, 10);
        ledcAttachPin(motor_pins[i], motor_channels[i]);
        ledcWrite(motor_channels[i], 0);
    }

    // LEDs
    FastLED.addLeds<SK9822, PIN_LED_DATA, PIN_LED_CLK, RBG, DATA_RATE_MHZ(12)>(leds, 4);

    // Linefollower
    pinMode(PIN_LF_LR, OUTPUT);

    // Sleep initialization to 60 seconds for sleep mode
    esp_sleep_enable_timer_wakeup(60 * 1000 * 1000);

    // Define MAC-address, which controller wants to pair to
    PS4.begin("11:11:11:11:11:11");

    #ifdef DEBUG_LOG
        Serial.println("LOG\tsetup: SUCCESS");
    #endif
}



void loop(void)
{
    if(micros() > (last_time_check+5000))                           // Undervoltage protection
    {
        check_undervoltage();
        last_time_check = micros();
    }

    controller_drive();
}
