#include "AnalogIn.h"
#include "BufferedSerial.h"
#include "DigitalIn.h"
#include "InterruptIn.h"
#include "PinNamesTypes.h"
#include "Timer.h"
#include "stdint.h"
#include "mbed.h"
#include <cstdint>
#include <cstdio>
#include <MessageStructure.hpp>
#include <Controller.hpp>
#include <SerialBridge.hpp>
#include <MbedHardwareSerial.hpp>


////////////////////////////////////
//  Private Defenition
//  Left cross key
#define LC_UP PC_10
#define LC_LEFT PC_12
#define LC_RIGHT PD_2
#define LC_DOWN PC_11
//  Right cross key
#define RC_UP PB_3
#define RC_LEFT PB_4
#define RC_RIGHT PB_5
#define RC_DOWN PA_10

//  Left button
#define LB_1 PB_14
#define LB_2 PB_13

//  Right button
#define RB_1 PB_1
#define RB_2 PB_15

//  JoyStick
#define JOY_LEFT_X PA_0
#define JOY_LEFT_Y PA_1
#define JOY_RIGHT_X PA_4
#define JOY_RIGHT_Y PB_0

//  Emergency stop
#define EMERGENCY_STOP PA_13

//  IM920
#define IM920_BAUDRATE 115200
#define IM920_TX PA_15
#define IM920_RX PB_7
#define IM920_XMIT PH_1
#define IM920_SLEEP PC_2
#define IM920_STATUS PC_3
#define IM920_BUSY PC_0
#define IM920_RESET PC_1

//  LED
#define LED_1 PB_9
#define LED_2 PB_8
#define LED_3 PC_9
#define LED_4 PC_8
#define LED_5 PC_6

#define JOYSTIC_DEADZONE 0.05

#define SHOOTER_POWER_FAST_DIFFERENCE 0.10
#define SHOOTER_POWER_SLOW_DIFFERENCE 0.01

#define LOOP_RATE 100
#define COOLTIME_MS 250

#define SUB_CONTROLLER_ID 0

////////////////////////////////////
//  Private Variable
//  Emergency Stop
DigitalIn emergency_stop(EMERGENCY_STOP);

//  LC Interrupt
InterruptIn lc_up(LC_UP, PullDown);
InterruptIn lc_down(LC_DOWN, PullDown);
InterruptIn lc_left(LC_LEFT, PullDown);

//  RC Interrupt
InterruptIn rc_left(RC_LEFT, PullDown);
InterruptIn rc_right(RC_RIGHT, PullDown);
//  RC DigitalIn
DigitalIn rc_up(RC_UP, PullDown);
DigitalIn rc_down(RC_DOWN, PullDown);

//  LB DigitalIn
DigitalIn lb_1(LB_1, PullDown);

//  RB Interrupt
InterruptIn rb_1(RB_1, PullDown);
InterruptIn rb_2(RB_2, PullDown);

//  JoyStick
AnalogIn joy_x(JOY_LEFT_X);
AnalogIn joy_y(JOY_LEFT_Y);
AnalogIn joy_theta(JOY_RIGHT_X);

//  UART
SerialDev *dev = new MbedHardwareSerial(new BufferedSerial(IM920_TX, IM920_RX, IM920_BAUDRATE));
SerialBridge serial(dev, 1024);

Controller controller_msg;

//  Intruppt timer
Timer interrupt_timer;

//  need to transfer
bool _need_transfer = false;

////////////////////////////////////
//  Private Prototype Function
//  Initializer callback
static void initialize_callback();
//  Transfer packet
void transfer_packet();

//  Update controller
void update_controller();

//  insert filltered value into controller type
void fillter_analog_value();

//  LC interrupt
void lc_up_callback();
void lc_down_callback();
void lc_left_callback();

//  RC interrupt
void rc_left_callback();
void rc_right_callback();

//  RB interrupt
void rb_1_callback();
void rb_2_callback();

//  timer
bool is_cooltime();
void reset_cooltime();


int main()
{
    printf("Launch\n\r");

    //  init serial bridge
    serial.add_frame(SUB_CONTROLLER_ID, &controller_msg);

    //  init message
    controller_msg.data.all_reload = false;
    controller_msg.data.emergency_switch = false;
    controller_msg.data.movement_mode = 0;
    controller_msg.data.movement.x = 0.0;
    controller_msg.data.movement.y = 0.0;
    controller_msg.data.movement.z = 0.0;
    controller_msg.data.shooter.action = 0;
    controller_msg.data.shooter.power = 0.0;
    controller_msg.data.shooter.num = 0;
    controller_msg.data.face = 0;

    interrupt_timer.start();

    initialize_callback();

    for(int i = 0 ; ; i++) {
        //  update controller
        update_controller();

        //  publish
        transfer_packet();

        if(_need_transfer)
        {
            controller_msg.data.all_reload = false;
            controller_msg.data.shooter.action = 0;

            _need_transfer = false;
        }

        wait_us(LOOP_RATE * 1000);
    }
}

static void initialize_callback()
{
    //  lc
    lc_up.rise(lc_up_callback);
    lc_down.rise(lc_down_callback);
    lc_left.rise(lc_left_callback);
    //  rc
    rc_left.rise(rc_left_callback);
    rc_right.rise(rc_right_callback);
    //  rb
    rb_1.rise(rb_1_callback);
    rb_2.rise(rb_2_callback);
}

//  Transfer packet
void transfer_packet()
{
    serial.write(SUB_CONTROLLER_ID);
}

//  update controller
void update_controller()
{
    //  emergency stop
    controller_msg.data.emergency_switch = emergency_stop;

    //  update joystick
    fillter_analog_value();

    //  shooter action
    if(controller_msg.data.shooter.action != 3)
    {
      if(rc_up && rc_down)
      {
        //  No! No! Don't press the combination of UP + DOWN
      }
      else if (rc_up)
      {
        controller_msg.data.shooter.action = 1;
      }
      else if (rc_down)
      {
        controller_msg.data.shooter.action = 2;
      }
      else if (!rc_up && !rc_down){
        controller_msg.data.shooter.action = 0;
      }
    }
}

//  insert filltered value into controller type
void fillter_analog_value()
{
    //  Joy
    float x = (joy_x.read() - 0.5) * 2;
    float y = (joy_y.read() - 0.5) * 2;
    float theta = (joy_theta.read() - 0.5) * 2;

    //  deadzone
    controller_msg.data.movement.x = -JOYSTIC_DEADZONE < x && x < JOYSTIC_DEADZONE ? 0.0 : x;
    controller_msg.data.movement.y = -JOYSTIC_DEADZONE < y && y < JOYSTIC_DEADZONE ? 0.0 : y;
    controller_msg.data.movement.z = -JOYSTIC_DEADZONE < theta && theta < JOYSTIC_DEADZONE ? 0.0 : theta;
}

//  LC interrupt
void lc_up_callback()
{
    if(is_cooltime())
    {
        return;
    }

    float power = controller_msg.data.shooter.power;

    //  increase
    power += lb_1 ? SHOOTER_POWER_FAST_DIFFERENCE: SHOOTER_POWER_SLOW_DIFFERENCE;

    //  limit
    controller_msg.data.shooter.power = power > 1.0 ? 1.0: power;

    reset_cooltime();
}

void lc_down_callback()
{
    if(is_cooltime())
    {
        return;
    }

    float power = controller_msg.data.shooter.power;

    //  increase
    power -= lb_1 ? SHOOTER_POWER_FAST_DIFFERENCE: SHOOTER_POWER_SLOW_DIFFERENCE;

    //  limit
    controller_msg.data.shooter.power = power < 0.0 ? 0.0: power;

    reset_cooltime();
}

void lc_left_callback()
{
    if(is_cooltime())
    {
        return;
    }


    int8_t mode = controller_msg.data.face + 1;

    //  limitter
    controller_msg.data.face = mode >= 4 ? 1 : mode;

    reset_cooltime();
}


//  RC interrupt
void rc_left_callback()
{
    if(is_cooltime())
    {
        return;
    }

    controller_msg.data.all_reload = true;

    //  publish
    // transfer_packet();
    _need_transfer = true;
    //  reset
    // controller_msg.data.all_reload = false;

    reset_cooltime();
}

void rc_right_callback()
{
    if(is_cooltime())
    {
        return;
    }

    int8_t mode = controller_msg.data.movement_mode + 1;

    //  limitter
    controller_msg.data.movement_mode = mode >= 3 ? 0: mode;

    reset_cooltime();
}

//  RB interrupt
void rb_1_callback()
{
    if(is_cooltime())
    {
        return;
    }

    int mode = controller_msg.data.shooter.num + 1;

    //  limitter
    controller_msg.data.shooter.num = mode >= 3 ? 0: mode;

    reset_cooltime();
}

void rb_2_callback()
{
    if(is_cooltime())
    {
        return;
    }

    controller_msg.data.shooter.action = 3;
    
    //  publish
    // transfer_packet();
    _need_transfer = true;
    //  reset
    // controller_msg.data.shooter.action = 0;

    reset_cooltime();
}

bool is_cooltime()
{
    return interrupt_timer.read_ms() < COOLTIME_MS;
}

void reset_cooltime()
{
    interrupt_timer.reset();
}