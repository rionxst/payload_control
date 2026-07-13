#include <Arduino.h>
#include <ESP32Servo.h>

Servo leftServo;
Servo rightServo;
Servo yawServo;

// ---------------- Pin ---------------- 
const int LEFT_PIN  = 18;
const int RIGHT_PIN = 19;
const int YAW_PIN   = 21;

// -------------- PWM ------------------ 
const int SERVO_FREQ   = 50;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;

// ---------- Servo Angle --------------- 
const int LEFT_OPEN   = 30;
const int LEFT_CLOSE  = 90;

const int RIGHT_OPEN  = 150;
const int RIGHT_CLOSE = 90;

int yawAngle = 90;

bool gripperOpened = false;
bool armed = false;

String inputLine = "";

/* ========================================= */

void replyOK(String msg)
{
    Serial.print("OK,");
    Serial.println(msg);
}

void replyERR(String msg)
{
    Serial.print("ERR,");
    Serial.println(msg);
}

/* ========================================= */

void openGripper()
{
    leftServo.write(LEFT_OPEN);
    rightServo.write(RIGHT_OPEN);
    gripperOpened = true;
}

void closeGripper()
{
    leftServo.write(LEFT_CLOSE);
    rightServo.write(RIGHT_CLOSE);
    gripperOpened = false;
}

bool setYaw(int angle)
{
    if(angle < 0 || angle > 180)
        return false;

    yawAngle = angle;

    yawServo.write(angle);

    return true;
}

/* ========================================= */
void printStatus()
{
    Serial.print("OK,STATUS,");

    Serial.print("ARMED=");
    Serial.print(armed);

    Serial.print(",GRIPPER=");

    if(gripperOpened)
        Serial.print("OPEN");
    else
        Serial.print("CLOSE");

    Serial.print(",YAW=");
    Serial.println(yawAngle);
}

/* ========================================= */

void processSet(String remain)
{
    int comma = remain.indexOf(',');

    if(comma < 0)
    {
        replyERR("BAD_FORMAT");
        return;
    }

    int id = remain.substring(0, comma).toInt();

    int value = remain.substring(comma + 1).toInt();

    if(!armed)
    {
        replyERR("NOT_ARMED");
        return;
    }

    switch(id)
    {

    // =====================
    // Gripper
    // =====================

    case 0:

        if(value == 0)
        {
            closeGripper();
            replyOK("GRIPPER,CLOSE");
        }
        else
        {
            openGripper();
            replyOK("GRIPPER,OPEN");
        }

        break;

    // =====================
    // Yaw
    // =====================

    case 1:

        if(setYaw(value))
        {
            Serial.print("OK,YAW,");
            Serial.println(value);
        }
        else
        {
            replyERR("YAW_RANGE");
        }

        break;

    default:

        replyERR("BAD_SERVO");

        break;
    }
}

/* ========================================= */

void handleCommand(String cmd)
{
    cmd.trim();

    cmd.toUpperCase();

    if(cmd == "PING")
    {
        replyOK("PONG");
        return;
    }

    if(cmd == "STATUS?")
    {
        printStatus();
        return;
    }

    if(cmd == "ARM")
    {
        armed = true;
        replyOK("ARMED");
        return;
    }

    if(cmd == "DISARM")
    {
        armed = false;
        replyOK("DISARMED");
        return;
    }

    if(cmd.startsWith("SET,"))
    {
        processSet(cmd.substring(4));
        return;
    }

    replyERR("BAD_COMMAND");
}

/* ========================================= */

void setup()
{
    Serial.begin(115200);

    delay(1000);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);

    leftServo.setPeriodHertz(SERVO_FREQ);
    rightServo.setPeriodHertz(SERVO_FREQ);
    yawServo.setPeriodHertz(SERVO_FREQ);

    leftServo.attach(
        LEFT_PIN,
        SERVO_MIN_US,
        SERVO_MAX_US);

    rightServo.attach(
        RIGHT_PIN,
        SERVO_MIN_US,
        SERVO_MAX_US);

    yawServo.attach(
        YAW_PIN,
        SERVO_MIN_US,
        SERVO_MAX_US);

    openGripper();

    setYaw(90);

    Serial.println("READY,GRIPPER_CONTROLLER,V1");

    printStatus();
}

/* ========================================= */

void loop()
{
    while(Serial.available())
    {
        char c = Serial.read();

        if(c == '\n' || c == '\r')
        {
            if(inputLine.length())
            {
                handleCommand(inputLine);

                inputLine = "";
            }
        }
        else
        {
            if(inputLine.length() < 100)
            {
                inputLine += c;
            }
            else
            {
                inputLine = "";
                replyERR("LINE_TOO_LONG");
            }
        }
    }
}