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
const int LEFT_OPEN   = 120;
const int LEFT_CLOSE  = 15;

const int RIGHT_OPEN  = 120;
const int RIGHT_CLOSE = 15;

int yawAngle = 90;   // yaw 현재각(홈=90°, 중립). setYaw의 n단계 분할 이동 시작점

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
    // ===== 표준(위치제어) 서보용 — n단계 분할 이동 + 단계별 0.3초 delay =====
    // Yaw를 각도 제어 서보로 교체함. write(angle)로 목표각(0~180)에 바로 정지 가능.
    // 급격한 점프 대신 현재각(yawAngle) → 목표각을 YAW_STEPS 단계로 선형 분할하여
    // 순차 이동하며, 각 단계마다 0.3초 대기해 부드럽게 이동한다.
    // (총 이동시간 ≈ YAW_STEPS × 0.3초 — 거리와 무관하게 일정)

    const int YAW_STEPS = 10;                      // 몇 번에 나눠 이동할지 (n)
    const unsigned long YAW_STEP_DELAY_MS = 300UL; // 각 단계 사이 대기 (0.3초)

    if(angle < 0 || angle > 180)
        return false;

    int start = yawAngle;

    if(angle == start)
    {
        // 이미 목표각 → 분할 이동 없이 즉시 반영(부팅/중복 명령 시 불필요한 3초 지연 방지)
        yawServo.write(angle);
        yawAngle = angle;
        return true;
    }

    // 현재각 → 목표각을 n단계로 선형 보간하며 순차 이동
    for(int i = 1; i <= YAW_STEPS; i++)
    {
        int stepAngle = start + (int)lroundf((float)(angle - start) * i / YAW_STEPS);
        yawServo.write(stepAngle);
        delay(YAW_STEP_DELAY_MS);
    }

    yawAngle = angle;   // 마지막 단계에서 목표각 도달
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

    leftServo.write(90);
    rightServo.write(90);

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

    setYaw(90);   // 홈 90°(중립) 확립 (yawAngle==90 → 분할 이동 없이 즉시 반영)

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