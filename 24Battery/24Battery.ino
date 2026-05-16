// ============================================================
// BTS7960 Plotter Motor Test
// Arduino Mega / UNO
//
// Serial Commands:
// CONT    -> Continuous rotation
// DASH    -> Dashed mode (2 sec ON / 1 sec OFF)
// STOP    -> Stop motor
// SPEED x -> Set speed (0 - 255)
//
// Example:
// SPEED 180
// CONT
// DASH
// STOP
// ============================================================

// ---------------- MOTOR PINS ----------------
const int RPWM = 38;
const int LPWM = 39;
const int R_EN = 40;
const int L_EN = 41;

// ---------------- VARIABLES ----------------
int motorSpeed = 180;

bool continuousMode = false;
bool dashedMode = false;

unsigned long previousMillis = 0;
bool motorState = false;

// DASH TIMING
const unsigned long ON_TIME  = 2000; // 2 sec
const unsigned long OFF_TIME = 1000; // 1 sec

// ============================================================
void setup()
{
    Serial.begin(9600);

    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);

    // Enable BTS7960
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    stopMotor();

    Serial.println("=== BTS7960 Plotter Test ===");
    Serial.println("Commands:");
    Serial.println("CONT");
    Serial.println("DASH");
    Serial.println("STOP");
    Serial.println("SPEED 180");
}

// ============================================================
void loop()
{
    readSerialCommands();

    // -------- CONTINUOUS MODE --------
    if (continuousMode)
    {
        runMotorForward(motorSpeed);
    }

    // -------- DASHED MODE --------
    if (dashedMode)
    {
        unsigned long currentMillis = millis();

        if (motorState)
        {
            // Motor currently ON
            if (currentMillis - previousMillis >= ON_TIME)
            {
                previousMillis = currentMillis;
                motorState = false;
                stopMotor();
            }
        }
        else
        {
            // Motor currently OFF
            if (currentMillis - previousMillis >= OFF_TIME)
            {
                previousMillis = currentMillis;
                motorState = true;
                runMotorForward(motorSpeed);
            }
        }
    }
}

// ============================================================
// SERIAL COMMANDS
// ============================================================
void readSerialCommands()
{
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        // ---------------- CONT ----------------
        if (cmd == "CONT")
        {
            continuousMode = true;
            dashedMode = false;

            Serial.println("Continuous Mode");
        }

        // ---------------- DASH ----------------
        else if (cmd == "DASH")
        {
            continuousMode = false;
            dashedMode = true;

            previousMillis = millis();
            motorState = true;

            runMotorForward(motorSpeed);

            Serial.println("Dashed Mode");
        }

        // ---------------- STOP ----------------
        else if (cmd == "STOP")
        {
            continuousMode = false;
            dashedMode = false;

            stopMotor();

            Serial.println("Motor Stopped");
        }

        // ---------------- SPEED ----------------
        else if (cmd.startsWith("SPEED"))
        {
            int value = cmd.substring(6).toInt();

            if (value >= 0 && value <= 255)
            {
                motorSpeed = value;

                Serial.print("Speed Set To: ");
                Serial.println(motorSpeed);
            }
            else
            {
                Serial.println("Speed must be 0-255");
            }
        }
    }
}

// ============================================================
// MOTOR FUNCTIONS
// ============================================================

// Forward rotation
void runMotorForward(int spd)
{
    analogWrite(RPWM, spd);
    analogWrite(LPWM, 0);
}

// Stop motor
void stopMotor()
{
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
}