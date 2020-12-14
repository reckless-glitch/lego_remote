#include <Arduino.h>

#include <PS4Controller.h>
#include <Ticker.h>
#include <controller.cpp>
#include <bluetoothCOM.cpp>

#define BUILDIN_LED 2

bool output = false;

#define INPUT_UPDATE_TIME 50

float ENGNInput = 0;
float SRVOInput = 0;
bool boostInput = false;
bool lightsInput = false;

// █  █ ▄▀▀▄ █    ▀█▀▀ ▄▀▀▄ ▄▀▀█ █▀▀▀      ▄▀▀▄ ▄▀▀▄ █▄ █ ▀█▀▀ █▀▀▄ ▄▀▀▄ █
// █ █  █  █ █     █   █▄▄█ █ ▄▄ █■■       █    █  █ █▀▄█  █   █▄▄▀ █  █ █
// ██   ▀▄▄▀ █▄▄▄  █   █  █ ▀▄▄▀ █▄▄▄      ▀▄▄▀ ▀▄▄▀ █ ▀█  █   █ ▀▄ ▀▄▄▀ █▄▄▄

#define SHUTDOWN_LOW_VOLTAGE false

#define VOLTAGE_MEASURED_OFFSET 0.15
#define VOLTAGE_MEASURED_MAX 3.3

#define VOLTAGE_LIPO_LOW 3.4
#define VOLTAGE_LIPO_FULL 4.2

#define VOLTAGE_OUTPUT_MAX 12
#define VOLTAGE_OUTPUT_MIN 1.2
#define VOLTAGE_OUTPUT_DEFAULT 9

#define BOOST_TIME_MAX 2000

// ADC measuring

#define ADC_MAXIMUM 4095
#define RESISTOR_A 15000
#define RESISTOR_B 82000

// PINS
#define VOLTAGE_READ_PIN 35

// PWM
#define PWM_PIN_BASE_A 12 //---> drv7881 BASE IN1
#define PWM_PIN_BASE_B 13 //connect ground istead to drv7881 BASE IN2

#define PWM_PIN_SRVO_A 26 //---> drv7881 SRVO IN1
#define PWM_PIN_SRVO_B 27 //---> drv7881 SRVO IN2

#define PWM_PIN_ENGN_A 32 //---> drv7881 ENGINE IN1
#define PWM_PIN_ENGN_B 33 //---> drv7881 ENGINE IN1

#define PWM_CH_BASE_A 1
#define PWM_CH_BASE_B 2
#define PWM_CH_SRVO_A 3
#define PWM_CH_SRVO_B 4
#define PWM_CH_ENGN_A 5
#define PWM_CH_ENGN_B 6

// Setting PWM properties
#define PWM_FREQ_ENGN 2000
#define PWM_FREQ_SRVO 2000
#define PWM_FREQ_BASE 8000
#define PWM_RESOL 8

int16_t pwmMaxDuty;
uint16_t dutyCyleBASE;
uint16_t dutyCyleSRVO;
uint16_t dutyCyleENGN;

Ticker voltageTicker;
uint8_t voltageUPDATE_TIME = 50;

int ADCValue = 0;
float voltageMeasured = 0;
float voltageInput = 99;
float voltageBASE;
float voltageSRVO;
float voltageENGN;
uint8_t numberOfLipoCells = 0;

bool shutdown = false;

void setupPWM(uint8_t pin, uint8_t channel, int freq)
{
    ledcAttachPin(pin, channel);
    ledcSetup(channel, freq, PWM_RESOL);
}

void getInputVoltage()
{
    ADCValue = analogRead(VOLTAGE_READ_PIN);
    voltageMeasured = (VOLTAGE_MEASURED_OFFSET + (VOLTAGE_MEASURED_MAX * ADCValue / ADC_MAXIMUM));
    voltageInput = voltageMeasured * (RESISTOR_A + RESISTOR_B) / RESISTOR_A;
    bluetoothSendPlot(voltageInput);
    bluetoothSendPlot(voltageENGN);
}


float setOutputVoltage(uint8_t chA, uint8_t chB, float voltage, uint16_t *dutyCyle)
{
    *dutyCyle = 0; 
    if (abs(voltage) < VOLTAGE_OUTPUT_MIN)
    {
        voltage = 0;
        dutyCyle = 0;
        ledcWrite(chA, 0);
        ledcWrite(chB, 0);
    }
    else if (voltage > 0)
    {
        voltage = min(voltage, min((float)VOLTAGE_OUTPUT_MAX, voltageInput));
        *dutyCyle = round(pwmMaxDuty * voltage / voltageInput);
        ledcWrite(chA, 0);
        ledcWrite(chB, *dutyCyle);
    }
    else if (voltage < 0)
    {
        voltage = max(voltage, -min((float)VOLTAGE_OUTPUT_MAX, voltageInput));
        *dutyCyle = round(pwmMaxDuty * (-voltage / voltageInput));
        ledcWrite(chA, *dutyCyle);
        ledcWrite(chB, 0);
    }
    return voltage;
}

bool boostEngaged = false;

void voltageShutdown()
{
    setOutputVoltage(PWM_CH_BASE_A, PWM_CH_BASE_B, 0, &dutyCyleBASE);
    setOutputVoltage(PWM_CH_SRVO_A, PWM_CH_SRVO_B, 0, &dutyCyleSRVO);
    setOutputVoltage(PWM_CH_ENGN_A, PWM_CH_ENGN_B, 0, &dutyCyleENGN);
    voltageTicker.detach();
}

void voltageUpdate()
{
    if (shutdown)
    {
        voltageShutdown();
        return;
    }

    getInputVoltage();

    if (SHUTDOWN_LOW_VOLTAGE && voltageInput < numberOfLipoCells * VOLTAGE_LIPO_LOW)
    {
        shutdown = true;
        bluetoothSendMessage("LOW VOLTAGE: " + String(voltageInput / numberOfLipoCells) + " V per cell");
        voltageShutdown();
        return;
    }

    voltageBASE = setOutputVoltage(PWM_CH_BASE_A, PWM_CH_BASE_B, VOLTAGE_OUTPUT_DEFAULT, &dutyCyleBASE);
    voltageSRVO = setOutputVoltage(PWM_CH_SRVO_A, PWM_CH_SRVO_B, VOLTAGE_OUTPUT_DEFAULT * SRVOInput,&dutyCyleSRVO);
    if (boostEngaged)
        voltageENGN = setOutputVoltage(PWM_CH_ENGN_A, PWM_CH_ENGN_B, VOLTAGE_OUTPUT_MAX, &dutyCyleENGN);
    else
        voltageENGN = setOutputVoltage(PWM_CH_ENGN_A, PWM_CH_ENGN_B, VOLTAGE_OUTPUT_DEFAULT * ENGNInput, &dutyCyleENGN);
}

void voltageSetup()
{
    pwmMaxDuty = pow(2, PWM_RESOL) - 1;
    bluetoothSendMessage("Setting PWM to resolution " + String(pwmMaxDuty) + " ... Measuring Voltage: ");
    getInputVoltage();
    bluetoothSendMessage(String(voltageInput, 1) + " V, please stand by");
    delay(500);
    getInputVoltage();
    bluetoothSendMessage("Voltage: " + String(voltageInput, 1) + " V, done  ");
    while (voltageInput > numberOfLipoCells * VOLTAGE_LIPO_FULL)
        numberOfLipoCells++;
    bluetoothSendMessage("Assuming an " + String(numberOfLipoCells) + " cell lipo Battery , setting pwms\n");
    delay(500);

    setupPWM(PWM_PIN_BASE_A, PWM_CH_BASE_A, PWM_FREQ_BASE);
    setupPWM(PWM_PIN_BASE_B, PWM_CH_BASE_B, PWM_FREQ_BASE);

    setupPWM(PWM_PIN_SRVO_A, PWM_CH_SRVO_A, PWM_FREQ_SRVO);
    setupPWM(PWM_PIN_SRVO_B, PWM_CH_SRVO_B, PWM_FREQ_SRVO);

    setupPWM(PWM_PIN_ENGN_A, PWM_CH_ENGN_A, PWM_FREQ_ENGN);
    setupPWM(PWM_PIN_ENGN_B, PWM_CH_ENGN_B, PWM_FREQ_ENGN);

    voltageTicker.attach_ms(voltageUPDATE_TIME, voltageUpdate);
}

// █▄ ▄█ ▄▀▀▄ ▀█▀ █▄ █
// █▀█▀█ █▄▄█  █  █▀▄█
// █   █ █  █ ▄█▄ █ ▀█

#define UPDATE_TIME 100
Ticker updateTicker;
uint32_t systemTime = 0;

void shutDown()
{
    Serial.println("SHUTDOWN initiated");
    Serial.end();
    bluetoothSendMessage("SHUTDOWN initiated");
    updateTicker.detach();
    voltageShutdown();
    bluetoothShutdown();
    esp_deep_sleep_start();
}


uint16_t boostTimer = BOOST_TIME_MAX;

void managaeBoost()
{
    static uint lastUpdateTime = 0;
    static uint16_t updateTime = 0; //time since last update in ms
    updateTime = systemTime - lastUpdateTime;
    lastUpdateTime = systemTime;
    if (boostEngaged)
    {
        if (boostInput)
        {
            if (boostTimer < updateTime)
            {
                boostTimer = 0;
                boostEngaged = false;
                boostInput = false;
            }
            else
            {
                boostTimer -= updateTime; //count booster time
            }
        }
        else
        {
            boostEngaged = false;
            boostTimer /= 2; // boost timer will be reduced after release to prevent small boost pushes
        }
        
    }
    else 
    {
        if (boostTimer < BOOST_TIME_MAX)
            boostTimer += updateTime / 3; //cooldown booster
        else
            boostTimer = BOOST_TIME_MAX;
    }

    boostEngaged = (boostInput && boostTimer > 0);
}

void managaeLights()
{
}

void update()
{
    systemTime += UPDATE_TIME;
    if (shutdown)
    {
        shutDown();
        return;
    }
    managaeBoost();
    managaeLights();
    bluetoothSendMessage(
         "|T| " + String((float)systemTime / 1000, 2) 
        + " |V/~| IN " + String(voltageInput, 1) 
        + " /"+String(pwmMaxDuty)
        + " | BA " + String(voltageBASE, 1) 
        + " /"+String(dutyCyleBASE)
        + " | <> " + String(voltageSRVO, 1) 
        + " /" + String(dutyCyleSRVO)
        + " | EN " + String(voltageENGN, 1) 
        + " /"+String(dutyCyleENGN)
        + " |BST| " + (boostEngaged ? " >> " : " -- ") 
        +String(boostTimer)
        + (shutdown ? "| SHUTDOWN" : "")
    );
}

void setup()
{
    Serial.begin(115200);
    Serial.println("initialising");
    /*OLED_setup();
    Serial.println("oled done");
    GUI_setup();
    Serial.println("gui done");
    output = OLED_display(true);
    */

    pinMode(BUILDIN_LED, OUTPUT);
    //controllerSetup(INPUT_UPDATE_TIME);
    bluetoothSetup(INPUT_UPDATE_TIME, &ENGNInput, &SRVOInput, &boostInput, &lightsInput);
    voltageSetup();
    updateTicker.attach_ms(UPDATE_TIME, update);
}

void loop()
{
    vTaskDelete(NULL);
}