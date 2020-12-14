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
bool boostEngaged = false;
bool lightsInput = false;
bool lightAenabled = false;
bool lightBenabled = false;


// █  █ ▄▀▀▄ █    ▀█▀▀ ▄▀▀▄ ▄▀▀█ █▀▀▀      ▄▀▀▄ ▄▀▀▄ █▄ █ ▀█▀▀ █▀▀▄ ▄▀▀▄ █
// █ █  █  █ █     █   █▄▄█ █ ▄▄ █■■       █    █  █ █▀▄█  █   █▄▄▀ █  █ █
// ██   ▀▄▄▀ █▄▄▄  █   █  █ ▀▄▄▀ █▄▄▄      ▀▄▄▀ ▀▄▄▀ █ ▀█  █   █ ▀▄ ▀▄▄▀ █▄▄▄

#define SHUTDOWN_LOW_VOLTAGE_ALLOWED false

#define VOLTAGE_MEASURED_OFFSET 0.15
#define VOLTAGE_MEASURED_MAX 3.3

#define VOLTAGE_LIPO_LOW_CRITICAL 3.4
#define VOLTAGE_LIPO_LOW 3.5
#define VOLTAGE_LIPO_FULL 4.2

#define VOLTAGE_OUTPUT_MAX 12.0f
#define VOLTAGE_OUTPUT_MIN 2.1f
#define VOLTAGE_OUTPUT_DEFAULT 9.0f

#define BOOST_TIME_MAX 2000

// ADC measuring

#define ADC_MAXIMUM 4095
#define RESISTOR_A 15000
#define RESISTOR_B 82000

// PINS
#define VOLTAGE_READ_PIN 35

// PWM
#define PWM_PIN_LED_A 12 //---> drv8833 IN1
#define PWM_PIN_LED_B 13 //---> drv8833 IN3

#define PWM_PIN_SRVO_A 26 //---> drv7881 SRVO IN1
#define PWM_PIN_SRVO_B 27 //---> drv7881 SRVO IN2

#define PWM_PIN_ENGN_A 32 //---> drv7881 ENGINE IN1
#define PWM_PIN_ENGN_B 33 //---> drv7881 ENGINE IN1

#define PWM_CH_LED_A 1
#define PWM_CH_LED_B 2

#define PWM_CH_SRVO_A 3
#define PWM_CH_SRVO_B 4
#define PWM_CH_ENGN_A 5
#define PWM_CH_ENGN_B 6

// Setting PWM properties
#define PWM_FREQ_ENGN 2000
#define PWM_FREQ_SRVO 2000
#define PWM_FREQ_LED 2000
#define PWM_RESOL 8

int16_t pwmMaxDuty;
uint16_t dutyCycleLED;
uint16_t dutyCyleSRVO;
uint16_t dutyCyleENGN;

Ticker voltageTicker;
uint8_t voltageUPDATE_TIME = 50;

int ADCValue = 0;
float voltageMeasured = 0;
float voltageInput = 99;

float voltageSRVO;
float voltageENGN;
uint8_t numberOfLipoCells = 0;
String debugString="BOOTING";
bool lowVoltage = false;
bool lowVoltageCritical = false;
bool inputVoltageError = false;
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

#define VOLTAGE_DELTA_UP_MAX 0.8f 
#define VOLTAGE_DELTA_DOWN_MAX 0.8f



int16_t setOutput(uint8_t ch, bool enable)
{

    uint16_t dutyCycle = enable?pwmMaxDuty:0;
    ledcWrite(ch, dutyCycle);

}

int16_t setOutputVoltage(uint8_t chA, uint8_t chB, float inputVoltage, float targetVoltage, float *currentVoltage)
{

    if(*currentVoltage != targetVoltage)
    { 
        if(abs(targetVoltage)<VOLTAGE_OUTPUT_MIN)
        targetVoltage=0;


        // move voltage up and down slowly
        if (*currentVoltage>0)
        {
            if (targetVoltage > *currentVoltage)
                targetVoltage = min(targetVoltage, *currentVoltage + VOLTAGE_DELTA_UP_MAX);
            else if ( targetVoltage < *currentVoltage)
                targetVoltage = max(targetVoltage,*currentVoltage - VOLTAGE_DELTA_DOWN_MAX);
        }
        else
        {
            if (targetVoltage > *currentVoltage)
                targetVoltage = min(targetVoltage,*currentVoltage + VOLTAGE_DELTA_DOWN_MAX);
            
            if ( targetVoltage < *currentVoltage)
                targetVoltage = max(targetVoltage, *currentVoltage - VOLTAGE_DELTA_UP_MAX);
        }
    }
 
    static uint16_t dutyCycle =0;

    if (targetVoltage > 0)
    {
        //savety
        targetVoltage = min(targetVoltage, min((float)VOLTAGE_OUTPUT_MAX, inputVoltage));
        dutyCycle = round( (float)pwmMaxDuty * targetVoltage / inputVoltage);
        if(targetVoltage!=*currentVoltage)
        {
            *currentVoltage = targetVoltage;
            ledcWrite(chA, 0);
            ledcWrite(chB, dutyCycle);
        }
       
    }
    else if (targetVoltage < 0)
    {
        //savety
        targetVoltage = max(targetVoltage, -min((float)VOLTAGE_OUTPUT_MAX, inputVoltage));
        dutyCycle = round( (float)pwmMaxDuty * (-targetVoltage / inputVoltage));
        if( *currentVoltage != targetVoltage)
        {
            *currentVoltage = targetVoltage;
            ledcWrite(chA, dutyCycle);
            ledcWrite(chB, 0);
        }
    }
    else
    {
        dutyCycle =0;
        if(*currentVoltage != targetVoltage)
        {
            *currentVoltage=targetVoltage;
            ledcWrite(chA, 0);
            ledcWrite(chB, 0);
        }
    }
    
    return dutyCycle;
}



void voltageShutdown()
{
    dutyCyleSRVO = setOutputVoltage(PWM_CH_SRVO_A, PWM_CH_SRVO_B, 1, 0, &voltageSRVO);
    dutyCyleENGN = setOutputVoltage(PWM_CH_ENGN_A, PWM_CH_ENGN_B, 1, 0, &voltageENGN);
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
    if(voltageMeasured<0.1)
    {
        inputVoltageError = true;
        return;
    }

    inputVoltageError=false;
    
    if (voltageInput < numberOfLipoCells * VOLTAGE_LIPO_LOW)
    {
        lowVoltage = true;
    }

    if (voltageInput < numberOfLipoCells * VOLTAGE_LIPO_LOW_CRITICAL)
    {
        lowVoltageCritical =true;
    }


    //set voltage engine
    if (boostEngaged)
        dutyCyleENGN = setOutputVoltage(PWM_CH_ENGN_A, PWM_CH_ENGN_B, voltageInput, VOLTAGE_OUTPUT_MAX, &voltageENGN);
    else
        dutyCyleENGN = setOutputVoltage(PWM_CH_ENGN_A, PWM_CH_ENGN_B, voltageInput,  VOLTAGE_OUTPUT_DEFAULT * ENGNInput, &voltageENGN);

        // set voltage servo
    dutyCyleSRVO = setOutputVoltage(PWM_CH_SRVO_A, PWM_CH_SRVO_B,VOLTAGE_OUTPUT_DEFAULT, VOLTAGE_OUTPUT_DEFAULT * SRVOInput,&voltageSRVO);
    
    setOutput(PWM_CH_LED_A,lightAenabled);
    setOutput(PWM_CH_LED_B,lightBenabled);
}

void voltageSetup()
{
    pwmMaxDuty = pow(2, PWM_RESOL) - 1;
    bluetoothSendMessage("Setting PWM to resolution " + String(pwmMaxDuty) + " ... Measuring Voltage: ");
    getInputVoltage();
    bluetoothSendMessage(String(voltageInput, 1) + " V, please stand by");
    delay(500);
    getInputVoltage();
    debugString="Voltage: " + String(voltageInput, 1) + " V, done  ";
    while (voltageInput > numberOfLipoCells * VOLTAGE_LIPO_FULL)
        numberOfLipoCells++;
    debugString+="Assuming a " + String(numberOfLipoCells) + " cell lipo Battery , setting pwms";
    delay(500);

    setupPWM(PWM_PIN_LED_A, PWM_CH_LED_A, PWM_FREQ_LED);
    setupPWM(PWM_PIN_LED_B, PWM_CH_LED_B, PWM_FREQ_LED);

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
    debugString+="\nSHUTDOWN initiated";
    updateTicker.detach();
    voltageShutdown();
    bluetoothShutdown();
    esp_deep_sleep_start();
}


uint16_t boostTimer = BOOST_TIME_MAX;

void manageBoost()
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



void manageLights()
{
    static uint8_t ledCycle = 0;
    if (++ledCycle*UPDATE_TIME>1000) ledCycle = 0;

    if(ledCycle==0 && BTconnected) digitalWrite(BUILDIN_LED,HIGH);
    else if(BTreceived) digitalWrite(BUILDIN_LED,HIGH);
    else digitalWrite(BUILDIN_LED,LOW);
    static uint8_t lightMode =0;
    
    if(lightsInput)
    {
        if(++lightMode>3)
        lightMode=0; 
        switch (lightMode)
        {
        case 1:
            lightAenabled = true;
            lightBenabled = true;
            break;
        case 2:
            lightAenabled = true;
            lightBenabled = false;
            break;
        case 3:
            lightAenabled = false;
            lightBenabled = true;
            break;
        default:
            lightAenabled = false;
            lightBenabled = false;
            break;
        }
        lightsInput=false;
    }
}



void update()
{
    systemTime += UPDATE_TIME;
    if(lowVoltageCritical)
    {
        debugString = "WARNING: VOLTAGE CRITICAL";

        if( SHUTDOWN_LOW_VOLTAGE_ALLOWED)
        {        
            shutdown = true;
            debugString += " - shutdown imminent";
        }
        else
        {
            debugString += " - shut down system and recharge battery to prevent permanent battery damage and explosions";
        }
    }

    if (inputVoltageError)
    {
        debugString +="\n WARNING: Input Voltage Error";
        /* code */
    }
    


    if (shutdown)
    {
        shutDown();
        bluetoothSendMessage(debugString);
        return;
    }
    manageBoost();
    manageLights();
    bluetoothSendMessage(debugString);
    bluetoothSendMessage(
         "|T| " + String((float)systemTime / 1000, 2) 
        + " |V/~| IN " + String(voltageInput, 1) 
        + " /"+String(pwmMaxDuty)
        + " | <> " + String(voltageSRVO, 1) 
        + " /" + String(dutyCyleSRVO)
        + " | EN " + String(voltageENGN, 1) 
        + " /"+String(dutyCyleENGN)
        + " || BST" + (boostEngaged ? ">>" : "--") 
        +String(boostTimer)
        + " | LGT "+(lightAenabled?"A":"-")+(lightBenabled?"B":"-")
        + (shutdown ? " || SHUTDOWN" : "")
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