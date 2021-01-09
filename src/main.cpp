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
bool INPUTboost = false;
bool boostEngaged = false;
bool INPUTlightToggleA = false;
bool INPUTlightToggleB = false;
bool INPUTlightToggleC = false;
bool lightEnabledA = true;
bool lightEnabledB = true;
bool lightEnabledC = true;


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
#define RESISTOR_GND 10000
#define RESISTOR_VIN 100000

// PIN ADC in
#define VOLTAGE_READ_PIN 35 //---> res A & B

// PINs PWM motors to 7881
#define PIN_PWM_ENG_1 25 
#define PIN_PWM_ENG_2 26 

// PINs PWM servo to 7881
#define PIN_PWM_SRV_1 32 
#define PIN_PWM_SRV_2 33 

// PINs PWM LED to input 8833
#define PIN_PWM_LED_BOOST 5 
#define PIN_PWM_LED_A 18 
#define PIN_PWM_LED_B 19 
#define PIN_PWM_LED_C 21 

#define CH_PWM_LED_A 1
#define CH_PWM_LED_B 2
#define CH_PWM_LED_C 3
#define CH_PWM_LED_BOOST 4

#define CH_PWM_SRVO_A 5
#define CH_PWM_SRVO_B 6
#define CH_PWM_ENGN_A 7
#define CH_PWM_ENGN_B 8

// Setting PWM properties
#define PWM_FREQ_ENGN 2000
#define PWM_FREQ_SRVO 2000
#define PWM_FREQ_LED 2000
#define PWM_RESOL 8

#define LED_MODE_OFF 0
#define LED_MODE_MANUAL 1
#define LED_MODE_NO_CONNECTION 2
#define LED_MODE_LOW_BATTERY 3

uint8_t LEDmode = 1;

int16_t pwmMaxDuty;
uint16_t dutyCycleLED;
uint16_t dutyCyleSRVO;
uint16_t dutyCyleENGN;

Ticker voltageTicker;
uint8_t voltageUpdateTime = 50;

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
    voltageInput = voltageMeasured * (RESISTOR_GND + RESISTOR_VIN) / RESISTOR_GND;
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
    dutyCyleSRVO = setOutputVoltage(CH_PWM_SRVO_A, CH_PWM_SRVO_B, 1, 0, &voltageSRVO);
    dutyCyleENGN = setOutputVoltage(CH_PWM_ENGN_A, CH_PWM_ENGN_B, 1, 0, &voltageENGN);
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
        dutyCyleENGN = setOutputVoltage(CH_PWM_ENGN_A, CH_PWM_ENGN_B, voltageInput, VOLTAGE_OUTPUT_MAX, &voltageENGN);
    else
        dutyCyleENGN = setOutputVoltage(CH_PWM_ENGN_A, CH_PWM_ENGN_B, voltageInput,  VOLTAGE_OUTPUT_DEFAULT * ENGNInput, &voltageENGN);

        // set voltage servo
    dutyCyleSRVO = setOutputVoltage(CH_PWM_SRVO_A, CH_PWM_SRVO_B,VOLTAGE_OUTPUT_DEFAULT, VOLTAGE_OUTPUT_DEFAULT * SRVOInput,&voltageSRVO);
    
    setOutput(CH_PWM_LED_BOOST,boostEngaged);
    setOutput(CH_PWM_LED_A,lightEnabledA);
    setOutput(CH_PWM_LED_B,lightEnabledB);
    setOutput(CH_PWM_LED_C,lightEnabledC);

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

    setupPWM(PIN_PWM_LED_A, CH_PWM_LED_A, PWM_FREQ_LED);
    setupPWM(PIN_PWM_LED_B, CH_PWM_LED_B, PWM_FREQ_LED);
    setupPWM(PIN_PWM_LED_C, CH_PWM_LED_C, PWM_FREQ_LED);
    setupPWM(PIN_PWM_LED_BOOST, CH_PWM_LED_BOOST, PWM_FREQ_LED);

    setupPWM(PIN_PWM_SRV_1, CH_PWM_SRVO_A, PWM_FREQ_SRVO);
    setupPWM(PIN_PWM_SRV_2, CH_PWM_SRVO_B, PWM_FREQ_SRVO);

    setupPWM(PIN_PWM_ENG_1, CH_PWM_ENGN_A, PWM_FREQ_ENGN);
    setupPWM(PIN_PWM_ENG_2, CH_PWM_ENGN_B, PWM_FREQ_ENGN);

    voltageTicker.attach_ms(voltageUpdateTime, voltageUpdate);

    //test Led

    setOutput(CH_PWM_LED_A,true);
    delay(500);
    setOutput(CH_PWM_LED_B,true);
    delay(500);
    setOutput(CH_PWM_LED_C,true);
    delay(500);
    setOutput(CH_PWM_LED_BOOST,true);
    delay(3000);
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
        if (INPUTboost)
        {
            if (boostTimer < updateTime)
            {
                boostTimer = 0;
                boostEngaged = false;
                INPUTboost = false;
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

    boostEngaged = (INPUTboost && boostTimer > 0);
}



void manageLights()
{
    static uint8_t ledCycle = 0;
    if (++ledCycle*UPDATE_TIME>1000) ledCycle = 0;

    if(ledCycle==0 && BTconnected) digitalWrite(BUILDIN_LED,HIGH);
    else if(BTreceived) digitalWrite(BUILDIN_LED,HIGH);
    else digitalWrite(BUILDIN_LED,LOW);
    
    if(INPUTlightToggleA)
    {
        lightEnabledA = !lightEnabledA;
        INPUTlightToggleA=false;
    } 
    else if(INPUTlightToggleB)
    {
        lightEnabledB = !lightEnabledB;
        INPUTlightToggleB=false;
    } 
    else if(INPUTlightToggleC)
    {
        lightEnabledC = !lightEnabledC;
        INPUTlightToggleC=false;
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
        + " | (≡ "+(lightEnabledA?"A":"-")+(lightEnabledB?"B":"-")+(lightEnabledC?"C":"-")
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
    bluetoothSetup(INPUT_UPDATE_TIME, &ENGNInput, &SRVOInput, &INPUTboost, &INPUTlightToggleA, &INPUTlightToggleB, &INPUTlightToggleC);
    voltageSetup();
    updateTicker.attach_ms(UPDATE_TIME, update);
}

void loop()
{
    vTaskDelete(NULL);
}