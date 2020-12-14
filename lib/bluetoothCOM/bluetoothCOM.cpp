
// █▀▀▄ █    █  █ █▀▀▀ ▀█▀▀ ▄▀▀▄ ▄▀▀▄ ▀█▀▀ █  █
// █▀▀▄ █    █  █ █■■   █   █  █ █  █  █   █■■█
// █▄▄▀ █▄▄▄ ▀▄▄▀ █▄▄▄  █   ▀▄▄▀ ▀▄▄▀  █   █  █

#include "BluetoothSerial.h"
#include <Ticker.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
Ticker bluetoothTicker;
const String bluetoothName = "THE TUMBLER";

float *speed; 
float *steer;
 bool *boost; 
 bool *lights;

void processMessage(String *message)
{
    static bool boostPressed = false;
    static bool lightPressed = false;
    if (message->startsWith("sp "))
    {
        message->remove(0, 3);
        *speed = constrain(message->toFloat(),-1,1);
        //SerialBT.print("speed set " + String(*speed) + ".\n");
    }
    else if (message->startsWith("lr "))
    {
        message->remove(0, 3);
        *steer = constrain(message->toFloat(),-1,1);
        //SerialBT.print("steer set " + String(*steer) + ".\n");
    }
    else if (message->startsWith("inp "))
    {
        message->remove(0, 4);
        int input = message->toInt();
        if (input < 6) // button slider is pushed on the boost side
        {
            if (!boostPressed)
            {
                boostPressed = true;
                *boost = true;
                SerialBT.print("Boost pressed\n");
            }
            lightPressed = false;
        }
        else if (input > 6) // button slider is pushed on the lights side
        {
            boostPressed = false;
            if (*boost)
            {
                SerialBT.print("Boost released\n");
                *boost = false;
            }
            if (!lightPressed)
            {
                lightPressed = true;
                *lights = true;
                SerialBT.print("lights pressed\n");
            }
        }
        else // button slider is released
        {
            if (*boost)
            {
                SerialBT.print("Boost released\n");
                *boost = false;
            }
            boostPressed = false;
            lightPressed = false;
        }
    }
    else
    {
        SerialBT.print("unknown command: " + *message + ".\n");
    }
    *message = "";
}

void bluetoothSendMessage(String message)
{
    SerialBT.print("msg "+message+"\n");
}

void bluetoothSendPlot(float value)
{
    SerialBT.print("plt "+String(value,2)+"\n");
}

void bluetoothUpdate()
{
    if (Serial.available())
        SerialBT.write(Serial.read());

    if (SerialBT.available())
    {
        String Message = "";
        char c;
        while (SerialBT.available())
        {
            c = SerialBT.read(); //read a letter
            if (c != '\n')
                Message += c; //add the letter
            else
                processMessage(&Message);
        }
        if (Message != "")
        {
            SerialBT.print("leftovers: " + Message + ".\n");
        }
    }
}

void bluetoothSetup(uint16_t inputUpdateTime, float *speedInput, float *steerInput, bool *boostInput, bool *lightsInput)
{
    speed = speedInput;
    steer = steerInput;
    boost = boostInput;
    lights = lightsInput;
    SerialBT.begin(bluetoothName); //Bluetooth device name
    Serial.println("bluetooth started, connect to: " + bluetoothName);
    bluetoothTicker.attach_ms(inputUpdateTime, bluetoothUpdate);
}

void bluetoothShutdown()
{
    Serial.println("closing bluetooth connection\n ... good bye" + bluetoothName);
    bluetoothTicker.detach();
    delay(100);
    SerialBT.end();
}
