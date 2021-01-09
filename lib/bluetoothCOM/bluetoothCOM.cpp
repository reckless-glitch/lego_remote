
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
 bool *togglelightA;
 bool *togglelightB;
 bool *togglelightC;
 bool BTreceived = false;
 bool BTconnected =false;

void processMessage(String *message)
{
    static bool boostPressed = false;
    static bool inputPressed = false;
    if (message->startsWith("sp "))
    {
        message->remove(0, 3);
        *speed = message->toFloat();
        //SerialBT.print("speed set " + String(*speed) + ".\n");
        if(!boostPressed && *speed>1.5f)
        {
            boostPressed = true;
            *boost = true;
            SerialBT.print("Boost pressed\n");
        }
        else if (boostPressed && *speed<1.5f)
        {
            boostPressed = false;
            *boost = false;
            SerialBT.print("Boost released\n");
        }
        *speed = constrain(*speed,-1,1);
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
    
        if (input < 30) // button slider is pushed on the left side
        {
            if(!inputPressed)
            {
                inputPressed=true;
                *togglelightA=true;
            }
        }
        else if (input > 35 && input < 60) // button slider is pushed on the center
        {
            if(!inputPressed)
            {
                 inputPressed=true;
                 *togglelightB=true;
            }
        }
        else if (input > 70)
        {
            if(!inputPressed)
            {
                 inputPressed=true;
                 *togglelightC=true;
            }
        } 
        else // button slider is released
        {
            inputPressed = false;
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
    BTreceived = SerialBT.available();
    BTconnected = SerialBT.connected();
    if (BTreceived)
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

void bluetoothSetup(uint16_t inputUpdateTime, float *speedInput, float *steerInput, bool *boostInput, bool *toggleA, bool *toggleB, bool *toggleC)
{
    speed = speedInput;
    steer = steerInput;
    boost = boostInput;
    togglelightA = toggleA;
    togglelightB = toggleB;
    togglelightC = toggleC;
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
