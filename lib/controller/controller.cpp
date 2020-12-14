// ▄▀▀▄ ▄▀▀▄ █▄ █ ▀█▀▀ █▀▀▄ ▄▀▀▄ █    █    █▀▀▀ █▀▀▄
// █    █  █ █▀▄█  █   █▄▄▀ █  █ █    █    █■■  █▄▄▀
// ▀▄▄▀ ▀▄▄▀ █ ▀█  █   █ ▀▄ ▀▄▄▀ █▄▄▄ █▄▄▄ █▄▄▄ █ ▀▄

#include <PS4Controller.h>
#include <Ticker.h>
#define MAC_ADRESS "11:11:11:11:11:11" //empty
//# define MAC_ADRESS "f0:6e:0b:de:01:01" //hyperglitch
//# define MAC_ADRESS "b8:27:eb:b7:c2:f6")) //apparat

/*
char macs[][18] = // 17 is the length of the MAC + 1 ( for the '\0' at the end )
{
    "f0:6e:0b:de:01:01" //hyperglitch
    ,"b8:27:eb:b7:c2:f6" //apparat
    //more macs here:
    //,":::::"
};
*/

Ticker controllerTicker;

uint8_t controllerLED[3] = {255, 255, 0};
uint16_t controllerFlash[2] = {100, 400}; //ms
uint8_t controllerRumble = 0;

void onControllerEvent()
{
    if (!PS4.isConnected())
    {
        Serial.print("no PS4 controller connected");
    }
    return;


    if (PS4.event.button_down.l2)
    {
        Serial.print("l2 button at ");
        Serial.println(PS4.data.analog.button.l2, DEC);
    }
    if (PS4.data.button.r2)
    {
        Serial.print("r2 button at ");
        Serial.println(PS4.data.analog.button.r2, DEC);
    }

    if (PS4.event.analog_move.stick.lx)
    {
        Serial.print("Left Stick x at ");
        Serial.println(PS4.data.analog.stick.lx, DEC);
    }
    if (PS4.event.analog_move.stick.ly)
    {
        Serial.print("Left Stick y at ");
        Serial.println(PS4.data.analog.stick.ly, DEC);
    }
    if (PS4.event.analog_move.stick.rx)
    {
        Serial.print("Right Stick x at ");
        Serial.println(PS4.data.analog.stick.rx, DEC);
    }
    if (PS4.event.analog_move.stick.ry)
    {
        Serial.print("Right Stick y at ");
        Serial.println(PS4.data.analog.stick.ry, DEC);
    }
    Serial.println("-endOfEvent");
}

void onConnection()
{

    if (PS4.isConnected())
    {
        Serial.println("Controller connected.");
        Serial.print("Battey Percent : ");
        Serial.println(PS4.data.status.battery, DEC);
        PS4.setLed(255, 255, 0);
        PS4.output.flashOn = true;
    }
}

void controllerSetLED(uint8_t r, uint8_t g, uint8_t b, uint16_t on = 0, uint16_t off = 0)
{
    controllerLED[0] = r;
    controllerLED[1] = g;
    controllerLED[2] = b;
    controllerFlash[0] = on;
    controllerFlash[1] = off;
}

void controllerUpdate()
{
    if (PS4.isConnected())
    {
        // color of front light
        PS4.setLed(controllerLED[0], controllerLED[1], controllerLED[2]);

        // front light flashes, Params: on in ms, off in ms, Range: 0->2550
        // Set to 0,0 for the light to remain on
        PS4.setFlashRate(controllerFlash[0], controllerFlash[1]);

        // rumble of the controllers,
        // Params: Weak rumble intensity, Strong rumble intensity, Range: 0->255
        PS4.setRumble(controllerRumble, 0);

        // Sends data set in the above three instructions to the controller
        PS4.sendToController();
        // Don't send data to the controller immediately, will cause buffer overflow
    }
    Serial.print("waiting for controller");
}

bool controllerSetup(uint16_t inputUpdateTime)
{
    if (PS4.begin(MAC_ADRESS))
    {
        PS4.attach(onControllerEvent);
        PS4.attachOnConnect(onConnection);
        Serial.print("waiting for controller with mac ");
        Serial.println(MAC_ADRESS);
        controllerTicker.attach_ms(inputUpdateTime, controllerUpdate);
        controllerSetLED(255, 255, 0);
        return true;
    }

    /*
    Serial.println("searching controller by MAC");
    for (uint8_t i = 0; i < sizeof(macs); i++)
        if (PS4.begin(macs[i]))
        {
            Serial.println("found MAC:" + String(macs[i]));
            PS4.attach(onControllerEvent);
            PS4.attachOnConnect(onConnection);
            return true;
        }
    */
    Serial.print("MAC adress error: ");
    Serial.println(MAC_ADRESS);
    return false;
}
