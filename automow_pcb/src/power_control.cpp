#include <WProgram.h>
#include <ros.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Metro.h>

#include <automow_pcb/Status.h>
#include <automow_pcb/CutterControl.h>

// Digital IO
#define pin_leftCutterCheck         2
#define pin_rightCutterCheck        3
#define pin_leftCutterControl       4
#define pin_rightCutterControl      5
#define pin_ledHigh                 6
#define pin_ledMid                  7
#define pin_ledLow                  8
// One-Wire Bus Pins
#define pin_temperatureTop          9
#define pin_temperatureBot          10
// Analog Inputs
#define pin_voltage                 0
#define pin_current                 1

char batteryState = 0;
#define BS_CHARGING 1
#define BS_DISCHARGING 2
#define BS_CRITICAL 3

char stateOfCharge;

bool ledState = LOW;
bool cutterLeftState;
bool cutterRightState;

OneWire oneWireTop(PIN_TEMP_TOP);  
OneWire oneWireBot(PIN_TEMP_BOT);
DallasTemperature tempTop(&oneWireTop);
DallasTemperature tempBot(&oneWireBot);
DeviceAddress topAddress;
DeviceAddress botAddress;

Metro ledMetro = Metro(500);
Metro msgMetro = Metro(250);

ros::NodeHandle nh;

automow_pcb::Status status_msg;
ros::Publisher status_pub("power_status", &status_msg);

ros::Subscriber<automow_pcb::CutterControl> cutter_sub("cutter_control", 
                                                        &cb_cutterControl)


void cb_cutterControl(const automow_pcb::CutterControl& msg)
{
    digitalWrite(pin_leftCutterControl,msg.cutter_1);
    digitalWrite(pin_rightCutterControl,msg.cutter_2);
    cutterLeftState = HIGH;
    cutterRightState = HIGH;
}

void updateBatteryDisplay(void)
{
    if(ledState == HIGH)
        ledState = LOW;
    else
        ledState = HIGH;
   
    switch(batteryState)
    {
    case BS_CHARGING:
        digitalWrite(pin_ledHigh,ledState);
        digitalWrite(pin_ledMid,LOW);
        digitalWrite(pin_ledLow,LOW);
        break;
    case BS_CRITICAL:
        digitalWrite(pin_ledHigh,LOW);
        digitalWrite(pin_ledMid,LOW);
        digitalWrite(pin_ledLow,ledState);
        break;
    default:
        unsigned char temp = stateOfCharge/20;
        bool a,b,c;
        switch(temp){
            case 5:
               a=1;b=0;c=0;
               break;
            case 4:
               a=1;b=0;c=0;
               break;
            case 3:
               a=1;b=1;c=0;
               break;
            case 2:
               a=0;b=1;c=0;
               break;
            case 1:
               a=0;b=1;c=1;
               break;
        }
        digitalWrite(pin_ledHigh,a);
        digitalWrite(pin_ledMid,b);
    }
}

void setup()
{
    pinMode(13, OUTPUT);

    // Set up all of the Digital IO pins.
    pinMode(pin_leftCutterCheck,INPUT);
    pinMode(pin_rightCutterCheck,INPUT);
    pinMode(pin_leftCutterControl,OUTPUT);
    pinMode(pin_rightCutterControl,OUTPUT);
    // Turn off the cutters by default
    digitalWrite(pin_leftCutterControl,LOW);
    digitalWrite(pin_rightCutterControl,LOW);
    pinMode(pin_ledHigh,OUTPUT);
    pinMode(pin_ledMid,OUTPUT);
    pinMode(pin_ledLow,OUTPUT);

    nh.initNode(); 
    nh.advertise(status_pub);
    nh.subscribe(cutter_sub);

    tempTop.begin();
    tempBot.begin();

    if (!tempTop.getAddress(topAddress,0))
    {
        nh.logwarn("No Temperature Sensor Detected");
        status_msg.temperature_1 = 0;
    } else {
        tempTop.setResolution(topAddress, 9);
        tempTop.setWaitForConversion(false);
        tempTop.requestTemperatures();
    }
    if (!tempBot.getAddress(botAddress,0))
    {
        nh.logwarn("No Temperature Sensor Detected");
        status_msg.temperature_2 = 0;
    } else {
        tempBot.setResolution(botAddress, 9);
        tempBot.setWaitForConversion(false);
        tempBot.requestTemperatures();
    }

}


void loop()
{
    status_msg.voltage = analogRead(PIN_VOLTAGE);
    status_msg.current = analogRead(PIN_CURRENT);

    float ADCVolts = 0.03 * status_msg.voltage + 0.027;
    if (ADCVolts > 25.5)
    {
        batteryState = BS_CHARGING;
        stateOfCharge = 100; 
    }   
    else if(ADCVolts < 23)
    {
        batteryState = BS_CRITICAL;
        stateOfCharge = (char)(ADCVolts  * 40 - 900); 
    }
    else
    {
        batteryState = BS_DISCHARGING;
        stateOfCharge = (char)(ADCVolts * 40 - 900); 
    }

    status_msg.cutter_1 = (digitalRead(CUTTER_L_CHECK) ? FALSE : TRUE);
    status_msg.cutter_2 = (digitalRead(CUTTER_R_CHECK) ? FALSE : TRUE);

    if (ledMetro.check() == 1)
    {
        if (cutterLeftState && !status_msg.cutter_1)
        {
            status_msg.cutter_1 = FALSE;
            cutterLeftState = LOW;
            digitalWrite(pin_leftCutterControl, LOW);
            nh.logerror("Estop Left Cutter");
        }
        if (cutterRightState && !status_msg.cutter_2)
        {
            status_msg.cutter_2 = FALSE;
            cutterRightState = LOW;
            digitalWrite(pin_rightCutterControl, LOW);
            nh.logerror("Estop Right Cutter");
        }

        updateBatteryDisplay();
    }


    if (msgMetro.check() == 1)
    {
        status_msg.temperature_1 = tempTop.getTempCByIndex(0);
        status_msg.temperature_2 = tempBot.getTempCByIndex(0);
        tempTop.requestTemperatures();
        tempBot.requestTemperatures();
        status_pub.publish( &status_msg );
    }
    nh.spinOnce();
}
