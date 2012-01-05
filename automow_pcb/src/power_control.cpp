#include <WProgram.h>
#include <ros.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <automow_pcb/Status.h>
#include <automow_pcb/CutterControl.h>

#define CUTTER_L_CONTROL 4
#define CUTTER_R_CONTROL 5
#define CUTTER_L_CHECK 2
#define CUTTER_R_CHECK 3

#define PIN_TEMP_TOP 9
#define PIN_TEMP_BOT 10

#define PIN_VOLTAGE 0
#define PIN_CURRENT 1

#define LED_1 6
#define LED_2 7
#define LED_3 8

OneWire oneWireTop(PIN_TEMP_TOP);  
OneWire oneWireBot(PIN_TEMP_BOT);
DallasTemperature tempTop(&oneWireTop);
DallasTemperature tempBot(&oneWireBot);
DeviceAddress topAddress;
DeviceAddress botAddress;

ros::NodeHandle nh;
automow_pcb::Status status_msg;

ros::Publisher status_pub("power_status", &status_msg);

void cb_cutterControl(const automow_pcb::CutterControl& msg)
{
    digitalWrite(CUTTER_L_CONTROL, msg.cutter_1);
    digitalWrite(CUTTER_R_CONTROL, msg.cutter_2);
}

void setup()
{
    pinMode(13, OUTPUT);
    pinMode(PIN_VOLTAGE, INPUT);
    pinMode(PIN_CURRENT, INPUT);
    pinMode(CUTTER_L_CONTROL, OUTPUT);
    pinMode(CUTTER_R_CONTROL, OUTPUT);
    pinMode(CUTTER_L_CHECK, INPUT);
    pinMode(CUTTER_R_CHECK, INPUT);
    digitalWrite(CUTTER_L_CONTROL, LOW);
    digitalWrite(CUTTER_R_CONTROL, LOW);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);

    nh.initNode(); 
    nh.advertise(status_pub);

    tempTop.begin();
    tempBot.begin();

    if (!tempTop.getAddress(topAddress,0))
    {
        nh.logwarn("No Temperature Sensor Detected") 
    } else {
        tempTop.setResolution(topAddress, 9);
        tempTop.setWaitForConversion(false);
        tempTop.requestTemperatures();
    }
    if (!tempBot.getAddress(botAddress,0))
    {
        nh.logwarn("No Temperature Sensor Detected")
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
    float ADCVolts = 0.030024 * analogRead(PIN_VOLTAGE) + 0.027055;
    float ADCA
    status_pub.publish( &status_msg );
    nh.spinOnce();

    status_msg.cutter_1 = (digitalRead(CUTTER_L_CHECK) ? FALSE : TRUE);
    status_msg.cutter_2 = (digitalRead(CUTTER_R_CHECK) ? FALSE : TRUE);



}
