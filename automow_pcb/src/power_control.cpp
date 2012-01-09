#include <WProgram.h>
#include <avr/pgmspace.h>
#include <ros.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Metro.h>

#include <automow_node/Automow_PCB.h>
#include <automow_node/Cutters.h>

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

ros::NodeHandle nh;

automow_node::Automow_PCB msgStatus;
using automow_node::Cutters;

enum BatteryState { DISCHARGING = 0,
					CHARGING_RECOVERY = 1,
					CHARGING = 2,
					TRICKLE_CHARGING = 3,
					DISCHARGING_CRITICAL = 4,
					ERROR = 5};

bool ledState = LOW;
bool leftCutterState = LOW;
bool rightCutterState = LOW;
uint8_t batteryState = ERROR;
uint8_t stateOfCharge = 100;
int16_t prev_current = 512;
uint16_t prev_voltage = 0;

void cb_cutters(const Cutters::Request &req, Cutters::Response &res)
{
	digitalWrite(pin_leftCutterControl, req.cutter_1);
	digitalWrite(pin_rightCutterControl, req.cutter_2);
	leftCutterState = req.cutter_1;
	rightCutterState = req.cutter_2;
	delay(100);
	res.cutter_1 = (digitalRead(pin_leftCutterCheck) ? FALSE : TRUE);
	res.cutter_2 = (digitalRead(pin_rightCutterCheck) ? FALSE : TRUE);
}


ros::Publisher status_pub("/automow_pcb/status", &msgStatus);
ros::ServiceServer<Cutters::Request, Cutters::Response> cutter_srv("cutters", &cb_cutters);

OneWire oneWireTop(pin_temperatureTop);
OneWire oneWireBot(pin_temperatureBot);
DallasTemperature temperatureTop(&oneWireTop);
DallasTemperature temperatureBot(&oneWireBot);
DeviceAddress topAddress;
DeviceAddress botAddress;

void updateBatteryDisplay(void)
{
	if(ledState == HIGH)
		ledState = LOW;
	else
		ledState = HIGH;
   
    switch(batteryState) {
	case ERROR:
		digitalWrite(pin_ledHigh, LOW);
		digitalWrite(pin_ledMid, LOW);
		digitalWrite(pin_ledLow, LOW);
		break;
	case CHARGING:
		digitalWrite(pin_ledHigh, ledState);
		digitalWrite(pin_ledMid, ledState);
		digitalWrite(pin_ledLow, LOW);
		break;
	case TRICKLE_CHARGING:
		digitalWrite(pin_ledHigh, ledState);
		digitalWrite(pin_ledMid, LOW);
		digitalWrite(pin_ledLow, LOW);
		break;
	case DISCHARGING:
        bool a,b,c;
        switch(stateOfCharge/20){
		case 5:
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
		case 0:
		   a=0;b=0;c=1;
		   break;
        }
        digitalWrite(pin_ledHigh, a);
        digitalWrite(pin_ledMid, b);
		digitalWrite(pin_ledLow, c);
		break;
	case DISCHARGING_CRITICAL:
		digitalWrite(pin_ledHigh, LOW);
		digitalWrite(pin_ledMid, LOW);
		digitalWrite(pin_ledLow, ledState);
		break;
	default:
		break;
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

	// Initialize the rear panel LED outputs
    pinMode(pin_ledHigh,OUTPUT);
    pinMode(pin_ledMid,OUTPUT);
	pinMode(pin_ledLow,OUTPUT);
	digitalWrite(pin_ledHigh, LOW);
	digitalWrite(pin_ledMid, LOW);
	digitalWrite(pin_ledLow, LOW);
	
	temperatureTop.begin();
    temperatureBot.begin();
    
    // Make sure we have temperature sensors, if not, set to something
    // unreasonable. This would be 0 in Alabama.
    if(!temperatureTop.getAddress(topAddress,0))
    {
        msgStatus.temperature_1 = 0.0;
    } else {
        temperatureTop.setResolution(topAddress,9);
        temperatureTop.setWaitForConversion(false);
        temperatureTop.requestTemperatures();
    }
    if(!temperatureBot.getAddress(botAddress,0))
    {
        msgStatus.temperature_2 = 0.0;
    } else {
        temperatureBot.setResolution(botAddress,9);
        temperatureBot.setWaitForConversion(false);
        temperatureBot.requestTemperatures();
    }
    nh.initNode(); 
	nh.advertise(status_pub);
	nh.advertiseService(cutter_srv);
}

void loop()
{	
	msgStatus.voltage = (12 * analogRead(pin_voltage) + 4*prev_voltage)/16;
	prev_voltage = msgStatus.voltage;
	msgStatus.voltage *= 30;
	msgStatus.current = (12 * analogRead(pin_current) + 4*prev_current)/16;
	prev_current = msgStatus.current;
	msgStatus.current = 333*(msgStatus.current - 501);
    
	if(abs(msgStatus.current) <= 1000 )
	{
		// Current is less than 1 amp in either direction,
		// Batteries are (probably?) disconnected.
		batteryState = ERROR;
	}
    else if(msgStatus.voltage < 20000)
    {
        batteryState = ERROR;
        stateOfCharge = 0;
    }
	else if(msgStatus.current > 0)
	{
		// If current is positive, we are charging
		stateOfCharge = 100;
		if (msgStatus.current < 3000)
			batteryState = TRICKLE_CHARGING;
		else
			batteryState = CHARGING;
	} else {
		// Otherwise, we are discharging.
		stateOfCharge = msgStatus.voltage/25 - 900;
		if (stateOfCharge > 100)
			stateOfCharge = 100;
		if (stateOfCharge < 20)
			batteryState = DISCHARGING_CRITICAL;
		else
			batteryState = DISCHARGING;
	}
	msgStatus.battery_state = batteryState;
	msgStatus.charge = stateOfCharge;
	updateBatteryDisplay();
	
	msgStatus.temperature_1 = temperatureTop.getTempCByIndex(0);
	msgStatus.temperature_2 = temperatureBot.getTempCByIndex(0);
	temperatureTop.requestTemperatures();
	temperatureBot.requestTemperatures();
	
	msgStatus.cutter_1 = (digitalRead(pin_leftCutterCheck) ? FALSE : TRUE);
	msgStatus.cutter_2 = (digitalRead(pin_rightCutterCheck) ? FALSE : TRUE);
	
	if(leftCutterState && !msgStatus.cutter_1)
	{
		digitalWrite(pin_leftCutterControl, LOW);
		leftCutterState = LOW;
		msgStatus.cutter_1 = FALSE;
	}
	if(rightCutterState && !msgStatus.cutter_2)
	{
		digitalWrite(pin_rightCutterControl, LOW);
		rightCutterState = LOW;
		msgStatus.cutter_2 = FALSE;
	}
	
	status_pub.publish( &msgStatus );
	nh.spinOnce();
	delay(400);
}
