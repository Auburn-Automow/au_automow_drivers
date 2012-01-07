#include <WProgram.h>
#include <ros.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Metro.h>

#include <automow_node/BatteryStatus.h>
#include <automow_node/Cutters.h>
#include <automow_node/Temperatures.h>

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

automow_node::BatteryStatus msgBatteryStatus;

enum BatteryState { DISCHARGING = 0,
					CHARGING_RECOVERY,
					CHARGING,
					TRICKLE_CHARGING,
					DISCHARGING_CRITICAL,
					ERROR};

bool ledState = LOW;
uint8_t batteryState = ERROR;
uint8_t stateOfCharge = 100;

ros::Publisher battery_pub("battery_status", &msgBatteryStatus);

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
	
    nh.initNode(); 
	nh.advertise(battery_pub);
}

int16_t prev_current = 512;
uint16_t prev_voltage = 0;

void loop()
{	
	msgBatteryStatus.voltage = (12 * analogRead(pin_voltage) + 4*prev_voltage)/16;
	prev_voltage = msgBatteryStatus.voltage;
	msgBatteryStatus.voltage *= 30;
	msgBatteryStatus.current = (12 * analogRead(pin_current) + 4*prev_current)/16;
	prev_current = msgBatteryStatus.current;
	msgBatteryStatus.current = 333*(msgBatteryStatus.current - 502);
    
	if(abs(msgBatteryStatus.current) <= 1000 || msgBatteryStatus.voltage < 20)
	{
		// Current is less than 1 amp in either direction,
		// Batteries are disconnected.
		batteryState = ERROR;
		stateOfCharge = 0;
	}
	else if(msgBatteryStatus.current > 0)
	{
		// If current is positive, we are charging
		stateOfCharge = 100;
		if (msgBatteryStatus.current < 3000)
			batteryState = TRICKLE_CHARGING;
		else
			batteryState = CHARGING;
	}
	else
	{
		// Otherwise, we are discharging.
		stateOfCharge = msgBatteryStatus.voltage/25 - 900;
		if (stateOfCharge > 100)
			stateOfCharge = 100;
		if (stateOfCharge < 20)
			batteryState = DISCHARGING_CRITICAL;
		else
			batteryState = DISCHARGING;
	}
	
	msgBatteryStatus.battery_state = batteryState;
	msgBatteryStatus.charge = stateOfCharge;

	updateBatteryDisplay();
	
	battery_pub.publish( &msgBatteryStatus );
	
	nh.spinOnce();
	delay(750);
}
