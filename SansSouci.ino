#include <EEPROMex.h>
#include <EEPROMVar.h>

#include <Time.h>

#include <TimeAlarms.h>


// *** SendandReceiveBinaryArguments ***

// This example expands the previous SendandReceiveArguments example. The Arduino will 
//  receive and send multiple Binary values, demonstrating that this is more compact and faster. 
// Since the output is not human readable any more, the logging is disabled and the NewLines 
// are removed
//
// It adds a demonstration of how to:
// - Send binary parameters
// - Receive binary parameters,#define


#include <CmdMessenger.h>  // CmdMessenger
#include <OneWire.h>		//One Wire Library
#include <DallasTemperature.h> 
#include <PID_v1.h>


//#include "pitches.h"


#define SERIAL_BAUD 9600
#define NUM_OF_THERMOMETERS 4

//For the interrupt and debouncing
long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

//Define Pins
#define ALARM_RESET_BUTTON_PIN 2
#define AUX_PIN 3
#define ONE_WIRE_BUS 4
#define PUMP_PIN 5
#define SSR_PIN 6
#define RIMSENABLE_PIN 7
#define ALARM_PIN 8
#define TIMER_ALARM_HW_ENABLED 11
#define TEMP_ALARM_HW_ENABLED 12
#define LED_PIN 13

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress thermometers[NUM_OF_THERMOMETERS];
bool thermometersActive[NUM_OF_THERMOMETERS];
bool thermometerAlarmTriggered[NUM_OF_THERMOMETERS];

bool TempAlarmActive = 0;
uint8_t  WhichThermometerAlarmActive = 0;
bool TimerAlarmActive = 0;

bool AuxOn = 0;
bool PumpOn = 0;
bool RimsEnable = 0;



// Database and time variables
long startAcqMillis                = 0;
bool acquireData                   = false;
const unsigned long sampleInterval = 60; // second interval

//RIMS & PID variables
//Define Variables we'll be connecting to
double SetPoint, Input, Output;
uint8_t rimsThermoNumber = 3;
double Kp = 2;
double Ki = 5;
double Kd = 1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &SetPoint,2,5,1, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;


// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial);

// This is the list of recognized commands. These can be commands that can either be sent or received. 
// In order to receive, attach a callback function to these events
enum
{
    ReturnUnknownCmd, //0
    ReturnStatus, // 1
    SetTempAlarmHigh,//2
    SetTempAlarmLow,//3
    ClearTempAlarms,//4
    SetTimer,//5
    ResetAlarm,//6
    StartLogging,//7
    StopLogging,//8
    SetPIDSetPoint,//9
    SetPIDWindowSize,//10
    SetPIDKp,//11
    SetPIDKi,//12
    SetPIDKd,//13
    TurnOnRims, //14
    TurnOnPump, // 15
    TurnOnAux,  //16
    SetInitialClock //17
};

enum
{
  addressPIDSetPoint,
  addressPIDWindowSize,
  addressPIDKp,
  addressPIDKi,
  addressPIDKd,
  addressAcquireData,
  ADDRESSARRAYCOUNT
};

int addressEE[ADDRESSARRAYCOUNT];

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(onReturnUnknownCmd);                                 
  cmdMessenger.attach(ReturnStatus,           onReturnStatus);					
  cmdMessenger.attach(SetTempAlarmHigh,  	  onSetTempAlarmHigh);			
  cmdMessenger.attach(SetTempAlarmLow,  	  onSetTempAlarmLow);			
  cmdMessenger.attach(ClearTempAlarms,  	  onClearTempAlarms);			
  cmdMessenger.attach(SetTimer,  			  onSetTimer);					
  cmdMessenger.attach(ResetAlarm,		      onResetAlarm);				
  cmdMessenger.attach(StartLogging,           onStartLogging);				
  cmdMessenger.attach(StopLogging,            onStopLogging);				
  cmdMessenger.attach(SetPIDSetPoint,         onSetPIDSetPoint);			
  cmdMessenger.attach(SetPIDWindowSize,       onSetPIDWindowSize);			
  cmdMessenger.attach(SetPIDKp,               onSetPIDKp);					
  cmdMessenger.attach(SetPIDKi,               onSetPIDKi);					
  cmdMessenger.attach(SetPIDKd,				  onSetPIDKd);					
  cmdMessenger.attach(TurnOnRims,			  onTurnOnRims);				
  cmdMessenger.attach(TurnOnPump,			  onTurnOnPump);				
  cmdMessenger.attach(TurnOnAux,			  onTurnOnAux);					
  cmdMessenger.attach(SetInitialClock,		  onSetInitialClock);			
}

void setEEPromAddress()
{
  addressEE[addressPIDSetPoint]  = EEPROM.getAddress(sizeof(double));
  addressEE[addressPIDWindowSize] = EEPROM.getAddress(sizeof(double));
  addressEE[addressPIDKp] = EEPROM.getAddress(sizeof(double));
  addressEE[addressPIDKi] = EEPROM.getAddress(sizeof(double));
  addressEE[addressPIDKd] = EEPROM.getAddress(sizeof(double));
  addressEE[addressAcquireData] = EEPROM.getAddress(sizeof(bool));
  
  // for debugging purposes
//  Serial.print("addressPIDSetPoint|"); Serial.print(addressEE[addressPIDSetPoint]); Serial.println(";");  
//  Serial.print("addressPIDWindowSize|"); Serial.print(addressEE[addressPIDWindowSize]); Serial.println(";");
//  Serial.print("addressPIDKp|"); Serial.print(addressEE[addressPIDKp]); Serial.println(";");
//  Serial.print("addressPIDKi|"); Serial.print(addressEE[addressPIDKi]); Serial.println(";");
//  Serial.print("addressPIDKd|"); Serial.print(addressEE[addressPIDKd]); Serial.println(";");
}

void updateLocalFromEEPROM()
{
  uint8_t tempAcuireData = EEPROM.read(addressEE[addressAcquireData]);
  acquireData = (tempAcuireData == 1);
  
  SetPoint =  EEPROM.readDouble(addressEE[addressPIDSetPoint]);  
  WindowSize = EEPROM.readDouble(addressEE[addressPIDWindowSize]);
  Kp = EEPROM.readDouble(addressEE[addressPIDKp]);
  Ki = EEPROM.readDouble(addressEE[addressPIDKi]);
}


// ------------------  C A L L B A C K S -----------------------
// Called when a received command has no attached function
void onReturnUnknownCmd()
{
  cmdMessenger.sendCmd(ReturnUnknownCmd,"Command without attached callback");
}


//Used to provide general status()
void onReturnStatus()
{
 // String printMsg;

  Serial.print("{");
  getTemps();
  getAlarmStatus();
  getTimerStatus();
  
  Serial.print("\"acquireData\": "); Serial.print(acquireData); Serial.print(", ");
  Serial.print("\"pumpOn\": "); Serial.print(PumpOn); Serial.print(", ");
  Serial.print("\"auxOn\": "); Serial.print(AuxOn); Serial.print(", ");
  Serial.print("\"rimsEnable\": "); Serial.print(RimsEnable); Serial.print(", ");
  Serial.print("\"arduinoTime\": ");
  Serial.print(now());
  Serial.print(",");
  Serial.print("\"arduinoTimeLong\": \"");
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year()); 
  Serial.print("\","); 
  Serial.print("\"setPoint\": "); Serial.print(SetPoint); Serial.print(", ");
  Serial.print("\"windowSize\": "); Serial.print(WindowSize); Serial.print(", ");
  
  Serial.print("\"kp\":"); Serial.print(Kp); Serial.print(", ");
  Serial.print("\"ki\":"); Serial.print(Ki); Serial.print(", ");
  Serial.print("\"kd\":"); Serial.print(Kd); Serial.print(", ");
  Serial.print("\"output\": "); Serial.print(Output); Serial.print(", ");
  Serial.print("\"millis\":");
  Serial.print(millis());
  Serial.print(",");
  Serial.print("\"windowStartTime\":"); Serial.print(windowStartTime); Serial.print(", ");
  Serial.print("\"outputTime\":");
  Serial.print(millis()-windowStartTime);

  Serial.println("}");
}


void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}




//  Called function to send back all of the temperature probes' temperature
//  
//  The return value will be "ReturnTemps,SensorA|TempA,SensorB|TempB...."
void getTemps()
{
  
  Serial.print("\"thermometers\": [ ");

  byte i;
  double temp;
  
  for (i = 0; i< NUM_OF_THERMOMETERS; i ++)
  {
    if (thermometersActive[i])
    {
	  Serial.print("{");
	  Serial.print("\"id\":"); Serial.print(i); Serial.print(",");
      temp = sensors.getTempF(thermometers[i]);

      Serial.print("\"temp\":"); Serial.print(temp); Serial.print(",");
      printAlarmInfoByIndex(i);
	        	  
      Serial.print("\"sensor\":");
      for (byte k=0; k < 8; k++)
      {
        Serial.print(thermometers[i][k], DEC);
      }
      

      if (i == rimsThermoNumber) {
        bool isRims = true;
        Serial.print(",\"isRims\":"); Serial.print(isRims); 
      }

      Serial.print("}");
      if (i != NUM_OF_THERMOMETERS -1) {
        Serial.print(",");
      }
	  
    }

  }

  Serial.print(" ], ");
  

}


// Called function to send back specific temperature probe's temperature
void onSetTempAlarmHigh()
{
  byte i = cmdMessenger.readIntArg();
  float tempF = cmdMessenger.readFloatArg();

  onSetTempAlarmHigh(tempF, i);
  onReturnStatus();
}
void onSetTempAlarmHigh(float inTemp, int whichTemp)
{
  if (thermometersActive[whichTemp])
  {
    sensors.setHighAlarmTemp(thermometers[whichTemp], sensors.toCelsius(inTemp));
  }
}


// Called function to send back specific temperature probe's temperature
void onSetTempAlarmLow()
{
  byte whichWire = cmdMessenger.readIntArg();
  float tempF = cmdMessenger.readFloatArg();

  onSetTempAlarmLow(tempF, whichWire);
  onReturnStatus();
}
void onSetTempAlarmLow(float inTemp, int whichTemp)
{
  if (thermometersActive[whichTemp])
  {
    sensors.setLowAlarmTemp(thermometers[whichTemp], sensors.toCelsius(inTemp));
  }
}


// Called function to send back specific temperature probe's temperature
void onClearTempAlarms()
{
  byte i = cmdMessenger.readIntArg();

  if (thermometersActive[i])
  {
	sensors.setHighAlarmTemp(thermometers[i], 125);
	sensors.setLowAlarmTemp(thermometers[i], -10);
  }

  onReturnStatus();

}


// Called function to send back specific temperature probe's temperature
void onSetTimer()
{
  float minutes = cmdMessenger.readFloatArg(); 
  

  AlarmId id = Alarm.timerOnce(minutes * 60, timerAlarmHandler);
  onReturnStatus();
}


void onInterrupt()
{
  if((long)(micros() - last_micros) >= debouncing_time * 1000) {
    onResetAlarm();
    last_micros = micros();
  }
}


// Called function to send back specific temperature probe's temperature
void onResetAlarm()
{
  turnOffAlarm();
  
  uint8_t deviceCount = sensors.getDeviceCount();
  float t=0;

  // there was race conditions where it's just best to reset all pressing alarms.
  if (TempAlarmActive == 1) 
  {
    for(uint8_t i = 0; i < deviceCount; i++) {
      t = sensors.getTempC(thermometers[i]);
      if (sensors.hasAlarm(thermometers[i])) {
        if(t >= sensors.getHighAlarmTemp(thermometers[i])) {
            sensors.setHighAlarmTemp(thermometers[i], 125);
        }
        if (t <= sensors.getLowAlarmTemp(thermometers[i])) {
            sensors.setLowAlarmTemp(thermometers[i], -10);
        }
      }
    }
  }
  
  
  TempAlarmActive = 0;
  WhichThermometerAlarmActive = 0;
  TimerAlarmActive = 0;
  onReturnStatus();
}

bool checkAlarm(DeviceAddress deviceAddress)
{
  if (sensors.hasAlarm(deviceAddress))
  {
   return true;
  }
  else 
  {
    return false;
  }
}

void getAlarmStatus()
{
  Serial.print("\"tempAlarmActive\":"); Serial.print(TempAlarmActive); Serial.print(",");
  Serial.print("\"timerAlarmActive\":"); Serial.print(TimerAlarmActive); Serial.print(",");
  Serial.print("\"whichThermoAlarm\":"); Serial.print(WhichThermometerAlarmActive); Serial.print(",");
}

void onStartLogging()
{
  acquireData    = true;
  EEPROM.update(addressEE[addressAcquireData],1);  
  onReturnStatus();
}

void onStopLogging()
{
  acquireData    = false;
  EEPROM.update(addressEE[addressAcquireData],0);  
  onReturnStatus();
}

void onSetPIDSetPoint()
{
  SetPoint = cmdMessenger.readFloatArg();
  EEPROM.updateDouble(addressEE[addressPIDSetPoint],SetPoint);  
  onReturnStatus();
}
void onSetPIDWindowSize()
{
  WindowSize = cmdMessenger.readFloatArg();
  EEPROM.updateDouble(addressEE[addressPIDWindowSize],WindowSize);  
  onReturnStatus();
}
void onSetPIDKp()
{
  Kp = cmdMessenger.readFloatArg();
  EEPROM.updateDouble(addressEE[addressPIDKp],Kp);  
  myPID.SetTunings(Kp, Ki, Kd);
  onReturnStatus();

}
void onSetPIDKi()
{
  Ki = cmdMessenger.readFloatArg();
  EEPROM.updateDouble(addressEE[addressPIDKi],Ki);  
  myPID.SetTunings(Kp, Ki, Kd);
  onReturnStatus();
}

void onSetPIDKd()
{
  Kd = cmdMessenger.readFloatArg();
  EEPROM.updateDouble(addressEE[addressPIDKd],Kd);  
  myPID.SetTunings(Kp, Ki, Kd);
  onReturnStatus();
}

void onTurnOnRims()
{
  RimsEnable = cmdMessenger.readIntArg();
  digitalWrite(RIMSENABLE_PIN, !RimsEnable);
  onReturnStatus();
}
void onTurnOnPump()
{
  PumpOn = cmdMessenger.readIntArg();
  
  digitalWrite(PUMP_PIN, !PumpOn);
  onReturnStatus();
}
void onTurnOnAux()
{
  AuxOn = cmdMessenger.readIntArg();
  digitalWrite(AUX_PIN, !AuxOn);
  onReturnStatus();
}
void onSetInitialClock()
{
   byte hr = cmdMessenger.readIntArg();
   byte min = cmdMessenger.readIntArg();
   byte sec = cmdMessenger.readIntArg();
   byte month = cmdMessenger.readIntArg();
   byte day = cmdMessenger.readIntArg();
   byte yr = cmdMessenger.readIntArg();

  setTime(hr,min,sec,day,month,yr); // set time to Saturday 8:29:00am Jan 1 2011
  onReturnStatus();
}


// function that will be called when an alarm condition exists during DallasTemperatures::processAlarms();
void alarmHandler(uint8_t* deviceAddress)
{
  TempAlarmActive = 1;
  WhichThermometerAlarmActive = whichThermometer(deviceAddress);
  int alarmEn = digitalRead(TEMP_ALARM_HW_ENABLED);
  turnOnAlarm();
}

byte whichThermometer(DeviceAddress deviceAddress)
{
   
   for (byte i = 0; i< NUM_OF_THERMOMETERS ;i ++)
   {
     byte k;
     for (k = 0; k < 8; k++)
     {
       if (thermometers[i][k] != deviceAddress[k])
	 {
	   break;
	 }
     }
     if (k == 8) return i; // made it all the way through
   }
   return 255;
}



void timerAlarmHandler()
{
  TimerAlarmActive = 1;
  int alarmEn = digitalRead(TIMER_ALARM_HW_ENABLED);
  
  turnOnAlarm();
}

// function that will be called when an alarm condition exists during DallasTemperatures::processAlarms();
void turnOnAlarm()
{
  digitalWrite(ALARM_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);
}

// function that will be called when an alarm condition exists during DallasTemperatures::processAlarms();
void turnOffAlarm()
{
  digitalWrite(ALARM_PIN, HIGH);
  digitalWrite(LED_PIN, LOW);
}
 
 void setupPID()
{
  windowStartTime = millis();
  
  //initialize the variables we're linked to
  //  SetPoint = 100;
  // no longer needed, SetPoint gets set by the EEPROM

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}



// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


// Returns if it has been more than interval (in ms) ago. Used for periodic actions
bool hasExpired(unsigned long &prevTime, unsigned long interval) {
  if (  millis() - prevTime > interval ) {
    prevTime = millis();
    return true;
  } else     
    return false;
}

void printAlarmInfoByIndex(byte i)
{
  char temp;
  Serial.print("\"highAlarm\":");
  temp = sensors.getHighAlarmTemp(thermometers[i]);
  Serial.print(DallasTemperature::toFahrenheit(temp), DEC);  
  Serial.print(",");

  Serial.print("\"lowAlarm\":");
  temp = sensors.getLowAlarmTemp(thermometers[i]);
  Serial.print(DallasTemperature::toFahrenheit(temp), DEC);
  Serial.print(","); 
}

void thermometerLoopCB()
{ 
// ask the devices to measure the temperature
  sensors.requestTemperatures();

  // call alarm handler function defined by sensors.setAlarmHandler
  // for each device reporting an alarm
  sensors.processAlarms();
  
  //update the RIMS input
  Input = sensors.getTempF(thermometers[rimsThermoNumber]);
}

void sendDataLogingCB()
{
  if (acquireData == true) {
    int i;
    double temp;
    Serial.print("{\"DATALOGGING\": {");

    for (i = 0; i< NUM_OF_THERMOMETERS; i ++)
    {
      if (thermometersActive[i])
      {
        if (i != 0) {
          Serial.print(",");
        }
        temp = sensors.getTempF(thermometers[i]);
        Serial.print("\"temp"); Serial.print(i); Serial.print("\":"); Serial.print(temp); 
      }
    }  
    Serial.print(", ");
    Serial.print("\"setPoint\": "); Serial.print(SetPoint); Serial.print(", ");
    Serial.print("\"kp\":"); Serial.print(Kp); Serial.print(", ");
    Serial.print("\"ki\":"); Serial.print(Ki); Serial.print(", ");
    Serial.print("\"kd\":"); Serial.print(Kd); Serial.print(", ");
    Serial.print("\"output\": "); Serial.print(Output); Serial.print(" ");
    Serial.println("} }");
  }
}

void getTimerStatus()
{
	int numAvailable = 0;
	bool timersAdded = false;
	time_t alarmTime;
  
        Serial.print("\"clearTimers\":1,");
        Serial.print("\"timers\": [");
	 for (uint8_t i=0; i < dtNBR_ALARMS; i++)
	 {
		dtAlarmPeriod_t alarmPeriodType = Alarm.readType(i);

		if (alarmPeriodType == dtNotAllocated)
		{
		  numAvailable ++;
		}
		else if (alarmPeriodType == dtTimer && Alarm.isOneShotType(i) == 1)
		{
           if(timersAdded == true) 
		   {
		     Serial.print(",");
		   }

		   alarmTime = Alarm.getNextTrigger(i); 
		   Serial.print("\"");
		   Serial.print(hour(alarmTime));
                   Serial.print(":");
		   Serial.print(minute(alarmTime));
		   Serial.print(":");
		   Serial.print(second(alarmTime));
		   Serial.print("\"");
                   timersAdded = true;
		}
	 }
         Serial.print("],");

	 Serial.print("\"timersNotAllocated\":");
	 Serial.print(numAvailable);
	 Serial.print(",");

	 Serial.print("\"totalTimers\":");
	 Serial.print(dtNBR_ALARMS);
	 Serial.print(",");
}


// Process incoming serial data, and perform callbacks
void processIncomingSerial()
{
	cmdMessenger.feedinSerialData(); 
}


// --------------------------------------------------
// ------------------ M A I N  ----------------------
// --------------------------------------------------
// ------------------- Setup ------------------------

// Setup function
void setup() 
{
  bool didNotFindAllSensors = false;
  bool didNotFindOneSensor = true;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);
  digitalWrite(AUX_PIN, HIGH);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(AUX_PIN, OUTPUT);

  
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(RIMSENABLE_PIN, HIGH);
  pinMode(RIMSENABLE_PIN, OUTPUT);
  
  pinMode(TIMER_ALARM_HW_ENABLED,INPUT);
  pinMode(TEMP_ALARM_HW_ENABLED,INPUT);
    
  //tone(ALARM_PIN, 262, 100);
  digitalWrite(ALARM_PIN, HIGH);
  pinMode(ALARM_PIN, OUTPUT);
  attachInterrupt(0, onInterrupt, RISING);
  
  String debugMessage;

  // Listen on serial connection for messages from the pc
  Serial.begin(SERIAL_BAUD); 


  // ----------- CMD MESSENGER -----------------------
  // Adds newline to every command
  cmdMessenger.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

    
  // ----------- DALLAS ONE WIRE----------------------
  // start up the library 
  sensors.begin();
  
  delay(750);
  
  // search for temperature devices on the bus, initialize their alarms
  for (byte i=0; i < NUM_OF_THERMOMETERS; i++)
  {
    thermometersActive[i] = true;
    if (!sensors.getAddress(thermometers[i], i)) 
    {    
         thermometersActive[i] = false;
	     didNotFindAllSensors = true;
         continue;
	}	
	else 
	{
	    didNotFindOneSensor = false;
	}

	     
    sensors.setAlarmHandler(&alarmHandler);
  
  } // for
  

  if (didNotFindAllSensors == true){
    Serial.println("{\"noSensors\": true}");
  }

 
  setEEPromAddress();
  updateLocalFromEEPROM();
  setupPID();

  //Alarm.timerRepeat(1, thermometerLoopCB);  // processes alarms & thermometers every 15 seconds.
  Alarm.timerRepeat(sampleInterval, sendDataLogingCB);
  Alarm.timerRepeat(10, onReturnStatus );
  Alarm.timerRepeat(1, processIncomingSerial);
  
  
  Alarm.timerRepeat(1,thermometerLoopCB); 
  }



// -------------------------------------------------
// ------------------- Loop ------------------------
// -------------------------------------------------
void loop() 
{
   
   if (RimsEnable)
   {
   
        myPID.Compute();  
    
    /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if(millis() - windowStartTime>WindowSize)
    { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    if(Output < millis() - windowStartTime) digitalWrite(SSR_PIN,LOW);
    else digitalWrite(SSR_PIN,HIGH);
  }
  else // RIMS disabled, so have this always low
  {
    digitalWrite(SSR_PIN,LOW);
  }

  Alarm.delay(5);
} 
