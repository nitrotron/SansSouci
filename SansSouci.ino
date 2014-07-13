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
#define NUM_OF_THERMOMETERS 3

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
uint8_t rimsThermoNumber = 0;
int Kp = 2;
int Ki = 5;
int Kd = 1;

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
    GetTemps,//2
    GetTemp,//3
    GetSensors,//4
    GetSensor,//5
    GetTempAlarms,//6
    SetTempAlarmHigh,//7
    SetTempAlarmLow,//8
    ClearTempAlarms,//9
    GetTimer,//10
    SetTimer,//11
    ResetAlarm,//12  
    GetAlarmStatus, //13
    StartLogging,//14
    StopLogging,//15
    SetPIDSetPoint,//16
	SetPIDWindowSize,//17
	SetPIDKp,//18
	SetPIDKi,//19
	SetPIDKd,//20
	TurnOnRims, //21
	TurnOnPump, // 22
	TurnOnAux,  //23
	SetInitialClock //24
};

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessenger.attach(onReturnUnknownCmd);
  cmdMessenger.attach(ReturnStatus,           onReturnStatus);
  cmdMessenger.attach(GetTemps,               onGetTemps);
  cmdMessenger.attach(GetTemp,  			  onGetTemp);
  cmdMessenger.attach(GetSensors,  			  onGetSensors);
  cmdMessenger.attach(GetSensor,  			  onGetSensor);
  cmdMessenger.attach(GetTempAlarms,  		  onGetTempAlarms);
  cmdMessenger.attach(SetTempAlarmHigh,  	  onSetTempAlarmHigh);
  cmdMessenger.attach(SetTempAlarmLow,  	  onSetTempAlarmLow);
  cmdMessenger.attach(ClearTempAlarms,  	  onClearTempAlarms);
  cmdMessenger.attach(GetTimer,  			  onGetTimer);
  cmdMessenger.attach(SetTimer,  			  onSetTimer);
  cmdMessenger.attach(ResetAlarm,		      onResetAlarm);
  cmdMessenger.attach(GetAlarmStatus,         onGetAlarmStatus);
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




// ------------------  C A L L B A C K S -----------------------
// Called when a received command has no attached function
void onReturnUnknownCmd()
{
  cmdMessenger.sendCmd(ReturnUnknownCmd,"Command without attached callback");
}

//Used to provide general status()
void onReturnStatus()
{
  onGetTemps();
  onGetSensors();
  onGetTempAlarms();
  onGetAlarmStatus();
  GetTimerStatus();
  Serial.print("PumpOn|");
  Serial.print(PumpOn);
  Serial.println(";");
  Serial.print("AuxOn|");
  Serial.print(AuxOn);
  Serial.println(";");
  Serial.print("RimsEnable|");
  Serial.print(RimsEnable);
  Serial.println(";");
  Serial.print("ArduinoTime|");
  Serial.print(now());
  Serial.println(";");
  Serial.print("SetPoint|");
  Serial.print(SetPoint);
  Serial.print(";");
  Serial.print("WindowSize|");
  Serial.print(WindowSize);
  Serial.print(";");
  Serial.print("Kp|");
  Serial.print(Kp);
  Serial.print(";");
  Serial.print("Ki|");
  Serial.print(Ki);
  Serial.print(";");
  Serial.print("Kd|");
  Serial.print(Kd);
  Serial.print(";");

  

}

//  Called function to send back all of the temperature probes' temperature
//  
//  The return value will be "ReturnTemps,SensorA|TempA,SensorB|TempB...."
void onGetTemps()
{
  
  
  byte i;
  for (i = 0; i< NUM_OF_THERMOMETERS -1 ;i ++)
  {
    if (thermometersActive[i])
    {
	  Serial.print("Thermometer");
	  Serial.print(i);
	  Serial.print("|");
	  Serial.print(sensors.getTempF(thermometers[i]));
	  Serial.print(",");
    }
  }
  if (thermometersActive[i])
    {
	  Serial.print("Thermometer");
	  Serial.print(i);
	  Serial.print("|");
	  Serial.print(sensors.getTempF(thermometers[i]));
    }
 
  Serial.print(";");
  Serial.println();
}
//  Called function to send back all of the temperature probes' temperature
void onGetTemp()
{
  byte i = cmdMessenger.readIntArg();
  //cmdMessenger.sendCmdStart(ReturnTemps);
  
  Serial.print("INFO@");
  if (thermometersActive[i])
  {
      Serial.print("Thermometer");
	  Serial.print(i);
	  Serial.print("|");
	  Serial.print(sensors.getTempF(thermometers[i]));
    //cmdMessenger.sendCmdArg(i);
    //cmdMessenger.sendCmdArg(sensors.getTempF(thermometers[i]));
  }
  else
  {
     Serial.print("Thermometer");
	  Serial.print(i);
	  Serial.print("|");
	  Serial.print("-99");
  }
  
  Serial.print(";");
  Serial.println();
  //cmdMessenger.sendCmdEnd();    
}


// Called function to send back specific temperature probe's temperature
void onGetSensors()
{
  Serial.print("INFO@");
  byte i;
  for (i = 0; i< NUM_OF_THERMOMETERS -1 ;i ++)
  {
    if (thermometersActive[i])
    {
      Serial.print("ThermometerSensor");
      Serial.print(i);
      Serial.print("|");
      for (byte k=0; k < 8; k++)
      {
        Serial.print(thermometers[i][k], DEC);
      }
      Serial.print(",");
    }
  }
  if (thermometersActive[i])
    {
      Serial.print("ThermometerSensor");
      Serial.print(i);
      Serial.print("|");
      for (byte k=0; k < 8; k++)
      {
        Serial.print(thermometers[i][k], DEC);
      }
      
    }
  Serial.println(";");
  //cmdMessenger.sendCmdEnd();    
}
// Called function to send back specific temperature probe's temperature
void onGetSensor()
{
  byte i = cmdMessenger.readIntArg();

  //cmdMessenger.sendCmdStart(ReturnSensors);
  Serial.print("INFO@");
  if (thermometersActive[i])
  {
    Serial.print("Thermometer");
	Serial.print(i);
	Serial.print("|");
    for (byte k=0; k < 8; k++)
    {
	  Serial.print(thermometers[i][k], HEX);
      //cmdMessenger.sendCmdArg(thermometers[i][k]);
    }
  }
  Serial.println(";");
  //cmdMessenger.sendCmdEnd();    
}


// Called function to send back specific temperature probe's temperature
void onGetTempAlarms()
{
   Serial.print("INFO@");
   byte i;
   for (i = 0; i< NUM_OF_THERMOMETERS -1 ;i ++)
   {
     if (thermometersActive[i])
     {
       printAlarmInfoByIndex(i);
	   Serial.print(",");
     }
   }
   if (thermometersActive[i])
   {
     printAlarmInfoByIndex(i);
   }
   Serial.println(";");
 
}

// Called function to send back specific temperature probe's temperature
void onSetTempAlarmHigh()
{
  byte i = cmdMessenger.readIntArg();
  float tempF = cmdMessenger.readFloatArg();


  if (thermometersActive[i])
  {
    sensors.setHighAlarmTemp(thermometers[i], sensors.toCelsius(tempF));
  }


}


// Called function to send back specific temperature probe's temperature
void onSetTempAlarmLow()
{
  byte whichWire = cmdMessenger.readIntArg();
  float tempF = cmdMessenger.readFloatArg();

  if (thermometersActive[whichWire])
  {
    sensors.setLowAlarmTemp(thermometers[whichWire], sensors.toCelsius(tempF));

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

}
// Called function to send back specific temperature probe's temperature
void onGetTimer()
{
  AlarmID_t id = cmdMessenger.readIntArg(); 
  time_t minutes = Alarm.read(id);
  
  Serial.print("INFO@TimerAlarm");
  Serial.print(id);
  Serial.print("|");
  Serial.print(minutes);
  Serial.println(";");

}
// Called function to send back specific temperature probe's temperature
void onSetTimer()
{
  float minutes = cmdMessenger.readFloatArg(); 
  

  AlarmId id = Alarm.timerOnce(minutes * 60, timerAlarmHandler);

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
  TempAlarmActive = 0;
  WhichThermometerAlarmActive = 0;
  TimerAlarmActive = 0;

//  Serial.println("INFO@AlarmOn|0;");
}

void onGetAlarmStatus()
{
  Serial.print("INFO@TempAlarmActive|");
  Serial.print(TempAlarmActive);
  Serial.print(",TimerAlarmActive|");
  Serial.print(TimerAlarmActive);
  Serial.print(",WhichThermoAlarm|");
  Serial.print(WhichThermometerAlarmActive);
  Serial.println(";");
}

void onStartLogging()
{
  // Start data acquisition
  startAcqMillis = millis();
  acquireData    = true;
  Serial.println("INFO@StartLogging");
}

void onStopLogging()
{
  // Stop data acquisition
  acquireData    = false;
  Serial.println("INFO@ StopLogging");
}

void onSetPIDSetPoint()
{
  SetPoint = cmdMessenger.readFloatArg();
  Serial.print("INFO@PIDSetPoint|");
  Serial.print(SetPoint);
  Serial.println(";");
}
void onSetPIDWindowSize()
{
  WindowSize = cmdMessenger.readFloatArg();
  Serial.print("INFO@PIDWindowSize|");
  Serial.print(WindowSize);
  Serial.println(";");
}
void onSetPIDKp()
{
  Kp = cmdMessenger.readFloatArg();
  Serial.print("INFO@PIDKp|");
  Serial.print(Kp);
  Serial.println(";");
  myPID.SetTunings(Kp, Ki, Kd);

}
void onSetPIDKi()
{
  Ki = cmdMessenger.readFloatArg();
  Serial.print("INFO@PIDKi|");
  Serial.print(Ki);
  Serial.println(";");
  myPID.SetTunings(Kp, Ki, Kd);
}

void onSetPIDKd()
{
  Kd = cmdMessenger.readFloatArg();
  Serial.print("INFO@PIDKd|");
  Serial.print(Kd);
  Serial.println(";");
  myPID.SetTunings(Kp, Ki, Kd);
}

void onTurnOnRims()
{
  RimsEnable = cmdMessenger.readIntArg();
  digitalWrite(RIMSENABLE_PIN, RimsEnable);
}
void onTurnOnPump()
{
  PumpOn = cmdMessenger.readIntArg();
  digitalWrite(PUMP_PIN, PumpOn);
}
void onTurnOnAux()
{
  AuxOn = cmdMessenger.readIntArg();
  digitalWrite(AUX_PIN, AuxOn);
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
}

// function that will be called when an alarm condition exists during DallasTemperatures::processAlarms();
void alarmHandler(uint8_t* deviceAddress)
{
  TempAlarmActive = 1;
  WhichThermometerAlarmActive = whichThermometer(deviceAddress);
  int alarmEn = digitalRead(TEMP_ALARM_HW_ENABLED);
  if (alarmEn == 1)
  {
    turnOnAlarm();
  }

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
  if (alarmEn == 1)
  {
    turnOnAlarm();
  }
}

// function that will be called when an alarm condition exists during DallasTemperatures::processAlarms();
void turnOnAlarm()
{
//  Serial.println("INFO@AlarmOn|1;");
  tone(ALARM_PIN, 262, 100);
  digitalWrite(LED_PIN, HIGH);

}

// function that will be called when an alarm condition exists during DallasTemperatures::processAlarms();
void turnOffAlarm()
{

  noTone(ALARM_PIN);
  digitalWrite(LED_PIN, LOW);
}
 
 void setupPID()
{
  windowStartTime = millis();
  
  //initialize the variables we're linked to
  SetPoint = 100;

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

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
}

void printAlarms(uint8_t deviceAddress[])
{
  char temp;
  temp = sensors.getHighAlarmTemp(deviceAddress);
  Serial.print("High Alarm: ");
  Serial.print(DallasTemperature::toFahrenheit(temp),DEC);
  Serial.print("F | Low Alarm: ");
  temp = sensors.getLowAlarmTemp(deviceAddress);
  Serial.print(DallasTemperature::toFahrenheit(temp),DEC);
  Serial.print("F");
}

// main function to print information about a device
void printData(DeviceAddress deviceAddress)
{
  Serial.print("Device Address: ");
  printAddress(deviceAddress);
  Serial.print(" ");
  printTemperature(deviceAddress);
  Serial.println();
}

void checkAlarm(DeviceAddress deviceAddress)
{
  if (sensors.hasAlarm(deviceAddress))
  {
    Serial.print("ALARM: ");
    printData(deviceAddress);
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
void printCurrentTemp(DeviceAddress deviceAddress)
{
  printAddress(deviceAddress);
  printTemp(deviceAddress);
  Serial.println();
}



void printTemp(DeviceAddress deviceAddress)
{
  float tempF = sensors.getTempF(deviceAddress);
  if (tempF != DEVICE_DISCONNECTED)
  {
    Serial.print("Current Temp F: ");
    Serial.print(tempF);
  }
  else Serial.print("DEVICE DISCONNECTED");
  Serial.print(" ");
}

void printAlarmInfo(DeviceAddress deviceAddress)
{
  char temp;
  printAddress(deviceAddress);
  Serial.print("_HIGH|");

  temp = sensors.getHighAlarmTemp(deviceAddress);
  Serial.print(DallasTemperature::toFahrenheit(temp), DEC);
  
  Serial.print(",");

  printAddress(deviceAddress);
  Serial.print("_LOW|");

  temp = sensors.getLowAlarmTemp(deviceAddress);
  Serial.print(DallasTemperature::toFahrenheit(temp), DEC);
  
}

void printAlarmInfoByIndex(byte i)
{
  char temp;
  Serial.print("ThermometerHighAlarm");
  Serial.print(i);
  Serial.print("|");
  //Serial.print("_HIGH|");

  temp = sensors.getHighAlarmTemp(thermometers[i]);
  Serial.print(DallasTemperature::toFahrenheit(temp), DEC);
  
  Serial.print(",");

  Serial.print("ThermometerLowAlarm");
  Serial.print(i);
  Serial.print("|");


  temp = sensors.getLowAlarmTemp(thermometers[i]);
  Serial.print(DallasTemperature::toFahrenheit(temp), DEC);
 
}

void thermometerLoopCB()
{ 
// ask the devices to measure the temperature
  sensors.requestTemperatures();

  // call alarm handler function defined by sensors.setAlarmHandler
  // for each device reporting an alarm
  sensors.processAlarms();
}

void sendInfoCB()
{
    Serial.println();
    Serial.println();
    Serial.println("===================== Print General Info Begin =====================");
	for (byte i=0; i < NUM_OF_THERMOMETERS; i++)
     {
       if (thermometersActive[i])
         printCurrentTemp(thermometers[i]);
     }

	 printTimerNextTrigger();

	 for (uint8_t i=0; i < dtNBR_ALARMS; i++)
	 {
		dtAlarmPeriod_t alarmPeriodType = Alarm.readType(i);
		if (alarmPeriodType == dtNotAllocated)
		  printDTNotAllocated(i);
		else if (alarmPeriodType == dtTimer)
		  printDTTimer(i);
	    else
		  printDTAlarm(i);
	 }
    Serial.println("===================== Print General Info End =====================");
    Serial.println();
    Serial.println();
}

void GetTimerStatus()
{
	int numAvailable = 0;
	time_t alarmTime;
        Serial.println("ClearTimers|1;");
	 for (uint8_t i=0; i < dtNBR_ALARMS; i++)
	 {
		dtAlarmPeriod_t alarmPeriodType = Alarm.readType(i);

		if (alarmPeriodType == dtNotAllocated)
		{
		  numAvailable ++;
		}
		else if (alarmPeriodType == dtTimer && Alarm.isOneShotType(i) == 1)
		{
		   alarmTime = Alarm.getNextTrigger(i); 
		   Serial.print("Timer");
		   Serial.print(i);
		   Serial.print("|");
		   Serial.print(hour(alarmTime));
           Serial.print(":");
		   Serial.print(minute(alarmTime));
		   Serial.print(":");
		   Serial.print(second(alarmTime));
		   Serial.println(";");
		}
	 }
	 Serial.print("TimersNotAllocated|");
	 Serial.print(numAvailable);
	 Serial.println(";");
	 Serial.print("TotalTimers|");
	 Serial.print(dtNBR_ALARMS);
	 Serial.println(";");
}

void sendDataLogingCB()
{
  if (acquireData)
  {
    for (byte i=0; i < NUM_OF_THERMOMETERS; i++)
    {
      float tempF = sensors.getTempF(thermometers[i]);
	  if (tempF != DEVICE_DISCONNECTED)
	  {
	    Serial.print("DB_WRITE@Device|");
	    Serial.print(i);
	    Serial.print(",Temperature|");
	    Serial.print(tempF);
	    Serial.print(",TimeDiff|");
	  }
	}
  }
}

void printTimerNextTrigger()
{
  Serial.println("================= Print Timer Next Trigger ======================");
  time_t nextTrigger = Alarm.getNextTrigger();
  time_t nowT = Alarm.getDigitsNow(dtSecond);
  time_t whenTrigger = nextTrigger - nowT;
  Serial.print("INFO@ The next Timer/Alarm to trigger is in ");
  Serial.print(whenTrigger);
  Serial.print(" seconds;");
  Serial.print(" Now=");
  Serial.print(nowT);
  Serial.print("; nextTrigger=");
  Serial.print(nextTrigger);
  Serial.println();
}

void  printDTNotAllocated(uint8_t timer)
{
  Serial.print("INFO@ Timer/Alarm"); 
  Serial.print(timer);
  Serial.print(" not allocated");
  Serial.println();
}
void  printDTTimer(uint8_t timer)
{
  time_t alarmTime = Alarm.read(timer);
  
  Serial.print("INFO@ Timer"); 
  Serial.print(timer);
  Serial.print(" set to:");

  Serial.print(alarmTime);
  Serial.println();
}
void  printDTAlarm(uint8_t timer)
{
  time_t alarmTime = Alarm.read(timer);
  
  Serial.print("INFO@ Alarm"); 
  Serial.print(timer);
  Serial.print(" set to:");

  Serial.print(alarmTime);
  Serial.println();
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
  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(RIMSENABLE_PIN, OUTPUT);
  
  pinMode(TIMER_ALARM_HW_ENABLED,INPUT);
  pinMode(TEMP_ALARM_HW_ENABLED,INPUT);
    
  tone(ALARM_PIN, 262, 100);
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

  
   

  // search for temperature devices on the bus, initialize their alarms
  for (byte i=0; i < NUM_OF_THERMOMETERS; i++)
  {
    thermometersActive[i] = true;
    if (!sensors.getAddress(thermometers[i], i)) 
	{
	  Serial.print("Unable to find address for Device "); 
	  Serial.print(i);
	  Serial.println();
      thermometersActive[i] = false;
	  continue;
	}
    

    // alarm when temp is higher than max
    sensors.setHighAlarmTemp(thermometers[i], 125);
    
    // alarm when temp is lower than min
    sensors.setLowAlarmTemp(thermometers[i], -10);

	// alarmHandler() will get called when a thermometer low/high alarm
	// is met.
    sensors.setAlarmHandler(&alarmHandler);
    
    
  } // for
  
  setupPID();

  //Alarm.timerRepeat(1, thermometerLoopCB);  // processes alarms & thermometers every 15 seconds.
  //Alarm.timerRepeat(sampleInterval, sendDataLogingCB);
  Alarm.timerRepeat(10, onReturnStatus );
  Alarm.timerRepeat(1, processIncomingSerial);

}

// -------------------------------------------------
// ------------------- Loop ------------------------
// -------------------------------------------------
void loop() 
{

   thermometerLoopCB();
  //if (sensors.requestTemperaturesByIndex(rimsThermoNumber))
  //{
    //Input = sensors.getTempFByIndex(rimsThermoNumber);
    Input = sensors.getTempF(thermometers[rimsThermoNumber]);
    myPID.Compute();
  //}

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if(millis() - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output < millis() - windowStartTime) digitalWrite(SSR_PIN,HIGH);
  else digitalWrite(SSR_PIN,LOW);

  Alarm.delay(125);
} 
