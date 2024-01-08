#include <ArduinoLowPower.h>
#include <MKRNB.h>
//#include <Arduino_PMIC.h> // Akku IC
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <math.h>
#include <DS3231.h> // RTC
#include <Wire.h>  // I2C für RTC
#include <limits>

int iDeepSleepTime_ms = 300000; // 5 min. // Zeit in ms, in der das Modul in den Sleep Zustand versetzt wird

volatile boolean bInterrupt = false;

//
// Defines
//
#define LOGDEBUG false        // Debugausgaben
#define DUMMY false           // Dummydaten beim Abstandssensor
#define WAIT_SERIAL false     // Soll daraf gewartet werden, dass serielle Verbindung vorhanden ist
#define USE_NB true           // Narrowband Kommunikationsmodul verwenden
#define SEND_START_SMS true   // Zu Beginn SMS senden
#define USE_DEEPSLEEP true   // Controller in den Ruhezustand versetzen
#define SET_RTC false         // RTC in setup () auf korrekte Uhrzeit stellen

#define PIN_INTERRUPT 0
#define PIN_TRIGGER 1
#define PIN_ECHO 2
#define PIN_SUPPLY 3

//#define USE_SERIAL_1        // Serial Port 1 (USART) verwenden

#ifdef USE_SERIAL_1
#define SERIALBEGIN(a)   (Serial1.begin(a))
#define SERIALPRINT(b)   (Serial1.print(b))
#define SERIALPRINTLN(c) (Serial1.println(c))
#define SERIALFLUSH      Serial1.flush ()
#define SERIALEND        Serial1.end ()
#define SERIALWAIT       while(!Serial1){}
#else
#define SERIALBEGIN(a)   (Serial.begin(a))
#define SERIALPRINT(b)   (Serial.print(b))
#define SERIALPRINTLN(c) (Serial.println(c))
#define SERIALFLUSH      Serial.flush()
#define SERIALEND        Serial.end()
#define SERIALWAIT       while(!Serial){}
#endif

//
// Narrowband Lib
//
NB nbAccess;
NB_SMS sms;

const char PINNUMBER[] = "0000";   // Pin

//telephone number to send sms
const char remoteNum1[20] = "012345678910";  // First person
const char remoteNum2[20] = "012345678910";  // Second person
boolean bConnected = false;
char message[160] = ""; // Message

//
// RTC
//
DS3231 myRTC;

bool century = false;
bool h12Flag;
bool pmFlag;

// Zustände des Moduls
enum class eState
{
  Ok = 0,
  Alert = 1
};

// Aktueller Zustand
eState State = eState::Ok;

// Zustand beim letzten Durchlauf
eState LastState = State;

// Maximale Alarmnachrichten pro Tag
const int iMaxCntAlarmUltrasonic = 3;
int iCntAlarmUltrasonic = 0;

const int iMaxCntAlarmSwitch = 3;
int iCntAlarmSwitch = 0;

//
// Battery
//
const int iLipoLover_mV = 3000; // Untere Schwelle LiPo Akku
const int iLipoUpper_mV = 4200; // Obere Schwelle LiPo Akku
const int iMaxDiff_mV = iLipoUpper_mV - iLipoLover_mV; // Für Berechnung der Batteriekapazität
const double dVoltageDividerFactor = 1.4681; // Batterie--220k--ADC1--470k--SG
unsigned long ulLoopCount = 0;

//
// Ultraschall Sensor
//

// Messbereich des Sensors
const int iMinDistance_cm = 15;    // kleinster Abstand, der mit dem Sensor gemessen werden kann
const int iMaxDistance_cm = 400;   // größter Abstand, der mit dem Sensor gemessen werden kann

// Gemessene Max/Min Werte
int iDistanceMin_cm = INT32_MAX;   // maximaler gemessener Wert
int iDistanceMax_cm = INT32_MIN;   // minimaler gemessener Wert
float fTempMin_Deg = std::numeric_limits<float>::max ();
float fTempMax_Deg = std::numeric_limits<float>::min ();

// Absoluter Abstand für Alarm
const int iDistanceLimitLower_cm = 70; // Abstand, ab dem der Alarm ausgelöst wird

// Prozentualer Abstand für Alarm
int iReferenceDistance_cm = 0;  // Referenzabstand, der nach dem Einschalten gemessen wird
int iDistanceLimitLowerPercent_cm = 0;
double dLimitPercent = 80.0;  // Prozentualer Abstand, ab dem Alarm ausgelöst wird. Bsp. iReferenceDistance_cm = 100, dLimitPercent = 40 => Alarm ab 40 cm

// Letzter gemessener Wert
int iDist_cm_1 = 0;

// Soll absolut oder relativ Alarm ausgelöst werden
enum class eMeasureMode
{
  Absolute = 0,
  Relative = 1
};

eMeasureMode MeasureMode = eMeasureMode::Relative;

//
// Distanz in cm messen
// !!! Die Spannung muss vorher eingeschaltet sein!
//
int MeasureDistance_cm (void)
{
  if (LOGDEBUG == true)
  {
    SERIALPRINTLN ("Start distance measurement");
  }

  int iDuration_us = 0;
  int iDistance_cm = 0;

  digitalWrite (PIN_TRIGGER, LOW);
  delayMicroseconds (20);
  //delayMicroseconds (20);
  digitalWrite (PIN_TRIGGER, HIGH);
  //delayMicroseconds (10);

  iDuration_us = pulseIn (PIN_ECHO, HIGH);
  iDistance_cm = int (iDuration_us * 343) / 20000;

  if (LOGDEBUG == true)
  {
    SERIALPRINT ("Duration (us): "); SERIALPRINT (iDuration_us);
    SERIALPRINT (" Distance (cm): "); SERIALPRINTLN (iDistance_cm);
  }

  return iDistance_cm;
}

//
// Bestimmte Anzahl an Messungen durchführen und Mittelwert zurückgeben
// //Bestimmte Anzahl an Messungen durchführen und Median zurückgeben
//
boolean MeasureAndCalculate (int x_iCount, int& r_iMedianDistance_cm, int& r_iValueCount, unsigned long& r_ulDuration_ms)
{
  unsigned long ulStart = millis ();

  digitalWrite (PIN_SUPPLY, HIGH); // Switch ON Supply
  delay (500);

  std::vector<int> vecValues;
  int iCounter = 0;
  int iSum = 0;

  while ((vecValues.size () < x_iCount) && (iCounter <= 100)) // Solange, bis genügend Messwerte erreicht sind, oder nach 100 Messungen
  {
    int iDistance_cm = 0;

    if (DUMMY == true)
    {
      iDistance_cm = iCounter * 10;
    }
    else
    {
      iDistance_cm = MeasureDistance_cm ();
    }

    iCounter++;

    if ((iDistance_cm > iMinDistance_cm) && (iDistance_cm < iMaxDistance_cm))
    {
      // nur Messwerte innerhalb des Messbereichs verwenden
      vecValues.push_back (iDistance_cm);
      iSum += iDistance_cm;
      //SERIALPRINTLN ("Messwert ok");
    }
    else
    {
      //SERIALPRINTLN ("Messwert nicht ok!");
    }

    delay (20);
  }

  digitalWrite (PIN_SUPPLY, LOW); // Switch OFF Supply

  // Check 1: Es müssen mindestens halb so viele Messwerte gültig sein, wie gemessen werden sollten
  r_iValueCount = (int)vecValues.size ();

  if (r_iValueCount < (x_iCount / 2))
  {
    r_ulDuration_ms = millis () - ulStart;
    return false;
  }

  // Check 2: Standardabweichung darf nicht zu hoch sein
  double dAverage = (double) (iSum / x_iCount);

  double dSumDev = 0.0;

  for (auto it = vecValues.begin (); it != vecValues.end (); ++ it)
  {
    dSumDev += ((*it - dAverage) * (*it - dAverage));
  }

  double dVar = dSumDev / vecValues.size ();

  double dStandardDev = sqrt (dVar);

  if (dStandardDev > 8.0)
  {
    r_ulDuration_ms = millis () - ulStart;
    return false;
  }
  
  r_iMedianDistance_cm = (int)dAverage;

  // Median
  if (0)
  {
    // Messwerte sortieren 
    std::sort (vecValues.begin (), vecValues.end ());

    // Index des Median
    size_t medianIndex = vecValues.size () / 2;
    
    // Median
    r_iMedianDistance_cm = vecValues[medianIndex];
  }

  // int iCount = 0;
  // for (auto it = vecValues.begin (); it != vecValues.end (); ++ it)
  // {
  //   SERIALPRINT ("Wert "); SERIALPRINT (iCount); SERIALPRINT (": "); SERIALPRINTLN (*it);
  //   iCount++;
  // }

  r_ulDuration_ms = millis () - ulStart;
  return true;
}

//
// Referenzabstand messen
//
bool MeasureReference (void)
{
  SERIALPRINTLN ("Start reference measurement");

  unsigned long ulStartTime = millis ();
  int iCountValid = 0;
  unsigned long ulMeasureDuration_ms = 0;

  while (MeasureAndCalculate (20, iReferenceDistance_cm, iCountValid, ulMeasureDuration_ms) != true)
  {
    // Solange messen, bis Referenzabstand gemessen werden konnte
    delay (2000);
  }
  
  iDistanceLimitLowerPercent_cm = int ((dLimitPercent * (double) iReferenceDistance_cm) / 100.0);

  unsigned long ulReferenceDuration_ms = millis () - ulStartTime;
  
  SERIALPRINT ("Reference (cm): "); SERIALPRINT (iReferenceDistance_cm);
  SERIALPRINT (" Duration (ms): "); SERIALPRINT (ulReferenceDuration_ms);
  SERIALPRINT (" Alert at distance (cm): "); SERIALPRINTLN (iDistanceLimitLowerPercent_cm);
  SERIALPRINTLN ("");

  return true;
}

//
// Spannung an Batterie lesen
//
bool MeasureBattery (int& r_iVoltage_mV, int& r_iPercent)
{
  int iVal = analogRead (A1); // 1023 = 3.3V

  double dVoltage_mV = (iVal * 3300 * dVoltageDividerFactor) / 1024;

  // Spannung im gültigen Bereich
  if ((int) dVoltage_mV >= iLipoLover_mV)
  {
    r_iVoltage_mV = (int) dVoltage_mV;
    double dVoltageDifference_mV = dVoltage_mV - (double) iLipoLover_mV; // Max: 4,2V-3,5V = 0,7V
    
    double dBatCapacity_percent = (dVoltageDifference_mV / (double) iMaxDiff_mV) * 100.0;

    // Clipping bei 100%
    if (dBatCapacity_percent > 100.0)
      dBatCapacity_percent = 100.0;

    r_iPercent = int (dBatCapacity_percent);

    return true;
  }
  else
  {
    r_iVoltage_mV = 0;
    r_iPercent = 0;
    return false;
  }
}

//
// Verbinden LTE
//
boolean Connect (long x_lTries = 10)
{
  if (bConnected == true)
  {
    return true;
  }
  // int iIsAlive = nbAccess.isAccessAlive ();
  // SERIALPRINT ("Is Alive: "); SERIALPRINTLN (iIsAlive);

  for (int i = 0; i < x_lTries; i++)
  {
    if (nbAccess.begin (PINNUMBER) == NB_READY)
    {
      bConnected = true;
      break;
    } 
    else
    {
      delay (1000);
    }
  }

  SERIALPRINT ("Connected: "); SERIALPRINTLN (bConnected);

  // iIsAlive = nbAccess.isAccessAlive ();
  // SERIALPRINT ("Is Alive: "); SERIALPRINTLN (iIsAlive);

  return bConnected;
}

//
// SMS senden 
//
boolean SendSms (void)
{
  // sms text
  char txtMsg[160] = "Alarm! Abstand Sensor <-> Medium kleiner 100 cm";

  // send the message
  sms.beginSMS (remoteNum1);
  sms.print (txtMsg);
  sms.endSMS ();

  return true;  
}

//
// SMS senden 
//
boolean SendSms (const char* x_pszPhonenumber, const char* x_pszText)
{
  // send the message
  sms.beginSMS (x_pszPhonenumber);
  sms.print (x_pszText);
  sms.endSMS ();

  return true;
}

//
// Wird einmalig ausgeführt
//
void setup ()
{
  // Pins
  pinMode (PIN_TRIGGER, OUTPUT);
  pinMode (PIN_ECHO, INPUT);
  pinMode (PIN_INTERRUPT, INPUT);

  pinMode (PIN_SUPPLY, OUTPUT);
  digitalWrite (PIN_SUPPLY, LOW);

  pinMode (LED_BUILTIN, OUTPUT); // LED
  digitalWrite (LED_BUILTIN, LOW);

  delay (500);

  // Serial
  SERIALBEGIN (9600);

  if (WAIT_SERIAL == true)
  {
    while (!Serial) 
    {
      ;// wait for serial port to connect. Needed for native USB port only
    }
  }

  // Start the I2C interface
  Wire.begin ();

  if (SET_RTC == true)
  {
    // ToDo: User Input
    // ToDo: In eigene Fkt auslagern

    myRTC.setYear (23);
    myRTC.setMonth (03);
    myRTC.setDate (12);
    myRTC.setDoW (7);
    myRTC.setHour (20);
    myRTC.setMinute (24);
    myRTC.setSecond (0);
  }

  SERIALPRINTLN ("Power On!");

  // prepare parameter values for setting a new alarm time
  int alarmBits = 0b00001000; // Alarm 1 when hour, minutes, second match
  //int alarmBits = 0b00001100; // Alarm 1 when minutes, second match
  bool alarmH12 = false; // interpret hour in 24-hour mode
  bool alarmPM = false; // irrelevant in 24-hour mode, but it needs a value
  bool alarmIsDay = true; // interpret "day" value as a date in the month

  // Setup alarm one to fire every second
  myRTC.turnOffAlarm (1);
  myRTC.setA1Time (0, 10, 00, 0, alarmBits, alarmIsDay, alarmH12, alarmPM);
  myRTC.turnOnAlarm (1);
  myRTC.checkIfAlarm (1);

  Wire.end ();

  SERIALPRINTLN ("Clock alarm set done.");

  // Referenzabstand messen
  if (MeasureMode == eMeasureMode::Relative)
  {
    MeasureReference ();
    iDist_cm_1 = iReferenceDistance_cm; // letzter Wert ist der Wert der Referenzmessung
  }

  SERIALPRINTLN ("Measure reference done.");

  if ((USE_NB == true) && (SEND_START_SMS == true))
  {
    // Welches Limit wird verwendet? Relativ oder absolut
    int iDistanceLimitToCompare_cm = 0;
    
    if (MeasureMode == eMeasureMode::Relative)
    {
      iDistanceLimitToCompare_cm = iDistanceLimitLowerPercent_cm;
    }
    else if (MeasureMode == eMeasureMode::Absolute)
    {
      iDistanceLimitToCompare_cm = iDistanceLimitLower_cm;
    }

    //
    // Battery
    //
    int iVoltage_mV = 0, iPercent = 0;
    MeasureBattery (iVoltage_mV, iPercent);

    char txtMsg[160] = "";
    sprintf (txtMsg, "Switch: %d Reference/cm: %d Limit/cm: %d (%.2lf%%) Voltage/mv: %d (%d%%)",
             digitalRead (PIN_INTERRUPT), iReferenceDistance_cm, iDistanceLimitToCompare_cm, dLimitPercent, iVoltage_mV, iPercent);   
    SERIALPRINTLN (txtMsg);

    SERIALPRINTLN ("Try to connect");

    if (Connect () == true)
    {
      if (SendSms (remoteNum2, txtMsg) == true)
      {
        SERIALPRINTLN ("Send SMS OK.");
      }

      if (SendSms (remoteNum1, txtMsg) == true)
      {
        SERIALPRINTLN ("Send SMS OK.");
      }
    }
  }

  LowPower.attachInterruptWakeup (PIN_INTERRUPT, InterruptCallback, RISING);

  if (USE_DEEPSLEEP == true) // Verbindung wird jedes mal in loop () wieder aufgebaut
  {
    SERIALFLUSH;
    SERIALEND;
  }
}

//
// main
//
void loop ()
{
  ulLoopCount++;
  
  // PinStatus des IR Pins setzen. Hier den Pin nicht noch mal neu einlesen.
  PinStatus StatusPinInterrupt = LOW;
  if (bInterrupt == true)
  {
    StatusPinInterrupt = HIGH;
  }

  if (USE_DEEPSLEEP == true) // Waked up from deepsleep -> Connect to serial port
  {
    Wire.begin ();
    SERIALBEGIN (9600);

    if (WAIT_SERIAL == true)
      SERIALWAIT;
  }

  //
  // Battery
  //
  int iVoltage_mV = 0, iPercent = 0;
  if (MeasureBattery (iVoltage_mV, iPercent) == true)
  {
    SERIALPRINT ("Voltage (mV): "); SERIALPRINT (iVoltage_mV);
    SERIALPRINT (" Percent: "); SERIALPRINTLN (iPercent);
  }
  else
  {
    SERIALPRINTLN ("No battery connected.");
  }

  //
  // Temperature
  //
  float fTemp_Deg = myRTC.getTemperature ();
  // Max/Min Werte
  if (fTemp_Deg < fTempMin_Deg)
    fTempMin_Deg = fTemp_Deg;
  if (fTemp_Deg > fTempMax_Deg)
    fTempMax_Deg = fTemp_Deg;

  //
  // Abstand
  //
  int iCountValid = 0;
  int iDist_cm = 0;
  unsigned long ulMeasureDuration_ms = 0;
  boolean bSuccess = MeasureAndCalculate (20, iDist_cm, iCountValid, ulMeasureDuration_ms);
  SERIALPRINT ("Median: Success: "); SERIALPRINT (bSuccess);
  SERIALPRINT (" Distance (cm): "); SERIALPRINT (iDist_cm);
  SERIALPRINT (" Valid values: "); SERIALPRINT (iCountValid);
  SERIALPRINT (" Duration (ms): "); SERIALPRINTLN (ulMeasureDuration_ms);
    // Max/Min Werte speichern
  if (bSuccess == true)
  {
    if (iDist_cm < iDistanceMin_cm)
      iDistanceMin_cm = iDist_cm;
    if (iDist_cm > iDistanceMax_cm)
      iDistanceMax_cm = iDist_cm;
  }

  // Welches Limit wird verwendet? Relativ oder absolut
  int iDistanceLimitToCompare_cm = 0;
  
  if (MeasureMode == eMeasureMode::Relative)
  {
    iDistanceLimitToCompare_cm = iDistanceLimitLowerPercent_cm;
  }
  else if (MeasureMode == eMeasureMode::Absolute)
  {
    iDistanceLimitToCompare_cm = iDistanceLimitLower_cm;
  }

  // Alarm, wenn: (Messung erfolgreich UND Abstand zu gering)
  boolean bAlarmUltrasonic = (bSuccess == true) &&                        // Messung erfolgreich
                             (iDist_cm <= iDistanceLimitToCompare_cm) &&  // Limit unterschritten
                             ((iDist_cm_1 - iDist_cm) <= 200);            // Sensor "springt" nicht zwischen Min/Max Werten (das ist der Fall, wenn er dejustiert ist)

  // letzten gemessenen Wert übernehmen
  iDist_cm_1 = iDist_cm;

  // Alarm, wenn Ultraschallsensor auslöst ODER Interrupt
  if ((bAlarmUltrasonic == true) || (bInterrupt == true))
  {
    State = eState::Alert;

    // Wechsel von Ok -> Alarm signalisieren
    if (LastState == eState::Ok)
    {
      digitalWrite (LED_BUILTIN, HIGH);

      SERIALPRINT ("Alarm! ");
      SERIALPRINT ("Float switch: "); SERIALPRINT (StatusPinInterrupt);
      SERIALPRINT (" Distance/cm: "); SERIALPRINT (iDist_cm);
      SERIALPRINT (" Limit: "); SERIALPRINTLN (iDistanceLimitToCompare_cm);
      
      if ((USE_NB == true) && 
          (((bAlarmUltrasonic == true) && (iCntAlarmUltrasonic < iMaxCntAlarmUltrasonic)) || ((bInterrupt == true) && (iCntAlarmSwitch < iMaxCntAlarmSwitch))))
      {
        SERIALPRINTLN ("Try to connect");

        if (Connect () == true)
        {
          char txtMsg[160] = "";

          sprintf (txtMsg, "Alarm! Switch: %d Distance/cm: %d Limit/cm: %d", StatusPinInterrupt, iDist_cm, iDistanceLimitToCompare_cm);
          
          boolean bSendSuccess1 = true;
          boolean bSendSuccess2 = true;

          bSendSuccess2 &= SendSms (remoteNum2, txtMsg);
          bSendSuccess1 &= SendSms (remoteNum1, txtMsg);

          // Sofern das Senden min. einer Nachricht geklappt hat, den Zähler inkrementieren.
          // Es könnte sein, dass eine Nummer nicht i.O. ist, dann muss man aber den Zähler wenigstens bei der funktionierenden Nachricht inkrementieren.
          if ((bSendSuccess1 == true) || (bSendSuccess2 == true))
          {
            // Den entsprechenden Zähler hochzählen
            if (bInterrupt == true)
              iCntAlarmSwitch ++;
            if (bAlarmUltrasonic == true)
              iCntAlarmUltrasonic++;
            
            SERIALPRINTLN ("Send SMS OK.");
          }
        }
      }

    }
  }
  else
  {
    State = eState::Ok;

    // Wechsel von Alarm -> Ok signalisieren
    if (LastState == eState::Alert)
    {
      digitalWrite (LED_BUILTIN, LOW);
      SERIALPRINTLN ("State ok again.");
    }
  }

  SERIALPRINTLN ("");

  // Den letzten Zustand merken
  LastState = State;

  // Interrupt löschen
  if (digitalRead (PIN_INTERRUPT) == LOW)
  {
    bInterrupt = false;
  }

  //
  // Status SMS senden
  //
  if (myRTC.checkIfAlarm (1, false))
  {
    SERIALPRINTLN ("Clock alarm.");

    if (USE_NB == true)
    {
      SERIALPRINTLN ("Try to connect");

      if (Connect () == true)
      {
        char txtMsg[160] = "";

        sprintf (txtMsg, "Switch: %d Dist/cm (cur;min;max): %d;%d;%d Voltage/mV: %d (%d%%) Temp/deg (cur;min;max): %.1lf;%.1lf;%.1lf",
                 digitalRead (PIN_INTERRUPT),
                 iDist_cm, iDistanceMin_cm, iDistanceMax_cm,
                 iVoltage_mV, iPercent,
                 fTemp_Deg, fTempMin_Deg, fTempMax_Deg);

        if (SendSms (remoteNum2, txtMsg) == true)
        {
          SERIALPRINTLN ("Send SMS OK.");
        }

        if (SendSms (remoteNum1, txtMsg) == true)
        {
          SERIALPRINTLN ("Send SMS OK.");
        }
      }
    }

    myRTC.checkIfAlarm (1, true); // Alarm Löschen
    
    // Reset Max/Min
    iDistanceMin_cm = INT32_MAX;   // maximaler gemessener Wert
    iDistanceMax_cm = INT32_MIN;   // minimaler gemessener Wert
    fTempMin_Deg = std::numeric_limits<float>::max ();
    fTempMax_Deg = std::numeric_limits<float>::min ();
    
    // Reset Alarm Messages
    iCntAlarmSwitch = 0;
    iCntAlarmUltrasonic = 0;
  }

  //
  // Deep Sleep
  //
  if (USE_DEEPSLEEP == true)
  {
    // Sleep Mode (ms)
    if (bConnected == true)
    {
      SERIALPRINTLN ("NB Shutdown");
      SERIALPRINTLN ("");    
      nbAccess.shutdown ();
      bConnected = false;
    }

    // Serielle Verbindung beenden
    SERIALFLUSH;
    SERIALEND;

    Wire.end ();
    LowPower.deepSleep (iDeepSleepTime_ms);
  }
}

void InterruptCallback ()
{
  // Hiernach wird loop () ausgeführt
  bInterrupt = true;
}