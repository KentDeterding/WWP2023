#include <Wire.h>               //Include library for UART communication
#include <Adafruit_MCP4725.h>   //Include DAC library
#include <Adafruit_INA260.h>    //Include power sensor library
#include <PA12.h>
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_MCP4725 dac;           //Initailize DAC

//Pin Declarations
#define Linear_Actuator_Enable 16
#define Linear_Actuator_Tx 1
#define Linear_Actuator_Rx 0
#define RPM_Pin 30
#define PCC_Relay_Pin 33
#define EStop_Pin 11
#define PCC_Disconnect_Pin 29

//Linear Actuators
#define ID_NUM 0
PA12 myServo(&Serial1, Linear_Actuator_Enable, 1);
//          (&Serial, enable_pin,  Tx Level)

//Adjustable Variables
uint16_t regulate_RPM_Setpoint = 5000;
uint16_t regulate_Power_Setpoint = 35000;//(mW)
float optimal_Theta = 7;
float cutin_Theta = 15;
float brake_Theta = 95;
float DAC_Voltage_Cutin = 0;

//Active Pitch Variables
uint16_t theta_Position;
float theta;
float actual_Theta;

//INA260 Reading Variables
uint16_t L_Power = 0;   //Load Power (mW)
uint16_t L_Voltage = 0; //Load Voltage (mV)
uint16_t L_Current = 0; //Load Current (mA)

//RPM Reading Variables
volatile uint16_t RPM_Filtered;
volatile uint16_t RPM_Raw;
volatile uint16_t RPM_Raw_Previous;
volatile unsigned long TempStore_RPM = 0;
volatile bool FirstRead_RPM = true;
volatile unsigned long Saved_RPM [10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile unsigned long RPMSum = 0;
volatile unsigned long Htime1, Htime2;
volatile int i = 0;
volatile int F = 0;

//Load Variables
float DAC_Voltage;
float DAC_Voltage_Previous;
float DAC_Perturbation = 10;
float DAC_Voltage_Maximum = 3300;//(mV)
float Effective_Load_Resistance = 4;
float VIratio;


//State Machine Variables
enum States {StartUp, Optimize, Regulate, EStop_Safety, Discontinuity_Safety, EStop_Safety_Restart, Discontinuity_Safety_Restart};
States State = StartUp;
enum PCC_Disconnect_States {Wait, RelayOn, Reconnection};
PCC_Disconnect_States PCC_Disconnect_State = Wait;

//Timer Intervals
unsigned Fast_Interval = 10;
unsigned Medium_Interval = 50;
unsigned Slow_Interval = 500;
unsigned EStop_Safety_Restart_Interval = 2000;
unsigned Discontinuity_Safety_Restart_Interval = 5000;
unsigned Pitch_Transient_Interval = 800;
unsigned RPM_Timeout_Interval = 200;
unsigned Discontinuity_Power_Down_Interval = 3000;

//Timers
unsigned long Timer_Fast;
unsigned long Timer_Medium;
unsigned long Timer_Slow;
unsigned long Timer_EStop_Safety_Restart;
unsigned long Timer_Discontinuity_Safety_Restart;
unsigned long Timer_RPM_Transient;
unsigned long Timer_RPM_Timeout;
unsigned long Timer_Discontinuity_Power_Down;

bool load_Resistance_Tracking_Enable = false;
bool E_Stop = false;
bool PCC_Disconnected = false;
bool PCC_Relay = false;
bool state_Machine_Enable = true;


void setup() {
  //Linear Actuator
  myServo.begin(32);
  myServo.movingSpeed(ID_NUM, 800);
  Serial.begin(9600);           //Begin serial monitor
  dac.begin(0x66);              //Initialize DAC
  ina260.begin();               //Initialize INA 260

  initialize_Pins();

  PCC_Relay = true;
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
  delay (4000);
  theta = cutin_Theta;
  set_Theta();
  delay (4000);
  PCC_Relay = false;
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
  delay (4000);
  DAC_Voltage = DAC_Voltage_Cutin;

  initialize_Timers();
}

void loop() {
  if (millis() - Timer_Fast >= Fast_Interval)
  {
    Timer_Fast = millis();
    read_EStop();
    manage_State();
  }

  if (millis() - Timer_Medium >= Medium_Interval)
  {
    Timer_Medium = millis();
    read_RPM();
  }

  if (millis() - Timer_Slow >= Slow_Interval)
  {
    Timer_Slow = millis();
    read_INA260();
    read_Linear_Actuator_Position();
    VIratio_optimize_load();
    set_Load();
    PC_Comms ();
  }
}

void manage_State() {
  if (state_Machine_Enable) {
    switch (State)
    {
      case StartUp:
        if (RPM_Filtered > 500) {
          State = Optimize;
          load_Resistance_Tracking_Enable = true;
          theta = optimal_Theta;
          set_Theta();
        }
        break;

      case Optimize:
        if (E_Stop)
        {
          State = EStop_Safety;
        }
        if (!digitalRead(PCC_Disconnect_Pin) && (ina260.readBusVoltage() < 50))
        {
          State = Discontinuity_Safety;
          Timer_Discontinuity_Power_Down = millis();
        }
        if (RPM_Filtered > regulate_RPM_Setpoint)
        {
          State = Regulate;
        }
        break;

      case Regulate:
        regulate_RPM();
        if (E_Stop)
        {
          State = EStop_Safety;
        }
        if (!digitalRead(PCC_Disconnect_Pin) && (ina260.readBusVoltage() < 50))
        {
          State = Discontinuity_Safety;
          Timer_Discontinuity_Power_Down = millis();
        }
        if (RPM_Filtered < 0.85 * regulate_RPM_Setpoint)
        {
          State = Optimize;
          theta = optimal_Theta;
          set_Theta();
        }
        break;

      case EStop_Safety:
        PCC_Relay = true;
        digitalWrite(PCC_Relay_Pin, PCC_Relay);
        theta = brake_Theta;
        set_Theta();
        load_Resistance_Tracking_Enable = false;
        if (!E_Stop)
        {
          State = EStop_Safety_Restart;
          Timer_EStop_Safety_Restart = millis();
        }
        break;

      case EStop_Safety_Restart:
        theta = optimal_Theta;
        set_Theta();
        if ((millis() - Timer_EStop_Safety_Restart >= EStop_Safety_Restart_Interval) && (RPM_Filtered > 500))
        {
          load_Resistance_Tracking_Enable = true;
          PCC_Relay = false;
          digitalWrite(PCC_Relay_Pin, PCC_Relay);
          State = Optimize;
        }
        break;

      case Discontinuity_Safety:
        PCC_Relay = true;
        digitalWrite(PCC_Relay_Pin, PCC_Relay);
        theta = brake_Theta;
        set_Theta();
        load_Resistance_Tracking_Enable = false;
        if (millis() - Timer_Discontinuity_Power_Down >= Discontinuity_Power_Down_Interval)
        {
          if (!digitalRead(PCC_Disconnect_Pin)) {
            Timer_Discontinuity_Safety_Restart = millis();
            State = Discontinuity_Safety_Restart;
          }
        }
        break;

      case Discontinuity_Safety_Restart:
        theta = optimal_Theta;
        set_Theta();
        if ((millis() - Timer_Discontinuity_Safety_Restart >= Discontinuity_Safety_Restart_Interval) && (RPM_Filtered > 500))
        {
          load_Resistance_Tracking_Enable = true;
          PCC_Relay = false;
          digitalWrite(PCC_Relay_Pin, PCC_Relay);
          State = Optimize;
        }
        break;

      default:
        State = StartUp;
        load_Resistance_Tracking_Enable = false;
        break;
    }
  }
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
}

void initialize_Timers() {
  Timer_Fast = millis();
  Timer_Medium = millis();
  Timer_Slow = millis();
  Timer_EStop_Safety_Restart = millis();
  Timer_Discontinuity_Safety_Restart = millis();
  Timer_RPM_Transient = millis();
  Timer_RPM_Timeout = millis();
  Timer_Discontinuity_Power_Down = millis();
}

void initialize_Pins() {
  pinMode(Linear_Actuator_Enable, OUTPUT);
  pinMode(RPM_Pin, INPUT);
  pinMode(PCC_Relay_Pin, OUTPUT);
  pinMode(EStop_Pin, INPUT);
  pinMode(PCC_Disconnect_Pin, INPUT);
}

void read_EStop() {
  E_Stop = digitalRead(EStop_Pin);
}

void read_INA260() {
  L_Power = ina260.readPower();
  L_Voltage = ina260.readBusVoltage();
  L_Current = ina260.readCurrent();

}

void read_Linear_Actuator_Position() {
  actual_Theta = (myServo.presentPosition(ID_NUM) - 208.084) / 31.3479;
}

void regulate_RPM() {
  if (millis() - Timer_RPM_Transient >= Pitch_Transient_Interval)
  {
    Timer_RPM_Transient = millis();
    if (RPM_Filtered < regulate_RPM_Setpoint || RPM_Filtered > 1.05 * regulate_RPM_Setpoint)
    {
      theta = theta + 0.003 * (RPM_Filtered - regulate_RPM_Setpoint);
      set_Theta();
    }
  }
}

void set_Load() {
  if (DAC_Voltage > 3300) {
    DAC_Voltage = 3300;
  }
  if (DAC_Voltage < 0) {
    DAC_Voltage = 0;
  }
  dac.setVoltage(((DAC_Voltage + (DAC_Voltage_Maximum * 0.5) / 4096) / DAC_Voltage_Maximum) * (4095), false);   //Set DAC voltage
}

void set_Theta()
{
  if (theta > 95.0) {
    theta = 95.0;
  }
  if (theta < optimal_Theta) {
    theta = optimal_Theta;
  }
  theta_Position = (int)(31.3479 * theta + 208.084);
  myServo.goalPosition(ID_NUM, theta_Position);
}

void read_RPM() {
  attachInterrupt(digitalPinToInterrupt(RPM_Pin), RPM_Interupt, RISING);    //run ISR on rising edge
  if (millis() - Timer_RPM_Timeout >= RPM_Timeout_Interval)
  {
    FirstRead_RPM = true;
    Timer_RPM_Timeout = millis();
    Saved_RPM [0] = 0;
    Saved_RPM [1] = 0;
    Saved_RPM [2] = 0;
    Saved_RPM [3] = 0;
    Saved_RPM [4] = 0;
    Saved_RPM [5] = 0;
    Saved_RPM [6] = 0;
    Saved_RPM [7] = 0;
    Saved_RPM [8] = 0;
    Saved_RPM [9] = 0;
    RPM_Filtered = 0;
    RPMSum = 0;
  }
}

void PC_Comms () {
  //---------------------------------------------------------------------------------------
  if (Serial)
  {
    Serial.println("----------------------------------------------------------");
    Serial.println();
    Serial.println("                   WILDCAT WIND POWER");
    Serial.println("                           2023");
    Serial.println();
    Serial.println("----------------------------------------------------------");
    Serial.println();

    Serial.print("(s) State Machine:                      ");
    Serial.println(state_Machine_Enable ? "Enabled" : "Disabled");

    Serial.print("State:                                  ");
    switch (State)
    {
      case StartUp:
        Serial.println("StartUp");
        break;

      case Optimize:
        Serial.println("Optimize");
        break;

      case Regulate:
        Serial.println("Regulate");
        break;

      case EStop_Safety:
        Serial.println("EStop Safety");
        break;

      case Discontinuity_Safety:
        Serial.println("Discontinuity Safety");
        break;

      case EStop_Safety_Restart:
        Serial.println("EStop Safety Restart");
        break;

      case Discontinuity_Safety_Restart:
        Serial.println("Discontinuity Safety Restart");
        break;

      default:
        Serial.println("Error");
        break;
    }

    Serial.print("(p) PCC Relay:                          ");
    Serial.println(PCC_Relay ? "On" : "Off");

    Serial.print("Emergency Switch:                       ");
    Serial.println(E_Stop ? "On" : "Off");


    Serial.print("Turbine Side Voltage:                   ");
    Serial.println(digitalRead(PCC_Disconnect_Pin) ? "Low" : "High");

    Serial.print("Load Discontinuity:                     ");
    Serial.println(PCC_Disconnected ? "True" : "False");
    Serial.println();

    Serial.print("(t) Theta (0° - 95°):                   ");
    Serial.print(theta);
    Serial.println("°");

    Serial.print("Real Theta (From Linear Actuator):      ");
    Serial.print(actual_Theta);
    Serial.println("°");

    Serial.println();

    Serial.print("(l) Load Resistance Tracking:           ");
    Serial.println(load_Resistance_Tracking_Enable ? "Enabled" : "Disabled");

    Serial.print("(r) Effective Load Resistance:          ");
    Serial.print(Effective_Load_Resistance);
    Serial.println(" Ω");

    Serial.print("(v) Load DAC Voltage ( 0mV - 3300mV ):  ");
    Serial.print(DAC_Voltage);
    Serial.println(" mV");

    Serial.println();
    Serial.print("RPM:                                    ");
    Serial.println(RPM_Filtered);
    Serial.println();

    Serial.print("Load Voltage:                           ");
    Serial.print(L_Voltage);
    Serial.println(" mV");

    Serial.print("Load Current:                           ");
    Serial.print(L_Current);
    Serial.println(" mA");

    Serial.print("Load Power:                             ");
    Serial.print(L_Power);
    Serial.println(" mW");

    Serial.println("----------------------------------------------------------");

    Serial.println();
  }
  if (Serial.available() > 0)
  {
    uint8_t cmd = Serial.read();

    switch (cmd)
    {
      case 's':
        state_Machine_Enable = !state_Machine_Enable;
        break;

      case 'l':
        load_Resistance_Tracking_Enable = !load_Resistance_Tracking_Enable;
        break;

      case 'p':
        PCC_Relay = !PCC_Relay;
        digitalWrite(PCC_Relay_Pin, PCC_Relay);
        break;

      case 't':
        theta = Serial.parseFloat();
        set_Theta();
        break;

      case 'v':
        DAC_Voltage = Serial.parseFloat();
        break;

      case 'r':
        Effective_Load_Resistance = Serial.parseFloat();
        break;

      default:
        Serial.println("Command not recognized");
        break;
    }
  }
}

void VIratio_optimize_load() {
  VIratio = 1 / Effective_Load_Resistance;
  if (load_Resistance_Tracking_Enable) {
    if (L_Power > regulate_Power_Setpoint) {
      DAC_Voltage_Previous = DAC_Voltage;                          //Check if power regulation is needed
      DAC_Voltage = DAC_Voltage - DAC_Perturbation;          //Regulate power
    }
    else {
      if (L_Voltage > 5) {
        if (L_Current - (VIratio * L_Voltage) < 0) {           //if the expression is negative, then load voltage is too high and current can be increased
          DAC_Voltage_Previous = DAC_Voltage;
          DAC_Voltage = DAC_Voltage + DAC_Perturbation;
        }
        else if (L_Current - (VIratio * L_Voltage) > 0) {        //if the expression is positive, then load current is too high and must be reduced
          DAC_Voltage_Previous = DAC_Voltage;
          DAC_Voltage = DAC_Voltage - DAC_Perturbation;
        }
        else {
          DAC_Voltage = DAC_Voltage - DAC_Perturbation;
        }
      }
    }
  }
}


void RPM_Interupt() {
  TempStore_RPM = micros();
  Timer_RPM_Timeout = millis();
  if (digitalRead(RPM_Pin) == HIGH) {
    if (FirstRead_RPM) {
      Htime1 = TempStore_RPM;                                               //capture first rising edge time
      FirstRead_RPM = false;
    }
    else {
      Htime2 = TempStore_RPM;                                               //capture second rising edge time
      FirstRead_RPM = true;
      //noInterrupts();
      detachInterrupt(digitalPinToInterrupt(RPM_Pin));
      RPM_Raw = 15000000 / (Htime2 - Htime1);                               //calculate rpm from period
      if (RPM_Raw < 6000) {
        if (abs(RPM_Raw - RPM_Raw_Previous) < 600) {                        //compare new rpm to previous
          if ( i < 9 ) {                                                    //cycle through array
            i++;
          }
          else {
            i = 0;
          }
          RPM_Raw_Previous = RPM_Raw;                                       //set comparative value
          RPMSum = RPMSum - Saved_RPM [i];                                  //subtract oldest value from sum
          Saved_RPM [i] = RPM_Raw;                                          //insert new value into array
          RPMSum = RPMSum + RPM_Raw;                                        //add new value to sum
          RPM_Filtered = (RPMSum / 10);
          F = 0;                                                            //reset number of fails
        }
        else {
          F ++;                                                             //increment failed compared value
          if ( F > 5 ) {
            RPM_Raw_Previous = RPM_Raw;                                     //reset the comparative value
          }
        }
      }
    }
  }
}
