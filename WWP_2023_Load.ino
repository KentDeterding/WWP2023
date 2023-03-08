#include <Wire.h>                                           //Include library for UART communication
#include <Adafruit_MCP4725.h>                               //Include DAC library
#include <Adafruit_INA260.h>                                //Include power sensor library
#include <PA12.h>
Adafruit_INA260 ina260 = Adafruit_INA260();
Adafruit_MCP4725 dac;                                       //Initailize DAC

//Pin Declarations
#define Linear_Actuator_Enable 16
#define Linear_Actuator_Tx 1
#define Linear_Actuator_Rx 0
#define RPM_Pin 30
#define PCC_Relay_Pin 33
#define EStop_Pin 11
#define PCC_Disconnect_Pin 29                               //Pin reads low for a high voltage on the turbine side (DOES NOT DIRECTLY IMPLY PCC IS DISCONNECTED)

//Linear Actuators
#define ID_NUM 0
PA12 myServo(&Serial1, Linear_Actuator_Enable, 1);
//          (&Serial, enable_pin,  Tx Level)

//Adjustable Variables
uint16_t regulate_RPM_Setpoint = 3000;                      //Maximum RPM value
uint16_t regulate_Power_Setpoint = 25000;                   //Maximum Power Value(mW)
float optimal_Theta = 7;                                    //Theta value for the entirety of the power curve task
float cutin_Theta = 25;                                     //Theta value for startup
float brake_Theta = 95;                                     //Theta value for safety tasks
float DAC_Voltage_Cutin = 0;                                //DAC voltage for startup (mV)

//Active Pitch Variables
uint16_t theta_Position;                                    //Integer value sent to linear actuator (DO NOT WRITE TO)
float theta;                                                //Angle of blades (WRITE TO THIS TO CHANGE PITCH) (0° - 95°)
float actual_Theta;                                         //Angle of blades recieved from linear actuator

//INA260 Reading Variables
uint16_t L_Power = 0;                                       //Load Power (mW)
uint16_t L_Voltage = 0;                                     //Load Voltage (mV)
uint16_t L_Current = 0;                                     //Load Current (mA)

//RPM Reading Variables
volatile uint16_t RPM_Filtered;                             //RPM value after filtering has been done (USE THIS VALUE FOR CALCULATIONS)
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
float DAC_Voltage;                                          //Current Load DAC Voltage
float DAC_Voltage_Previous;                                 //Previous Load DAC Voltage
float DAC_Perturbation = 10;                                //Step size of DAC voltage adjustment during resistance tracking
float DAC_Voltage_Maximum = 3300;                           //Maximum voltage output of load DAC (mV)
float Effective_Load_Resistance = 4;                        //Resistance to which tracking aims to achieve
float VIratio;                                              //Current divided by voltage


//State Machine Variables
enum States {StartUp, Resistance_Tracking, Regulate, EStop_Safety, Discontinuity_Safety, EStop_Safety_Restart, Discontinuity_Safety_Restart};
States State = StartUp;

//Timer Intervals
unsigned Fast_Interval = 10;                                //Runs at this interval: State Machine, E-Stop Checking, Discontinuity Checking
unsigned Medium_Interval = 50;                              //Runs at this interval: RPM Reading
unsigned Slow_Interval = 500;                               //Runs at this interval: Reading INA260, Reading Linear Actuator Position, Resistance Tracking, PC Comms
unsigned EStop_Safety_Restart_Interval = 2000;              //Time from E-Stop button being returned to normal to entering Resistance Tracking state
unsigned EStop_Safety_CoolDown_Interval = 5000;
unsigned Discontinuity_Safety_Restart_Interval = 3000;      //Time from PCC being recconected to entering Resistance Tracking state (longer to allow cap charging)
unsigned Discontinuity_Safety_CoolDown_Interval = 5000;
unsigned Pitch_Transient_Interval = 800;                    //Time between pitch adjustments during RPM regulation
unsigned RPM_Timeout_Interval = 200;                        //After this amount of time without an RPM reading, RPM is set to zero
unsigned Discontinuity_Power_Down_Interval = 5000;          //Time interval needed for turbine side caps to discharge

//Timers
unsigned long Timer_Fast;
unsigned long Timer_Medium;
unsigned long Timer_Slow;
unsigned long Timer_EStop_Safety_Restart;
unsigned long Timer_EStop_Safety_CoolDown;
unsigned long Timer_Discontinuity_Safety_Restart;
unsigned long Timer_Discontinuity_Safety_CoolDown;
unsigned long Timer_RPM_Transient;
unsigned long Timer_RPM_Timeout;
unsigned long Timer_Discontinuity_Power_Down;

bool load_Resistance_Tracking_Enable = false;
bool E_Stop = false;                                        //True if E-Stop button pressed
bool PCC_Disconnected = false;                              //True if PCC disconnected
bool PCC_Relay = false;                                     //If set true, PCC will supply power from load side to turbine side
bool state_Machine_Enable = true;                           //Set to false if wanting to do manual testing


void setup() {
  //Linear Actuator
  myServo.begin(32);
  myServo.movingSpeed(ID_NUM, 800);                         //Set speed of linear actuator movement
  Serial.begin(9600);                                       //Begin serial monitor
  dac.begin(0x66);                                          //Initialize DAC
  ina260.begin();                                           //Initialize INA 260

  initialize_Pins();                                        //Initialize all pins used in the program

  PCC_Relay = true;                                         //Turn on PCC relay to allow blades to pitch to cut-in pitch
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
  delay (4000);                                             //Allow time for turbine side caps to charge
  theta = cutin_Theta;                                      //Set theta to best pitch for cut-in
  set_Theta();
  delay (4000);                                             //Allow time for pitching to occur
  PCC_Relay = false;
  digitalWrite(PCC_Relay_Pin, PCC_Relay);                   //Turn off PCC to allow power to flow through load
  DAC_Voltage = DAC_Voltage_Cutin;                          //Set to low load for better cut-in

  initialize_Timers();                                      //Set all timers to current millis() value
}

void loop() {
  if (millis() - Timer_Fast >= Fast_Interval)
  {
    Timer_Fast = millis();
    read_EStop();                                           //Read signal from E-Stop switch
    manage_State();                                         //Run state machine
  }

  if (millis() - Timer_Medium >= Medium_Interval)
  {
    Timer_Medium = millis();
    read_RPM();                                             //Gather RPM
  }

  if (millis() - Timer_Slow >= Slow_Interval)
  {
    Timer_Slow = millis();
    read_INA260();                                          //Gather load power, voltage, and current
    read_Linear_Actuator_Position();                        //Gather actual linear actuator posistion
    Resistance_Track_Load();                                //Find DAC voltage setpoint
    set_Load();                                             //Set DAC voltage
    PC_Comms ();                                            //Check serial monitor and print to serial monitor
  }
}

void manage_State() {
  if (state_Machine_Enable) {
    switch (State)
    {
      case StartUp:                                         //Only enters upon restart of micro
        if (RPM_Filtered > 700) {                           //Transistion out of start-up once RPM is achieved
          State = Resistance_Tracking;                      //Enter normal operation state
          load_Resistance_Tracking_Enable = true;           //Allow load to maximize power output
          theta = optimal_Theta;                            //Set pitch to optimal
          set_Theta();
        }
        break;

      case Resistance_Tracking:                             //Operates in this state during the duration of the power curve
        if (E_Stop && (millis() - Timer_EStop_Safety_CoolDown >= EStop_Safety_CoolDown_Interval))
        {
          State = EStop_Safety;                             //Enter estop state when button is pushed and the cooldown period is over
        }
        if (!digitalRead(PCC_Disconnect_Pin) && (ina260.readBusVoltage() < 50) && (millis() - Timer_Discontinuity_Safety_CoolDown >= Discontinuity_Safety_CoolDown_Interval))
        {
          State = Discontinuity_Safety;                     //Enter PCC Discontinuity when there is a high voltage on turbine side, low voltage on load side, and cooldown period is over 
          Timer_Discontinuity_Power_Down = millis();        //Begin timer that allows time for turbine caps to discharge
        }
        if (RPM_Filtered > regulate_RPM_Setpoint)
        {
          State = Regulate;                                 //Enter RPM regulation when RPM exceeds the setpoint
        }
        break;

      case Regulate:                                        //Operates in this state during the durabilty task, this state regulates RPM
        regulate_RPM();
        if (E_Stop && (millis() - Timer_EStop_Safety_CoolDown >= EStop_Safety_CoolDown_Interval))
        {
          State = EStop_Safety;                             //Enter estop state when button is pushed and the cooldown period is over
        }
        if (!digitalRead(PCC_Disconnect_Pin) && (ina260.readBusVoltage() < 50)&& (millis() - Timer_Discontinuity_Safety_CoolDown >= Discontinuity_Safety_CoolDown_Interval))
        {
          State = Discontinuity_Safety;                     //Enter PCC Discontinuity when there is a high voltage on turbine side, low voltage on load side, and cooldown period is over
          Timer_Discontinuity_Power_Down = millis();        //Begin timer that allows time for turbine caps to discharge
        }
        if (RPM_Filtered < 0.85 * regulate_RPM_Setpoint)
        {
          State = Resistance_Tracking;                      //Once RPM is below the setpoint, enter back into normal state
          theta = optimal_Theta;                            //Set pitch to optimal
          set_Theta();
        }
        break;

      case EStop_Safety:
        PCC_Relay = true;                                   //Provide power to turbine side from the load side
        //digitalWrite(PCC_Relay_Pin, PCC_Relay);
        theta = brake_Theta;                                //Feather blades
        set_Theta();
        load_Resistance_Tracking_Enable = false;            //Quit changing load DAC voltage
        if (!E_Stop)
        {
          State = EStop_Safety_Restart;                     //Once button isn't pushed, enter restart state
          Timer_EStop_Safety_Restart = millis();
        }
        break;

      case EStop_Safety_Restart:
        theta = optimal_Theta;                              //Set pitch to optimal
        set_Theta();
        if ((millis() - Timer_EStop_Safety_Restart >= EStop_Safety_Restart_Interval) && (RPM_Filtered > 500))
        {                                                   //Allow time to get to correct pitch and come up to speed
          load_Resistance_Tracking_Enable = true;           //Allow load to maximize power output
          PCC_Relay = false;                                //Turn off PCC Relay
          //digitalWrite(PCC_Relay_Pin, PCC_Relay);
          State = Resistance_Tracking;                      //Return to normal state
          Timer_EStop_Safety_CoolDown = millis();           //Reset estop cooldown, the system cannot reenter the estop safety state during this time
        }
        break;

      case Discontinuity_Safety:
        PCC_Disconnected = true;                            //Show PCC is disconnected on serial monitor
        PCC_Relay = true;                                   //Provide power to turbine side from the load side
        //digitalWrite(PCC_Relay_Pin, PCC_Relay);
        theta = brake_Theta;                                //Feather blades
        set_Theta();
        load_Resistance_Tracking_Enable = false;            //Quit changing load DAC voltage
        if (millis() - Timer_Discontinuity_Power_Down >= Discontinuity_Power_Down_Interval)
        {                                                   //Allow time for turbine caps to discharge
          if (!digitalRead(PCC_Disconnect_Pin)) {           //Wait for turbine power to be restored, cable reconnection
            Timer_Discontinuity_Safety_Restart = millis();  
            State = Discontinuity_Safety_Restart;           //Enter restart state
            PCC_Disconnected = false;                       //Show PCC is connected on serial monitor
          }
        }
        break;

      case Discontinuity_Safety_Restart:
        theta = optimal_Theta;                              //Set pitch to optimal
        set_Theta();
        if ((millis() - Timer_Discontinuity_Safety_Restart >= Discontinuity_Safety_Restart_Interval) && (RPM_Filtered > 500))
        {                                                   //Allow time to get to correct pitch and come up to speed
          load_Resistance_Tracking_Enable = true;           //Allow load to maximize power output
          PCC_Relay = false;                                //Turn off PCC Relay
          //digitalWrite(PCC_Relay_Pin, PCC_Relay);
          Timer_Discontinuity_Safety_CoolDown = millis();   //Reset discontinuity cooldown, the system cannot reenter the discontinuity safety state during this time
          State = Resistance_Tracking;                      //Return to normal state
        }
        break;

      default:
        State = StartUp;                                    //Default to startup state
        load_Resistance_Tracking_Enable = false;            //Allow load to maximize power output
        break;
    }
  }
  digitalWrite(PCC_Relay_Pin, PCC_Relay);
}

void initialize_Timers() {                                  //Initialize all timers, only occurs upon startup
Timer_Fast = millis();
Timer_Medium = millis();
Timer_Slow = millis();
Timer_EStop_Safety_Restart = millis();
Timer_EStop_Safety_CoolDown = millis();
Timer_Discontinuity_Safety_Restart = millis();
Timer_Discontinuity_Safety_CoolDown = millis();
Timer_RPM_Transient = millis();
Timer_RPM_Timeout = millis();
Timer_Discontinuity_Power_Down = millis();
}

void initialize_Pins() {                                    //Initialize all pins, only occurs upon startup
  pinMode(Linear_Actuator_Enable, OUTPUT);
  pinMode(RPM_Pin, INPUT);
  pinMode(PCC_Relay_Pin, OUTPUT);
  pinMode(EStop_Pin, INPUT);
  pinMode(PCC_Disconnect_Pin, INPUT);
}

void read_EStop() {
  E_Stop = digitalRead(EStop_Pin);                          //Read estop pin
}

void read_INA260() {
  L_Power = ina260.readPower();                             //Read load power
  L_Voltage = ina260.readBusVoltage();                      //Read load voltage
  L_Current = ina260.readCurrent();                         //Read load current

}

void read_Linear_Actuator_Position() {                      //Gather linear actuator position and convert it to degrees (0-95)
  actual_Theta = (myServo.presentPosition(ID_NUM) - 208.084) / 31.3479;
}

void regulate_RPM() {
  if (millis() - Timer_RPM_Transient >= Pitch_Transient_Interval)
  {                                                         //Only change RPM once per transient interval
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

      case Resistance_Tracking:
        Serial.println("Resistance Tracking");
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

void Resistance_Track_Load() {
  VIratio = 1 / Effective_Load_Resistance;
  if (load_Resistance_Tracking_Enable) {
    if (L_Power > regulate_Power_Setpoint) {
      DAC_Voltage_Previous = DAC_Voltage;                   //Check if power regulation is needed
      DAC_Voltage = DAC_Voltage - DAC_Perturbation;         //Regulate power
      //DAC_Voltage = DAC_Voltage - sqrt(L_Power - regulate_Power_Setpoint) * XXXX;
    }
    else {
      if (L_Voltage > 50) {
        if (L_Current - (VIratio * L_Voltage) < 0) {        //if the expression is negative, then load voltage is too high and current can be increased
          if (L_Power > 0.85 * regulate_Power_Setpoint) {
            DAC_Voltage = DAC_Voltage + DAC_Perturbation;         //Regulate power
            //DAC_Voltage = DAC_Voltage + sqrt(L_Power - regulate_Power_Setpoint) * XXXX;
          }
          else {
          DAC_Voltage = DAC_Voltage - (L_Current - (VIratio * L_Voltage)) * .03; //increased DAC voltage by an amount
          }
          DAC_Voltage_Previous = DAC_Voltage;
        }
        else if (L_Current - (VIratio * L_Voltage) > 0) {   //if the expression is positive, then load current is too high and must be reduced
          DAC_Voltage_Previous = DAC_Voltage;
          DAC_Voltage = DAC_Voltage - (L_Current - (VIratio * L_Voltage)) * .03; //decrease DAC voltage by an amount 
        }
      }
      else {
        DAC_Voltage = DAC_Voltage - DAC_Perturbation;
      }

    }
  }
}


void RPM_Interupt() {
  TempStore_RPM = micros();
  Timer_RPM_Timeout = millis();
  if (digitalRead(RPM_Pin) == HIGH) {
    if (FirstRead_RPM) {
      Htime1 = TempStore_RPM;                               //capture first rising edge time
      FirstRead_RPM = false;
    }
    else {
      Htime2 = TempStore_RPM;                               //capture second rising edge time
      FirstRead_RPM = true;
      //noInterrupts();
      detachInterrupt(digitalPinToInterrupt(RPM_Pin));
      RPM_Raw = 15000000 / (Htime2 - Htime1);               //calculate rpm from period
      if (RPM_Raw < 6000) {
        if (abs(RPM_Raw - RPM_Raw_Previous) < 600) {        //compare new rpm to previous
          if ( i < 9 ) {                                    //cycle through array
            i++;
          }
          else {
            i = 0;
          }
          RPM_Raw_Previous = RPM_Raw;                       //set comparative value
          RPMSum = RPMSum - Saved_RPM [i];                  //subtract oldest value from sum
          Saved_RPM [i] = RPM_Raw;                          //insert new value into array
          RPMSum = RPMSum + RPM_Raw;                        //add new value to sum
          RPM_Filtered = (RPMSum / 10);
          F = 0;                                            //reset number of fails
        }
        else {
          F ++;                                             //increment failed compared value
          if ( F > 5 ) {
            RPM_Raw_Previous = RPM_Raw;                     //reset the comparative value
          }
        }
      }
    }
  }
}
