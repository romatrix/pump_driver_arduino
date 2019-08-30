/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

const int POTENTIOMETER = A0;    // select the input pin for the potentiometer
//int sensorValue = 0;  // variable to store the value coming from the sensor
//int digitalVal[2] = {0};
const int ERR_LED = 2;
const int POMP = 3;
const int WATER_IN = 5;
const int WATER_OUT = 7;
//int errorLed = 0;

unsigned long lastMeasuredTime = 0;

//class Log
//{
//  public:
//  enum LEVEL
//  {
//    TRACE,
//    DEBUG,
//    INFO,
//    ERROR
//  }
//  
//  Log(LEVEL level)
//  {
//    
//  }
//  
//  void operator()(const char* fun, const char* info)
//  {
//    Serial.print(fun);
//    Serial.println(info);
//  }
//};


/////////////////////////////////////////////////////////////////////////// DEBOUNCER //////////////////////////////////////////////////////////////////////////////////////
class Debouncer
{
public:
  Debouncer(int port, unsigned long stableTime): m_port(port),
                                   m_stableTime(stableTime)
                                   {}

  bool getState(int &state)
  {
    if(m_stable){
      state = m_lastState;
    }

    return m_stable;
  }

  void update(unsigned long delta)
  {
      int state = digitalRead(m_port);

      if(state == m_lastState){
        m_currentTime += delta;
      } else {
        m_currentTime = 0;
        m_stable = false;
      }

      if(m_currentTime >= m_stableTime){
        m_currentTime = m_stableTime;
        m_stable = true;
      }

      m_lastState = state;
  }

  bool m_stable = false;
  int m_port;
  int m_lastState = -1;
  unsigned long m_stableTime;
  unsigned long m_currentTime = 0;
};

class IAlarmListener
{
  public:
  virtual void onTimeout(int id) = 0;
};

class IStateListener
{
  public:
  virtual void onChangeState(int oldState, int newState) = 0;
};

class ITimeoutValueListener
{
  public:
  virtual void onValueChanged(int value) = 0;
};


/////////////////////////////////////////////////////////////////////////// SENSOR //////////////////////////////////////////////////////////////////////////////////////
class Sensor
{
  public:
  Sensor(int port, unsigned long stableTime, IStateListener *stateListener):
                                               m_port(port),
                                               m_debouncer(port, stableTime),
                                               m_stateListener(stateListener)
                                               {}
  void update(unsigned long delta)
  {
    int state = 0;
    m_debouncer.update(delta);
    
    if(!m_debouncer.getState(state)){
      Serial.print(__FUNCTION__);
      Serial.print(" debouncer not stable, object:");
      Serial.println((int)this, HEX);
      return;
    }

    if(state != m_lastState){
      onChangeState(state);
      m_lastState = state;
    }
  }

  void onChangeState(int state)
  {
//    Serial.print(__FUNCTION__);
//    Serial.print(" state:");
//    Serial.println(state);
//    ;
    if(!m_stateListener){
      return;
    }

    m_stateListener->onChangeState(m_lastState, state);
  }

  int getState()
  {
    return m_lastState;
  }

  int m_lastState = -1;
  Debouncer m_debouncer;
  IStateListener *m_stateListener = nullptr;

  int m_port;
};

/////////////////////////////////////////////////////////////////////////// ALARM //////////////////////////////////////////////////////////////////////////////////////
class Alarm
{
  public:

  void update(unsigned long delta, int state)
  {
    if(!m_alarmListener){
      return;
    }

    if(1 == state || m_ignoreState)
    {
      m_elapsed += delta;

      if(m_elapsed >= m_timeout)
      {
        onAlarm();
      }
    }else{
        onClearAlarm();
    }
  }

  void onClearAlarm()
  {
    Serial.print(__FUNCTION__);
    Serial.print(" id:");
    Serial.println(m_id);

    m_elapsed = 0;
    m_alarmListener = nullptr;
  }

  void onAlarm()
  {
    Serial.print(__FUNCTION__);
    Serial.print(" id:");
    Serial.println(m_id);
    m_alarmListener->onTimeout(m_id);
    m_alarmListener = nullptr;
  }

  void setAlarm(IAlarmListener* alarmListener, unsigned long timeout, int id, bool ignoreState = false)
  {
    Serial.print(__FUNCTION__);
    Serial.print("id:");
    Serial.println(id);
    m_ignoreState = ignoreState;
    m_timeout = timeout;
    m_elapsed = 0;
    m_alarmListener = alarmListener;
    m_id = id;
  }

  IAlarmListener* m_alarmListener = nullptr;
  int m_lastState = 0;
  unsigned long m_elapsed = 0;
  unsigned long m_timeout = 0;
  bool m_ignoreState = false;
  int m_id = -1;
};

/////////////////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////////////////////////////
class Motor
{
  public:

  void turnOn()
  {
    Serial.println(__FUNCTION__);
    digitalWrite(POMP, HIGH);
  }

  void turnOff()
  {
    Serial.println(__FUNCTION__);
    digitalWrite(POMP, LOW);
  }

  void onWaterError()
  {
    Serial.println(__FUNCTION__);
    digitalWrite(ERR_LED, LOW);
    digitalWrite(POMP, LOW);
    m_waterError = true;
  }

  void onClearWaterError()
  {
    Serial.println(__FUNCTION__);
    digitalWrite(ERR_LED, HIGH);
    m_waterError = false;
    //digitalWrite(POMP, HIGH);
  }

  bool waterErrorOccured()
  {
    return m_waterError;
  }

  bool m_waterError = false;
};

/////////////////////////////////////////////////////////////////////////// WATER INPUT //////////////////////////////////////////////////////////////////////////////////////
class IWaterInListener
{
  public:
  virtual void onWaterIn(bool state) = 0;
};

class WaterInput : public IStateListener
{
  public:
  WaterInput(IWaterInListener &listener):m_sensor(WATER_IN, 100, this),
                            m_listener(listener)
  {}

  void update(unsigned long delta)
  {
    m_sensor.update(delta);
  }

  void onChangeState(int oldState, int newState) override
  {
    m_listener.onWaterIn(newState);
  }

  int getState()
  {
    return m_sensor.getState();
  }

  Sensor m_sensor;
  IWaterInListener& m_listener;
};

/////////////////////////////////////////////////////////////////////////// WATER OUTPUT //////////////////////////////////////////////////////////////////////////////////////
class IWaterOutListener
{
  public:
  virtual void onWaterOut(bool state) = 0;
};

class WaterOutput : public IStateListener
{
  public:
  WaterOutput(IWaterOutListener& listener):m_sensor(WATER_OUT, 100, this),
                            m_listener(listener)
                            {}
  
  void update(unsigned long delta)
  {
    m_sensor.update(delta);
  }

  void onChangeState(int oldState, int newState) override
  {
      m_listener.onWaterOut(newState);
  }

  int getState()
  {
    return m_sensor.getState();
  }

  Sensor m_sensor;
  IWaterOutListener& m_listener;
};


/////////////////////////////////////////////////////////////////////////// POMP DRIVER //////////////////////////////////////////////////////////////////////////////////////
class PompDriver: public ITimeoutValueListener, IWaterInListener, IWaterOutListener, IAlarmListener
{
  constexpr static int NO_WATER = 0;
  constexpr static int WATER_OFF = 1;
  
  constexpr static int NO_WATER_TIMEOUT = 10000;
  constexpr static int DEFAULT_WATER_OFF_TIMEOUT = 10000;
  
  public:
  PompDriver(): m_waterInput(*this),
                m_waterOutput(*this)
                
  {
    
  };

  void update(unsigned long delta)
  {
    if(m_motor.waterErrorOccured()){
      return;
    }
    
    m_waterOutput.update(delta);
    m_waterInput.update(delta);

    m_waterAlarm.update(delta, m_waterInput.getState());
  }

  void onValueChanged(int value) override
  {
    m_waterOffTimeout = value * 10;

    Serial.print(__FUNCTION__);
    Serial.print(" value:");
    Serial.println(m_waterOffTimeout);
  }

  void onWaterOut(bool state) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" state:");
    Serial.println(state);
    
    if(0 == state){
      m_motor.turnOn();
      m_waterAlarm.setAlarm(this, NO_WATER_TIMEOUT, NO_WATER);
    }
  }

  void onWaterIn(bool state) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" state:");
    Serial.println(state);
    
    if(1 == state){
      if(1 == m_waterOutput.getState()){
        m_waterAlarm.setAlarm(this, m_waterOffTimeout, WATER_OFF, true);
      }
    }
  }

  void onTimeout(int id) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" id:");
    Serial.println(id);
    
    switch(id){
      case WATER_OFF:
        m_motor.turnOff();
      break;
      case NO_WATER:
        m_motor.onWaterError();
      break;
      default:
      ;//ERROR
    }
  }
  
  Motor m_motor;
  WaterInput m_waterInput;
  WaterOutput m_waterOutput;
  unsigned long m_waterOffTimeout = DEFAULT_WATER_OFF_TIMEOUT;
  Alarm m_waterAlarm;
};


/////////////////////////////////////////////////////////////////////////// TIMEOUT READER //////////////////////////////////////////////////////////////////////////////////////
class TimeoutReader
{
  public:
  void setListener(ITimeoutValueListener *listener)
  {
    m_listener = listener;
  }

  void init()
  {
    int value = readValue();
    onValueChanged(value);
  }
  
  void update()
  {
    int value = readValue();

//    Serial.print(__PRETTY_FUNCTION__);
//    Serial.print(" value:");
//    Serial.println(value);


    if(valueNotChanged(value)){
      return;
    } else {
      onValueChanged(value); 
    }
  }

  void onValueChanged(int value)
  {
    m_lastValue = value;

    if(m_listener){
      m_listener->onValueChanged(value);
    }
  }

  bool valueNotChanged(int value)
  {
    if(value > m_lastValue){
      return value - m_lastValue <= TRESHOLD;
    } else {
      return m_lastValue- value <= TRESHOLD;
    }
  }

  int readValue()
  {
    return analogRead(POTENTIOMETER);
  }
  ITimeoutValueListener *m_listener = nullptr;
  
  int m_lastValue = 0;
  const int TRESHOLD = 50;
};

/////////////////////////////////////////////////////////////////////////// TIMER //////////////////////////////////////////////////////////////////////////////////////
class Timer
{
  public:
  unsigned long m_lastTime = 0;
  unsigned long m_currentTime = 0;

  void init()
  {
    m_lastTime = millis();
  }

  void measureBegin()
  {
    m_currentTime = millis();
  }

  unsigned long getLapsedTime()
  {
    return m_currentTime - m_lastTime;
  }

  void measureEnd()
  {
    m_lastTime = m_currentTime;
  }
};

PompDriver g_pompDriver;
TimeoutReader g_timeoutReader;
Timer g_timer;


/////////////////////////////////////////////////////////////////////////// SETUP //////////////////////////////////////////////////////////////////////////////////////
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  //pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(POTENTIOMETER, INPUT);
  
  pinMode(WATER_IN, INPUT);
  pinMode(WATER_OUT, INPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(POMP, OUTPUT);

  digitalWrite(ERR_LED, HIGH);
  digitalWrite(POMP, LOW);
  
  Serial.begin(115200);
  //Serial.print("qwqwqw");

  g_timeoutReader.setListener(&g_pompDriver);
  g_timeoutReader.init();
  g_timer.init();
}

//int getOutputTimout()
//{
//  return analogRead(POTENTIOMETER) / 100;
//}

//unsigned long lapsedTime()


/////////////////////////////////////////////////////////////////////////// LOOP //////////////////////////////////////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  g_timer.measureBegin();
  
  g_timeoutReader.update();
  delay(100);
  g_pompDriver.update(g_timer.getLapsedTime());
  
  g_timer.measureEnd();
//  unsigned long lapsedTime = millis();
//  //sensorValue = analogRead(sensorPin);
//  digitalVal[0] = digitalRead(5);
//  digitalVal[1] = digitalRead(7);
//  Serial.println(digitalVal[0]);
//  Serial.println(digitalVal[1]);
//
//  delay(1000);                       // wait for a second
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  digitalWrite(ERR_LED, HIGH);
//  digitalWrite(POMP, HIGH);
//  
//  delay(1000);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  digitalWrite(ERR_LED, LOW);
//  digitalWrite(POMP, LOW);
}
