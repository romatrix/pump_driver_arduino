
const int TIMEOUT_POTENTIOMETER   = A0;    // select the input pin for the potentiometer
const int PRESSURE_SENSOR = A1;
const int ERR_LED = 2;
const int POMP = 3;
const int PRESSURE_POTENTIOMETER = A4;
const int WATER_IN = 5;
const int WATER_OUT = 7;

unsigned long lastMeasuredTime = 0;


/////////////////////////////////////////////////////////////////////////// DEBOUNCER //////////////////////////////////////////////////////////////////////////////////////
/// \brief The Debouncer class
class IDebouncer
{
    public:
    virtual bool getState(int &state) = 0;
    virtual void update(unsigned long delta) = 0;
};

class IPotentiometerDataListener
{
  public:
  virtual void onValueChanged(int value, int port) = 0;
};

/////////////////////////////////////////////////////////////////////////// POTENTIOMETER DATA READER //////////////////////////////////////////////////////////////////////////////////////
class PotentiometerDataReader
{
  public:
  void setListener(IPotentiometerDataListener *listener)
  {
    m_listener = listener;
  }

  void init(int pin)
  {
    m_pin = pin;
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
      m_listener->onValueChanged(value, m_pin);
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
    return analogRead(m_pin);
  }
  IPotentiometerDataListener *m_listener = nullptr;

  int m_lastValue = 0;
  const int TRESHOLD = 50;
  int m_pin = 0;
};


class DigitalDebouncer: public IDebouncer
{
public:
  DigitalDebouncer(int port, unsigned long stableTime): m_port(port),
                                   m_stableTime(stableTime)
                                   {}

  bool getState(int &state) override
  {
    if(m_stable){
      state = m_lastState;
    }

    return m_stable;
  }

  void update(unsigned long delta) override
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


class AnalogDebouncer: public IDebouncer
{
public:
  AnalogDebouncer(int port, unsigned long stableTime, int minOffset): m_port(port),
                                   m_stableTime(stableTime), m_minOffset(minOffset)
                                   {
  }

  bool getState(int &state) override
  {
    if(m_stable){
      state = m_lastState;
    }

    return m_stable;
  }

  void update(unsigned long delta) override
  {
      int value = readSensorValue();
      int state = getSensorState(value);

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

  int readSensorValue()
  {
      return analogRead(m_port) - m_minOffset;
  }

  int getSensorState(int value)
  {
    if(value > m_stateChangeTreshold){
      return true;
    } else if (value < m_stateChangeTreshold - TRESHOLD){
      return false;
    } else {
      return m_lastState;
    }
  }

  void setStateChangeTreshold(int treshold)
  {
    m_stateChangeTreshold = treshold;
  }

  bool m_stable = false;
  int m_port;
  int m_lastState = -1;
  unsigned long m_stableTime;
  unsigned long m_currentTime = 0;
  int m_minOffset;
  int m_stateChangeTreshold = -1;
  const int TRESHOLD = 50;
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


/////////////////////////////////////////////////////////////////////////// SENSOR //////////////////////////////////////////////////////////////////////////////////////
class Sensor
{
  public:
  Sensor(IDebouncer &debouncer, IStateListener *stateListener):
                                               m_debouncer(debouncer),
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
  IDebouncer& m_debouncer;
  IStateListener *m_stateListener = nullptr;
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

///////////////////////////////////////
class WaterInput : public IStateListener
{
  static constexpr int STABLE_TIME = 100;

  public:
  WaterInput(IWaterInListener &listener):m_debouncer(WATER_IN, STABLE_TIME),
                            m_sensor(m_debouncer, this),
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

  DigitalDebouncer m_debouncer;
  Sensor m_sensor;
  IWaterInListener& m_listener;
};

/////////////////////////////////////////////////////////////////////////// WATER OUTPUT //////////////////////////////////////////////////////////////////////////////////////
class IWaterOutListener
{
  public:
  virtual void onWaterOut(bool state) = 0;
};

///////////////////////////////////////
class WaterOutput : public IStateListener
{
  static constexpr int STABLE_TIME = 500;
  static constexpr int MIN_VALUE = 90;

  public:
  WaterOutput(IWaterOutListener& listener):m_debouncer(PRESSURE_SENSOR, STABLE_TIME, MIN_VALUE),
                            m_sensor(m_debouncer, this),
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

  void setMaxPressureValue(int value)
  {
    m_debouncer.setStateChangeTreshold(value);
  }

  AnalogDebouncer m_debouncer;
  Sensor m_sensor;
  IWaterOutListener& m_listener;
};


/////////////////////////////////////////////////////////////////////////// POMP DRIVER //////////////////////////////////////////////////////////////////////////////////////
class PompDriver: public IPotentiometerDataListener, IWaterInListener, IWaterOutListener, IAlarmListener
{
  constexpr static int NO_WATER = 0;
  constexpr static int WATER_OFF = 1;

  constexpr static int NO_WATER_TIMEOUT = 10000;
  constexpr static int DEFAULT_WATER_OFF_TIMEOUT = 10000;

  public:
  PompDriver(): m_waterInput(*this),
                m_waterOutput(*this)
  {}

  void update(unsigned long delta)
  {
    if(m_motor.waterErrorOccured()){
      return;
    }

    m_waterOutput.update(delta);
    m_waterInput.update(delta);

    m_waterAlarm.update(delta, m_waterInput.getState());
  }

  void onValueChanged(int value, int port) override
  {
    switch(port){
    case TIMEOUT_POTENTIOMETER:
        setTimeoutValue(value);
        break;
    case PRESSURE_POTENTIOMETER:
        setMaxPressureValue(value);
        break;
    }
  }

  void setTimeoutValue(int value)
  {
    m_waterOffTimeout = value * 10;

    Serial.print(__FUNCTION__);
    Serial.print(" value:");
    Serial.println(m_waterOffTimeout);
  }

  void setMaxPressureValue(int value)
  {
    Serial.print(__FUNCTION__);
    Serial.print(" value:");
    Serial.println(m_waterOffTimeout);

    m_waterOutput.setMaxPressureValue(value);
  }

  void onWaterOut(bool state) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" state:");
    Serial.println(state);

    if(0 == state){
      m_motor.turnOn();
      m_waterAlarm.setAlarm(this, NO_WATER_TIMEOUT, NO_WATER);
    } else {
        m_motor.turnOff();
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

/////////////////////////////////////////////////////////////////////////// GLOBALS //////////////////////////////////////////////////////////////////////////////////////
PompDriver g_pompDriver;
PotentiometerDataReader g_timeoutValueReader;
PotentiometerDataReader g_maxPressureValueReader;
Timer g_timer;


/////////////////////////////////////////////////////////////////////////// SETUP //////////////////////////////////////////////////////////////////////////////////////
// the setup function runs once when you press reset or power the board

void hardwareSetup()
{
    Serial.begin(115200);

    pinMode(WATER_IN, INPUT);
    //pinMode(WATER_OUT, INPUT);
    pinMode(ERR_LED, OUTPUT);
    pinMode(POMP, OUTPUT);

    digitalWrite(ERR_LED, HIGH);
    digitalWrite(POMP, LOW);
}

void softwareSetup()
{
    g_timeoutValueReader.setListener(&g_pompDriver);
    g_timeoutValueReader.init(TIMEOUT_POTENTIOMETER);

    g_maxPressureValueReader.init(PRESSURE_POTENTIOMETER);
    g_maxPressureValueReader.setListener(&g_pompDriver);
    g_timer.init();
}

void setup()
{
    hardwareSetup();
    softwareSetup();
}


/////////////////////////////////////////////////////////////////////////// LOOP //////////////////////////////////////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  g_timer.measureBegin();

  g_timeoutValueReader.update();
  g_maxPressureValueReader.update();

  delay(100);
  g_pompDriver.update(g_timer.getLapsedTime());

  g_timer.measureEnd();
}
