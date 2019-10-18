
const int LOW_PRESSURE_POTENTIOMETER   = A0;    // select the input pin for the potentiometer
const int HIGH_PRESSURE_POTENTIOMETER = A1;
const int ERR_LED = 2;
const int POMP = 3;
const int RESET = 4;
const int PRESSURE_SENSOR = A4;
const int WATER_IN = 5;
const int WATER_OUT = 7;

unsigned long lastMeasuredTime = 0;
String SIM_ON = "SIM ON";
String SIM_OFF = "SIM OFF";
bool g_simulatedSensor = false;
int g_simulatedSensorValue = 100;


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
  const int TRESHOLD = 5;
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
      int value= analogRead(m_port) - m_minOffset;

      if(true == g_simulatedSensor){
          value = g_simulatedSensorValue;
      }
//      Serial.print(__FUNCTION__);
//      Serial.print(" value:");
//      Serial.print(value);
//      Serial.print(" raw value:");
//      Serial.println(value + m_minOffset);
      return value;
  }

  int getSensorState(int value)
  {
    if(value > m_highLevelTreshold){
//              Serial.print(__FUNCTION__);
//              Serial.print(" value high:");
//              Serial.println(value);
      return true;
    } else if (value < m_lowLevelTreshold){
//        Serial.print(__FUNCTION__);
//        Serial.print(" value low:");
//        Serial.println(value);
      return false;
    } else {
//        Serial.print(__FUNCTION__);
//        Serial.print(" value treshold:");
//        Serial.println(value);
      return m_lastState != -1 ? m_lastState : false;
    }
  }

  void setHighLevelTreshold(int treshold)
  {
    m_highLevelTreshold = treshold;
  }

  void setLowLevelTreshold(int treshold)
  {
    m_lowLevelTreshold = treshold;
  }

  bool m_stable = false;
  int m_port;
  int m_lastState = -1;
  unsigned long m_stableTime;
  unsigned long m_currentTime = 0;
  int m_minOffset;
  int m_highLevelTreshold = -1;
  int m_lowLevelTreshold = -1;
  const int TRESHOLD = 200;
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
//      Serial.print(__FUNCTION__);
//      Serial.print(" debouncer not stable, object:");
//      Serial.println((int)this, HEX);
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

  void clearAlarm()
  {
      onClearAlarm();
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
    enum WaterFlowState{
        eWaterFlow,
        eWaterStop
    };

    virtual void onWaterIn(WaterFlowState state) = 0;
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
      switch(newState){
        case 0:
            return m_listener.onWaterIn(IWaterInListener::eWaterFlow);
        case 1:
            return m_listener.onWaterIn(IWaterInListener::eWaterStop);
      }
  }

  IWaterInListener::WaterFlowState getState()
  {
      switch(m_sensor.getState()){
        case 0:
            return IWaterInListener::eWaterFlow;
        case 1:
            return IWaterInListener::eWaterStop;
      }
  }

  DigitalDebouncer m_debouncer;
  Sensor m_sensor;
  IWaterInListener& m_listener;
};

/////////////////////////////////////////////////////////////////////////// WATER OUTPUT //////////////////////////////////////////////////////////////////////////////////////
class IWaterOutListener
{
public:
    enum WaterPressureState{
        eLowWaterPressure,
        eHighWaterPressure
    };

  virtual void onWaterOut(WaterPressureState state) = 0;
};

///////////////////////////////////////
class WaterOutput : public IStateListener
{
  static constexpr int STABLE_TIME = 30;
  static constexpr int MIN_VALUE = 0;

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
      switch(newState){
      case 0:
          m_listener.onWaterOut(IWaterOutListener::eLowWaterPressure);
          break;
      case 1:
          m_listener.onWaterOut(IWaterOutListener::eHighWaterPressure);
          break;
      }

  }

  IWaterOutListener::WaterPressureState getState()
  {
      switch(m_sensor.getState()){
      case 0:
          return IWaterOutListener::eLowWaterPressure;
      case 1:
          return IWaterOutListener::eHighWaterPressure;
      }
  }

  void setHighPressureValue(int value)
  {
    m_debouncer.setHighLevelTreshold(value);
  }

  void setLowPressureValue(int value)
  {
    m_debouncer.setLowLevelTreshold(value);
  }

  AnalogDebouncer m_debouncer;
  Sensor m_sensor;
  IWaterOutListener& m_listener;
};


/////////////////////////////////////////////////////////////////////////// POMP DRIVER //////////////////////////////////////////////////////////////////////////////////////
class PompDriver: public IPotentiometerDataListener, IWaterInListener, IWaterOutListener, IAlarmListener
{
  constexpr static int NO_WATER = 0;

  constexpr static int NO_WATER_TIMEOUT = 10000;
  constexpr static int RESTART_AFTER_WATER_ERROR_TIMEOUT_SEC = 60 * 20;

  public:
  PompDriver(): m_waterInput(*this),
                m_waterOutput(*this)
  {}

  void update(unsigned long delta)
  {
    if(m_motor.waterErrorOccured()){
        m_restartAfterWaterError.update(delta, m_waterInput.getState());
      return;
    }

    m_waterOutput.update(delta);
    m_waterInput.update(delta);

    m_waterAlarm.update(delta, m_waterInput.getState());
  }

  void onValueChanged(int value, int port) override
  {
    switch(port){
    case LOW_PRESSURE_POTENTIOMETER:
        setLowPressureValue(value);
        break;
    case HIGH_PRESSURE_POTENTIOMETER:
        setHighPressureValue(value);
        break;
    }
  }

  void setLowPressureValue(int value)
  {
      Serial.print(__FUNCTION__);
      Serial.print(" value:");
      Serial.println(value);

      m_waterOutput.setLowPressureValue(value / 2);
  }

  void setHighPressureValue(int value)
  {
    Serial.print(__FUNCTION__);
    Serial.print(" value:");
    Serial.println(value);

    m_waterOutput.setHighPressureValue(value / 2);
  }

  void onWaterOut(WaterPressureState state) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" state:");
    Serial.println(state);

    if(m_motor.waterErrorOccured()){
        Serial.print(__FUNCTION__);
        Serial.println(" waterErrorOccured, cannot turn on motor");
        return;
    }

    switch(state){
    case eLowWaterPressure:
        m_motor.turnOn();
        m_waterAlarm.setAlarm(this, NO_WATER_TIMEOUT, NO_WATER, true);
    break;
    case eHighWaterPressure:
        m_motor.turnOff();
        m_waterAlarm.clearAlarm();
    break;
    }
  }

  void onWaterIn(WaterFlowState state) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" state:");
    Serial.println(state);

    switch(state){
    case eWaterFlow:
        switch(m_waterOutput.getState()){
        case eHighWaterPressure:
            m_motor.turnOff();
            m_waterAlarm.clearAlarm();
        break;
        case eLowWaterPressure:
            m_waterAlarm.setAlarm(this, NO_WATER_TIMEOUT, NO_WATER, true);
        break;
        }
    break;
    case eWaterStop:
        switch(m_waterOutput.getState()){
        case eLowWaterPressure:
            m_waterAlarm.setAlarm(this, NO_WATER_TIMEOUT, NO_WATER, true);
        break;
        }
    break;
    }
}

  void onTimeout(int id) override
  {
    Serial.print(__FUNCTION__);
    Serial.print(" id:");
    Serial.println(id);

    switch(id){
      case NO_WATER:
        if(eWaterStop == m_waterInput.getState() && eLowWaterPressure == m_waterOutput.getState()){
            m_motor.onWaterError();
            resetBoardAfter(RESTART_AFTER_WATER_ERROR_TIMEOUT_SEC);
        }
      break;
      default:
      ;//ERROR
    }
  }

  void delaySec(int sec)
  {
      for(int i = 0; i < sec; ++i){
          delay(1000);
      }
  }

  void resetBoardAfter(int delayValue)
  {
    Serial.println(__FUNCTION__);

    delaySec(delayValue);
    void(* resetFunc) (void) = 0;
    resetFunc(); //call reset
    Serial.println("dupa");
  }

  Motor m_motor;
  WaterInput m_waterInput;
  WaterOutput m_waterOutput;
  Alarm m_waterAlarm;
  Alarm m_restartAfterWaterError;
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
PotentiometerDataReader g_lowPressureValueReader;
PotentiometerDataReader g_highPressureValueReader;
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

    digitalWrite(RESET, HIGH);
    delay(200);
    pinMode(RESET, OUTPUT);


    digitalWrite(ERR_LED, HIGH);
    digitalWrite(POMP, LOW);
}

void softwareSetup()
{
    g_lowPressureValueReader.setListener(&g_pompDriver);
    g_lowPressureValueReader.init(LOW_PRESSURE_POTENTIOMETER);

    g_highPressureValueReader.setListener(&g_pompDriver);
    g_highPressureValueReader.init(HIGH_PRESSURE_POTENTIOMETER);
    g_timer.init();
}

void setup()
{
    hardwareSetup();
    //softwareSetup();
}


void readCmd()
{
    String inString = "";
    int valPos = -1;
    String cmd = "";
    //Serial.println("dupa");
    //return;
    while(Serial.available() > 0) // Don't read unless
     {
        int data = Serial.read();
        //Serial.println(data);

        if(data != '\n'){
            inString += (char)data;
            if (data == '='){
              valPos = inString.length();
            }
            continue;
        }

            if(inString == SIM_ON){
                g_simulatedSensor = true;
                inString = "";
                //delay(10);
                //Serial.println(SIM_ON);
                cmd = inString;
                continue;
            } else if(inString == SIM_OFF){
                g_simulatedSensor = false;
                Serial.println(SIM_OFF);
                inString = "";
                continue;
            } else {
              delay(10);
              Serial.println("UNKNOWN CMD: " + inString);
            }

//        if (data == '\n' && inString.length() > 0 && g_simulatedSensor) {
//            g_simulatedSensorValue = inString.toInt();
//            Serial.print("SIM ON, VALUE=");
//            Serial.println(inString.toInt());
//        }
     }
}

/////////////////////////////////////////////////////////////////////////// LOOP //////////////////////////////////////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  readCmd();

//  g_timer.measureBegin();

//  g_lowPressureValueReader.update();
//  g_highPressureValueReader.update();

  delay(5);
//  g_pompDriver.update(g_timer.getLapsedTime());

//  g_timer.measureEnd();
}
