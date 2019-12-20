#include "ProfileController.h"

ProfileController::ProfileController() {
}

void ProfileController::init() {
  _initialized = false;
  _tcInitialized = false;
  _pidInitialized = false;

  setLearnedConstants(1.0f, 1.0f, 1.0f);
  
  memset(_name, '\0', sizeof(_name));
  _status = PS_IDLE;
  _progress = 0;

  _numHeatingElements = 0;
  
  _pidEnabled = true;
  initPID();

  _profileLoaded = false;
  _profileLine = 0;
  _jobStartTime = 0;

  resetCommandArguments();
  _cmdFinished = true;

  _graphPos = 0;
  _graphPosLastXfer = 0;
  clearGraph();

  _proOverviewUpdateNeeded = false;
  _conOverviewUpdateNeeded = false;
  _heatOverviewUpdateNeeded = false;

  _lastTemperatureSetpoint = 0.0f;

  _lastUpdate.thermocouple = 0;
  _lastUpdate.controller = 0;
  _lastUpdate.pwm = 0;
}

bool ProfileController::isInitialized() {
  if (_initialized)
    return true;
    
  if (!_initialized && _tcInitialized && _pidInitialized && _numHeatingElements > 0)
    _initialized = true;

  return _initialized;
}

void ProfileController::setLearnedConstants(float power, float inertia, float insulation) {
  _learnedPower = power;
  _learnedInertia = inertia;
  _learnedInsulation = insulation;

  setPIDConstants(2.0f, 0.01f, map(constrain(_learnedInertia, 30, 100), 30, 100, 30, 75));
}

void ProfileController::update() {
  if (!_initialized) {
    Serial.println("ERROR: ProfileController NOT Initialized!");
    return;
  }

  if (_status == PS_PREHEATING || _status == PS_RUNNING || _status == PS_PAUSED) {
    if (elapsedTime(_pid.lastUpdate) >= PID_RATE)
      updatePID();
    
    if (elapsedTime(_lastUpdate.pwm) >= PWM_PERIOD)
      restartPWMPeriod();
    else
      updatePWM();

    if (_status == PS_RUNNING && _cmdFinished) {
      _cmdFinished = false;

      if (!_pidEnabled)
        enablePID();
        
      readNextCommand();
    }
    else {
      bool done = true;

      if (_status == PS_PREHEATING || _status == PS_PAUSED)
        done = false;

      if (_cmdArgs.holdTime != 0) {
        Serial.print("startTime = ");
        Serial.print(_cmdArgs.startTime);
        Serial.print(", holdTime = ");
        Serial.println(_cmdArgs.holdTime);
      }
      if (_cmdArgs.holdTime != 0 && elapsedTime(_cmdArgs.startTime) < _cmdArgs.holdTime)
        done = false;

      if ((int16_t)_cmdArgs.startTemperature != (int16_t)_cmdArgs.endTemperature && (
        ((int16_t)_cmdArgs.startTemperature < (int16_t)_cmdArgs.endTemperature && (int16_t)getTemperature() < (int16_t)_cmdArgs.endTemperature) ||
        ((int16_t)_cmdArgs.startTemperature > (int16_t)_cmdArgs.endTemperature && (int16_t)getTemperature() > (int16_t)_cmdArgs.endTemperature)
        )) {
          done = false;
      }

      if (done)
        _cmdFinished = true;
    }
  }

  if (_thermocouple.dataReady() && _thermocouple.readTemperatures()) {
    _graph[_graphPos++] = getTemperature();

    if (_graphPos >= GRAPH_POINT_COUNT)
      _graphPos = 0;

    if (_graphPosLastXfer == _graphPos)
      _graphPosLastXfer++;

    if (_graphPosLastXfer >= GRAPH_POINT_COUNT)
      _graphPosLastXfer = 0;
  }
}

void ProfileController::updatePWM() {
  for (uint8_t i = 0; i < _numHeatingElements; i++) {
    if (i % 2 == 0 && _heater[i].getPower() > 0.0f && _heater[i].isActive()) {
      if (elapsedTime(_lastUpdate.pwm) >= (_heater[i].getPower() / 100.0f) * PWM_PERIOD) {
        _heater[i].turnOff();
      }
    }
    else if (i % 2 == 1 && _heater[i].getPower() > 0.0f && !_heater[i].isActive()) {
      if (elapsedTime(_lastUpdate.pwm) >= ((100.0f - _heater[i].getPower()) / 100.0f) * PWM_PERIOD) {
        _heater[i].turnOn();
      }
    }
  }
}

void ProfileController::updatePID() {
  if (_pidEnabled == false || (_cmdArgs.endTemperature == 0.0f && _lastTemperatureSetpoint == 0))
    return;

  // Controleo3 Base Power Calculation
  
  float maxRatio = 0.0f, r;
  for (uint8_t i = 0; i < _numHeatingElements; i++) {
    r = _heater[i].getHeaterRatio();
    if (r > maxRatio)
      maxRatio = r;
  }

  float tempError = (_cmdArgs.endTemperature > 0.0f ? _cmdArgs.pidTemperature : _lastTemperatureSetpoint) - getTemperature();
  //float tempError = (_cmdArgs.endTemperature > 0.0f ? _cmdArgs.endTemperature : _lastTemperatureSetpoint) - getTemperature();

  if (_cmdArgs.pidTemperature == 0.0f)
    _cmdArgs.pidTemperature = _cmdArgs.endTemperature;
  else {
    if (_cmdArgs.pidTemperature >= _cmdArgs.endTemperature) {
      _cmdArgs.pidTemperature = _cmdArgs.endTemperature;
      _cmdArgs.riseRate = 0.0f;
    }
    else
      _cmdArgs.pidTemperature = getTemperature() + _cmdArgs.riseRate;
  }
  
  double basePower = _cmdArgs.pidTemperature * 0.83 * _learnedPower / 100;
  uint16_t insulationPower = map(_learnedInsulation, 0, 300, map(_cmdArgs.pidTemperature, 0, 400, 0, 20), 0);
  double risePower = _cmdArgs.riseRate * basePower * 2.0f;

  double biasFactor = 2.0 * maxRatio / (_heater[TOP].getHeaterRatio() + _heater[BOTTOM].getHeaterRatio());

  double totalBasePower = (basePower + (float)insulationPower + risePower) * biasFactor;

  Serial.print("BasePower = ");
  Serial.print(basePower);
  Serial.print(", insulationPower = ");
  Serial.print(insulationPower);
  Serial.print(", risePower = ");
  Serial.print(risePower);
  Serial.print(", totalBasePower = "); Serial.println(totalBasePower);

  if (totalBasePower > 100.0)
    totalBasePower = 100.0;
  else if (totalBasePower < 0.0)
    totalBasePower = 0.0;

  // Regular PID stuff

  
  Serial.print("tempError Calculation: endTemperature = "); Serial.print(_cmdArgs.endTemperature);
  Serial.print(", pidTemperature = "); Serial.print(_cmdArgs.pidTemperature);
  Serial.print(", lastTempSetpoint = "); Serial.println(_lastTemperatureSetpoint);
  
  float pTerm, iTerm, dTerm;

  if (_pid.lastUpdate == 0)
    _pid.lastUpdate = millis();

  float tDelta = elapsedTime(_pid.lastUpdate) / 1000.0f;
  _pid.lastUpdate = millis();

  _pid.iErrorSum += tempError * tDelta;

  if (_pid.iErrorSum > PID_MAXERRORSUM)
    _pid.iErrorSum = PID_MAXERRORSUM;
  else if (_pid.iErrorSum < -PID_MAXERRORSUM)
    _pid.iErrorSum = -PID_MAXERRORSUM;

  if (tempError < 0.0f)
    pTerm = tempError * (_pid.p * 2.0f); // If over temp, force it down quicker
  else
    pTerm = tempError * _pid.p;
    
  iTerm = _pid.iErrorSum * _pid.i;
  dTerm = (((_cmdArgs.endTemperature > 0.0f? _cmdArgs.endTemperature : _lastTemperatureSetpoint) - getTemperature()) - _pid.previousError) * _pid.d;

  _pid.value = pTerm + iTerm + dTerm;
  _pid.previousError = _cmdArgs.endTemperature - getTemperature();

  Serial.print("TempError = "); Serial.println(tempError);
  Serial.print("PID Values: P = "); Serial.print(pTerm);
  Serial.print(", i = "); Serial.print(iTerm);
  Serial.print(", d = "); Serial.println(dTerm);

  if (_pid.value > PID_MAXINFLUENCE)
    _pid.value = PID_MAXINFLUENCE;
  else if (_pid.value < -PID_MAXINFLUENCE)
    _pid.value = -PID_MAXINFLUENCE;

  totalBasePower += _pid.value;
  Serial.print("new totalBasePower = "); Serial.println(totalBasePower);

  if (totalBasePower > 100.0)
    totalBasePower = 100.0;
  else if (totalBasePower < 0.0)
    totalBasePower = 0.0;

  Serial.print("Final Heater Powers: ");
  for (uint8_t i = 0; i < _numHeatingElements; i++) {
    _heater[i].setPower(totalBasePower * _heater[i].getHeaterRatio() / maxRatio);
    Serial.print(_heater[i].getPower());
    Serial.print(", ");
  }
  Serial.println("\n");

  _heatOverviewUpdateNeeded = true;
}

void ProfileController::restartPWMPeriod() {
  for (uint8_t i = 0; i < _numHeatingElements; i++) {
    if (i % 2 == 0 && _heater[i].getPower() > 0.0f)
      _heater[i].turnOn();
    else
      _heater[i].turnOff();
  }

  _lastUpdate.pwm = millis();
}

void ProfileController::initPID() {
  _pid.iErrorSum = 0.0f;
  _pid.previousError = 0.0f;
  _pid.value = 0.0f;
  _pid.lastUpdate = 0;
}

void ProfileController::setPIDConstants(float p, float i, float d) {
  _pid.p = p;
  _pid.i = i;
  _pid.d = d;

  _pidInitialized = true;
  isInitialized();
}

void ProfileController::enablePID() {
  _pidEnabled = true;
  initPID();
}

void ProfileController::disablePID() {
  _pidEnabled = false;
}

void ProfileController::addHeatingElement(const int8_t relayPin, const bool activeLow, float maxPower, float ratio) {
  if (_numHeatingElements >= MAX_HEATINGELEMENTS)
    return;
  
  _heater[_numHeatingElements].init(relayPin, activeLow);
  setHeatingElementMaxPower((HeatingElement)_numHeatingElements, maxPower);
  setHeatingElementRatio((HeatingElement)_numHeatingElements++, ratio);
  
  isInitialized();
}

void ProfileController::addThermocouple(int8_t cs, int8_t drdy) {
  _thermocouple.init(cs, drdy);

  _tcInitialized = true;
  isInitialized();

  _thermocouple.startTemperatureSampling();
}

bool ProfileController::isHeatingElementActive(HeatingElement h) {
  if (h >= _numHeatingElements)
    return false;
    
  return _heater[h].isActive();
}

float ProfileController::getHeatingElementPower(HeatingElement h) {
  if (h >= _numHeatingElements)
    return 0.0f;
    
  return _heater[h].getPower();
}

void ProfileController::setHeatingElementPower(HeatingElement h, float power) {
  if (h >= _numHeatingElements)
    return;
    
  _heater[h].setPower(power);
}

void ProfileController::setHeatingElementMaxPower(HeatingElement h, float power) {
  if (h >= _numHeatingElements)
    return;
    
  _heater[h].setMaxPower(power);
}

float ProfileController::getHeatingElementRatio(HeatingElement h) {
  if (h >= _numHeatingElements)
    return 0.0f;
    
  return _heater[h].getHeaterRatio();
}

void ProfileController::setHeatingElementRatio(HeatingElement h, float ratio) {
  if (h >= _numHeatingElements)
    return;
    
  _heater[h].setHeaterRatio(ratio);
}

void ProfileController::resetCommandArguments() {
  _cmdArgs.startTime = 0;
  _cmdArgs.holdTime = 0;
  _cmdArgs.startTemperature = 0.0f;
  _cmdArgs.endTemperature = 0.0f;
  _cmdArgs.pidTemperature = 0.0f;
  _cmdArgs.riseRate = 10.0f;
}

void ProfileController::setCommandArguments(float temperature) {
  #if defined(DEBUG)
  Serial.println("profileController.cpp: setCommandArguments( temperature = ");
  Serial.print(temperature);
  Serial.println(" )");
  #endif
  
  resetCommandArguments();

  _cmdArgs.startTemperature = getTemperature();
  _cmdArgs.endTemperature = temperature;
}

void ProfileController::setCommandArguments(uint16_t seconds) {
  #if defined(DEBUG)
  Serial.print("profileController.cpp: setCommandArguments( seconds = ");
  Serial.print(seconds);
  Serial.println(" )");
  #endif
  
  resetCommandArguments();

  _cmdArgs.startTime = millis();
  _cmdArgs.holdTime = seconds * 1000;
}

void ProfileController::setCommandArguments(float temperature, float rate) {
  #if defined(DEBUG)
  Serial.print("profileController.cpp: setCommandArguments( temperature = ");
  Serial.print(temperature);
  Serial.print(", rate = ");
  Serial.print(rate);
  Serial.println(" )");
  #endif
  
  resetCommandArguments();

  _cmdArgs.startTemperature = getTemperature();
  _cmdArgs.endTemperature = temperature;

  _cmdArgs.riseRate = rate;
}

void ProfileController::setCommandArguments(float temperature, uint16_t seconds) {
  #if defined(DEBUG)
  Serial.print("profileController.cpp: setCommandArguments( temperature = ");
  Serial.print(temperature);
  Serial.print(", seconds = ");
  Serial.print(seconds);
  Serial.println(" )");
  #endif
  
  resetCommandArguments();

  _cmdArgs.startTime = millis();
  _cmdArgs.holdTime = seconds * 1000;

  _cmdArgs.startTemperature = getTemperature();
  _cmdArgs.endTemperature = temperature;
}

void ProfileController::setCommandArguments(float temperature, uint16_t seconds, float rate) {
  #if defined(DEBUG)
  Serial.print("profileController.cpp: setCommandArguments( temperature = ");
  Serial.print(temperature);
  Serial.print(", seconds = ");
  Serial.print(seconds);
  Serial.print(", rate = ");
  Serial.print(rate);
  Serial.println(" )");
  #endif
  
  resetCommandArguments();

  _cmdArgs.startTime = millis();
  _cmdArgs.holdTime = seconds * 1000;

  _cmdArgs.startTemperature = getTemperature();
  _cmdArgs.endTemperature = temperature;
  
  _cmdArgs.riseRate = rate;
}

void ProfileController::readNextCommand() {
  if (_status == PS_RUNNING && _profileLoaded) {
    if (!_profile.available()) {
      stopJob();
      return;
    }

    char c, *token, buff[48] = {0};
    const char s[2] = " ";
    uint8_t i, pos = 0;
    int16_t t;
    uint16_t secs;
    float f, goalTemp, rate;

    _profileLine++;
    
    while (_profile.available() && c != '\n') {
      c = _profile.read();
      if (c != -1 && c != '\n')
        buff[pos++] = c;
    }

    #if defined(DEBUG)
    Serial.print("profileController.cpp: readNextCommand() -> ");
    Serial.println(buff);
    #endif

    token = strtok(buff, s);

    if (token == NULL)
      token = buff;
    
    if (strcmp(token, "MaxPower") == 0) {
      i = 0;
      token = strtok(NULL, s);
      while (token != NULL && i < _numHeatingElements) {
        setHeatingElementMaxPower((HeatingElement)i++, atof(token));
        token = strtok(NULL, s);
      }

      _cmdFinished = true;
    }
    else if (strcmp(token, "Ratio") == 0) {
      i = 0;
      token = strtok(NULL, s);
      while (token != NULL && i < _numHeatingElements) {
        setHeatingElementRatio((HeatingElement)i++, atof(token));
        token = strtok(NULL, s);
      }

      _cmdFinished = true;
    }
    else if (strcmp(token, "Temperature") == 0) {
      token = strtok(NULL, s);
      f = atof(token);

      rate = 0.0f;
      secs = 0;
      goalTemp = 0.0f;
      if (f > TEMPERATURE_MIN && f < TEMPERATURE_MAX)
        goalTemp = f;

      token = strtok(NULL, s);
      while (token != NULL) {
        if (strcmp(token, "Increment") == 0) {
          token = strtok(NULL, s);
          if (token == NULL)
            break;
            
          f = atof(token);
          if (goalTemp > 0.0f) {
            if (goalTemp - getTemperature() < 0.0f && f > 0.0f)
              f *= -1.0f;
            else if (goalTemp - getTemperature() > 0.0f && f < 0.0f)
              f *= -1.0f;
              
            if (f > RATE_MIN && f < RATE_MAX)
              rate = f;
          }
        }
        else if (strcmp(token, "In") == 0) {
          token = strtok(NULL, s);
          if (token == NULL)
            break;
            
          t = atoi(token);
          if (goalTemp > 0.0f) {
            f = (goalTemp - getTemperature()) / (float)t;
            if (f > RATE_MIN && f < RATE_MAX)
              rate = f;
          }
        }
        else if (strcmp(token, "For") == 0) {
          token = strtok(NULL, s);
          if (token == NULL)
            break;
            
          t = atoi(token);
          if (t > TIME_MIN && t < TIME_MAX)
            secs = t;
        }

        token = strtok(NULL, s);
      }

      if (goalTemp > 0.0f) {
        _lastTemperatureSetpoint = goalTemp;
        if (rate == 0.0f)
          rate = _lastTemperatureSetpoint - getTemperature();

        if (secs == 0)
          setCommandArguments(_lastTemperatureSetpoint, rate);
        else
          setCommandArguments(_lastTemperatureSetpoint, secs, rate);

        _cmdArgs.pidTemperature = getTemperature();
      }
    }
    else if (strcmp(token, "TemperatureUnder") == 0) {
      token = strtok(NULL, s);
      f = atof(token);
      if (f > TEMPERATURE_MIN && f < TEMPERATURE_MAX) {
        _lastTemperatureSetpoint = f;

        if (f < getTemperature())
          setCommandArguments(_lastTemperatureSetpoint);
      }
    }
    else if (strcmp(token, "HoldTemperature") == 0) {
      token = strtok(NULL, s);

      if (strcmp(token, "For") != 0) {
        f = atof(token);
        token = strtok(NULL, s);
      }

      if (strcmp(token, "For") == 0) {
        token = strtok(NULL, s);
        secs = atoi(token);

        if (f > TEMPERATURE_MIN && f < TEMPERATURE_MAX) {
          if (secs > TIME_MIN && secs < TIME_MAX) {
            _lastTemperatureSetpoint = f;
            setCommandArguments(_lastTemperatureSetpoint, secs);
          }
        }
      }
    }
    else if (strcmp(token, "HoldPower") == 0) {
      i = 0;
      token = strtok(NULL, s);
      while (token != NULL) {
        if (strcmp(token, "For") == 0)
          break;
        else {
          if (i < _numHeatingElements)
            setHeatingElementPower((HeatingElement)i++, atof(token));
          token = strtok(NULL, s);
        }
      }

      token = strtok(NULL, s);
      if (token != NULL) {
        secs = atoi(token);
        
        if (secs > TIME_MIN && secs < TIME_MAX) {
          setCommandArguments(secs);
          disablePID();
        }
      }
    }
  }
}

bool ProfileController::startJob(char *name) {
  #if defined(DEBUG)
  Serial.print("profileController.cpp: startJob -> ");
  Serial.println(name);
  #endif
  
  if (_status != PS_IDLE && _status != PS_COMPLETE)
    return false;

  stopJob();

  char fname[32];
  strcpy(fname, "/profiles/");
  strcat(fname, name);
  
  _profile = SPIFFS.open(fname, "r");
  if (!_profile) {
    Serial.println("ERROR: Could not open file!");
    return false;
  }

  _profileLoaded = true;
  _profileLine = 0;
  
  clearGraph();

  strcpy(_name, name);
  _jobStartTime = millis();
  _status = PS_RUNNING;

  _proOverviewUpdateNeeded = true;

  return true;
}

void ProfileController::pauseJob() {
  if (_status == PS_RUNNING) {
    if (!_cmdFinished) {
      _cmdArgs.endTemperature = getTemperature();
      if (_cmdArgs.holdTime > 0) {
        _cmdArgs.holdTime -= elapsedTime(_cmdArgs.startTime);
      }
    }

    _status = PS_PAUSED;
    _proOverviewUpdateNeeded = true;
  }
}

void ProfileController::resumeJob() {
  if (_status == PS_PAUSED) {
    if (!_cmdFinished) {
      _cmdArgs.endTemperature = _lastTemperatureSetpoint;
      if (_cmdArgs.holdTime > 0)
        _cmdArgs.startTime = millis();
    }
      
    _status = PS_RUNNING;
    _proOverviewUpdateNeeded = true;
  }
}

void ProfileController::stopJob() {
  if (_profileLoaded) {
    _profile.close();
    _profileLoaded = false;
    
    _status = PS_IDLE;
    _proOverviewUpdateNeeded = true;
    _heatOverviewUpdateNeeded = true;
  }
  else if(_status == PS_PREHEATING) {
    _status = PS_IDLE;
    _proOverviewUpdateNeeded = true;
    _heatOverviewUpdateNeeded = true;
  }

  resetCommandArguments();

  for (uint8_t i = 0; i < _numHeatingElements; i++)
    setHeatingElementPower((HeatingElement)i, 0.0f);
}

void ProfileController::clearGraph() {
  if (_status != PS_RUNNING && _status != PS_PAUSED) {
    _graph[0] = 0.0f;
    _graphPos = 0;
    _graphPosLastXfer = 0;
  }
}

char* ProfileController::getName() {
  return _name;
}

ProfileStatus ProfileController::getStatus() {
  return _status;
}

uint8_t ProfileController::getProgress() {
  return _progress;
}

float ProfileController::getTemperature() {
  return _thermocouple.getTemperature();
}

void ProfileController::setTemperature(float c) {
  if (c < TEMPERATURE_MIN) {
    if (_status == PS_PREHEATING) {
      _status = PS_IDLE;
      resetCommandArguments();
      
      for (uint8_t i = 0; i < _numHeatingElements; i++)
        setHeatingElementPower((HeatingElement)i, 0.0f);
    }
    
    return;
  }
  else if (c > TEMPERATURE_MAX)
    return;
    
  if (_status == PS_IDLE)
    _status = PS_PREHEATING;

  if (_status == PS_PREHEATING)
    setCommandArguments(c);
}

char* ProfileController::getProfileList(char *ret) {
  strcpy(ret, "{ \"profileList\": [");

  Dir dir = SPIFFS.openDir("/profiles");
  
  if (dir.next()) {
    strcat(ret, "\"");
    strcat(ret, dir.fileName().c_str());
    strcat(ret, "\"");
  }
  else
    return "{\"profileList\": []}";
    
  while (dir.next()) {
    strcat(ret, ", \"");
    strcat(ret, dir.fileName().c_str());
    strcat(ret, "\"");
  }

  strcat(ret, "]}");

  return ret;
}

char* ProfileController::getProfileOverview(char *ret) {
  _proOverviewUpdateNeeded = false;
  
  strcpy(ret, "{ \"status\": \"");

  switch(_status) {
    case PS_IDLE:
      strcat(ret, "Idle");
      break;
    case PS_PREHEATING:
      strcat(ret, "Preheating");
      break;
    case PS_RUNNING:
      strcat(ret, "Running");
      break;
    case PS_PAUSED:
      strcat(ret, "Paused");
      break;
    case PS_COMPLETE:
      strcat(ret, "Complete");
      break;
    default:
      strcat(ret, "Unknown");
      break;
  }

  strcat(ret, "\", \"name\": \"");
  if (_status == PS_RUNNING || _status == PS_PAUSED || _status == PS_COMPLETE) {
    char buff[10];
    itoa(_progress, buff, 10);
    
    strcat(ret, _name);
    strcat(ret, "\", \"progress\": ");
    strcat(ret, buff);
  }
  else
    strcat(ret, "\", \"progress\": -1");

  strcat(ret, " }");

  return ret;
}

char* ProfileController::getConnectionOverview(char *ret) {
  _conOverviewUpdateNeeded = false;
  
  strcpy(ret, "{}");
  
  return ret;
}

char* ProfileController::getHeaterOverview(char *ret) {
  _heatOverviewUpdateNeeded = false;
  
  if (_numHeatingElements < 1) {
    strcpy(ret, "{}");
    return ret;
  }

  strcpy(ret, "{ \"power\": [");
  
  for (uint8_t i = 0; i < _numHeatingElements; i++) {
    char buff[10];
    dtostrf(getHeatingElementPower((HeatingElement)i), 6, 1, buff);
    strcat(ret, buff);

    if (i < _numHeatingElements - 1)
      strcat(ret, ", ");
  }

  strcat(ret, "]}");
  return ret;
}

bool ProfileController::graphDataAvailable() {
  if (_graphPosLastXfer == _graphPos)
    return false;
  else
    return true;
}

float ProfileController::getGraphData() {
  float t = _graph[_graphPosLastXfer++];
  if (_graphPosLastXfer >= GRAPH_POINT_COUNT)
      _graphPosLastXfer = 0;
      
  return t;
}

bool ProfileController::proOverviewUpdateNeeded() {
  return _proOverviewUpdateNeeded;
}

bool ProfileController::conOverviewUpdateNeeded() {
  return _conOverviewUpdateNeeded;
}

bool ProfileController::heatOverviewUpdateNeeded() {
  return _heatOverviewUpdateNeeded;
}

uint32_t ProfileController::elapsedTime(uint32_t startTime) {
  uint32_t t = millis();
  
  if (t > startTime)
    return t - startTime;
  else
    return 0xFFFFFFFF - startTime + t;
}
