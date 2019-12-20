#ifndef PROFILECONTROLLER_H
#define PROFILECONTROLLER_H


#define MAX_HEATINGELEMENTS       4

#define TEMPERATURE_MIN           25.0f    // Celcius
#define TEMPERATURE_MAX           350.0f   // Celcius
#define TEMPERATURE_SAMPLE_DELAY  990

#define TIME_MIN                  0     // Seconds
#define TIME_MAX                  600   // Seconds

#define RATE_MIN                  -20   // C Per S
#define RATE_MAX                  20    // C Per S

#define PWM_PERIOD                1000  // Milliseconds
#define PID_RATE                  1000  // Milliseconds
#define PID_MAXINFLUENCE          30.0f    // % Power
#define PID_MAXERRORSUM           100.0f

#define GRAPH_POINT_COUNT         480

#include <ArduinoJson.h>
#include <Arduino.h>
#include <FS.h>

#include "globals.h"
#include "MAX31856.h"
#include "Heater.h"


typedef enum ProfileStatus {
  PS_IDLE,
  PS_PREHEATING,
  PS_RUNNING,
  PS_PAUSED,
  PS_COMPLETE
};

typedef enum HeatingElement {
  TOP,
  BOTTOM,
  BOOST,
  AUX
};

typedef struct PID {
  float p;
  float i;
  float d;
  float iErrorSum;
  float previousError;
  float value;
  uint32_t lastUpdate;
};

typedef struct CommandArguments {
  uint32_t startTime;
  uint32_t holdTime;
  float startTemperature;
  float endTemperature;
  float pidTemperature;
  float riseRate;
};

struct {
  uint32_t thermocouple;
  uint32_t pwm;
  uint32_t controller;
} _lastUpdate;

class ProfileController {
  public:
    ProfileController();
    void init();
    bool isInitialized();

    void setLearnedConstants(float power, float inertia, float insulation);
    
    void update();
    void updatePWM();
    void updatePID();
    
    void restartPWMPeriod();

    void initPID();
    void setPIDConstants(float p, float i, float d);
    void enablePID();
    void disablePID();

    void addHeatingElement(int8_t relayPin, bool activeLow, float maxPower, float ratio);
    void addThermocouple(int8_t cs, int8_t drdy);

    bool isHeatingElementActive(HeatingElement h);
    float getHeatingElementPower(HeatingElement h);
    void setHeatingElementPower(HeatingElement h, float power);
    void setHeatingElementMaxPower(HeatingElement h, float power);

    float getHeatingElementRatio(HeatingElement h);
    void setHeatingElementRatio(HeatingElement h, float ratio);

    void resetCommandArguments();
    void setCommandArguments(float temperature);
    void setCommandArguments(uint16_t seconds);
    void setCommandArguments(float temperature, float rate);
    void setCommandArguments(float temperature, uint16_t seconds);
    void setCommandArguments(float temperature, uint16_t seconds, float rate);

    void readNextCommand();

    bool startJob(char *name);
    void pauseJob();
    void resumeJob();
    void stopJob();

    void clearGraph();

    char* getName();
    ProfileStatus getStatus();
    uint8_t getProgress();

    float getTemperature();
    void setTemperature(float c);

    char* getProfileList(char *ret);
    char* getProfileOverview(char *ret);
    char* getConnectionOverview(char *ret);
    char* getHeaterOverview(char *ret);

    bool graphDataAvailable();
    float getGraphData();

    bool proOverviewUpdateNeeded();
    bool conOverviewUpdateNeeded();
    bool heatOverviewUpdateNeeded();
    
    uint32_t elapsedTime(uint32_t startTime);

  private:
    bool _initialized;
    bool _tcInitialized;
    bool _pidInitialized;

    float _learnedPower;
    float _learnedInsulation;
    float _learnedInertia;

    char _name[24];
    ProfileStatus _status;
    uint8_t _progress;

    uint8_t _numHeatingElements;

    float _graph[GRAPH_POINT_COUNT];
    uint16_t _graphPos;
    uint16_t _graphPosLastXfer;

    bool _proOverviewUpdateNeeded;
    bool _conOverviewUpdateNeeded;
    bool _heatOverviewUpdateNeeded;
    
    File _profile;
    bool _profileLoaded;
    uint16_t _profileLine;
    
    uint32_t _jobStartTime;

    Heater _heater[3];
    MAX31856 _thermocouple;

    PID _pid;
    bool _pidEnabled;

    float _lastTemperatureSetpoint;

    CommandArguments _cmdArgs;
    bool _cmdFinished;
};


#endif
