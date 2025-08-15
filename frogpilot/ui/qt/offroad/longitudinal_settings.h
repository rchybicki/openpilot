#pragma once

#include <map>
#include <set>

#include "frogpilot/ui/qt/offroad/frogpilot_settings.h"

class FrogPilotLongitudinalPanel : public FrogPilotListWidget {
  Q_OBJECT

public:
  explicit FrogPilotLongitudinalPanel(FrogPilotSettingsWindow *parent);

signals:
  void openSubPanel();
  void openSubSubPanel();

protected:
  void showEvent(QShowEvent *event) override;

private:
  void updateMetric(bool metric, bool bootRun);
  void updateToggles();

  bool customPersonalityOpen;
  bool hasDashSpeedLimits;
  bool hasPCMCruise;
  bool isGM;
  bool isHKGCanFd;
  bool isToyota;
  bool isTSK;
  bool slcOpen;

  int tuningLevel;

  float longitudinalActuatorDelay;
  float startAccel;
  float stopAccel;
  float stoppingDecelRate;
  float stoppingErrorFactor;
  float stoppingAccelMax1;
  float stoppingAccelMax2;
  float stoppingAccelMax3;
  float stoppingAccelMin1;
  float stoppingAccelMin2;
  float stoppingAccelMin3;
  float vEgoStarting;
  float vEgoStopping;

  std::map<QString, AbstractControl*> toggles;

  std::set<QString> advancedLongitudinalTuneKeys = {
    "LongitudinalActuatorDelay", "StartAccel", "StopAccel",
    "StoppingDecelRate", "StoppingErrorFactor",
    "StoppingAccelMax1", "StoppingAccelMax2", "StoppingAccelMax3",
    "StoppingAccelMin1", "StoppingAccelMin2", "StoppingAccelMin3",
    "VEgoStarting", "VEgoStopping"
  };
  std::set<QString> aggressivePersonalityKeys = {"AggressiveFollow", "AggressiveJerkAcceleration", "AggressiveJerkDeceleration", "AggressiveJerkDanger", "AggressiveJerkSpeed", "AggressiveJerkSpeedDecrease", "ResetAggressivePersonality"};
  std::set<QString> conditionalExperimentalKeys = {"CESpeed", "CESpeedLead", "CECurves", "CECscCurves", "CELead", "CEModelStopTime", "CENavigation", "CESignalSpeed", "ShowCEMStatus"};
  std::set<QString> curveSpeedKeys = {"CalibratedLateralAcceleration", "CalibrationProgress", "ResetCurveData", "ShowCSCStatus"};
  std::set<QString> customDrivingPersonalityKeys = {"AggressivePersonalityProfile", "RelaxedPersonalityProfile", "StandardPersonalityProfile", "TrafficPersonalityProfile"};
  std::set<QString> longitudinalTuneKeys = {"AccelerationProfile", "DecelerationProfile", "HumanAcceleration", "HumanFollowing", "ShortDistanceFactor", "LongDistanceFactor", "LeadDetectionThreshold", "MaxDesiredAcceleration", "TacoTune"};
  std::set<QString> qolKeys = {"CustomCruise", "CustomCruiseLong", "ForceStandstill", "ForceStops", "IncreasedStoppedDistance", "MapGears", "ReverseCruise", "SetSpeedOffset"};
  std::set<QString> relaxedPersonalityKeys = {"RelaxedFollow", "RelaxedJerkAcceleration", "RelaxedJerkDeceleration", "RelaxedJerkDanger", "RelaxedJerkSpeed", "RelaxedJerkSpeedDecrease", "ResetRelaxedPersonality"};
  std::set<QString> speedLimitControllerKeys = {"SLCOffsets", "SLCFallback", "SLCOverride", "SLCPriority", "SLCQOL", "SLCVisuals"};
  std::set<QString> speedLimitControllerOffsetsKeys = {"Offset1", "Offset2", "Offset3", "Offset4", "Offset5", "Offset6", "Offset7"};
  std::set<QString> speedLimitControllerQOLKeys = {"ForceMPHDashboard", "SetSpeedLimit", "SLCConfirmation", "SLCLookaheadHigher", "SLCLookaheadLower", "SLCMapboxFiller"};
  std::set<QString> speedLimitControllerVisualKeys = {"ShowSLCOffset", "SpeedLimitSources"};
  std::set<QString> standardPersonalityKeys = {"StandardFollow", "StandardJerkAcceleration", "StandardJerkDeceleration", "StandardJerkDanger", "StandardJerkSpeed", "StandardJerkSpeedDecrease", "ResetStandardPersonality"};
  std::set<QString> trafficPersonalityKeys = {"TrafficFollow", "TrafficJerkAcceleration", "TrafficJerkDeceleration", "TrafficJerkDanger", "TrafficJerkSpeed", "TrafficJerkSpeedDecrease", "ResetTrafficPersonality"};

  std::set<QString> parentKeys;

  FrogPilotParamValueControl *longitudinalActuatorDelayToggle;
  FrogPilotParamValueControl *startAccelToggle;
  FrogPilotParamValueControl *stopAccelToggle;
  FrogPilotParamValueControl *stoppingDecelRateToggle;
  FrogPilotParamValueControl *stoppingErrorFactorToggle;
  FrogPilotParamValueControl *stoppingAccelMax1Toggle;
  FrogPilotParamValueControl *stoppingAccelMax2Toggle;
  FrogPilotParamValueControl *stoppingAccelMax3Toggle;
  FrogPilotParamValueControl *stoppingAccelMin1Toggle;
  FrogPilotParamValueControl *stoppingAccelMin2Toggle;
  FrogPilotParamValueControl *stoppingAccelMin3Toggle;
  FrogPilotParamValueControl *vEgoStartingToggle;
  FrogPilotParamValueControl *vEgoStoppingToggle;

  FrogPilotSettingsWindow *parent;

  LabelControl *calibratedLateralAccelerationLabel;
  LabelControl *calibrationProgressLabel;

  QJsonObject frogpilotToggleLevels;

  Params params;
  Params params_cache{"/cache/params"};
  Params params_default{"/dev/shm/params_default"};
  Params params_memory{"/dev/shm/params"};
};
