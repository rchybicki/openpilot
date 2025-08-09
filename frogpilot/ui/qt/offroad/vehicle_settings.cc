#include <QRegularExpression>
#include <QTextStream>

#include "frogpilot/ui/qt/offroad/vehicle_settings.h"

QStringList getCarNames(const QString &carMake, QMap<QString, QString> &carModels) {
  static const QMap<QString, QString> makeMap = {
    {"acura", "honda"},
    {"audi", "volkswagen"},
    {"buick", "gm"},
    {"cadillac", "gm"},
    {"chevrolet", "gm"},
    {"chrysler", "chrysler"},
    {"cupra", "volkswagen"},
    {"dodge", "chrysler"},
    {"ford", "ford"},
    {"genesis", "hyundai"},
    {"gmc", "gm"},
    {"holden", "gm"},
    {"honda", "honda"},
    {"hyundai", "hyundai"},
    {"jeep", "chrysler"},
    {"kia", "hyundai"},
    {"lexus", "toyota"},
    {"lincoln", "ford"},
    {"man", "volkswagen"},
    {"mazda", "mazda"},
    {"nissan", "nissan"},
    {"ram", "chrysler"},
    {"seat", "volkswagen"},
    {"škoda", "volkswagen"},
    {"subaru", "subaru"},
    {"tesla", "tesla"},
    {"toyota", "toyota"},
    {"volkswagen", "volkswagen"}
  };

  QStringList carNameList;

  QFile valuesFile(QString("../car/%1/values.py").arg(makeMap.value(carMake, carMake)));
  if (!valuesFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return carNameList;
  }

  QTextStream in(&valuesFile);
  QString fileContent = in.readAll();
  valuesFile.close();

  fileContent.remove(QRegularExpression("#[^\n]*"));
  fileContent.remove(QRegularExpression("footnotes=\\[[^\\]]*\\],\\s*"));

  static const QRegularExpression carNameRegex("CarDocs\\(\\s*\"([^\"]+)\"[^)]*\\)");
  static const QRegularExpression platformRegex("((\\w+)\\s*=\\s*\\w+\\s*\\(\\s*\\[([\\s\\S]*?)\\]\\s*,)");
  static const QRegularExpression validNameRegex("^[A-Za-z0-9 \u0160.()-]+$");

  QRegularExpressionMatchIterator platformMatches = platformRegex.globalMatch(fileContent);
  while (platformMatches.hasNext()) {
    QRegularExpressionMatch platformMatch = platformMatches.next();
    QString platformName = platformMatch.captured(2);
    QString platformSection = platformMatch.captured(3);

    QRegularExpressionMatchIterator carNameMatches = carNameRegex.globalMatch(platformSection);
    while (carNameMatches.hasNext()) {
      QString carName = carNameMatches.next().captured(1);
      if (carName.contains(validNameRegex) && carName.count(" ") >= 1) {
        QStringList carNameParts = carName.split(" ");
        for (const QString &part : carNameParts) {
          if (part.compare(carMake, Qt::CaseInsensitive) == 0) {
            carNameList.append(carName);
            carModels[carName] = platformName;
          }
          break;
        }
      }
    }
  }

  carNameList.sort();
  return carNameList;
}

FrogPilotVehiclesPanel::FrogPilotVehiclesPanel(FrogPilotSettingsWindow *parent) : FrogPilotListWidget(parent), parent(parent) {
  QJsonObject shownDescriptions = QJsonDocument::fromJson(QString::fromStdString(params.get("ShownToggleDescriptions")).toUtf8()).object();
  QString className = this->metaObject()->className();

  if (!shownDescriptions.value(className).toBool(false)) {
    forceOpenDescriptions = true;
    shownDescriptions.insert(className, true);
    params.put("ShownToggleDescriptions", QJsonDocument(shownDescriptions).toJson(QJsonDocument::Compact).toStdString());
  }

  QStackedLayout *vehiclesLayout = new QStackedLayout();
  addItem(vehiclesLayout);

  FrogPilotListWidget *settingsList = new FrogPilotListWidget(this);

  ScrollView *vehiclesPanel = new ScrollView(settingsList, this);

  vehiclesLayout->addWidget(vehiclesPanel);

  QStringList makes = {
    "Acura", "Audi", "Buick", "Cadillac", "Chevrolet", "Chrysler",
    "CUPRA", "Dodge", "Ford", "Genesis", "GMC", "Holden", "Honda",
    "Hyundai", "Jeep", "Kia", "Lexus", "Lincoln", "MAN", "Mazda",
    "Nissan", "Ram", "SEAT", "Škoda", "Subaru", "Tesla", "Toyota",
    "Volkswagen"
  };

  ButtonControl *selectMakeButton = new ButtonControl(tr("Car Make"), tr("SELECT"));
  QObject::connect(selectMakeButton, &ButtonControl::clicked, [this, makes, selectMakeButton]() {
    QString makeSelection = MultiOptionDialog::getSelection(tr("Choose your car make"), makes, "", this);
    if (!makeSelection.isEmpty()) {
      params.put("CarMake", makeSelection.toStdString());
      selectMakeButton->setValue(makeSelection);
    }
  });
  settingsList->addItem(selectMakeButton);

  ButtonControl *selectModelButton = new ButtonControl(tr("Car Model"), tr("SELECT"));
  QObject::connect(selectModelButton, &ButtonControl::clicked, [this, selectModelButton]() {
    QString modelSelection = MultiOptionDialog::getSelection(tr("Choose your car model"), getCarNames(QString::fromStdString(params.get("CarMake")).toLower(), carModels), "", this);
    if (!modelSelection.isEmpty()) {
      params.put("CarModel", carModels.value(modelSelection).toStdString());
      params.put("CarModelName", modelSelection.toStdString());
      selectModelButton->setValue(modelSelection);
    }
  });
  settingsList->addItem(selectModelButton);

  forceFingerprint = new ParamControl("ForceFingerprint", tr("Disable Automatic Fingerprint Detection"), tr("<b>Force the selected fingerprint</b> and prevent it from ever changing."), "");
  settingsList->addItem(forceFingerprint);

  disableOpenpilotLong = new ParamControl("DisableOpenpilotLongitudinal", tr("Disable openpilot Longitudinal Control"), tr("<b>Disable openpilot longitudinal</b> and use the car's stock ACC instead."), "");
  QObject::connect(disableOpenpilotLong, &ToggleControl::toggleFlipped, [parent, this](bool state) {
    if (state) {
      if (FrogPilotConfirmationDialog::yesorno(tr("Are you sure you want to completely disable openpilot longitudinal control?"), this)) {
        if (started) {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      } else {
        params.putBool("DisableOpenpilotLongitudinal", false);
        disableOpenpilotLong->refresh();
      }
    }

    parent->updateVariables();
    updateToggles();
  });
  settingsList->addItem(disableOpenpilotLong);

  FrogPilotListWidget *gmList = new FrogPilotListWidget(this);
  FrogPilotListWidget *hkgList = new FrogPilotListWidget(this);
  FrogPilotListWidget *toyotaList = new FrogPilotListWidget(this);

  ScrollView *gmPanel = new ScrollView(gmList, this);
  ScrollView *hkgPanel = new ScrollView(hkgList, this);
  ScrollView *toyotaPanel = new ScrollView(toyotaList, this);

  vehiclesLayout->addWidget(gmPanel);
  vehiclesLayout->addWidget(hkgPanel);
  vehiclesLayout->addWidget(toyotaPanel);

  std::vector<std::tuple<QString, QString, QString, QString>> vehicleToggles {
    {"GMToggles", tr("General Motors Settings"), tr("<b>FrogPilot features for General Motors vehicles.</b>"), ""},
    {"ExperimentalGMTune", tr("FrogsGoMoo's Experimental Tune"), tr("<b>Experimental GM tune by FrogsGoMoo</b> that attempts to smoothen stopping and takeoff control. Use at your own risk!"), ""},
    {"LongPitch", tr("Smooth Pedal Response on Hills"), tr("<b>Smoothen acceleration and braking</b> when driving downhill/uphill."), ""},
    {"VoltSNG", tr("Stop-and-Go Hack"), tr("<b>Force stop-and-go</b> on the 2017 Chevy Volt."), ""},

    {"HKGToggles", tr("Hyundai/Kia/Genesis Settings"), tr("<b>FrogPilot features for Genesis, Hyundai, and Kia vehicles.</b>"), ""},
    {"NewLongAPI", tr("comma's New Longitudinal API"), tr("<b>comma's new gas and brake control system</b> that improves acceleration and braking but may cause issues on some Genesis/Hyundai/Kia vehicles."), ""},
    {"TacoTuneHacks", tr("\"Taco Bell Run\" Torque Hack"), tr("<b>The steering torque hack from comma's 2022 \"Taco Bell Run\".</b> Designed to increase steering torque at low speeds for left and right turns."), ""},

    {"ToyotaToggles", tr("Toyota/Lexus Settings"), tr("<b>FrogPilot features for Lexus and Toyota vehicles.</b>"), ""},
    {"ToyotaDoors", tr("Automatically Lock/Unlock Doors"), tr("<b>Automatically lock/unlock doors</b> when shifting in and out of drive."), ""},
    {"ClusterOffset", tr("Dashboard Speed Offset"), tr("<b>The speed offset openpilot uses to match the speed on the dashboard display.</b>"), ""},
    {"FrogsGoMoosTweak", tr("FrogsGoMoo's Personal Tweaks"), tr("<b>Personal tweaks by FrogsGoMoo for quicker acceleration and smoother braking.</b>"), ""},
    {"LockDoorsTimer", tr("Lock Doors On Ignition Off After"), tr("<b>Automatically lock the doors on ignition off</b> when no one is detected in the front seats."), ""},
    {"SNGHack", tr("Stop-and-Go Hack"), tr("<b>Force stop-and-go</b> on Lexus/Toyota vehicles without stock stop-and-go functionality."), ""}
  };

  for (const auto &[param, title, desc, icon] : vehicleToggles) {
    AbstractControl *vehicleToggle;

    if (param == "GMToggles") {
      ButtonControl *gmToggle = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(gmToggle, &ButtonControl::clicked, [vehiclesLayout, gmPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(gmPanel);
      });
      vehicleToggle = gmToggle;

    } else if (param == "HKGToggles") {
      ButtonControl *hkgToggle = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(hkgToggle, &ButtonControl::clicked, [vehiclesLayout, hkgPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(hkgPanel);
      });
      vehicleToggle = hkgToggle;

    } else if (param == "ToyotaToggles") {
      ButtonControl *toyotaToggle = new ButtonControl(title, tr("MANAGE"), desc);
      QObject::connect(toyotaToggle, &ButtonControl::clicked, [vehiclesLayout, toyotaPanel, this]() {
        openDescriptions(forceOpenDescriptions, toggles);
        vehiclesLayout->setCurrentWidget(toyotaPanel);
      });
      vehicleToggle = toyotaToggle;
    } else if (param == "ToyotaDoors") {
      std::vector<QString> lockToggles{"LockDoors", "UnlockDoors"};
      std::vector<QString> lockToggleNames{tr("Lock"), tr("Unlock")};
      vehicleToggle = new FrogPilotButtonToggleControl(param, title, desc, icon, lockToggles, lockToggleNames);
    } else if (param == "LockDoorsTimer") {
      std::map<float, QString> autoLockLabels;
      for (int i = 0; i <= 300; ++i) {
        autoLockLabels[i] = i == 0 ? tr("Never") : QString::number(i) + tr(" seconds");
      }
      vehicleToggle = new FrogPilotParamValueControl(param, title, desc, icon, 0, 300, QString(), autoLockLabels, 5);
    } else if (param == "ClusterOffset") {
      std::vector<QString> clusterOffsetButton{"Reset"};
      FrogPilotParamValueButtonControl *clusterOffsetToggle = new FrogPilotParamValueButtonControl(param, title, desc, icon, 1.000, 1.050, "x", std::map<float, QString>(), 0.001, false, {}, clusterOffsetButton, false, false);
      QObject::connect(clusterOffsetToggle, &FrogPilotParamValueButtonControl::buttonClicked, [clusterOffsetToggle, this]() {
        params.putFloat("ClusterOffset", params_default.getFloat("ClusterOffset"));
        clusterOffsetToggle->refresh();
      });
      vehicleToggle = clusterOffsetToggle;

    } else {
      vehicleToggle = new ParamControl(param, title, desc, icon);
    }

    toggles[param] = vehicleToggle;

    if (gmKeys.find(param) != gmKeys.end()) {
      gmList->addItem(vehicleToggle);
    } else if (hkgKeys.find(param) != hkgKeys.end()) {
      hkgList->addItem(vehicleToggle);
    } else if (toyotaKeys.find(param) != toyotaKeys.end()) {
      toyotaList->addItem(vehicleToggle);
    } else {
      settingsList->addItem(vehicleToggle);

      parentKeys.insert(param);
    }

    if (ButtonControl *buttonControl = qobject_cast<ButtonControl*>(vehicleToggle)) {
      QObject::connect(buttonControl, &ButtonControl::clicked, this, &FrogPilotVehiclesPanel::openSubPanel);
    }

    QObject::connect(vehicleToggle, &AbstractControl::hideDescriptionEvent, [this]() {
      update();
    });
    QObject::connect(vehicleToggle, &AbstractControl::showDescriptionEvent, [this]() {
      update();
    });
  }

  static_cast<FrogPilotParamValueControl*>(toggles["LockDoorsTimer"])->setWarning("<b>Warning:</b> openpilot can't detect if keys are still inside the car, so ensure you have a spare key to prevent accidental lockouts!");

  std::set<QString> rebootKeys = {"NewLongAPI", "TacoTuneHacks"};
  for (const QString &key : rebootKeys) {
    QObject::connect(static_cast<ToggleControl*>(toggles[key]), &ToggleControl::toggleFlipped, [key, this](bool state) {
      if (started) {
        if (key == "TacoTuneHacks" && state) {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        } else if (key != "TacoTuneHacks") {
          if (FrogPilotConfirmationDialog::toggleReboot(this)) {
            Hardware::reboot();
          }
        }
      }
    });
  }

  openDescriptions(forceOpenDescriptions, toggles);

  QObject::connect(uiState(), &UIState::offroadTransition, [this, selectMakeButton, selectModelButton]() {
    std::thread([this, selectMakeButton, selectModelButton]() {
      selectMakeButton->setValue(QString::fromStdString(params.get("CarMake", true)));
      selectModelButton->setValue(QString::fromStdString(params.get(params.get("CarModelName").empty() ? "CarModel" : "CarModelName")));
    }).detach();
  });

  QObject::connect(parent, &FrogPilotSettingsWindow::closeSubPanel, [vehiclesLayout, vehiclesPanel, this] {
    if (forceOpenDescriptions) {
      openDescriptions(forceOpenDescriptions, toggles);

      disableOpenpilotLong->showDescription();
      forceFingerprint->showDescription();
    }
    vehiclesLayout->setCurrentWidget(vehiclesPanel);
  });
  QObject::connect(uiState(), &UIState::uiUpdate, this, &FrogPilotVehiclesPanel::updateState);
}

void FrogPilotVehiclesPanel::showEvent(QShowEvent *event) {
  if (forceOpenDescriptions) {
    disableOpenpilotLong->showDescription();
    forceFingerprint->showDescription();
  }

  frogpilotToggleLevels = parent->frogpilotToggleLevels;
  hasExperimentalOpenpilotLongitudinal = parent->hasExperimentalOpenpilotLongitudinal;
  hasOpenpilotLongitudinal = parent->hasOpenpilotLongitudinal;
  hasPedal = parent->hasPedal;
  hasSNG = parent->hasSNG;
  isC3 = parent->isC3;
  isGM = parent->isGM;
  isHKG = parent->isHKG;
  isHKGCanFd = parent->isHKGCanFd;
  isToyota = parent->isToyota;
  isVolt = parent->isVolt;
  openpilotLongitudinalControlDisabled = parent->openpilotLongitudinalControlDisabled || params.getBool("DisableOpenpilotLongitudinal");
  tuningLevel = parent->tuningLevel;

  updateToggles();
}

void FrogPilotVehiclesPanel::updateState(const UIState &s) {
  if (!isVisible()) {
    return;
  }

  started = s.scene.started;
}

void FrogPilotVehiclesPanel::updateToggles() {
  for (auto &[key, toggle] : toggles) {
    if (parentKeys.find(key) != parentKeys.end()) {
      toggle->setVisible(false);
    }
  }

  for (auto &[key, toggle] : toggles) {
    if (parentKeys.find(key) != parentKeys.end()) {
      continue;
    }

    bool setVisible = tuningLevel >= frogpilotToggleLevels[key].toDouble();

    if (gmKeys.find(key) != gmKeys.end()) {
      setVisible &= isGM;
    } else if (hkgKeys.find(key) != hkgKeys.end()) {
      setVisible &= isHKG;
    } else if (toyotaKeys.find(key) != toyotaKeys.end()) {
      setVisible &= isToyota;
    }

    if (longitudinalKeys.find(key) != longitudinalKeys.end()) {
      setVisible &= hasOpenpilotLongitudinal;
    }

    if (key == "LockDoorsTimer") {
      setVisible &= !isC3;
    }

    else if (key == "SNGHack") {
      setVisible &= !hasPedal && !hasSNG;
    }

    else if (key == "TacoTuneHacks") {
      setVisible &= isHKGCanFd;
    }

    else if (key == "VoltSNG") {
      setVisible &= isVolt && !hasSNG;
    }

    toggle->setVisible(setVisible);

    if (setVisible) {
      if (gmKeys.find(key) != gmKeys.end()) {
        toggles["GMToggles"]->setVisible(true);
      } else if (hkgKeys.find(key) != hkgKeys.end()) {
        toggles["HKGToggles"]->setVisible(true);
      } else if (toyotaKeys.find(key) != toyotaKeys.end()) {
        toggles["ToyotaToggles"]->setVisible(true);
      }
    }
  }

  disableOpenpilotLong->setVisible((hasOpenpilotLongitudinal || openpilotLongitudinalControlDisabled) && !hasExperimentalOpenpilotLongitudinal && tuningLevel >= frogpilotToggleLevels["DisableOpenpilotLongitudinal"].toDouble());
  forceFingerprint->setVisible(tuningLevel >= frogpilotToggleLevels["ForceFingerprint"].toDouble());

  openDescriptions(forceOpenDescriptions, toggles);

  update();
}
