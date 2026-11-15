#include "frogpilot/ui/qt/onroad/frogpilot_buttons.h"

DrivingPersonalityButton::DrivingPersonalityButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(personality_button_width, btn_size);

  QObject::connect(frogpilotUIState(), &FrogPilotUIState::themeUpdated, this, &DrivingPersonalityButton::updateTheme);
  QObject::connect(this, &QPushButton::pressed, [] {
    frogpilotUIState()->setDistanceButtonPressed(true);
  });
  QObject::connect(this, &QPushButton::released, [] {
    frogpilotUIState()->setDistanceButtonPressed(false);
  });
}

void DrivingPersonalityButton::showEvent(QShowEvent *event) {
  updateTheme();
}

void DrivingPersonalityButton::hideEvent(QHideEvent *event) {
  setDown(false);
  frogpilotUIState()->setDistanceButtonPressed(false);
  currentGif.reset();

  QPushButton::hideEvent(event);
}

void DrivingPersonalityButton::updateTheme() {
  currentGif.clear();
  currentIcon.clear();
  currentImg = QPixmap();
}

void DrivingPersonalityButton::updateState(const UIState &s, const FrogPilotUIState &fs) {
  if (!isVisible()) {
    return;
  }

  static const QString personalityIcons[] = {"aggressive", "standard", "relaxed"};
  static const QString trafficIcon = "traffic";

  const QString &icon = fs.frogpilot_scene.traffic_mode_enabled ? trafficIcon : personalityIcons[static_cast<int>(s.scene.personality)];
  if (icon == currentIcon) {
    return;
  }
  currentIcon = icon;

  loadImage("../../frogpilot/assets/active_theme/distance_icons/" + icon, currentImg, currentGif, QSize(btn_size, btn_size), this, false);
}

void DrivingPersonalityButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  QPoint center(rightHandDM ? width() / 2 - UI_BORDER_SIZE / 2 : width() / 2 + UI_BORDER_SIZE / 2, height() / 2);
  drawIcon(p, center, currentGif ? currentGif->currentPixmap() : currentImg, Qt::transparent, 1.0);
}
