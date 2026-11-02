#pragma once

#include <QMouseEvent>
#include <QWidget>

#include "selfdrive/ui/ui.h"

class OnroadAlerts : public QWidget {
  Q_OBJECT

public:
  OnroadAlerts(QWidget *parent = 0);
  void updateState(const UIState &s);
  void clear();
  bool isStagedUpdateAlert() const;
  bool isStagedUpdateAlertAt(const QPoint &pos) const;

  // FrogPilot variables
  bool displayFerg = false;
  bool enableFerg = false;

  int alertHeight = 0;

  QJsonObject frogpilot_toggles;

protected:
  struct Alert {
    QString text1;
    QString text2;
    QString type;
    cereal::SelfdriveState::AlertSize size;
    cereal::SelfdriveState::AlertStatus status;

    bool equal(const Alert &other) const {
      return text1 == other.text1 && text2 == other.text2 && type == other.type;
    }
  };

  const QMap<cereal::SelfdriveState::AlertStatus, QColor> alert_colors = {
    {cereal::SelfdriveState::AlertStatus::NORMAL, QColor(0x15, 0x15, 0x15, 0xf1)},
    {cereal::SelfdriveState::AlertStatus::USER_PROMPT, QColor(0xDA, 0x6F, 0x25, 0xf1)},
    {cereal::SelfdriveState::AlertStatus::CRITICAL, QColor(0xC9, 0x22, 0x31, 0xf1)},

    // FrogPilot variables
    {static_cast<cereal::SelfdriveState::AlertStatus>(cereal::FrogPilotSelfdriveState::AlertStatus::FROGPILOT), QColor(0x17, 0x86, 0x44, 0xf1)},
  };

  void paintEvent(QPaintEvent*) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  OnroadAlerts::Alert getAlert(const SubMaster &sm, uint64_t started_frame);
  QRect alertRect() const;
  void updateMouseEventTransparency();

  QColor bg;
  Alert alert = {};
  QPixmap stagedUpdateIcon;
  bool stagedUpdateRebootTouch = false;

  // FrogPilot variables
  QPixmap ferg;
};
