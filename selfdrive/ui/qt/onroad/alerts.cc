#include "selfdrive/ui/qt/onroad/alerts.h"

#include <QPainter>
#include <map>

#include "selfdrive/ui/qt/util.h"

namespace {
constexpr int STAGED_UPDATE_BADGE_WIDTH = 650;
constexpr int STAGED_UPDATE_BADGE_HEIGHT = 130;
constexpr int STAGED_UPDATE_BADGE_MARGIN = 40;
}

OnroadAlerts::OnroadAlerts(QWidget *parent) : QWidget(parent) {
  stagedUpdateIcon = loadPixmap("../assets/icons_mici/settings/device/update.png", {64, 76});
}

void OnroadAlerts::updateState(const UIState &s) {
  Alert a = getAlert(*(s.sm), s.scene.started_frame);

  // FrogPilot variables
  static const QSet<QString> informational_alerts = {"calibrationIncomplete/permanent", "laneChange/warning", "noLaneAvailable/warning", "personalityChanged/warning", "preLaneChangeLeft/warning", "preLaneChangeRight/warning", "turningLeft/warning", "turningRight/warning"};
  if (informational_alerts.contains(a.type) && frogpilot_toggles.value(QLatin1String("hide_alerts")).toBool()) {
    a = {};
  }

  if (!alert.equal(a)) {
    alert = a;
    update();
    updateMouseEventTransparency();
  }
}

void OnroadAlerts::clear() {
  alert = {};
  update();

  stagedUpdateRebootTouch = false;
  updateMouseEventTransparency();
}

QRect OnroadAlerts::alertRect() const {
  if (alert.size == cereal::SelfdriveState::AlertSize::NONE) {
    return {};
  }

  if (isStagedUpdateAlert()) {
    return QRect((width() - STAGED_UPDATE_BADGE_WIDTH) / 2,
                 height() - STAGED_UPDATE_BADGE_HEIGHT - STAGED_UPDATE_BADGE_MARGIN,
                 STAGED_UPDATE_BADGE_WIDTH, STAGED_UPDATE_BADGE_HEIGHT);
  }

  int h = height();
  if (alert.size == cereal::SelfdriveState::AlertSize::SMALL) {
    h = 271;
  } else if (alert.size == cereal::SelfdriveState::AlertSize::MID) {
    h = 420;
  }

  const int margin = alert.size == cereal::SelfdriveState::AlertSize::FULL ? 0 : 40;
  return QRect(margin, height() - h + margin, width() - margin * 2, h - margin * 2);
}

bool OnroadAlerts::isStagedUpdateAlert() const {
  // every fullupdate.sh banner state renders as the compact badge; keep in sync with the
  // alertDebug texts published there
  return alert.text1 == "Update Running" || alert.text1 == "Update Ready" ||
         alert.text1 == "Preparing Restart" || alert.text1 == "Update Staged" ||
         alert.text1 == "Cruise Still On" || alert.text1 == "Restart Paused" ||
         alert.text1 == "Reboot Ready" || alert.text1 == "Reboot Pending";
}

bool OnroadAlerts::isStagedUpdateAlertAt(const QPoint &pos) const {
  return isStagedUpdateAlert() && alertRect().contains(pos);
}

void OnroadAlerts::updateMouseEventTransparency() {
  const bool staged_update = isStagedUpdateAlert();
  if (staged_update) {
    setMask(QRegion(alertRect()));
  } else {
    clearMask();
  }

  const bool transparent_for_mouse_events = !staged_update;
  if (testAttribute(Qt::WA_TransparentForMouseEvents) != transparent_for_mouse_events) {
    setAttribute(Qt::WA_TransparentForMouseEvents, transparent_for_mouse_events);
  }
}

void OnroadAlerts::mousePressEvent(QMouseEvent *event) {
  stagedUpdateRebootTouch = isStagedUpdateAlertAt(event->pos());
  if (stagedUpdateRebootTouch) {
    event->accept();
  } else {
    event->ignore();
  }
}

void OnroadAlerts::mouseReleaseEvent(QMouseEvent *event) {
  if (!stagedUpdateRebootTouch) {
    event->ignore();
    return;
  }

  stagedUpdateRebootTouch = false;
  event->accept();
}

OnroadAlerts::Alert OnroadAlerts::getAlert(const SubMaster &sm, uint64_t started_frame) {
  const cereal::SelfdriveState::Reader &ss = sm["selfdriveState"].getSelfdriveState();
  const uint64_t selfdrive_frame = sm.rcv_frame("selfdriveState");

  // FrogPilot variables
  const SubMaster &fpsm = *(frogpilotUIState()->sm);
  const cereal::FrogPilotSelfdriveState::Reader &fpss = fpsm["frogpilotSelfdriveState"].getFrogpilotSelfdriveState();

  Alert a = {};
  if (selfdrive_frame >= started_frame) {  // Don't get old alert.
    a = {ss.getAlertText1().cStr(), ss.getAlertText2().cStr(),
         ss.getAlertType().cStr(), ss.getAlertSize(), ss.getAlertStatus()};

    // FrogPilot variables
    if (a.size == cereal::SelfdriveState::AlertSize::NONE || fpss.getHasPriorityAlert()) {
      a = {fpss.getAlertText1().cStr(), fpss.getAlertText2().cStr(),
           fpss.getAlertType().cStr(), static_cast<cereal::SelfdriveState::AlertSize>(fpss.getAlertSize()), static_cast<cereal::SelfdriveState::AlertStatus>(fpss.getAlertStatus())};
    }
  }

  // FrogPilot variables
  static QString crash_log_path = "/data/error_logs/error.txt";
  bool crashed = QFile::exists(crash_log_path);
  if (displayFerg && !crashed) {
    displayFerg = false;
    update();
  }

  bool stock_alert = selfdrive_frame >= started_frame && ss.getAlertSize() != cereal::SelfdriveState::AlertSize::NONE && !fpss.getHasPriorityAlert();
  if (crashed && !stock_alert && fpss.getAlertSize() != cereal::FrogPilotSelfdriveState::AlertSize::FULL) {
    bool random_events = frogpilot_toggles.value(QLatin1String("random_events")).toBool();
    if (random_events && enableFerg) {
      a = {};

      if (!displayFerg) {
        displayFerg = true;
        ferg = loadPixmap("../../frogpilot/assets/random_events/icons/ferg.svg", {1080, 720});
        update();
      }
    } else {
      a = {random_events ? tr("openpilot crashed 💩") : tr("openpilot crashed"),
           tr("Please post the \"Error Log\" in the FrogPilot Discord!"),
           random_events ? "openpilotCrashedRandomEvent" : "openpilotCrashed",
           cereal::SelfdriveState::AlertSize::MID,
           cereal::SelfdriveState::AlertStatus::CRITICAL};
    }
  }

  if (!sm.updated("selfdriveState") && (sm.frame - started_frame) > 5 * UI_FREQ && !frogpilot_toggles.value(QLatin1String("force_onroad")).toBool()) {
    const int SELFDRIVE_STATE_TIMEOUT = 5;
    const int ss_missing = (nanos_since_boot() - sm.rcv_time("selfdriveState")) / 1e9;

    // Handle selfdrive timeout
    if (selfdrive_frame < started_frame) {
      // car is started, but selfdriveState hasn't been seen at all
      a = {tr("openpilot Unavailable"), tr("Waiting to start"),
           "selfdriveWaiting", cereal::SelfdriveState::AlertSize::MID,
           cereal::SelfdriveState::AlertStatus::NORMAL};
    } else if (ss_missing > SELFDRIVE_STATE_TIMEOUT && !Hardware::PC()) {
      // car is started, but selfdrive is lagging or died
      if (ss.getEnabled() && (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10) {
        a = {tr("TAKE CONTROL IMMEDIATELY"), tr("System Unresponsive"),
             "selfdriveUnresponsive", cereal::SelfdriveState::AlertSize::FULL,
             cereal::SelfdriveState::AlertStatus::CRITICAL};
      } else {
        a = {tr("System Unresponsive"), tr("Reboot Device"),
             "selfdriveUnresponsivePermanent", cereal::SelfdriveState::AlertSize::MID,
             cereal::SelfdriveState::AlertStatus::NORMAL};
      }
    }
  }
  return a;
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::SelfdriveState::AlertSize::NONE) {
    // FrogPilot variables
    alertHeight = 0;

    if (displayFerg) {
      QPainter p(this);
      p.drawPixmap(QPoint((width() - ferg.width()) / 2, (height() - ferg.height()) / 2), ferg);
    }

    return;
  }

  if (isStagedUpdateAlert()) {
    const QRect r = alertRect();
    alertHeight = r.height() + STAGED_UPDATE_BADGE_MARGIN;

    QPainter p(this);
    p.setRenderHints(QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform);
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0x70), 3));
    p.setBrush(QColor(0x15, 0x15, 0x15, 0xf1));
    p.drawRoundedRect(r, 38, 38);

    const QRect iconRect(r.x() + 38, r.y() + (r.height() - stagedUpdateIcon.height()) / 2,
                         stagedUpdateIcon.width(), stagedUpdateIcon.height());
    p.drawPixmap(iconRect, stagedUpdateIcon);

    const int textLeft = iconRect.right() + 28;
    const QRect titleRect(textLeft, r.y() + 20, r.right() - textLeft - 28, 54);
    const QRect subtitleRect(textLeft, r.y() + 72, r.right() - textLeft - 28, 38);
    p.setPen(Qt::white);
    p.setFont(InterFont(44, QFont::DemiBold));
    p.drawText(titleRect, Qt::AlignLeft | Qt::AlignVCenter, alert.text1.toUpper());
    p.setFont(InterFont(28));
    p.drawText(subtitleRect, Qt::AlignLeft | Qt::AlignVCenter, alert.text2.toUpper());
    return;
  }

  static std::map<cereal::SelfdriveState::AlertSize, const int> alert_heights = {
    {cereal::SelfdriveState::AlertSize::SMALL, 271},
    {cereal::SelfdriveState::AlertSize::MID, 420},
    {cereal::SelfdriveState::AlertSize::FULL, height()},
  };
  int h = alert_heights[alert.size];

  int margin = 40;
  int radius = 30;
  if (alert.size == cereal::SelfdriveState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
  }

  // FrogPilot variables
  alertHeight = h - margin;

  QRect r = QRect(0 + margin, height() - h + margin, width() - margin*2, h - margin*2);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setBrush(QBrush(alert_colors[alert.status]));
  p.drawRoundedRect(r, radius, radius);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.drawRoundedRect(r, radius, radius);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::SelfdriveState::AlertSize::SMALL) {
    p.setFont(fitInterFont(74, QFont::DemiBold, r.width(), {alert.text1}));
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::SelfdriveState::AlertSize::MID) {
    p.setFont(fitInterFont(88, QFont::Bold, r.width(), {alert.text1}));
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    p.setFont(fitInterFont(66, QFont::Normal, r.width(), {alert.text2}));
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::SelfdriveState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    p.setFont(InterFont(l ? 132 : 177, QFont::Bold));
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    p.setFont(InterFont(88));
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}
