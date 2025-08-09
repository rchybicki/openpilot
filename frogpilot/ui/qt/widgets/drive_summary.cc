#include "selfdrive/ui/qt/widgets/scrollview.h"

#include "frogpilot/ui/qt/widgets/drive_summary.h"

FrogPilotDriveSummary::FrogPilotDriveSummary(QWidget *parent, bool random_events) : QFrame(parent), displayRandomEvents(random_events) {
  QVBoxLayout *mainLayout = new QVBoxLayout(this);
  mainLayout->setContentsMargins(20, 20, 20, 20);
  mainLayout->setSpacing(15);

  titleLabel = new QLabel(random_events ? tr("Random Events Summary") : tr("Drive Summary"), this);
  titleLabel->setAlignment(Qt::AlignCenter);
  titleLabel->setStyleSheet(R"(
    QLabel {
      font-size: 50px;
      font-weight: bold;
      color: #FFFFFF;
      background-color: #444444;
      border-radius: 12px;
      padding: 12px 24px;
    }
  )");
  titleLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  titleLabel->setMaximumHeight(titleLabel->sizeHint().height());

  QWidget *containerWidget = new QWidget(this);
  QVBoxLayout *listLayout = new QVBoxLayout(containerWidget);
  listLayout->setSpacing(20);

  if (!displayRandomEvents) {
    listLayout->addWidget(createStatBox(tr("% of Drive With openpilot Engaged"), &engagementValue, this));
    listLayout->addWidget(createStatBox(tr("Drive Distance"), &frogPilotMetersValue, this));
    listLayout->addWidget(createStatBox(tr("Drive Time"), &trackedTimeValue, this));
    listLayout->addWidget(createStatBox(tr("Time Spent In \"Experimental Mode\""), &experimentalModeTimeValue, this));
  } else {
    randomEventsMap.insert("accel30", tr("UwUs"));
    randomEventsMap.insert("accel35", tr("Loch Ness Encounters"));
    randomEventsMap.insert("accel40", tr("Visits to 1955"));
    randomEventsMap.insert("dejaVuCurve", tr("Deja Vu Moments"));
    randomEventsMap.insert("firefoxSteerSaturated", tr("Internet Explorer Weeeeeeees"));
    randomEventsMap.insert("hal9000", tr("HAL 9000 Denials"));
    randomEventsMap.insert("openpilotCrashedRandomEvent", tr("openpilot Crashes"));
    randomEventsMap.insert("thisIsFineSteerSaturated", tr("This Is Fine Moments"));
    randomEventsMap.insert("toBeContinued", tr("To Be Continued Moments"));
    randomEventsMap.insert("vCruise69", tr("Noices"));
    randomEventsMap.insert("yourFrogTriedToKillMe", tr("Attempted Frog Murders"));
    randomEventsMap.insert("youveGotMail", tr("Total Mail Received"));
  }

  mainLayout->addWidget(titleLabel);
  mainLayout->addSpacing(10);

  if (displayRandomEvents) {
    ScrollView *scrollView = new ScrollView(containerWidget, this);
    mainLayout->addWidget(scrollView, 1);
    eventsListLayout = listLayout;
  } else {
    mainLayout->addWidget(containerWidget, 1);
  }

  setLayout(mainLayout);

  setStyleSheet(R"(
    QFrame {
      background-color: #2B2B2B;
    }
  )");

  QObject::connect(uiState(), &UIState::offroadTransition, [this](bool offroad) {
    if (!offroad) {
      snapshotStats = QJsonDocument::fromJson(QString::fromStdString(params.get("FrogPilotStats")).toUtf8()).object();
    }
  });
}

void FrogPilotDriveSummary::showEvent(QShowEvent *event) {
  bool is_metric = params.getBool("IsMetric");

  QJsonObject currentStats = QJsonDocument::fromJson(QString::fromStdString(params.get("FrogPilotStats")).toUtf8()).object();

  if (!displayRandomEvents) {
    std::function<int(const QString &)> diffInt = [currentStats, this](const QString &key) -> int {
      return currentStats.value(key).toInt() - snapshotStats.value(key).toInt();
    };

    std::function<QString(double)> format_distance = [&](double meters) {
      double value;
      QString unit;
      if (is_metric) {
        value = meters / 1000.0;
        unit = (value == 1.0) ? tr(" kilometer") : tr(" kilometers");
      } else {
        value = meters * METER_TO_MILE;
        unit = (value == 1.0) ? tr(" mile") : tr(" miles");
      }
      return QLocale().toString(qRound(value)) + unit;
    };

    std::function<QString(int)> format_time = [&](int seconds) {
      static int seconds_in_day = 60 * 60 * 24;
      static int seconds_in_hour = 60 * 60;

      int days = seconds / seconds_in_day;
      int hours = (seconds % seconds_in_day) / seconds_in_hour;
      int minutes = (seconds % seconds_in_hour) / 60;

      QString result;
      if (days > 0) result += QLocale().toString(days) + (days == 1 ? tr(" day ") : tr(" days "));
      if (hours > 0 || days > 0) result += QLocale().toString(hours) + (hours == 1 ? tr(" hour ") : tr(" hours "));
      result += QLocale().toString(minutes) + (minutes == 1 ? tr(" minute") : tr(" minutes"));
      return result.trimmed();
    };

    int override_time = diffInt("OverrideTime");
    int tracked_time = diffInt("TrackedTime");
    engagementValue->setText(QLocale().toString((tracked_time > 0) ? ((tracked_time - override_time) * 100 / tracked_time) : 0) + "%");
    experimentalModeTimeValue->setText(format_time(diffInt("ExperimentalModeTime")));
    frogPilotMetersValue->setText(format_distance(diffInt("FrogPilotMeters")));
    trackedTimeValue->setText(format_time(diffInt("TrackedTime")));
  } else {
    QJsonObject currentRandomEvents = currentStats.value("RandomEvents").toObject();
    QJsonObject snapshotRandomEvents = snapshotStats.value("RandomEvents").toObject();

    QList<QPair<QString, int>> eventsList;
    for (QMap<QString, QString>::const_iterator it = randomEventsMap.constBegin(); it != randomEventsMap.constEnd(); ++it) {
      int currentValue = currentRandomEvents.value(it.key()).toInt();
      int snapshotValue = snapshotRandomEvents.value(it.key()).toInt();
      int diffValue = currentValue - snapshotValue;

      if (diffValue > 0) {
        eventsList.append(qMakePair(it.value(), diffValue));
      }
    }

    std::sort(eventsList.begin(), eventsList.end(), [](const QPair<QString, int> &a, const QPair<QString, int> &b) {
      if (a.second != b.second) {
        return a.second > b.second;
      } else {
        return a.first.localeAwareCompare(b.first) < 0;
      }
    });

    while (QLayoutItem *child = eventsListLayout->takeAt(0)) {
      delete child->widget();
      delete child;
    }
    randomEventLabels.clear();

    if (eventsList.isEmpty()) {
      QLabel *noEventsLabel = new QLabel(tr("No Random Events Played!"), this);
      noEventsLabel->setAlignment(Qt::AlignCenter);
      noEventsLabel->setStyleSheet(R"(
        QLabel {
          font-size: 50px;
          font-weight: bold;
          color: #FFFFFF;
        }
      )");
      eventsListLayout->addWidget(noEventsLabel);
    } else {
      for (QList<QPair<QString, int>>::const_iterator it = eventsList.constBegin(); it != eventsList.constEnd(); ++it) {
        QLabel *valueLabel = nullptr;
        eventsListLayout->addWidget(createStatBox(it->first, &valueLabel, this));
        randomEventLabels.insert(it->first, valueLabel);
        valueLabel->setText(QLocale().toString(it->second));
      }
    }
  }
}

void FrogPilotDriveSummary::hideEvent(QHideEvent *event) {
  emit panelClosed();
}

QWidget *FrogPilotDriveSummary::createStatBox(const QString &title, QLabel **valueLabel, QWidget *parent) {
  QWidget *box = new QWidget(parent);

  QVBoxLayout *layout = new QVBoxLayout(box);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(8);

  QLabel *statTitleLabel = new QLabel(title, box);
  statTitleLabel->setAlignment(Qt::AlignCenter);
  statTitleLabel->setStyleSheet(R"(
    QLabel {
      font-size: 40px;
      font-weight: bold;
      color: #AAAAAA;
    }
  )");

  QLabel *value = new QLabel("-", box);
  value->setAlignment(Qt::AlignCenter);
  value->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  value->setStyleSheet(R"(
    QLabel {
      font-size: 75px;
      font-weight: bold;
      color: #FFFFFF;
    }
  )");

  *valueLabel = value;

  layout->addWidget(statTitleLabel);
  layout->addWidget(value, 1);

  return box;
}
