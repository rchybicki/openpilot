#include "selfdrive/ui/qt/onroad/hud.h"

#include <cmath>
#include <vector>

#include "selfdrive/ui/qt/util.h"

constexpr int SET_SPEED_NA = 255;

HudRenderer::HudRenderer() {}

void HudRenderer::updateState(const UIState &s) {
  is_metric = s.scene.is_metric;
  status = s.status;

  const SubMaster &sm = *(s.sm);
  if (sm.rcv_frame("carState") < s.scene.started_frame) {
    is_cruise_set = false;
    set_speed = SET_SPEED_NA;
    speed = 0.0;
    return;
  }

  const auto &controls_state = sm["controlsState"].getControlsState();
  const auto &car_state = sm["carState"].getCarState();

  // Handle older routes where vCruiseCluster is not set
  set_speed = car_state.getVCruiseCluster() == 0.0 ? controls_state.getVCruiseDEPRECATED() : car_state.getVCruiseCluster();
  is_cruise_set = set_speed > 0 && set_speed != SET_SPEED_NA;
  is_cruise_available = set_speed != -1;

  if (is_cruise_set && !is_metric) {
    set_speed *= KM_TO_MILE;
  }

  v_ego_raw = car_state.getVEgo();
  a_ego = car_state.getAEgo();
  brake_lights = car_state.getBrakeLightsDEPRECATED() || car_state.getBrakePressed();
  stopping = controls_state.getLongControlState() == cereal::CarControl::Actuators::LongControlState::STOPPING;

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen && !frogpilot_toggles.value("use_wheel_speed").toBool() ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = std::max<float>(0.0f, v_ego * (is_metric ? MS_TO_KPH : MS_TO_MPH));
}

void HudRenderer::draw(QPainter &p, const QRect &surface_rect) {
  p.save();

  // Draw header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, surface_rect.width(), UI_HEADER_HEIGHT, bg);


  if (is_cruise_available) {
    drawSetSpeed(p, surface_rect);
  }
  if (frogpilot_nvg->standstillDuration == 0 && !frogpilot_toggles.value("hide_speed").toBool()) {
    drawCurrentSpeed(p, surface_rect);
  }

  p.restore();
}

void HudRenderer::drawSetSpeed(QPainter &p, const QRect &surface_rect) {
  // Draw outer box + border to contain set speed
  const QSize default_size = {172, 204};
  QSize set_speed_size = is_metric ? QSize(200, 204) : default_size;

  // FrogPilot variables
  if (frogpilot_nvg->speedLimitHeight != 0) {
    set_speed_size.rheight() += frogpilot_nvg->speedLimitHeight;
    if (frogpilot_toggles.value("speed_limit_vienna").toBool()) {
      set_speed_size.rwidth() = 200;
    }
  }

  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2, 45), set_speed_size);

  if (!frogpilot_toggles.value("hide_max_speed").toBool()) {
    // Draw set speed box
    p.setPen(QPen(QColor(255, 255, 255, 75), 6));
    p.setBrush(QColor(0, 0, 0, 166));
    p.drawRoundedRect(set_speed_rect, 32, 32);

    // Colors based on status
    QColor max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
    QColor set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
    if (is_cruise_set) {
      set_speed_color = QColor(255, 255, 255);
      if (status == STATUS_DISENGAGED) {
        max_color = QColor(255, 255, 255);
      } else if (status == STATUS_OVERRIDE) {
        max_color = QColor(0x91, 0x9b, 0x95, 0xff);
      } else {
        max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
      }
    }

    // Draw "MAX" text
    p.setFont(InterFont(40, QFont::DemiBold));
    p.setPen(max_color);
    p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));

    // Draw set speed
    QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(set_speed)) : "–";
    p.setFont(InterFont(90, QFont::Bold));
    p.setPen(set_speed_color);
    p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);
  }

  // FrogPilot variables
  frogpilot_nvg->defaultSize = default_size;
  frogpilot_nvg->isCruiseSet = is_cruise_set;
  frogpilot_nvg->setSpeedRect = set_speed_rect;
  frogpilot_nvg->speed = speed;
}

void HudRenderer::drawCurrentSpeed(QPainter &p, const QRect &surface_rect) {
  QString speedStr = QString::number(std::nearbyint(speed));

  QColor speed_color = QColor(0xff, 0xff, 0xff);
  QColor unit_color = QColor(0xff, 0xff, 0xff, 200);
  if (stopping || brake_lights) {
    speed_color = QColor(0xde, 0x98, 0x00, 0xff);
    unit_color = QColor(0xde, 0x98, 0x00, 200);
    if (brake_lights && !stopping) {
      speed_color = QColor(0xde, 0x00, 0x00, 0xff);
      unit_color = QColor(0xde, 0x00, 0x00, 200);
    }
  }

  p.setFont(InterFont(176, QFont::Bold));
  drawTextColor(p, surface_rect.center().x(), 210, speedStr, speed_color);

  p.setFont(InterFont(66));
  drawTextColor(p, surface_rect.center().x(), 290, is_metric ? tr("km/h") : tr("mph"), unit_color);

  if (v_ego_raw < 1.0f) {
    const auto cp = (*uiState()->sm)["carParams"].getCarParams();
    std::vector<float> stopping_v_bp;
    std::vector<float> stopping_accel_max;
    std::vector<float> stopping_accel_min;
    for (float value : cp.getStoppingVbp()) {
      stopping_v_bp.push_back(value);
    }
    for (float value : cp.getStoppingAccelMax()) {
      stopping_accel_max.push_back(value);
    }
    for (float value : cp.getStoppingAccelMin()) {
      stopping_accel_min.push_back(value);
    }

    const bool has_stopping_data = stopping_v_bp.size() >= 2 &&
                                   stopping_v_bp.size() == stopping_accel_max.size() &&
                                   stopping_v_bp.size() == stopping_accel_min.size();

    float expected_accel_max = 0.0f;
    float expected_accel_min = 0.0f;
    if (has_stopping_data) {
      expected_accel_max = interpolateAccel(v_ego_raw, stopping_v_bp, stopping_accel_max);
      expected_accel_min = interpolateAccel(v_ego_raw, stopping_v_bp, stopping_accel_min);
    }

    const int left_x = surface_rect.center().x() - 490;
    const int y_center = 210;

    p.setFont(InterFont(40, QFont::Normal));

    QString max_str = has_stopping_data ? QString("Max: %1 m/s^2").arg(expected_accel_max, 0, 'f', 2) : QString("Max: N/A");
    QColor max_color = has_stopping_data ? QColor(255, 255, 255, 180) : QColor(255, 165, 0, 220);
    drawTextColor(p, left_x, y_center - 30, max_str, max_color);

    QString min_str = has_stopping_data ? QString("Min: %1 m/s^2").arg(expected_accel_min, 0, 'f', 2) : QString("Min: N/A");
    QColor min_color = has_stopping_data ? QColor(255, 255, 255, 180) : QColor(255, 165, 0, 220);
    drawTextColor(p, left_x, y_center + 30, min_str, min_color);

    const int right_x = left_x + 290;
    QString vego_str = QString::number(v_ego_raw, 'f', 2) + " m/s";
    QString aego_str = QString::number(a_ego, 'f', 2) + " m/s^2";

    p.setFont(InterFont(50, QFont::DemiBold));
    QColor vego_color = v_ego_raw <= 0.02f ? QColor(255, 0, 0, 255) : QColor(255, 255, 255, 255);
    drawTextColor(p, right_x, y_center - 30, vego_str, vego_color);

    QColor accel_color = has_stopping_data ? QColor(255, 255, 255, 255) : QColor(255, 165, 0, 255);
    if (has_stopping_data) {
      if (a_ego > expected_accel_max) {
        accel_color = QColor(0, 255, 0, 255);
      } else if (a_ego < expected_accel_min) {
        accel_color = QColor(255, 0, 0, 255);
      }
    }
    drawTextColor(p, right_x, y_center + 30, aego_str, accel_color);
  }
}

void HudRenderer::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void HudRenderer::drawTextColor(QPainter &p, int x, int y, const QString &text, const QColor &color) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

float HudRenderer::interpolateAccel(float v_ego, const std::vector<float> &bp, const std::vector<float> &vals) {
  if (bp.size() < 2 || bp.size() != vals.size()) {
    return 0.0f;
  }
  if (v_ego <= bp.front()) {
    return vals.front();
  }
  if (v_ego >= bp.back()) {
    return vals.back();
  }

  for (size_t i = 0; i < bp.size() - 1; ++i) {
    if (v_ego >= bp[i] && v_ego <= bp[i + 1]) {
      float factor = (v_ego - bp[i]) / (bp[i + 1] - bp[i]);
      return vals[i] + factor * (vals[i + 1] - vals[i]);
    }
  }
  return vals.back();
}
