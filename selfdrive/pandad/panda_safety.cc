#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

namespace {
std::string live_update_handoff_state_name(const std::string &state) {
  const size_t separator = state.find(':');
  return state.substr(0, separator);
}

bool is_live_update_handoff_state(const std::string &state_name) {
  return state_name == "diagnostic_requested" || state_name == "diagnostic" || state_name == "verifying" ||
         state_name == "ready" || state_name == "failed";
}
}

void PandaSafety::configureSafetyMode(bool is_onroad, bool controls_engaged) {
  const std::string handoff_state = params_.get("LiveUpdateHandoffState");
  const std::string handoff_state_name = live_update_handoff_state_name(handoff_state);
  const bool handoff_requested = is_live_update_handoff_state(handoff_state_name);
  if (is_onroad && (handoff_requested || live_update_handoff_mode_)) {
    if (controls_engaged && !live_update_handoff_mode_) {
      LOGE("refusing live update handoff safety mode while controls are engaged");
      return;
    }

    if (!live_update_handoff_mode_) {
      LOGW("entering live update handoff safety mode");
    }
    for (int i = 0; i < pandas_.size(); ++i) {
      const std::optional<health_t> state = pandas_[i]->get_state();
      if (!live_update_handoff_mode_ || !state || state->safety_mode_pkt != (uint8_t)cereal::CarParams::SafetyModel::ELM327) {
        pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
      }
    }
    if (!live_update_handoff_mode_) {
      live_update_handoff_mode_ = true;
      params_.remove("ControlsReady");
    }

    if (!handoff_requested) {
      LOGE("live update handoff state disappeared after entering diagnostic mode; holding ELM327 and blocking reboot");
      params_.put("LiveUpdateHandoffState", "failed");
    } else if (handoff_state_name == "diagnostic_requested") {
      params_.put("LiveUpdateHandoffState", "diagnostic");
    }
    return;
  }

  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    live_update_handoff_mode_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    for (int i = 0; i < pandas_.size(); ++i) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    }
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    for (int i = 0; i < pandas_.size(); ++i) {
      const uint16_t safety_param = (i > 0 || !obd_multiplexing_requested) ? 1U : 0U;
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    }
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

std::string PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return params_.get("CarParams");
}

void PandaSafety::setSafetyMode(const std::string &params_string) {
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();

  // FrogPilot variables
  std::string frogpilot_params_string = params_.get("FrogPilotCarParams");

  AlignedBuffer frogpilot_aligned_buf;
  capnp::FlatArrayMessageReader frogpilot_cmsg(frogpilot_aligned_buf.align(frogpilot_params_string.data(), frogpilot_params_string.size()));
  cereal::FrogPilotCarParams::Reader frogpilot_car_params = frogpilot_cmsg.getRoot<cereal::FrogPilotCarParams>();

  auto frogpilot_safety_configs = frogpilot_car_params.getSafetyConfigs();
  alternative_experience |= frogpilot_car_params.getAlternativeExperience();

  for (int i = 0; i < pandas_.size(); ++i) {
    // Default to SILENT safety model if not specified
    cereal::CarParams::SafetyModel safety_model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t safety_param = 0U;
    if (i < safety_configs.size()) {
      safety_model = safety_configs[i].getSafetyModel();
      safety_param = safety_configs[i].getSafetyParam();
    }

    // FrogPilot variables
    if (i < frogpilot_safety_configs.size()) {
      safety_param |= frogpilot_safety_configs[i].getSafetyParam();
    }

    LOGW("Panda %d: setting safety model: %d, param: %d, alternative experience: %d", i, (int)safety_model, safety_param, alternative_experience);
    pandas_[i]->set_alternative_experience(alternative_experience);
    pandas_[i]->set_safety_model(safety_model, safety_param);
  }
}
