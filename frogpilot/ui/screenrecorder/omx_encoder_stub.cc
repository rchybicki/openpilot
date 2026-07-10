#include "frogpilot/ui/screenrecorder/omx_encoder.h"

OmxEncoder::OmxEncoder(const char *path, int width, int height, int fps, int bitrate)
    : width(width), height(height), fps(fps), path(path) {}

OmxEncoder::~OmxEncoder() = default;

int OmxEncoder::encode_frame_rgba(const uint8_t *ptr, int in_width, int in_height, uint64_t ts) {
  return -1;
}

void OmxEncoder::encoder_open(const char *filename) {
  is_open = false;
}

void OmxEncoder::encoder_close() {
  is_open = false;
}
