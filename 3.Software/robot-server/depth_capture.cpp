/**
 * depth_capture — Orbbec Astra Pro depth frame capture tool
 *
 * Usage: ./depth_capture <output_png_path> [width] [height] [fps]
 * Default: 640 480 30  (also supports 1280 1024 7)
 *
 * Outputs JSON to stdout, errors to stderr.
 * Colormap: Red (close) → Yellow → Green → Cyan → Blue (far)
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <libobsensor/ObSensor.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <string>
#include <sstream>

static void red_to_blue(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    t = std::max(0.0f, std::min(1.0f, t));
    float h = t * 240.0f;
    int   i = (int)(h / 60.0f);
    float f = h / 60.0f - i;
    float q = 1.0f - f, u = f;
    float rv, gv, bv;
    switch (i % 6) {
        case 0: rv=1; gv=u; bv=0; break;
        case 1: rv=q; gv=1; bv=0; break;
        case 2: rv=0; gv=1; bv=u; break;
        case 3: rv=0; gv=q; bv=1; break;
        default: rv=0; gv=0; bv=1; break;
    }
    r=(uint8_t)(rv*255); g=(uint8_t)(gv*255); b=(uint8_t)(bv*255);
}

// Minimal JSON escaping for file paths
static std::string json_str(const std::string &s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"')  out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else out += c;
    }
    out += "\"";
    return out;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: depth_capture <output.png> [width] [height] [fps]\n";
        return 1;
    }
    std::string out_path = argv[1];
    int req_w   = argc > 2 ? std::stoi(argv[2]) : 640;
    int req_h   = argc > 3 ? std::stoi(argv[3]) : 480;
    int req_fps = argc > 4 ? std::stoi(argv[4]) : 30;

    try {
        ob::Context ctx;
        // Silence all SDK output so stdout stays clean for JSON
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);
        ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_OFF);

        auto devList = ctx.queryDeviceList();
        if (devList->deviceCount() == 0) {
            std::cerr << "No Orbbec device found\n";
            return 1;
        }

        auto device = devList->getDevice(0);
        ob::Pipeline pipe(device);
        auto config = std::make_shared<ob::Config>();
        config->enableVideoStream(OB_STREAM_DEPTH, req_w, req_h, req_fps, OB_FORMAT_Y11);
        pipe.start(config);

        // Warm up — discard first 10 frames, use 11th good one
        std::shared_ptr<ob::FrameSet> best = nullptr;
        int good = 0;
        for (int i = 0; i < 60 && good < 11; i++) {
            auto fs = pipe.waitForFrames(500);
            if (fs && fs->depthFrame()) {
                best = fs;
                good++;
            }
        }
        pipe.stop();

        if (!best || !best->depthFrame()) {
            std::cerr << "Failed to capture depth frame\n";
            return 1;
        }

        auto df     = best->depthFrame();
        int W       = df->width();
        int H       = df->height();
        float scale = df->getValueScale();
        uint16_t *raw = (uint16_t *)df->data();

        // Collect valid depths for stats
        std::vector<float> valid;
        valid.reserve(W * H);
        float sum = 0;
        uint16_t abs_min_raw = 65535, abs_max_raw = 0;

        for (int i = 0; i < W * H; i++) {
            if (raw[i] == 0) continue;
            float d = raw[i] * scale;
            valid.push_back(d);
            sum += d;
            if (raw[i] < abs_min_raw) abs_min_raw = raw[i];
            if (raw[i] > abs_max_raw) abs_max_raw = raw[i];
        }

        if (valid.empty()) {
            std::cerr << "No valid depth pixels\n";
            return 1;
        }

        std::sort(valid.begin(), valid.end());
        int n = (int)valid.size();
        float p5  = valid[n * 5  / 100];
        float p95 = valid[n * 95 / 100];
        float mean = sum / n;
        float abs_min = abs_min_raw * scale;
        float abs_max = abs_max_raw * scale;
        float valid_pct = (float)n * 100.0f / (W * H);

        // Build colorized PNG
        std::vector<uint8_t> img(W * H * 3);
        for (int i = 0; i < W * H; i++) {
            uint8_t &r = img[i*3+0], &g = img[i*3+1], &b = img[i*3+2];
            if (raw[i] == 0) { r = g = b = 30; continue; }
            float t = (raw[i] * scale - p5) / (p95 - p5);
            red_to_blue(t, r, g, b);
        }

        if (!stbi_write_png(out_path.c_str(), W, H, 3, img.data(), W * 3)) {
            std::cerr << "Failed to write PNG: " << out_path << "\n";
            return 1;
        }

        // Output JSON to stdout
        std::cout
            << "{"
            << "\"ok\":true,"
            << "\"width\":"       << W                  << ","
            << "\"height\":"      << H                  << ","
            << "\"valid_pixels\":" << n                 << ","
            << "\"total_pixels\":" << (W * H)           << ","
            << "\"valid_pct\":"   << (int)(valid_pct * 10) / 10.0f << ","
            << "\"depth_min_mm\":" << (int)abs_min       << ","
            << "\"depth_max_mm\":" << (int)abs_max       << ","
            << "\"depth_p5_mm\":"  << (int)p5            << ","
            << "\"depth_p95_mm\":" << (int)p95           << ","
            << "\"depth_mean_mm\":" << (int)mean         << ","
            << "\"output_path\":" << json_str(out_path)
            << "}"
            << std::endl;

        return 0;

    } catch (ob::Error &e) {
        std::cerr << "ObError: " << e.getMessage() << "\n";
        return 1;
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
