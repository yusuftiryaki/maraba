// kit_camera_shim — the one program that links Insta360's Camera SDK.
//
// robot_kit/camera.py is the protocol's other end and its definition:
// JSON lines on stdin / stdout, one request -> one response, plus unsolicited
// {"event": ...} lines from the SDK's notifications. kit_camera (ROS) starts
// this process; a crash in the closed-source SDK takes down only this, and a
// robot built without the SDK still builds (the shim is compiled separately:
// scripts/build_camera_shim.sh).
//
//   cmd          request fields          response data
//   connect                              {"model", "serial", "firmware"}
//   status                               {"recording", "battery_pct", "on_adapter",
//                                          "storage", "free_bytes"}
//   start        "resolution"            {}
//   stop                                 {"files": [...]}
//   sync_time    "utc_ms", "tz_offset_s" {}
//   media_time                           {"media_time"}
//
// Written against the SDK 2.1.8 headers (ins_camera::Camera, DeviceDiscovery);
// the call sequence follows the SDK's own example/common/camera_setup.h.
// ⚠️ NOT RUN AGAINST A CAMERA YET (2026-10-26): it compiles and links on
// x86-64 against SDK 2.1.8, and with no camera on the bus it answers
// "no camera found" (test_camera.py, with KIT_CAMERA_SHIM set).

#include <camera/camera.h>
#include <camera/device_discovery.h>

#include <cctype>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::mutex out_mutex;

void emit(const std::string& line) {
    std::lock_guard<std::mutex> lock(out_mutex);
    std::cout << line << "\n" << std::flush;
}

std::string quote(const std::string& text) {
    std::string out = "\"";
    for (char c : text) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            out += ' ';
        } else {
            out += c;
        }
    }
    return out + "\"";
}

// A flat JSON object's values, as raw text (strings unquoted). The requests
// camera.py sends are flat: {"id": 7, "cmd": "start", "resolution": "5.7K30"}.
std::map<std::string, std::string> parse_flat(const std::string& line) {
    std::map<std::string, std::string> out;
    size_t i = 0;
    auto skip = [&]() { while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i; };
    auto read_string = [&]() {
        std::string s;
        ++i;                                       // opening quote
        while (i < line.size() && line[i] != '"') {
            if (line[i] == '\\' && i + 1 < line.size()) ++i;
            s += line[i++];
        }
        ++i;                                       // closing quote
        return s;
    };
    skip();
    if (i >= line.size() || line[i] != '{') return out;
    ++i;
    while (i < line.size()) {
        skip();
        if (line[i] == '}') break;
        if (line[i] != '"') return {};
        std::string key = read_string();
        skip();
        if (i >= line.size() || line[i] != ':') return {};
        ++i;
        skip();
        std::string value;
        if (i < line.size() && line[i] == '"') {
            value = read_string();
        } else {
            while (i < line.size() && line[i] != ',' && line[i] != '}') value += line[i++];
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) value.pop_back();
        }
        out[key] = value;
        skip();
        if (i < line.size() && line[i] == ',') ++i;
    }
    return out;
}

const std::map<std::string, ins_camera::VideoResolution> RESOLUTIONS = {
    {"5.7K30", ins_camera::VideoResolution::RES_2880_2880P30},
    {"8K30", ins_camera::VideoResolution::RES_3840_3840P30},
};

const char* storage_name(ins_camera::CardState state) {
    switch (state) {
    case ins_camera::STOR_CS_PASS: return "ok";
    case ins_camera::STOR_CS_NOCARD: return "no_card";
    case ins_camera::STOR_CS_NOSPACE: return "full";
    case ins_camera::STOR_CS_INVALID_FORMAT: return "bad_format";
    case ins_camera::STOR_CS_WPCARD: return "write_protected";
    default: return "error";
    }
}

class Shim {
public:
    std::string handle(const std::string& cmd, const std::map<std::string, std::string>& req) {
        if (cmd == "connect") return connect();
        if (!camera_) throw std::runtime_error("not connected");
        if (cmd == "status") return status();
        if (cmd == "start") return start(req);
        if (cmd == "stop") return stop();
        if (cmd == "sync_time") {
            const uint64_t utc_ms = std::stoull(req.at("utc_ms"));
            const int32_t offset = static_cast<int32_t>(std::stol(req.count("tz_offset_s") ? req.at("tz_offset_s") : "0"));
            // camera.h: "current UTC timestamp in seconds (e.g. time(nullptr))".
            if (!camera_->SyncLocalTimeToCamera(utc_ms / 1000, offset))
                throw std::runtime_error("SyncLocalTimeToCamera failed");
            return "{}";
        }
        if (cmd == "media_time") {
            return "{\"media_time\": " + std::to_string(camera_->GetCameraMediaTime()) + "}";
        }
        throw std::runtime_error("unknown command " + cmd);
    }

private:
    std::string connect() {
        if (camera_ && camera_->IsConnected()) return info_;
        ins_camera::SetLogLevel(ins_camera::LogLevel::ERR);
        ins_camera::DeviceDiscovery discovery;
        auto list = discovery.GetAvailableDevices();
        if (list.empty()) throw std::runtime_error("no camera found (USB in Android mode? udev rule?)");
        const std::string serial = list[0].serial_number;
        const std::string name = list[0].camera_name;
        const std::string firmware = list[0].fw_version;
        auto camera = std::make_shared<ins_camera::Camera>(list[0].info);
        if (!camera->Open()) {
            discovery.FreeDeviceDescriptors(list);
            throw std::runtime_error("failed to open the camera");
        }
        discovery.FreeDeviceDescriptors(list);
        // Unplugging the kit must not stop a scan's recording: the video is on
        // the camera's own card and the robot only stops driving
        // (docs/kit-soketi.md). Off is the SDK's default; stated anyway.
        camera->EnableAutoStopRecordAfterDisconnet(false);
        camera->SetBatteryLowNotification([](int) { emit("{\"event\": \"battery_low\"}"); });
        camera->SetStorageFullNotification([]() { emit("{\"event\": \"storage_full\"}"); });
        camera->SetCaptureStoppedNotification(
            [](const std::string&, int) { emit("{\"event\": \"capture_stopped\"}"); });
        camera->SetTemperatureHighNotification([]() { emit("{\"event\": \"temperature_high\"}"); });
        camera_ = camera;
        info_ = "{\"model\": " + quote(name) + ", \"serial\": " + quote(serial) +
                ", \"firmware\": " + quote(firmware) + "}";
        return info_;
    }

    std::string status() {
        ins_camera::BatteryStatus battery{};
        const bool have_battery = camera_->GetBatteryStatus(battery);
        ins_camera::StorageStatus storage{};
        const bool have_storage = camera_->GetStorageState(storage);
        std::ostringstream out;
        out << "{\"recording\": " << (camera_->CaptureCurrentStatus() ? "true" : "false")
            << ", \"battery_pct\": " << (have_battery ? std::to_string(battery.battery_level) : "null")
            << ", \"on_adapter\": "
            << (have_battery ? (battery.power_type == ins_camera::ADAPTER ? "true" : "false") : "null")
            << ", \"storage\": " << (have_storage ? quote(storage_name(storage.state)) : "null")
            << ", \"free_bytes\": " << (have_storage ? std::to_string(storage.free_space) : "null")
            << "}";
        return out.str();
    }

    std::string start(const std::map<std::string, std::string>& req) {
        const std::string name = req.count("resolution") ? req.at("resolution") : "5.7K30";
        const auto res = RESOLUTIONS.find(name);
        if (res == RESOLUTIONS.end()) throw std::runtime_error("unknown resolution " + name);
        if (!camera_->SetVideoSubMode(ins_camera::SubVideoMode::VIDEO_NORMAL))
            throw std::runtime_error("SetVideoSubMode failed");
        ins_camera::RecordParams params;
        params.resolution = res->second;
        if (!camera_->SetVideoCaptureParams(params, ins_camera::CameraFunctionMode::FUNCTION_MODE_NORMAL_VIDEO))
            throw std::runtime_error("SetVideoCaptureParams failed (" + name + " on this model?)");
        if (!camera_->StartRecording()) throw std::runtime_error("StartRecording failed");
        return "{}";
    }

    std::string stop() {
        const auto url = camera_->StopRecording();
        if (url.Empty()) throw std::runtime_error("StopRecording returned no file");
        std::string files = "[";
        for (const auto& origin : url.OriginUrls()) {
            if (files.size() > 1) files += ", ";
            files += quote(origin);
        }
        return "{\"files\": " + files + "]}";
    }

    std::shared_ptr<ins_camera::Camera> camera_;
    std::string info_;
};

}  // namespace

int main() {
    Shim shim;
    std::string line;
    while (std::getline(std::cin, line)) {
        const auto req = parse_flat(line);
        const std::string id = req.count("id") ? req.at("id") : "null";
        try {
            const std::string data = shim.handle(req.count("cmd") ? req.at("cmd") : "", req);
            emit("{\"id\": " + id + ", \"ok\": true, \"data\": " + data + "}");
        } catch (const std::exception& error) {
            emit("{\"id\": " + id + ", \"ok\": false, \"error\": " + quote(error.what()) + "}");
        }
    }
    return 0;
}
