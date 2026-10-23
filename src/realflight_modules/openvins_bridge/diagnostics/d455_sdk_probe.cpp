// Standalone streaming diagnostic: no ROS, device reset, option or calibration writes.
#include <librealsense2/rs.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

struct Input {
    rs2_stream type;
    int index;
    int fps;
    rs2_format format;
    const char* label;
};

struct SensorRun {
    rs2::sensor sensor;
    std::string name;
    std::vector<rs2::stream_profile> profiles;
    bool opened = false;
    bool started = false;
};

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;
    const std::string mode = argc > 1 ? argv[1] : "all";
    const std::string serial = argc > 2 ? argv[2] : "";
    if (argc > 3 || (mode != "all" && mode != "imu" && mode != "ir")) {
        std::cerr << "Usage: d455_sdk_probe [all|imu|ir] [serial]\n";
        return 2;
    }

    std::vector<Input> inputs;
    if (mode != "ir") {
        inputs.push_back({RS2_STREAM_GYRO, 0, 400, RS2_FORMAT_MOTION_XYZ32F, "gyro"});
        inputs.push_back({RS2_STREAM_ACCEL, 0, 100, RS2_FORMAT_MOTION_XYZ32F, "accel"});
    }
    if (mode != "imu") {
        inputs.push_back({RS2_STREAM_INFRARED, 1, 30, RS2_FORMAT_Y8, "infra1"});
        inputs.push_back({RS2_STREAM_INFRARED, 2, 30, RS2_FORMAT_Y8, "infra2"});
    }
    auto counts = std::make_shared<std::array<std::atomic<unsigned long>, 4>>();
    for (auto& count : *counts) count.store(0);
    // Keep the context alive through stop/close, including exceptional exits.
    std::shared_ptr<rs2::context> context;
    std::vector<SensorRun> runs;
    int result = 1;
    try {
        rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
        rs2_error* error = nullptr;
        const int runtime = rs2_get_api_version(&error);
        rs2::error::handle(error);
        std::cout << "SDK header=" << RS2_API_VERSION_STR << " runtime=" << runtime
                  << " mode=" << mode << '\n';
        if (runtime != RS2_API_VERSION)
            throw std::runtime_error("SDK headers and runtime differ; check library paths");

        std::cout << "STAGE query_devices begin\n";
        context = std::make_shared<rs2::context>();
        auto devices = context->query_devices();
        std::vector<rs2::device> matches;
        for (auto device : devices) {
            if (device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) &&
                (serial.empty() || serial == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)))
                matches.push_back(device);
        }
        if (matches.size() != 1)
            throw std::runtime_error("Expected exactly one matching camera; connect it or specify serial");
        auto device = matches.front();
        for (const auto key : {RS2_CAMERA_INFO_NAME, RS2_CAMERA_INFO_SERIAL_NUMBER,
                              RS2_CAMERA_INFO_FIRMWARE_VERSION, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR,
                              RS2_CAMERA_INFO_PHYSICAL_PORT}) {
            if (device.supports(key))
                std::cout << rs2_camera_info_to_string(key) << '=' << device.get_info(key) << '\n';
        }
        std::cout << "STAGE query_sensors begin\n";
        std::vector<bool> found(inputs.size(), false);
        for (auto sensor : device.query_sensors()) {
            SensorRun run{sensor, sensor.get_info(RS2_CAMERA_INFO_NAME), {}};
            std::cout << "SENSOR " << run.name << '\n';
            for (const auto& profile : sensor.get_stream_profiles()) {
                if (profile.stream_type() == RS2_STREAM_GYRO ||
                    profile.stream_type() == RS2_STREAM_ACCEL)
                    std::cout << "PROFILE " << profile.stream_name() << " index="
                              << profile.stream_index() << " fps=" << profile.fps()
                              << " format=" << rs2_format_to_string(profile.format()) << '\n';
                for (size_t i = 0; i < inputs.size(); ++i) {
                    const auto& input = inputs[i];
                    if (found[i] || profile.stream_type() != input.type ||
                        profile.stream_index() != input.index || profile.fps() != input.fps ||
                        profile.format() != input.format) continue;
                    if (input.type == RS2_STREAM_INFRARED) {
                        auto video = profile.as<rs2::video_stream_profile>();
                        if (!video || video.width() != 848 || video.height() != 480) continue;
                    }
                    run.profiles.push_back(profile);
                    found[i] = true;
                    std::cout << "SELECT " << input.label << " fps=" << input.fps << '\n';
                }
            }
            if (!run.profiles.empty()) runs.push_back(run);
        }
        for (size_t i = 0; i < inputs.size(); ++i)
            if (!found[i]) throw std::runtime_error(std::string("Missing exact profile: ") + inputs[i].label);

        // Match the ROS 2.3.2 module-name ordering (Motion before Stereo).
        std::sort(runs.begin(), runs.end(), [](const SensorRun& a, const SensorRun& b) {
            return a.name < b.name;
        });
        for (auto& run : runs) {
            std::cout << "STAGE open begin: " << run.name << '\n';
            run.sensor.open(run.profiles);
            run.opened = true;
            std::cout << "STAGE open done: " << run.name << '\n';
            std::cout << "STAGE start begin: " << run.name << '\n';
            run.sensor.start([counts, inputs](rs2::frame frame) {
                auto profile = frame.get_profile();
                for (size_t i = 0; i < inputs.size(); ++i)
                    if (profile.stream_type() == inputs[i].type &&
                        profile.stream_index() == inputs[i].index)
                        (*counts)[i].fetch_add(1, std::memory_order_relaxed);
            });
            run.started = true;
            std::cout << "STAGE start done: " << run.name << '\n';
        }
        std::cout << "STAGE receive begin (10 seconds; cumulative frame counts)\n";
        for (int second = 1; second <= 10; ++second) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "COUNTS second=" << second;
            for (size_t i = 0; i < inputs.size(); ++i)
                std::cout << ' ' << inputs[i].label << '=' << (*counts)[i].load();
            std::cout << '\n';
        }
        result = 0;
        for (size_t i = 0; i < inputs.size(); ++i)
            if ((*counts)[i].load() == 0) result = 1;
    } catch (const rs2::error& error) {
        std::cerr << "SDK_ERROR " << error.get_failed_function() << '(' << error.get_failed_args()
                  << "): " << error.what() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "ERROR " << error.what() << '\n';
    }
    for (auto it = runs.rbegin(); it != runs.rend(); ++it) {
        try {
            if (it->started) {
                std::cout << "STAGE stop begin: " << it->name << '\n';
                it->sensor.stop();
            }
            if (it->opened) {
                std::cout << "STAGE close begin: " << it->name << '\n';
                it->sensor.close();
            }
        } catch (const std::exception& error) {
            std::cerr << "CLEANUP_ERROR " << error.what() << '\n';
            result = 1;
        }
    }
    std::cout << "RESULT " << (result == 0 ? "RECEIVED_ALL_REQUESTED_STREAMS" : "FAILED")
              << " (stream-presence test only; not a timing or calibration qualification)\n";
    return result;
}
