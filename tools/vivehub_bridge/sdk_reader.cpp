// VisionTactile adapter. Links the separately installed HTC SDK; no HID access.
#include <vut/vut_client.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <time.h>

namespace {
volatile sig_atomic_t running = 1;
void stop(int) { running = 0; }
uint64_t clock_ns(clockid_t clock) {
    timespec t{}; clock_gettime(clock, &t);
    return uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
struct Event {
    vut::Pose pose;
    uint32_t state;
    uint64_t mono, real, clock_span, generation, dropped;
};
void vector_json(const float* p, int size) {
    std::cout << '[';
    for (int i = 0; i < size; ++i) {
        if (i) std::cout << ',';
        if (std::isfinite(p[i])) std::cout << p[i];
        else std::cout << "null";
    }
    std::cout << ']';
}
}

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "usage: vt-vivehub-reader SOCKET\n"; return 2; }
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    // All callbacks only copy into bounded storage; stdout/SDK queries stay here.
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Event> queue;
    std::map<uint32_t, uint32_t> states;
    uint64_t generation = 0, dropped = 0;
    bool connected = false;
    vut::Client client(argv[1]);
    client.set_auto_reconnect(true, 200, 2000);
    client.on_connection_change([&](bool up) {
        std::lock_guard<std::mutex> lock(mutex);
        connected = up; ++generation; states.clear(); queue.clear();
        ready.notify_one();
    });
    client.on_status([&](const vut::Status& s) {
        std::lock_guard<std::mutex> lock(mutex);
        states[s.device_id] = s.error_code == 0 ? s.state : 255;
    });
    client.on_pose([&](const vut::Pose& p) {
        const auto before = clock_ns(CLOCK_MONOTONIC);
        const auto real = clock_ns(CLOCK_REALTIME);
        const auto clock_span = clock_ns(CLOCK_MONOTONIC) - before;
        const auto mono = before + clock_span / 2;
        std::lock_guard<std::mutex> lock(mutex);
        if (!connected) return;
        if (queue.size() >= 256) { queue.pop_front(); ++dropped; }
        const auto state = states.find(p.device_id);
        queue.push_back({p, state == states.end() ? 255U : state->second,
                         mono, real, clock_span, generation, dropped});
        ready.notify_one();
    });
    if (client.connect("visiontactile_readonly", vut::SUB_ALL) != 0) return 3;
    auto next_query = std::chrono::steady_clock::now();
    std::cout << std::setprecision(9);
    while (running && std::cout.good()) {
        // A subscription only sends state CHANGES: seed existing Ready states.
        // Never overwrite a newer callback with a synchronous query snapshot.
        if (std::chrono::steady_clock::now() >= next_query && client.is_connected()) {
            uint64_t query_generation;
            { std::lock_guard<std::mutex> lock(mutex); query_generation = generation; }
            std::vector<vut::Device> devices;
            if (client.list_devices(devices, 250) == 0) {
                std::lock_guard<std::mutex> lock(mutex);
                if (connected && query_generation == generation)
                    for (const auto& d : devices) states.emplace(d.device_id, d.state);
            }
            next_query = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        }
        Event e{};
        {
            std::unique_lock<std::mutex> lock(mutex);
            ready.wait_for(lock, std::chrono::milliseconds(50), [&]{ return !queue.empty(); });
            if (queue.empty()) continue;
            e = queue.front(); queue.pop_front();
        }
        const auto& p = e.pose;
        std::cout << "{\"type\":\"pose\",\"serial\":"
                  << std::quoted(std::string(p.serial, strnlen(p.serial, sizeof(p.serial))))
                  << ",\"device_id\":" << p.device_id << ",\"state\":" << e.state
                  << ",\"timestamp_ns\":" << p.timestamp_ns
                  << ",\"receive_monotonic_ns\":" << e.mono
                  << ",\"receive_realtime_ns\":" << e.real
                  << ",\"receive_clock_span_ns\":" << e.clock_span
                  << ",\"generation\":" << e.generation
                  << ",\"dropped_total\":" << e.dropped
                  << ",\"flags\":" << p.flags << ",\"position\":";
        vector_json(p.position, 3);
        std::cout << ",\"orientation\":"; vector_json(p.orientation, 4);
        std::cout << ",\"angular_velocity\":"; vector_json(p.angular_velocity, 3);
        std::cout << "}\n" << std::flush;
    }
    client.disconnect();
    return 0;
}
