#pragma once

#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace flexiv_rt {

/// Thread-safe singleton logger for libpyflexiv.
/// Pattern: [YYYY-MM-DD HH:MM:SS] [libpyflexiv] [level] message
/// (no milliseconds — keeps log output clean next to Python's logging)
inline std::shared_ptr<spdlog::logger>& logger() {
    static auto instance = [] {
        auto l = spdlog::stdout_color_mt("libpyflexiv");
        l->set_pattern("[%Y-%m-%d %H:%M:%S] [%n] [%^%l%$] %v");
        return l;
    }();
    return instance;
}

} // namespace flexiv_rt
