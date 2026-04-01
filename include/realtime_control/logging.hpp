#pragma once

#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace flexiv_rt {

/// Thread-safe singleton logger for libpyflexiv.
/// Pattern: [MM/DD HH:MM:SS.mmm] [libpyflexiv] [level] message
inline std::shared_ptr<spdlog::logger>& logger() {
    static auto instance = [] {
        auto l = spdlog::stdout_color_mt("libpyflexiv");
        l->set_pattern("[%m/%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
        return l;
    }();
    return instance;
}

} // namespace flexiv_rt
