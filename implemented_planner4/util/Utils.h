#pragma once

#include <ctime>
#include <string>
#include <unordered_map>
#include <iostream>
#include <chrono>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "nlohmann/json.hpp"
#include "common.h"

using std::string;
using std::cerr;
using std::cout;
using std::endl;
using std::chrono::steady_clock;
using std::chrono::duration;

// ====================================================================================================
// Timer Class - Performance measurement utility
// ====================================================================================================
// Provides timing functionality for measuring execution durations.
// Supports multiple named timers and duration tracking.
// ====================================================================================================
class Timer
{
public:
    // Record start time for a point
    steady_clock::time_point record_timepoint(const string & pkey);
    
    // Record duration between two points
    double record_duration(const string & old_pkey, const string & new_pkey);
    double record_duration(const string & old_pkey, const string & new_pkey, const string & dkey);

    // Get recorded times
    steady_clock::time_point get_timepoint(const string & pkey) const;
    double get_duration(const string & dkey, int mode=0) const;  // mode: 0=sum, 1=mean, 2=last

    // Print durations
    void print_duration(const string & dkey) const;

private:
    std::unordered_map<string,steady_clock::time_point> time_points;  // Start/end points
    std::unordered_map<string,double> time_durations;  // Accumulated durations
    std::unordered_map<string,size_t> time_duration_counters;  // Count of recordings
    std::unordered_map<string,double> time_durations_last;  // Last recorded duration
};

// ====================================================================================================
// Global Timer Instance
// ====================================================================================================
extern Timer g_timer;

// ====================================================================================================
// TimeLimiter Class - Execution time limiting utility
// ====================================================================================================
// Provides timeout functionality for operations that should not exceed a time limit.
// ====================================================================================================
class TimeLimiter {
public:
    steady_clock::time_point start_time;  // Start time
    double time_limit;  // Time limit in seconds

    TimeLimiter(double _time_limit): time_limit(_time_limit) {  // Constructor
        reset_timer();
    }

    TimeLimiter(const TimeLimiter & other) {  // Copy constructor
        time_limit = other.time_limit;
        start_time = other.start_time;
    }

    inline void reset_timer() {  // Reset start time
        start_time = steady_clock::now();
    }

    inline double get_elapsed_time() const {  // Get elapsed time
        auto now = steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);
        return duration.count();
    }

    inline bool has_timed_out() const {  // Check if timed out
        double elapse=get_elapsed_time();
        return elapse >= time_limit;
    }

    inline double get_remaining_time() const {  // Get remaining time
        return time_limit - get_elapsed_time();
    }
};

// ====================================================================================================
// MyLogger Class - Logging utility
// ====================================================================================================
// Provides logging functionality with file and console output.
// Uses spdlog library for efficient logging.
// ====================================================================================================
class MyLogger {
public:
    std::string name;  // Logger name
    std::shared_ptr<spdlog::logger> logger;  // Spdlog logger instance

    // Initialize logger with file prefix and name
    void init(std::string log_fp_prefix, std::string name="g", spdlog::level::level_enum level=spdlog::level::debug);
    
    // Set logging level
    void set_level(spdlog::level::level_enum level=spdlog::level::debug);

    // Logging methods
    template<typename... Args>
    void debug(std::string msg, const Args & ... args) {
        logger->debug(msg, args...);
    }

    template<typename... Args>
    void info(std::string msg, const Args & ... args) {
        logger->info(msg, args...);
    }

    template<typename... Args>
    void warn(std::string msg, const Args & ... args) {
        logger->warn(msg, args...);
    }

    template<typename... Args>
    void error(std::string msg, const Args & ... args) {
        logger->error(msg, args...);
    }

    // Flush logs
    void flush();

    // Get timestamp string
    static std::string get_timestep();
};

// ====================================================================================================
// Global Logger Instance
// ====================================================================================================
extern MyLogger g_logger;


