#include "util/Utils.h"

// ====================================================================================================
// Utility Classes Implementation
// ====================================================================================================
// This file contains implementations for Timer, TimeLimiter, and MyLogger classes.
// These provide timing, timeout, and logging utilities for the MAPF planner.
// ====================================================================================================

Timer g_timer;  // Global timer instance
MyLogger g_logger;  // Global logger instance

// ====================================================================================================
// Timer Class Implementation
// ====================================================================================================

// ====================================================================================================
// Timer::record_p - Record a time point
// ====================================================================================================
steady_clock::time_point Timer::record_timepoint(const string & pkey)
{
    auto t=steady_clock::now();  // Get current time
    time_points[pkey]=t;  // Store time point
    return t;
}

// ====================================================================================================
// Timer::get_p - Get a recorded time point
// ====================================================================================================
steady_clock::time_point Timer::get_timepoint(const string & pkey) const
{
    const auto & iter=time_points.find(pkey);  // Find time point
    if (iter!=time_points.end())
    {
        return iter->second;  // Return time point
    } else 
    {
        cerr<<"key "<<pkey<<" not found in time_points"<<endl;  // Error if not found
        exit(-1);
    }
}

// ====================================================================================================
// Timer::record_d - Record duration between two points
// ====================================================================================================
double Timer::record_duration(const string & old_pkey, const string & new_pkey, const string & dkey)
{
    auto new_p=record_timepoint(new_pkey);  // Record new point
    auto old_p=get_timepoint(old_pkey);  // Get old point
    auto d=duration<double>(new_p-old_p).count();  // Calculate duration

    const auto & iter=time_durations.find(dkey);  // Find duration record
    if (iter!=time_durations.end())
    {
        time_durations[dkey]+=d;  // Accumulate duration
        time_duration_counters[dkey]++;  // Increment counter
        time_durations_last[dkey]=d;  // Update last duration
    }
    else
    {
        time_durations[dkey]=d;  // Initialize duration
        time_duration_counters[dkey]=1;  // Initialize counter
        time_durations_last[dkey]=d;  // Set last duration
    }
    return d;
}

// ====================================================================================================
// Timer::record_d - Record duration with auto-generated key
// ====================================================================================================
double Timer::record_duration(const string & old_pkey, const string & new_pkey)
{
    return record_duration(old_pkey,new_pkey,new_pkey);  // Use new key as duration key
}

// ====================================================================================================
// Timer::get_d - Get recorded duration
// ====================================================================================================
double Timer::get_duration(const string & dkey, int mode) const
{
    const auto & iter=time_durations.find(dkey);  // Find duration
    double d=0;
    if (iter!=time_durations.end())
    {
        d=iter->second;  // Get accumulated duration
    }
    else
    {
        cerr<<"key "<<dkey<<" not found in time_durations"<<endl;  // Error if not found
        exit(-1);
    }   
    double ret=d;  // Default return sum
    if (mode==1)  // Mean
    {
        ret=ret/double(time_duration_counters.at(dkey));
    }
    if (mode==2)  // Last
    {
        return time_durations_last.at(dkey);
    }
    return ret;
}

// ====================================================================================================
// Timer::print_d - Print a duration
// ====================================================================================================
void Timer::print_duration(const string & dkey) const
{
    double sum_d=get_duration(dkey,0);  // Sum
    double mean_d=get_duration(dkey,1);  // Mean
    double last_d=get_duration(dkey,2);  // Last
}

// ====================================================================================================
// MyLogger Class Implementation
// ====================================================================================================

// ====================================================================================================
// MyLogger::init - Initialize logger
// ====================================================================================================
void MyLogger::init(std::string log_fp_prefix, std::string name, spdlog::level::level_enum level) {
    this->name=name;  // Set name

    std::string timestamp=get_timestep();  // Get timestamp
    std::string log_fp=log_fp_prefix+"_"+timestamp+".log";  // Log file path

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();  // Console sink
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_fp, true);  // File sink

    logger=std::make_shared<spdlog::logger>(name, spdlog::sinks_init_list({console_sink, file_sink}));  // Create logger
    logger->set_level(level);  // Set level
}

// ====================================================================================================
// MyLogger::get_timestep - Get timestamp string
// ====================================================================================================
std::string MyLogger::get_timestep(){
    auto now=time(0);  // Get current time
    auto _time = localtime(&now);  // Convert to local time
    char buffer[50];  // Buffer for formatted time
    strftime(buffer,50,"%y-%m-%d %H:%M:%S",_time);  // Format time
    return std::string(buffer);  // Return string
}

// ====================================================================================================
// MyLogger::set_level - Set logging level
// ====================================================================================================
void MyLogger::set_level(spdlog::level::level_enum level){
    logger->set_level(level);  // Set level
}

// ====================================================================================================
// MyLogger::flush - Flush logs
// ====================================================================================================
void MyLogger::flush() {
    logger->flush();  // Flush
}
