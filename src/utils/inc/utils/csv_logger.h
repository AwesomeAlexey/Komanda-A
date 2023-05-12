#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>
#include <cppmisc/json.h>
#include <cppmisc/traces.h>


class CSVLogger
{
public:
    CSVLogger(std::string const& filename, std::string const& format);
    ~CSVLogger();

    template <class ... Args>
    bool write(Args ... args)
    {
        if (sizeof...(args) != _narguments)
            throw_runtime_error("expect ", _narguments, "arguments, but given ", sizeof...(args));

        const int bufsz = 2 * _linesz;
        char buf[bufsz];

        int n = snprintf(buf, bufsz, _format.c_str(), args...);
        if (n <= 0)
        {
            warn_msg("can't format arguments, too long string");
            return false;
        }
        return _append_cache(buf, n);
    }

private:
    bool _append_cache(char const* s, int n);
    void _rotate_cache(int nbytes);
    void _saver_loop();

    const int _page_size = 0x400;

    std::mutex _change_cache_mutex;
    std::condition_variable _dump_cache_signal;
    std::thread _saver_thread;
    bool _stop;

    int _narguments;
    int _linesz;
    int _cache_volume;
    int _cache_filled;
    std::vector<char> _cache;
    std::string _title;
    std::string _format;
    FILE* _file;
};
