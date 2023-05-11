#include <utils/csv_logger.h>
#include <string_view>
#include <cstring>


struct Entry
{
    std::string name;
    std::string format;
};

int count_symbols(std::string_view const& format, char c)
{
    int i = 0;
    for (size_t pos = format.find_first_of(c); 
        pos != std::string_view::npos; 
        pos = format.find_first_of(c, pos + 1))
    {
        ++ i;
    }
    return i;
}

std::string_view strip(std::string_view const& s)
{
    size_t i1 = s.find_first_not_of(" \t\n\r");
    if (i1 == std::string_view::npos)
        return std::string_view();
    size_t i2 = s.find_last_not_of(" \t\n\r");
    if (i2 == std::string_view::npos)
        return std::string_view();
    return s.substr(i1, i2 - i1 + 1);
}

Entry fetch_entry(std::string_view const& s)
{
    size_t p = s.find_first_of('=');
    if (p == std::string_view::npos)
        throw_runtime_error("incorrect name=format value in ", s);
    auto name = strip(s.substr(0, p));
    if (name.size() == 0)
        throw_runtime_error("invalid name in ", s);
    auto format = strip(s.substr(p + 1));
    if (format.size() == 0)
        throw_runtime_error("invalid format in ", s);
    return { std::string(name), std::string(format) };
}

std::vector<Entry> fetch_entries(std::string_view const& format)
{
    std::vector<Entry> entries;
    entries.reserve(count_symbols(format, 'c') + 1);
    size_t pos = 0;

    while (true)
    {
        size_t nextpos = format.find_first_of(',', pos);
        if (nextpos == std::string_view::npos)
        {
            entries.emplace_back(fetch_entry(format.substr(pos)));
            break;
        }
        entries.emplace_back(fetch_entry(format.substr(pos, nextpos - pos)));
        pos = nextpos + 1;
    }
    return entries;
}

std::string get_format(std::vector<Entry> const& entries)
{
    const int n = entries.size();
    std::string format;
    format.reserve(n * 6);

    for (int i = 0; i < n; ++ i)
    {
        format.append(entries[i].format);
        if (i == n - 1)
            format.append("\n");
        else
            format.append(",");
    }
    return format;
}

std::string get_title(std::vector<Entry> const& entries)
{
    const int n = entries.size();
    int sz = 1;
    for (int i = 0; i < n; ++ i)
        sz += entries[i].name.size() + 1;

    std::string title;
    title.reserve(sz);

    for (int i = 0; i < n; ++ i)
    {
        title.append(entries[i].name);
        if (i == n - 1)
            title.append("\n");
        else
            title.append(",");
    }
    return title;
}

inline int round_up(int a, int b)
{
    return ((std::max(a + b - 1, 0)) / b) * b;
}

inline int round_down(int a, int b)
{
    return (a / b) * b;
}

CSVLogger::CSVLogger(std::string const& filename, std::string const& format)
{
    auto entries = fetch_entries(format);
    _narguments = entries.size();
    _format = get_format(entries);
    _title = get_title(entries);
    _linesz = _narguments * 16;
    _cache_volume = 1024 * _linesz;
    _cache_volume = std::max(_cache_volume, 4 * _page_size);
    _cache_volume = round_up(_cache_volume, _page_size);
    _cache.resize(_cache_volume);
    _cache_filled = 0;
    _append_cache(_title.c_str(), _title.size());

    _file = fopen(filename.c_str(), "w");
    if (!_file)
        throw_runtime_error("can't open file ", filename, " for writing");

    _saver_thread = std::thread(&CSVLogger::_saver_loop, this);
}

CSVLogger::~CSVLogger()
{
    _stop = true;
    _dump_cache_signal.notify_all();
    _saver_thread.join();
    if (_file)
    {
        fclose(_file);
        _file = nullptr;
    }
}

bool CSVLogger::_append_cache(char const* s, int n)
{
    {
        std::unique_lock<std::mutex> lock(_change_cache_mutex);
        if (_cache_filled + n > _cache_volume)
        {
            _dump_cache_signal.notify_all();
            warn_msg("cache is overflew, skipping");
            return false;
        }
        std::memcpy(&_cache[_cache_filled], s, n);
        _cache_filled += n;
    }

    if (_cache_filled >= _cache_volume * 3 / 4)
        _dump_cache_signal.notify_all();
    
    return true;
}

void CSVLogger::_rotate_cache(int nbytes)
{
    std::unique_lock<std::mutex> lock(_change_cache_mutex);
    const int newsz = _cache_filled - nbytes;
    if (newsz <= 0)
    {
        _cache.resize(0);
        return;
    }
    std::memmove(&_cache[0], &_cache[nbytes], newsz);
    _cache_filled = newsz;
}

void CSVLogger::_saver_loop()
{
    while (true)
    {
        {
            std::unique_lock<std::mutex> lock(_tasks_mutex);
            while (_cache_filled < _cache_volume * 3 / 4 && !_stop)
                _dump_cache_signal.wait(lock);
        }

        if (_stop)
            break;

        int sz = round_down(_cache_filled, _page_size);
        fwrite(_cache.data(), sz, 1, _file);
        _rotate_cache(sz);
    }

    fwrite(_cache.data(), _cache_filled, 1, _file);
}
