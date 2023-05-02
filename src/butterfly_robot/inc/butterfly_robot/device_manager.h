#pragma once

#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <assert.h>
#include <cppmisc/traces.h>
#include <cppmisc/throws.h>
#include <iostream>


class Devices
{
private:
    typedef std::unordered_map<std::string, std::shared_ptr<void>> storage;
    storage device_list;
    bool destroyed;

    Devices(Devices const&) = delete;

    Devices() : destroyed(false)
    {
        dbg_msg("Devices created");
    }

public:
    static Devices& get_instance()
    {
        static Devices instance;
        return instance;
    }

    ~Devices()
    {
        remove_all();
    }

    template <class T>
    void append(std::shared_ptr<T> device)
    {
        assert(!destroyed);

        std::string name = typeid(T).name();
        if (device_list.find(name) != device_list.end())
            throw_runtime_error("Storage device already added");

        device_list[name] = device;
    }

    template <class T>
    std::shared_ptr<T> get()
    {
        assert(!destroyed);

        std::string name = typeid(T).name();
        auto i = device_list.find(name);
        if (i == device_list.end())
            return nullptr;

        return std::static_pointer_cast<T>(i->second);
    }

    template <typename T>
    std::shared_ptr<T> capture_instance()
    {
        assert(!destroyed);

        auto dev = get<T>();
        if (dev == nullptr)
            return nullptr;

        if (dev.use_count() > 2)
            throw_runtime_error("Devices: device ", typeid(T).name(), " already captured");

        return get<T>();
    }

    void remove_all()
    {
        if (destroyed)
        {
            dbg_msg("devices: everything is already cleared");
            return;
        }

        dbg_msg("devices: begin to remove devices");
        destroyed = true;
        device_list.clear();
        dbg_msg("devices: device list cleared");
    }
};
