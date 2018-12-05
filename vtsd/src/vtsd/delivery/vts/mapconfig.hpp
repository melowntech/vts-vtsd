/**
 * Copyright (c) 2018 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef vtsd_vts_mapconfig_hpp_included_
#define vtsd_vts_mapconfig_hpp_included_

#include <mutex>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "vts-libs/vts/mapconfig.hpp"
#include "vts-libs/storage/streams.hpp"

#include "../driver.hpp"

namespace mapconfig {

namespace vts = vtslibs::vts;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;

vts::MapConfig augmentMapConfig(vts::MapConfig &&mc);

template <typename T>
class LazyConfigHolder {
public:
    LazyConfigHolder() = default;

    template <typename ...Args> T& operator()(Args &&...args) const {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!data_) { data_ = boost::in_place(std::forward<Args>(args)...); }
        return *data_;
    }

private:
    mutable std::mutex mutex_;
    mutable boost::optional<T> data_;
};

struct SerializedConfig {
    std::string data;
    vs::FileStat stat;

    SerializedConfig() = default;

    SerializedConfig(std::string inData, std::time_t lastModified
                   , const char *contentType)
        : data(std::move(inData))
        , stat(data.size(), lastModified, contentType)
    {}

    void set(std::string data, std::time_t lastModified
             , const char *contentType)
    {
        this->data = std::move(data);
        this->stat = vs::FileStat
            (this->data.size(), lastModified, contentType);
    }

    void send(Sink &sink) const {
        sink.content
            (data, DriverWrapper::fileinfo(stat, FileClass::ephemeral));
    }
};

struct BaseMapConfig : SerializedConfig {
    SerializedConfig mapConfig;
    SerializedConfig dirs;

    BaseMapConfig(const vts::MapConfig &mc, std::time_t lastModified);
};

struct MapConfig : BaseMapConfig {
    template <typename Source>
    MapConfig(const Source &source)
        : BaseMapConfig(augmentMapConfig(source.mapConfig())
                        , source.lastModified())
    {}
};

class PotentiallyProxiedMapConfig {
public:
    typedef std::unique_ptr<PotentiallyProxiedMapConfig> pointer;
    virtual ~PotentiallyProxiedMapConfig() {}

    virtual void sendMapConfig(Sink &sink, const vts::OProxy &proxy) const = 0;
    virtual void sendDirs(Sink &sink, const vts::OProxy &proxy) const = 0;

    template <typename Source>
    static pointer factory(const Source &source, bool proxiesAllowed);

    class Lazy;
};

class PotentiallyProxiedMapConfig::Lazy {
public:
    Lazy() = default;

    template <typename Source>
    const PotentiallyProxiedMapConfig&
    operator()(const Source &source, bool proxiesAllowed) const
    {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!data_) {
            data_ = PotentiallyProxiedMapConfig::factory
                (source, proxiesAllowed);
        }
        return *data_;
    }

private:
    mutable std::mutex mutex_;
    mutable PotentiallyProxiedMapConfig::pointer data_;
};

class SimpleMapConfig
    : private MapConfig
    , public PotentiallyProxiedMapConfig
{
public:
    template <typename Source>
    SimpleMapConfig(const Source &source)
        : MapConfig(source)
    {}

    virtual void sendMapConfig(Sink &sink, const vts::OProxy &proxy) const;
    virtual void sendDirs(Sink &sink, const vts::OProxy &proxy) const;
};

class ProxiedMapConfig : public PotentiallyProxiedMapConfig
{
public:
    template <typename Source>
    ProxiedMapConfig(const Source &source)
        : mc_(augmentMapConfig(source.mapConfig()))
        , lastModified_(source.lastModified())
    {}

    virtual void sendMapConfig(Sink &sink, const vts::OProxy &proxy) const;
    virtual void sendDirs(Sink &sink, const vts::OProxy &proxy) const;

private:
    vts::MapConfig mc_;
    std::time_t lastModified_;
};

struct Definition {
    SerializedConfig def;
    SerializedConfig debug;

    template <typename Source>
    Definition(const Source &source) {
        init(source.meshTilesConfig(), source.lastModified());
    }

private:
    void init(const vts::MeshTilesConfig &mtc, std::time_t lastModified);
};

template <typename Source>
PotentiallyProxiedMapConfig::pointer
PotentiallyProxiedMapConfig::factory(const Source &source, bool proxiesAllowed)
{
    if (proxiesAllowed) {
        return pointer(new ProxiedMapConfig(source));
    }

    return pointer(new SimpleMapConfig(source));
}

} // namespace mapconfig

#endif // vtsd_vts_mapconfig_hpp_included_
