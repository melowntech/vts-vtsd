/**
 * Copyright (c) 2017 Melown Technologies SE
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

#ifndef httpd_fileclass_hpp_included_
#define httpd_fileclass_hpp_included_

#include <ctime>
#include <array>

#include <boost/any.hpp>
#include <boost/program_options.hpp>

#include "utility/enum-io.hpp"

/** If adding into this enum leave unknown the last one!
 *  Make no holes, i.e. we can use values directly as indices to an array
 */
enum class FileClass { config, support, registry, data, ephemeral, unknown };

UTILITY_GENERATE_ENUM_IO(FileClass,
                         ((config))
                         ((support))
                         ((registry))
                         ((data))
                         ((ephemeral))
                         ((unknown))
                         )

class FileClassSettings {
public:
    static constexpr std::size_t storageSize =
        static_cast<int>(FileClass::unknown) + 1;
    typedef std::array<std::time_t, storageSize> Times;

    /** All file classes are disables by default. Setting value allows them
     *  automatically.
     */
    FileClassSettings() {
        // reset to defaults
        allowed_.fill(false);
        maxAges_.fill(0);
        staleWhileRevalidate_.fill(0);

        // ephemeral files are never cached
        setMaxAge(FileClass::ephemeral, -1);

        // unknown files are never cached -- for example directory listings
        setMaxAge(FileClass::unknown, -1);
    }

    void configuration(boost::program_options::options_description &od
                       , const std::string &prefix = "");

    void setMaxAge(FileClass fc, std::time_t value);
    std::time_t getMaxAge(FileClass fc) const;

    void setStaleWhileRevalidate(FileClass fc, std::time_t value);
    std::time_t getStaleWhileRevalidate(FileClass fc) const;

    void allow(FileClass fc, bool value);
    bool allowed(FileClass fc) const;

    void dump(std::ostream &os, const std::string &prefix) const;

private:
    std::array<bool, storageSize> allowed_;
    Times maxAges_;
    Times staleWhileRevalidate_;
};

// inlines

inline void FileClassSettings::setMaxAge(FileClass fc, std::time_t value)
{
    maxAges_[static_cast<int>(fc)] = value;
    allowed_[static_cast<int>(fc)] = true;
}

inline std::time_t FileClassSettings::getMaxAge(FileClass fc) const
{
    return maxAges_[static_cast<int>(fc)];
}

inline void
FileClassSettings::setStaleWhileRevalidate(FileClass fc, std::time_t value)
{
    staleWhileRevalidate_[static_cast<int>(fc)] = value;
}

inline std::time_t FileClassSettings::getStaleWhileRevalidate(FileClass fc)
    const
{
    return staleWhileRevalidate_[static_cast<int>(fc)];
}

inline void FileClassSettings::allow(FileClass fc, bool value)
{
    // do not allow unknown
    allowed_[static_cast<int>(fc)] = value;
}

inline bool FileClassSettings::allowed(FileClass fc) const
{
    return allowed_[static_cast<int>(fc)];
}

#endif // httpd_fileclass_hpp_included_
