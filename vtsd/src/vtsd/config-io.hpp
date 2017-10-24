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

#ifndef vtsd_config_io_hpp_included_
#define vtsd_config_io_hpp_included_

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "./config.hpp"

namespace detail {

extern std::vector<std::pair<std::string, DatasetProvider::value_type>>
    boolMapping;

namespace constants {
const std::string vts("vts");
const std::string vtsLegacy("vts-legacy");
const std::string slpk("slpk");
} // constants

typedef std::vector<boost::iterator_range<std::string::const_iterator>> Tokens;
} // namespace detail

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, DatasetProvider &dp)
{
    using boost::algorithm::split;
    using boost::algorithm::equals;
    using boost::algorithm::iequals;
    using boost::algorithm::is_any_of;
    using boost::algorithm::token_compress_on;

    std::string s;
    if (!(is >> s)) { return is; }

    detail::Tokens tokens;

    split(tokens, s, is_any_of(","), token_compress_on);

    dp.value = DatasetProvider::none;

    if (tokens.empty()) {
        is.setstate(std::ios::failbit);
        return is;
    }

    if (tokens.size() == 1) {
        for (const auto &bm : detail::boolMapping) {
            if (iequals(bm.first, tokens.front())) {
                dp.value = bm.second;
                return is;
            }
        }
    }

    for (const auto &atom : tokens) {
        if (equals(detail::constants::vts, atom)) {
            dp.value |= DatasetProvider::vts;
        } else if (equals(detail::constants::vtsLegacy, atom)) {
            dp.value |= DatasetProvider::vtsLegacy;
        } else if (equals(detail::constants::slpk, atom)) {
            dp.value |= DatasetProvider::slpk;
        } else {
            is.setstate(std::ios::failbit);
            return is;
        }
    }

    return is;
}

struct Separator {
    std::string value;
    mutable bool marker;

    Separator(const std::string &value = ",")
        : value(value), marker(false)
    {}
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Separator &s)
{
    if (s.marker) { os << s.value; }
    s.marker = true;
    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const DatasetProvider &dp)
{
    if (!dp) { return os << "none"; }

    Separator s;
    if (dp.value & DatasetProvider::vts) {
        os << s << detail::constants::vts;
    }

    if (dp.value & DatasetProvider::vtsLegacy) {
        os << s << detail::constants::vtsLegacy;
    }

    if (dp.value & DatasetProvider::slpk) {
        os << s << detail::constants::slpk;
    }
    return os;
}

#endif // vtsd_config_io_hpp_included_
