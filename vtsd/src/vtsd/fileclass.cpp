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

#include "./fileclass.hpp"

namespace po = boost::program_options;

void FileClassSettings
::configuration(po::options_description &od, const std::string &prefix)
{
    auto ao(od.add_options());

    const auto registerTimes([&](Times &times, const std::string &prefix
                                 , const std::string &what
                                 , const std::string &help)
    {
        for (auto fc : enumerationValues(FileClass())) {
            switch (fc) {
            case FileClass::unknown:
            case FileClass::ephemeral:
                continue;
            default:
                if (!allowed(fc)) { continue; }
            }

            auto name(boost::lexical_cast<std::string>(fc));
            ao((prefix + name).c_str()
               , po::value(&times[static_cast<int>(fc)])
               ->required()->default_value(times[static_cast<int>(fc)])
               , (what + " of file class <" + name + ">; " + help).c_str());
        }
    });

    registerTimes(maxAges_, prefix + "max-age."
                  , "Max age"
                  , ">=0: Cache-Control: max-age=???, "
                  "<0: Cache-Control=no-cache.");

    registerTimes(staleWhileRevalidate_, prefix + "stale-while-revalidate."
                  , "Stale-while-revalidate value "
                  , "if >0: adds stale-while-revalidate=??? after max-age.");
}

void FileClassSettings::dump(std::ostream &os, const std::string &prefix)
    const
{
    auto dump([&](const Times &times, const std::string &prefix)
    {
        for (auto fc : enumerationValues(FileClass())) {
            if (!allowed(fc)) { continue; }
            os << prefix << fc << " = " << times[static_cast<int>(fc)] << "\n";
        }
    });

    dump(maxAges_, prefix + "max-age.");
    dump(staleWhileRevalidate_, prefix + "stale-while-revalidate.");
}
