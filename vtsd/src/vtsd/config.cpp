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

#include "utility/format.hpp"

#include "./config.hpp"

namespace po = boost::program_options;

void LocationConfig::configuration(po::options_description &od
                                   , const std::string &prefix)
{
    od.add_options()
        ((prefix + "match").c_str()
         , po::value(&match)->default_value(match)
         , utility::format("Match type, one of {%s}."
                           , enumerationString(Match())).c_str())
        ((prefix + "dataset").c_str()
         , po::value(&enableDataset)->default_value(enableDataset)
         , "Enable dataset (tileset, storage, etc.) handling at "
         "this location.")
        ((prefix + "browser").c_str()
         , po::value(&enableBrowser)->default_value(enableBrowser)
         , "Enable built-in browser at this location. NB: only some drivers"
         " provide built-in browser.")
        ((prefix + "listing").c_str()
         , po::value(&enableListing)->default_value(enableListing)
         , "Enabled directory listing.")
        ((prefix + "root").c_str()
         , po::value(&root)
         , "Root directory (conflicts with alias).")
        ((prefix + "alias").c_str()
         , po::value(&alias)
         , "Alias this location to another directory (conflicts with root).")
        ((prefix + "vts.builtinBrowserUrl").c_str()
         , po::value(&vars["VTS_BUILTIN_BROWSER_URL"])
         , "URL of built in browser.")
        ((prefix + "proxyHeader").c_str()
         , po::value<std::string>()
         , "Name of proxy header.")
        ((prefix + "allowedProxy").c_str()
         , po::value<std::vector<std::string>>()
         , "Allowed proxy name. Can be used multiple times. "
         "Applicable only when proxyHeader is set.")
        ((prefix + "configClass").c_str()
         , po::value(&configClass)->default_value(configClass)->required()
         , "Config files (e.g. mapConfig.json, freelayer.json, dirs.json, ...)"
         " file class. Allowed values are \"ephemeral\" and \"config\" only.")
        ;

    fileClassSettings.configuration(od, prefix);
}

void LocationConfig::configure(const po::variables_map &vars
                               , const std::string &prefix)
{
    const auto rootName(prefix + "root");
    const auto aliasName(prefix + "alias");
    const auto rootCount(vars.count(rootName));
    const auto aliasCount(vars.count(aliasName));

    if (rootCount == aliasCount) {
        if (rootCount) {
            // both defined
            throw po::validation_error
                (po::validation_error::multiple_values_not_allowed
                 , rootName + "," + aliasName);
        }

        // none defined
        throw po::validation_error
            (po::validation_error::at_least_one_value_required
             , rootName + "," + aliasName);
    }

    if (match == Match::regex) {
        regex = boost::in_place(location);
    }

    const auto proxyHeaderName(prefix + "proxyHeader");
    if (vars.count(proxyHeaderName)) {
        proxyHeader = vars[proxyHeaderName].as<std::string>();
    }

    if (proxyHeader) {
        const auto allowedProxyName(prefix + "allowedProxy");
        if (vars.count(allowedProxyName)) {
            const auto raw
                (vars[allowedProxyName].as<std::vector<std::string>>());
            allowedProxies.insert(raw.begin(), raw.end());
        }

        // not present or empty?
        if (allowedProxies.empty()) {
            throw po::required_option(allowedProxyName);
        }

        for (const auto &proxy : allowedProxies) {
            if (proxy.empty()) {
                throw po::required_option(allowedProxyName);
            }
        }
    }

    switch (configClass) {
        case FileClass::ephemeral:
        case FileClass::config:
            break;

    default:
        throw po::validation_error
            (po::validation_error::invalid_option_value
             , prefix + "configClass"
             , boost::lexical_cast<std::string>(configClass));
    }
}

std::ostream& LocationConfig::dump(std::ostream &os, const std::string &prefix)
    const
{
    os << prefix << "dataset = " << enableDataset << "\n";
    if (enableDataset) {
        os << prefix << "browser = " << enableBrowser << "\n";
    }

    os << prefix << "listing = " << enableListing << "\n";

    if (!root.empty()) {
        os << prefix << "root = " << root << "\n";
    } else {
        os << prefix << "root = none\n";
    }

    if (!alias.empty()) {
        os << prefix << "alias = " << alias << "\n";
    } else {
        os << prefix << "alias = none\n";
    }

    if (enableDataset) {
        for (const auto &var : vars) {
            os << prefix << "variable " << var.first << " = "
               << var.second << "\n";
        }

        if (proxyHeader) {
            os << prefix << "proxyHeader = " << *proxyHeader << "\n";
            os << prefix << "allowedProxy = "
               << utility::join(allowedProxies, ", ") << "\n";
        }
    }

    os << prefix << "configClass = " << configClass << "\n";
    fileClassSettings.dump(os, prefix);

    return os;
}
