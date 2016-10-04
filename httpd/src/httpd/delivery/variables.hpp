#ifndef libvadstena_http_variables_hpp_included_
#define libvadstena_http_variables_hpp_included_

#include <utility>
#include <map>
#include <string>

class Variables {
public:
    typedef std::map<std::string, std::string> Map;

    Variables() = default;

    Variables(Map variables)
        : variables_(std::move(variables)) {}

    /** Add value to variables. Nothing happens if value already exists.
     */
    void add(const std::string &key, const std::string &value)
    {
        return add(key, value, false);
    }

    /** Replaces value in variables. Adds or rewrites existing field.
     */
    void replace(const std::string &key, const std::string &value)
    {
        return add(key, value, true);
    }

    /** Tries to find variable by its name. Returns null pointer if not found.
     */
    const std::string* find(const std::string &key) const {
        auto fvariables(variables_.find(key));
        if (fvariables == variables_.end()) { return nullptr; }
        return &fvariables->second;
    }

    void update(const Variables &src, bool overwrite);

    /** Get variables
     */
    const Map& variables() const { return variables_; }

    struct Wrapper {
        const Variables &vars;
        const Variables &defaults;
    };

private:
    void add(const std::string &key, const std::string &value
             , bool overwrite)
    {
        auto res(variables_.insert(Map::value_type(key, value)));
        if (overwrite && !res.second) { res.first->second = value; }
    }

    Map variables_;
};


inline void Variables::update(const Variables &src, bool overwrite)
{
    for (const auto &item : src.variables_) {
        add(item.first, item.second, overwrite);
    }
}

#endif // libvadstena_http_variables_hpp_included_
