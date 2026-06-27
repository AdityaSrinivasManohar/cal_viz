#pragma once

#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

// Minimal flag parser for "--key value" and "--flag" (boolean) style args.
// argv[0] is expected to be the subcommand name and is ignored.
struct Args {
    std::unordered_map<std::string, std::string> flags;

    static Args parse(int argc, char** argv) {
        Args a;
        for (int i = 1; i < argc; ++i) {
            std::string_view s = argv[i];
            if (!s.starts_with("--"))
                throw std::runtime_error(std::string("unexpected positional argument: ") + argv[i]);
            std::string key(s.substr(2));
            // Next token is the value unless it starts with "--" or doesn't exist.
            if (i + 1 < argc && !std::string_view(argv[i + 1]).starts_with("--")) {
                a.flags[key] = argv[++i];
            } else {
                a.flags[key] = "";  // boolean flag
            }
        }
        return a;
    }

    bool has(const std::string& key) const { return flags.count(key) > 0; }

    std::string get(const std::string& key, const std::string& def = "") const {
        auto it = flags.find(key);
        return it == flags.end() ? def : it->second;
    }

    std::string require(const std::string& key) const {
        auto it = flags.find(key);
        if (it == flags.end())
            throw std::runtime_error("missing required flag --" + key);
        return it->second;
    }

    int get_int(const std::string& key, int def) const {
        auto it = flags.find(key);
        return it == flags.end() ? def : std::stoi(it->second);
    }
};
