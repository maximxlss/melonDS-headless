#ifndef MELONDS_HEADLESS_OPTIONS_H
#define MELONDS_HEADLESS_OPTIONS_H

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>

struct Options
{
    std::string romPath;
    std::string screenshotPath;
    std::uint64_t maxFrames = 60;
    bool enableJit = false;
    bool showHelp = false;
};

namespace {

inline bool ParseU64(const char* value, std::uint64_t& out)
{
    if (!value || !*value)
        return false;

    errno = 0;
    char* end = nullptr;
    unsigned long long parsed = std::strtoull(value, &end, 10);
    if (errno == ERANGE)
        return false;
    if (end && *end != '\0')
        return false;
    out = static_cast<std::uint64_t>(parsed);
    return true;
}

} // namespace

inline void PrintUsage(const char* exe)
{
    std::printf(
        "Usage: %s [options] <rom.nds> [screenshot.png]\n"
        "\n"
        "Options:\n"
        "  --frames <n>       Run for n frames before taking screenshot/exit (default: 60, 0 = unlimited)\n"
        "  --jit              Enable the JIT recompiler (if available)\n"
        "  --help             Show this help\n",
        exe);
}

inline Options ParseArgs(int argc, char** argv)
{
    Options opts;
    for (int i = 1; i < argc; ++i)
    {
        const char* arg = argv[i];
        if (!std::strcmp(arg, "--help") || !std::strcmp(arg, "-h"))
        {
            opts.showHelp = true;
            return opts;
        }
        auto requireValue = [&](const char* opt) -> const char*
        {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "Missing value for %s\n", opt);
                opts.showHelp = true;
                return nullptr;
            }
            return argv[++i];
        };

        if (!std::strcmp(arg, "--frames"))
        {
            const char* value = requireValue("--frames");
            if (!value)
                return opts;
            std::uint64_t frames = 0;
            if (!ParseU64(value, frames))
            {
                std::fprintf(stderr, "Invalid --frames value: %s\n", value);
                opts.showHelp = true;
                return opts;
            }
            opts.maxFrames = frames;
            continue;
        }
        if (!std::strcmp(arg, "--jit"))
        {
            opts.enableJit = true;
            continue;
        }
        if (arg[0] == '-')
        {
            std::fprintf(stderr, "Unknown option: %s\n", arg);
            opts.showHelp = true;
            return opts;
        }
        if (opts.romPath.empty())
        {
            opts.romPath = arg;
            continue;
        }
        if (opts.screenshotPath.empty())
        {
            opts.screenshotPath = arg;
            continue;
        }
        std::fprintf(stderr, "Unexpected argument: %s\n", arg);
        opts.showHelp = true;
        return opts;
    }
    return opts;
}

#endif
