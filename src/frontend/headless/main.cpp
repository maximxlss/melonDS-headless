#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "Args.h"
#include "NDS.h"
#include "NDSCart.h"
#include "HeadlessOptions.h"
#include "HeadlessPlatform.h"
#include "Screenshot.h"

namespace fs = std::filesystem;

namespace {

constexpr int kExitOk = 0;
constexpr int kExitRomLoadFailed = 2;
constexpr int kExitRomParseFailed = 3;
constexpr int kExitGbaModeNotSupported = 12;
constexpr int kExitBadExceptionRegion = 13;
constexpr int kExitScreenshotFailed = 20;

bool LoadFile(const fs::path& path, std::unique_ptr<melonDS::u8[]>& outData, std::uint32_t& outLen)
{
    outLen = 0;
    outData.reset();

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open())
        return false;

    std::ifstream::pos_type length = file.tellg();
    if (length <= 0)
        return false;

    if (static_cast<std::uint64_t>(length) > 0xFFFFFFFFu)
        return false;

    outLen = static_cast<std::uint32_t>(length);
    outData = std::make_unique<melonDS::u8[]>(outLen);

    file.seekg(0, std::ios::beg);
    if (!file.read(reinterpret_cast<char*>(outData.get()), outLen))
    {
        outData.reset();
        outLen = 0;
        return false;
    }

    return true;
}

int ExitCodeForStopReason(melonDS::Platform::StopReason reason)
{
    switch (reason)
    {
        case melonDS::Platform::StopReason::GBAModeNotSupported:
            return kExitGbaModeNotSupported;
        case melonDS::Platform::StopReason::BadExceptionRegion:
            return kExitBadExceptionRegion;
        default:
            return kExitOk;
    }
}

} // namespace

int main(int argc, char** argv)
{
    Options opts = ParseArgs(argc, argv);
    if (opts.showHelp)
    {
        PrintUsage(argv[0]);
        return 0;
    }
    if (opts.romPath.empty())
    {
        PrintUsage(argv[0]);
        return 1;
    }

    std::unique_ptr<melonDS::u8[]> romData;
    std::uint32_t romLen = 0;
    if (!LoadFile(opts.romPath, romData, romLen))
    {
        std::fprintf(stderr, "Failed to load ROM: %s\n", opts.romPath.c_str());
        return kExitRomLoadFailed;
    }

    melonDS::NDSCart::NDSCartArgs cartArgs{
        .SDCard = std::nullopt,
        .SRAM = nullptr,
        .SRAMLength = 0,
    };

    melonDS::NDSArgs ndsArgs;
    if (!opts.enableJit)
        ndsArgs.JIT = std::nullopt;
    auto nds = std::make_unique<melonDS::NDS>(std::move(ndsArgs), nullptr);

    auto cart = melonDS::NDSCart::ParseROM(std::move(romData), romLen, nullptr, std::move(cartArgs));
    if (!cart)
    {
        std::fprintf(stderr, "Failed to parse ROM: %s\n", opts.romPath.c_str());
        return kExitRomParseFailed;
    }

    nds->SetNDSCart(std::move(cart));
    nds->Reset();

    fs::path romPath(opts.romPath);
    if (nds->NeedsDirectBoot())
    {
        nds->SetupDirectBoot(romPath.filename().string());
    }

    nds->Start();

    const bool frameLimitEnabled = opts.maxFrames != 0;
    std::uint64_t framesRun = 0;
    bool frameLimitReached = false;
    while (true)
    {
        nds->RunFrame();
        if (frameLimitEnabled)
        {
            ++framesRun;
            if (framesRun >= opts.maxFrames)
            {
                frameLimitReached = true;
                break;
            }
        }
        if (melonDS::Platform::Headless_StopRequested())
            break;
    }
    bool screenshotFailed = false;
    bool screenshotSaved = false;
    if (!opts.screenshotPath.empty())
    {
        const int front = nds->GPU.FrontBuffer;
        const auto* top = nds->GPU.Framebuffer[front][0].get();
        const auto* bottom = nds->GPU.Framebuffer[front][1].get();
        if (!top || !bottom)
        {
            std::fprintf(stderr, "Screenshot failed: framebuffer not ready\n");
            screenshotFailed = true;
        }
        else if (!WritePNG(opts.screenshotPath, top, bottom, 256, 192))
        {
            std::fprintf(stderr, "Screenshot failed: could not write %s\n", opts.screenshotPath.c_str());
            screenshotFailed = true;
        }
        else
        {
            screenshotSaved = true;
        }
    }

    if (frameLimitReached)
        nds->Stop(melonDS::Platform::StopReason::External);

    if (screenshotSaved)
        std::fprintf(stderr, "Saved screenshot: %s\n", opts.screenshotPath.c_str());
    else if (opts.screenshotPath.empty())
        std::fprintf(stderr, "No screenshot path provided; skipping capture\n");

    int exitCode = ExitCodeForStopReason(melonDS::Platform::Headless_StopReason());
    if (exitCode == kExitOk && screenshotFailed)
        exitCode = kExitScreenshotFailed;
    return exitCode;
}
