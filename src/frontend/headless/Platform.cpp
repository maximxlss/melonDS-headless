/*
    Copyright 2016-2025 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <dlfcn.h>

#include "HeadlessPlatform.h"
#include "Platform.h"
#include "SPI_Firmware.h"

namespace melonDS::Platform
{

namespace {


std::atomic<bool> g_stopRequested{false};
std::atomic<StopReason> g_stopReason{StopReason::Unknown};

const auto g_timeStart = std::chrono::steady_clock::now();

[[noreturn]] void ThrowUnimplemented(const char* feature)
{
    Log(LogLevel::Error, "Headless platform does not implement %s\n", feature);
    throw std::runtime_error(std::string("Headless platform does not implement ") + feature);
}

void WarnFileIO()
{
    Log(LogLevel::Warn, "Headless: file I/O is disabled\n");
}

} // namespace

bool Headless_StopRequested()
{
    return g_stopRequested.load(std::memory_order_relaxed);
}

StopReason Headless_StopReason()
{
    return g_stopReason.load(std::memory_order_relaxed);
}

void SignalStop(StopReason reason, void* userdata)
{
    (void)userdata;
    g_stopRequested.store(true, std::memory_order_relaxed);
    g_stopReason.store(reason, std::memory_order_relaxed);
}


std::string GetLocalFilePath(const std::string& filename)
{
    return filename;
}

FileHandle* OpenFile(const std::string& path, FileMode mode)
{
    (void)path;
    (void)mode;
    WarnFileIO();
    return nullptr;
}

FileHandle* OpenLocalFile(const std::string& path, FileMode mode)
{
    return OpenFile(GetLocalFilePath(path), mode);
}

bool CloseFile(FileHandle* file)
{
    (void)file;
    WarnFileIO();
    return false;
}

bool IsEndOfFile(FileHandle* file)
{
    (void)file;
    WarnFileIO();
    return true;
}

bool FileReadLine(char* str, int count, FileHandle* file)
{
    (void)str;
    (void)count;
    (void)file;
    WarnFileIO();
    return false;
}

u64 FilePosition(FileHandle* file)
{
    (void)file;
    WarnFileIO();
    return 0;
}

bool FileSeek(FileHandle* file, s64 offset, FileSeekOrigin origin)
{
    (void)file;
    (void)offset;
    (void)origin;
    WarnFileIO();
    return false;
}

void FileRewind(FileHandle* file)
{
    (void)file;
    WarnFileIO();
}

u64 FileRead(void* data, u64 size, u64 count, FileHandle* file)
{
    (void)data;
    (void)size;
    (void)count;
    (void)file;
    WarnFileIO();
    return 0;
}

bool FileFlush(FileHandle* file)
{
    (void)file;
    WarnFileIO();
    return false;
}

u64 FileWrite(const void* data, u64 size, u64 count, FileHandle* file)
{
    (void)data;
    (void)size;
    (void)count;
    (void)file;
    WarnFileIO();
    return 0;
}

u64 FileWriteFormatted(FileHandle* file, const char* fmt, ...)
{
    (void)file;
    (void)fmt;
    WarnFileIO();
    return 0;
}

u64 FileLength(FileHandle* file)
{
    (void)file;
    WarnFileIO();
    return 0;
}

bool FileExists(const std::string& name)
{
    (void)name;
    WarnFileIO();
    return false;
}

bool LocalFileExists(const std::string& name)
{
    (void)name;
    WarnFileIO();
    return false;
}

bool CheckFileWritable(const std::string& filepath)
{
    (void)filepath;
    WarnFileIO();
    return false;
}

bool CheckLocalFileWritable(const std::string& filepath)
{
    (void)filepath;
    WarnFileIO();
    return false;
}

void Log(LogLevel level, const char* fmt, ...)
{
    if (!fmt)
        return;

    FILE* out = (level == LogLevel::Error || level == LogLevel::Warn) ? stderr : stdout;
    va_list args;
    va_start(args, fmt);
    std::vfprintf(out, fmt, args);
    va_end(args);
}

struct Thread
{
    std::thread worker;
};

Thread* Thread_Create(std::function<void()> func)
{
    return new Thread{std::thread(std::move(func))};
}

void Thread_Free(Thread* thread)
{
    if (!thread)
        return;

    if (thread->worker.joinable())
        thread->worker.join();
    delete thread;
}

void Thread_Wait(Thread* thread)
{
    if (thread && thread->worker.joinable())
        thread->worker.join();
}

struct Semaphore
{
    std::mutex mutex;
    std::condition_variable cv;
    int count = 0;
};

Semaphore* Semaphore_Create()
{
    return new Semaphore();
}

void Semaphore_Free(Semaphore* sema)
{
    delete sema;
}

void Semaphore_Reset(Semaphore* sema)
{
    std::lock_guard<std::mutex> lock(sema->mutex);
    sema->count = 0;
}

void Semaphore_Wait(Semaphore* sema)
{
    std::unique_lock<std::mutex> lock(sema->mutex);
    sema->cv.wait(lock, [sema]() { return sema->count > 0; });
    --sema->count;
}

bool Semaphore_TryWait(Semaphore* sema, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(sema->mutex);
    if (timeout_ms == 0)
    {
        if (sema->count == 0)
            return false;
    }
    else
    {
        if (!sema->cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                [sema]() { return sema->count > 0; }))
        {
            return false;
        }
    }
    --sema->count;
    return true;
}

void Semaphore_Post(Semaphore* sema, int count)
{
    {
        std::lock_guard<std::mutex> lock(sema->mutex);
        sema->count += count;
    }
    sema->cv.notify_all();
}

struct Mutex
{
    std::mutex mutex;
};

Mutex* Mutex_Create()
{
    return new Mutex();
}

void Mutex_Free(Mutex* mutex)
{
    delete mutex;
}

void Mutex_Lock(Mutex* mutex)
{
    mutex->mutex.lock();
}

void Mutex_Unlock(Mutex* mutex)
{
    mutex->mutex.unlock();
}

bool Mutex_TryLock(Mutex* mutex)
{
    return mutex->mutex.try_lock();
}

void Sleep(u64 usecs)
{
    std::this_thread::sleep_for(std::chrono::microseconds(usecs));
}

u64 GetMSCount()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - g_timeStart).count();
}

u64 GetUSCount()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - g_timeStart).count();
}

void WriteNDSSave(const u8* savedata, u32 savelen, u32 writeoffset, u32 writelen, void* userdata)
{
    (void)savedata;
    (void)savelen;
    (void)writeoffset;
    (void)writelen;
    (void)userdata;
    WarnFileIO();
}

void WriteGBASave(const u8* savedata, u32 savelen, u32 writeoffset, u32 writelen, void* userdata)
{
    (void)savedata;
    (void)savelen;
    (void)writeoffset;
    (void)writelen;
    (void)userdata;
    WarnFileIO();
}

void WriteFirmware(const Firmware& firmware, u32 writeoffset, u32 writelen, void* userdata)
{
    (void)firmware;
    (void)writeoffset;
    (void)writelen;
    (void)userdata;
    WarnFileIO();
}

void WriteDateTime(int year, int month, int day, int hour, int minute, int second, void* userdata)
{
    (void)userdata;
    Log(LogLevel::Info, "Headless: RTC write requested %04d-%02d-%02d %02d:%02d:%02d\n",
        year, month, day, hour, minute, second);
}

void MP_Begin(void* userdata)
{
    (void)userdata;
    Log(LogLevel::Error, "Headless: local multiplayer is disabled\n");
}

void MP_End(void* userdata)
{
    (void)userdata;
}

int MP_SendPacket(u8* data, int len, u64 timestamp, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)len;
    (void)timestamp;
    return 0;
}

int MP_RecvPacket(u8* data, u64* timestamp, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)timestamp;
    return 0;
}

int MP_SendCmd(u8* data, int len, u64 timestamp, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)len;
    (void)timestamp;
    return 0;
}

int MP_SendReply(u8* data, int len, u64 timestamp, u16 aid, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)len;
    (void)timestamp;
    (void)aid;
    return 0;
}

int MP_SendAck(u8* data, int len, u64 timestamp, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)len;
    (void)timestamp;
    return 0;
}

int MP_RecvHostPacket(u8* data, u64* timestamp, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)timestamp;
    return 0;
}

u16 MP_RecvReplies(u8* data, u64 timestamp, u16 aidmask, void* userdata)
{
    (void)userdata;
    (void)data;
    (void)timestamp;
    (void)aidmask;
    return 0;
}

int Net_SendPacket(u8* data, int len, void* userdata)
{
    (void)data;
    (void)len;
    (void)userdata;
    Log(LogLevel::Error, "Headless: Wi-Fi/network is disabled; dropping packets\n");
    return 0;
}

int Net_RecvPacket(u8* data, void* userdata)
{
    (void)data;
    (void)userdata;
    Log(LogLevel::Error, "Headless: Wi-Fi/network is disabled; no packets available\n");
    return 0;
}

void Camera_Start(int num, void* userdata)
{
    (void)num;
    (void)userdata;
    ThrowUnimplemented("camera");
}

void Camera_Stop(int num, void* userdata)
{
    (void)num;
    (void)userdata;
    ThrowUnimplemented("camera");
}

void Camera_CaptureFrame(int num, u32* frame, int width, int height, bool yuv, void* userdata)
{
    (void)num;
    (void)frame;
    (void)width;
    (void)height;
    (void)yuv;
    (void)userdata;
    ThrowUnimplemented("camera");
}

void Mic_Start(void* userdata)
{
    (void)userdata;
    Log(LogLevel::Error, "Headless: microphone is disabled; returning silence\n");
}

void Mic_Stop(void* userdata)
{
    (void)userdata;
}

int Mic_ReadInput(s16* data, int maxlength, void* userdata)
{
    (void)data;
    (void)maxlength;
    (void)userdata;
    return 0;
}

struct AACDecoder {};

AACDecoder* AAC_Init()
{
    ThrowUnimplemented("AAC decoding");
}

void AAC_DeInit(AACDecoder* dec)
{
    (void)dec;
    ThrowUnimplemented("AAC decoding");
}

bool AAC_Configure(AACDecoder* dec, int frequency, int channels)
{
    (void)dec;
    (void)frequency;
    (void)channels;
    ThrowUnimplemented("AAC decoding");
}

bool AAC_DecodeFrame(AACDecoder* dec, const void* input, int inputlen, void* output, int outputlen)
{
    (void)dec;
    (void)input;
    (void)inputlen;
    (void)output;
    (void)outputlen;
    ThrowUnimplemented("AAC decoding");
}

bool Addon_KeyDown(KeyType type, void* userdata)
{
    (void)userdata;
    (void)type;
    return false;
}

void Addon_RumbleStart(u32 len, void* userdata)
{
    (void)userdata;
    Log(LogLevel::Info, "Headless: rumble requested for %u ms\n", len);
}

void Addon_RumbleStop(void* userdata)
{
    (void)userdata;
    Log(LogLevel::Info, "Headless: rumble stopped\n");
}

float Addon_MotionQuery(MotionQueryType type, void* userdata)
{
    (void)userdata;
    (void)type;
    return 0.0f;
}

struct DynamicLibrary
{
    void* handle = nullptr;
};

DynamicLibrary* DynamicLibrary_Load(const char* lib)
{
    void* handle = dlopen(lib, RTLD_NOW);
    if (!handle)
        return nullptr;

    auto* dyn = new DynamicLibrary();
    dyn->handle = handle;
    return dyn;
}

void DynamicLibrary_Unload(DynamicLibrary* lib)
{
    if (!lib)
        return;
    if (lib->handle)
        dlclose(lib->handle);
    delete lib;
}

void* DynamicLibrary_LoadFunction(DynamicLibrary* lib, const char* name)
{
    if (!lib || !lib->handle)
        return nullptr;
    return dlsym(lib->handle, name);
}

} // namespace melonDS::Platform
