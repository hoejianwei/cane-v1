// cane.cpp
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>
#include <algorithm>
#include <csignal>
#include <iomanip>

#include "RtAudio.h"
#include <libserialport.h>

#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"

// ---------------- Configuration ----------------
const unsigned int BAUD_RATE = 115200;

const std::string WAV_FILE_PATH = "/Users/jianwei/Downloads/SOPHIE_kick_wet_20.wav";
const double WAV_TRIGGER_THRESHOLD = 156.90;

const std::string CLICK_FILE_PATH = "/Users/jianwei/Downloads/Click.wav";
const double GYRO_CLICK_THRESHOLD_GX = 15.0;
const double GYRO_CLICK_THRESHOLD_REST = 30.0;

const std::string GZ_HIGH_FILE_PATH = "/Users/jianwei/Downloads/cymbal.wav";
const std::string GZ_LOW_FILE_PATH = "/Users/jianwei/Downloads/cymbal 2.wav";
const double GZ_TRIGGER_THRESHOLD = 5.0;

const double AX_COOLDOWN = 0.25;
const double GZ_COOLDOWN = 0.25;

// ---------------- WAV holder ----------------
struct WavData {
    std::vector<float> data;     // mono samples (converted if necessary)
    unsigned int sampleRate = 44100;
    unsigned int channels = 1;   // after conversion we store as mono
};

// ---------------- Voice (active playback) ----------------
struct Voice {
    const WavData* wav = nullptr;
    double pos = 0.0;        // fractional playhead in WAV sample frames
    double increment = 1.0;  // wav.sampleRate / streamSampleRate
    float volume = 1.0f;
    bool active = true;
};

// ---------------- Mixer (single stream) ----------------
class Mixer {
public:
    Mixer() : streamOpen(false), streamSampleRate(44100) {}

bool init() {
    try {
        rta = std::make_unique<RtAudio>();
    } catch (...) {
        std::cerr << "Failed to create RtAudio." << std::endl;
        return false;
    }

    if (rta->getDeviceCount() < 1) {
        std::cerr << "No audio devices found." << std::endl;
        return false;
    }

    // choose default device
    unsigned int dev = rta->getDefaultOutputDevice();
    RtAudio::DeviceInfo info = rta->getDeviceInfo(dev);

    // FORCE a safe sample rate for HDMI / CoreAudio
    unsigned int deviceSampleRate = 48000; // or 44100
    unsigned int bufferFrames = 256;       // low latency

    // setup output parameters
    RtAudio::StreamParameters outParams;
    outParams.deviceId = dev;
    outParams.nChannels = 2;
    outParams.firstChannel = 0;

    try {
        rta->openStream(&outParams, nullptr, RTAUDIO_FLOAT32,
                        deviceSampleRate, &bufferFrames, &Mixer::rtCallback, this);
        rta->startStream();
        streamSampleRate = deviceSampleRate;
        streamOpen = true;
        std::cout << "Mixer started on device '" << info.name 
                  << "' SR=" << streamSampleRate 
                  << " buffer=" << bufferFrames << std::endl;
    } catch (RtAudioErrorType &e) {
        std::cerr << "RtAudio Exception during init: " << e << std::endl;
        streamOpen = false;
        return false;
    }

    return true;
}


    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex);
        try {
            if (rta && rta->isStreamOpen()) {
                rta->stopStream();
                rta->closeStream();
            }
        } catch (...) {}
        streamOpen = false;
    }

    // Trigger a new voice (thread-safe)
    void trigger(const WavData* wav, float volume = 1.0f) {
        std::lock_guard<std::mutex> lock(mutex);
        Voice v;
        v.wav = wav;
        v.pos = 0.0;
        v.volume = volume;
        // increment = wavRate / streamRate (how many wav samples to advance per output sample)
        v.increment = (wav->sampleRate > 0) ? (static_cast<double>(wav->sampleRate) / streamSampleRate) : 1.0;
        v.active = true;
        voices.emplace_back(std::move(v));
    }

private:
    std::unique_ptr<RtAudio> rta;
    std::mutex mutex;
    std::vector<Voice> voices;
    std::atomic<bool> streamOpen;
    unsigned int streamSampleRate;

    // linear interpolation helper
    inline float linearSample(const std::vector<float>& d, double idx) {
        size_t i0 = static_cast<size_t>(floor(idx));
        size_t i1 = i0 + 1;
        if (i0 >= d.size()) return 0.0f;
        if (i1 >= d.size()) return d[i0];
        double frac = idx - static_cast<double>(i0);
        return static_cast<float>(d[i0] * (1.0 - frac) + d[i1] * frac);
    }

    // RtAudio static callback wrapper
    static int rtCallback(void *outputBuffer, void * /*inputBuffer*/, unsigned int nFrames,
                          double /*streamTime*/, RtAudioStreamStatus /*status*/, void *userData)
    {
        return static_cast<Mixer*>(userData)->audioCallback(outputBuffer, nFrames);
    }

    // real callback (mixes voices) - returns 0 to continue
    int audioCallback(void *outputBuffer, unsigned int nFrames) {
        float *out = static_cast<float*>(outputBuffer);

        // zero output buffer (stereo interleaved)
        unsigned int totalSamples = nFrames * 2;
        for (unsigned int i = 0; i < totalSamples; ++i) out[i] = 0.0f;

        std::lock_guard<std::mutex> lock(mutex);

        if (voices.empty()) return 0;

        // Mix each voice
        for (auto it = voices.begin(); it != voices.end();) {
            if (!it->active || it->wav == nullptr) {
                it = voices.erase(it);
                continue;
            }

            const std::vector<float>& data = it->wav->data;
            double pos = it->pos;
            double inc = it->increment;
            float vol = it->volume;

            bool voiceAlive = true;

            for (unsigned int frame = 0; frame < nFrames; ++frame) {
                // read sample (linear interpolation)
                float sample = linearSample(data, pos);
                // output to both L/R interleaved
                unsigned int outIndex = frame * 2;
                out[outIndex] += sample * vol;
                out[outIndex + 1] += sample * vol;

                pos += inc;
                if (pos >= static_cast<double>(data.size())) {
                    voiceAlive = false;
                    break;
                }
            }

            if (voiceAlive) {
                it->pos = pos;
                ++it;
            } else {
                // finished voice
                it = voices.erase(it);
            }
        }

        // clamp output to [-1..1]
        for (unsigned int i = 0; i < totalSamples; ++i) {
            if (out[i] > 1.0f) out[i] = 1.0f;
            else if (out[i] < -1.0f) out[i] = -1.0f;
        }

        return 0;
    }
};

// ---------------- Global objects ----------------
Mixer g_mixer;

// ---------------- SensorAudioProcessor (triggers mixer) ----------------
class SensorAudioProcessor {
public:
    using HighResClock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<HighResClock>;

    SensorAudioProcessor() :
        m_global_wav_volume(1.0),
        m_last_gx(0.0),
        m_current_click_interval(0.5)
    {
        auto now = HighResClock::now();
        m_ax_last_play_time = now;
        m_last_click_time = now;
        m_gz_high_last_play_time = now;
        m_gz_low_last_play_time = now;

        load_wav_file(WAV_FILE_PATH, m_wav_data);
        load_wav_file(CLICK_FILE_PATH, m_click_data);
        load_wav_file(GZ_HIGH_FILE_PATH, m_gz_high_data);
        load_wav_file(GZ_LOW_FILE_PATH, m_gz_low_data);
    }

    void load_wav_file(const std::string& path, WavData& wavData) {
        unsigned int channels;
        unsigned int sampleRate;
        drwav_uint64 totalFrames;
        float* pSampleData = drwav_open_file_and_read_pcm_frames_f32(
            path.c_str(), &channels, &sampleRate, &totalFrames, NULL
        );

        if (!pSampleData) {
            std::cerr << "Error loading WAV file: " << path << std::endl;
            return;
        }

        wavData.sampleRate = sampleRate;
        wavData.channels = channels;
        // convert to mono by averaging channels if necessary
        wavData.data.resize(totalFrames);
        for (drwav_uint64 i = 0; i < totalFrames; ++i) {
            float acc = 0.0f;
            for (unsigned int c = 0; c < channels; ++c) {
                acc += pSampleData[i * channels + c];
            }
            wavData.data[i] = acc / static_cast<float>(channels);
        }

        drwav_free(pSampleData, NULL);
        std::cout << "Loaded '" << path << "' - frames: " << totalFrames
                  << ", SR: " << sampleRate << ", ch: " << channels << std::endl;
    }

    void process_new_data(const std::vector<double>& values) {
        if (values.size() != 9) return;
        double ax = values[0], ay = values[1], az = values[2];
        double gx = values[3], gy = values[4], gz = values[5];

        auto now = HighResClock::now();

        // AX trigger
        if (!m_wav_data.data.empty()) {
            std::chrono::duration<double> d = now - m_ax_last_play_time;
            if ((std::abs(ax) > WAV_TRIGGER_THRESHOLD) && (d.count() > AX_COOLDOWN)) {
                m_ax_last_play_time = now;
                std::cout << "AX WAV Triggered: ax = " << std::fixed << std::setprecision(2) << ax << std::endl;
                g_mixer.trigger(&m_wav_data, static_cast<float>(m_global_wav_volume));
            }
        }

        // Click trigger (gyro)
        double delta_gx = std::abs(gx - m_last_gx);
        m_last_gx = gx;
        bool condition_met = (std::abs(gx) > GYRO_CLICK_THRESHOLD_GX) &&
                             ((std::abs(gy) + std::abs(gz)) < GYRO_CLICK_THRESHOLD_REST);
        if (condition_met) {
            if (delta_gx > 0.1) {
                double interval_s = std::clamp(200.0 / (delta_gx * 1000.0), 0.04, 0.5);
                m_current_click_interval = interval_s;
            }
            std::chrono::duration<double> click_d = now - m_last_click_time;
            if (click_d.count() > m_current_click_interval) {
                if (!m_click_data.data.empty()) {
                    g_mixer.trigger(&m_click_data, 1.0f);
                }
                m_last_click_time = now;
            }
        }

        // GZ triggers
        std::chrono::duration<double> gz_high_d = now - m_gz_high_last_play_time;
        if (!m_gz_high_data.data.empty()) {
            if ((gz > GZ_TRIGGER_THRESHOLD) && (gz_high_d.count() > GZ_COOLDOWN)) {
                m_gz_high_last_play_time = now;
                std::cout << "GZ HIGH Triggered: gz = " << std::fixed << std::setprecision(2) << gz << std::endl;
                g_mixer.trigger(&m_gz_high_data, 0.9f);
            }
        }
        std::chrono::duration<double> gz_low_d = now - m_gz_low_last_play_time;
        if (!m_gz_low_data.data.empty()) {
            if ((gz < -GZ_TRIGGER_THRESHOLD) && (gz_low_d.count() > GZ_COOLDOWN)) {
                m_gz_low_last_play_time = now;
                std::cout << "GZ LOW Triggered: gz = " << std::fixed << std::setprecision(2) << gz << std::endl;
                g_mixer.trigger(&m_gz_low_data, 0.9f);
            }
        }
    }

    void stop() {
        // Nothing to join here; mixer shutdown handled globally
    }

private:
    double m_global_wav_volume;
    WavData m_wav_data, m_click_data, m_gz_high_data, m_gz_low_data;
    TimePoint m_ax_last_play_time, m_last_click_time, m_gz_high_last_play_time, m_gz_low_last_play_time;
    double m_last_gx, m_current_click_interval;
};

// ---------------- Serial helper (unchanged logic) ----------------
std::string find_arduino_port() {
    struct sp_port **port_list;
    if (sp_list_ports(&port_list) != SP_OK) {
        std::cerr << "Error listing serial ports." << std::endl;
        return "";
    }
    std::string found;
    for (int i = 0; port_list[i] != NULL; ++i) {
        struct sp_port *p = port_list[i];
        const char *name = sp_get_port_name(p);
        const char *desc = sp_get_port_description(p);
        std::string sn = name ? name : "";
        std::string sd = desc ? desc : "";
        if (sd.find("Arduino") != std::string::npos ||
            sd.find("USB-SERIAL") != std::string::npos ||
            sn.find("usbmodem") != std::string::npos) {
            std::cout << "Found Arduino at: " << sn << std::endl;
            found = sn;
            break;
        }
    }
    sp_free_port_list(port_list);
    if (found.empty()) std::cerr << "Could not find Arduino." << std::endl;
    return found;
}

void serial_reader_thread(SensorAudioProcessor& processor, std::atomic<bool>& stop_event) {
    std::string port = find_arduino_port();
    if (port.empty()) return;

    struct sp_port *serial_port;
    if (sp_get_port_by_name(port.c_str(), &serial_port) != SP_OK) {
        std::cerr << "Cannot get port by name." << std::endl;
        return;
    }
    if (sp_open(serial_port, SP_MODE_READ) != SP_OK) {
        std::cerr << "Cannot open serial port." << std::endl;
        sp_free_port(serial_port);
        return;
    }

    sp_set_baudrate(serial_port, BAUD_RATE);
    sp_set_bits(serial_port, 8);
    sp_set_parity(serial_port, SP_PARITY_NONE);
    sp_set_stopbits(serial_port, 1);
    sp_set_flowcontrol(serial_port, SP_FLOWCONTROL_NONE);

    std::cout << "Connected to " << port << " at " << BAUD_RATE << " baud." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    sp_flush(serial_port, SP_BUF_INPUT);

    std::string line;
    char buf[1];

    while (!stop_event.load()) {
        int r = sp_blocking_read(serial_port, buf, 1, 10);
        if (r > 0) {
            char c = buf[0];
            if (c == '\n' || c == '\r') {
                if (!line.empty()) {
                    try {
                        std::stringstream ss(line);
                        std::string part;
                        std::vector<double> values;
                        while (std::getline(ss, part, ',')) {
                            values.push_back(std::stod(part));
                        }
                        if (values.size() == 9) processor.process_new_data(values);
                    } catch (...) {}
                    line.clear();
                }
            } else {
                line.push_back(c);
            }
        }
    }

    sp_close(serial_port);
    sp_free_port(serial_port);
    std::cout << "Serial closed." << std::endl;
}

// ---------------- Globals ----------------
std::atomic<bool> g_stop_flag(false);
void sigint_handler(int) {
    std::cout << "\nShutting down..." << std::endl;
    g_stop_flag.store(true);
}

// ---------------- Main ----------------
int main() {
    signal(SIGINT, sigint_handler);

    // Initialize mixer (single stream)
    if (!g_mixer.init()) {
        std::cerr << "Mixer init failed. Exiting." << std::endl;
        return 1;
    }

    // Create app objects
    SensorAudioProcessor processor;

    // Start serial reader
    std::thread reader(serial_reader_thread, std::ref(processor), std::ref(g_stop_flag));

    std::cout << "Application started. Press Ctrl+C to exit." << std::endl;

    while (!g_stop_flag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup
    if (reader.joinable()) reader.join();
    g_mixer.shutdown();

    std::cout << "Goodbye." << std::endl;
    return 0;
}
