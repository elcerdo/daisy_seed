
// clang-format off
#include "begin_ignore_warnings.h"

#include "pvoc/phase_vocoder.h"
#include "pvoc/parameters.h"

#include "daisy_pod.h"
#include "daisysp.h"

#include "end_ignore_warnings.h"
// clang-format on

#include "hid/disp/oled_display.h"
#include "dev/oled_ssd130x.h"
#include "util/oled_fonts.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <list>
#include <cmath>

constexpr size_t audio_block_size = 32;

// #define WITH_MIDI_USB
#define WITH_DISPLAY

daisy::DaisyPod pod;

struct OscData
{
    daisysp::Oscillator osc;
    bool                gate;
    float               top_on;
    float               top_off;
    // daisysp::Adsr       env;
    // daisysp::AdEnv      env;
};

using NoteToOscDatas = std::map<uint8_t, OscData>;

NoteToOscDatas note_to_osc_datas = {};

using FloatFrameArray = std::array<FloatFrame, audio_block_size>;

FloatFrameArray DSY_SDRAM_BSS buffer_fft;

PhaseVocoder vocoder;

daisy::CpuLoadMeter meter;

int32_t count_midi_clocks = 0;
bool    midi_transport    = false;
uint8_t master_waveform   = 0;

float master_volume  = .5f;
float master_attack  = .5f;
float master_decay   = .5f;
float master_sustain = .5f;
float master_release = .5f;

void audio_callback(daisy::AudioHandle::InterleavingInputBuffer  in,
                    daisy::AudioHandle::InterleavingOutputBuffer out,
                    size_t                                       size)
{
    meter.OnBlockStart();

    assert(buffer_fft.size() * 2 == size);
    for(size_t ii = 0; ii < size; ii += 2)
    {
        buffer_fft[ii / 2].r = in[ii + 0];
        buffer_fft[ii / 2].l = in[ii + 1];
    }

    // Parameters params;
    // vocoder.Process(params,
    //                 reinterpret_cast<const FloatFrame*>(in),
    //                 buffer_fft.data(),
    //                 buffer_fft.size());

    float top_now = pod.seed.system.GetNow() * 1e-3f;

    const auto dt = 1 / pod.AudioSampleRate();

    // pod.seed.PrintLine("!!!! " FLT_FMT3, FLT_VAR3(dt), FLT_VAR3(top_now));


    for(size_t ii = 0; ii < size; ii += 2)
    {
        float ss = 0;
        for(auto& [note, data] : note_to_osc_datas)
        {
            const auto  elapsed = (top_now - data.top_off);
            const float volume  = data.gate ? 1.f : elapsed > 0 ? .2 : 0;
            ss += data.osc.Process() * volume;
        }
        ss *= master_volume;
        out[ii + 0] = buffer_fft[ii / 2].l + ss;
        out[ii + 1] = buffer_fft[ii / 2].r + ss;

        top_now += dt;
    }

    meter.OnBlockEnd();
}

void midi_dump(const daisy::MidiEvent& event, daisy::DaisySeed& seed)
{
    using MesgType = daisy::MidiMessageType;
    using RTType   = daisy::SystemRealTimeType;

    switch(event.type)
    {
        case MesgType::NoteOff:
        {
            const auto note     = event.data[0];
            const auto velocity = event.data[1];
            seed.PrintLine("[midi] OFF channel %d note %d vel %d",
                           event.channel,
                           note,
                           velocity);
        }
        break;
        case MesgType::NoteOn:
        {
            const auto note     = event.data[0];
            const auto velocity = event.data[1];
            seed.PrintLine("[midi] ON channel %d note %d vel %d",
                           event.channel,
                           note,
                           velocity);
        }
        break;
        case MesgType::ControlChange:
        {
            const auto key   = event.data[0];
            const auto value = event.data[1];
            seed.PrintLine("[midi] CC channel %d key %d val %d",
                           event.channel,
                           key,
                           value);
        }
        break;
        case MesgType::SystemRealTime:
        {
            std::string label = "unknown";
            switch(event.srt_type)
            {
                case RTType::Start: label = "start"; break;
                case RTType::Continue: label = "continue"; break;
                case RTType::Stop: label = "stop"; break;
                default: break;
            }
            seed.PrintLine("[midi] SRT channel %d key %d label %s",
                           event.channel,
                           event.srt_type,
                           label.c_str());
        }
        break;
        case MesgType::ProgramChange:
        case MesgType::ChannelPressure:
        case MesgType::PitchBend:
        case MesgType::SystemCommon:
        case MesgType::ChannelMode:
        case MesgType::PolyphonicKeyPressure:
        default: break;
    }
}

void midi_callback(const daisy::MidiEvent& event,
                   const float             sample_rate,
                   const uint32_t          top_now)
{
    using MesgType = daisy::MidiMessageType;
    using RTType   = daisy::SystemRealTimeType;

    switch(event.type)
    {
        case MesgType::SystemRealTime:
        {
            switch(event.srt_type)
            {
                case RTType::TimingClock: count_midi_clocks++; break;
                case RTType::Start: midi_transport = true; break;
                case RTType::Continue: midi_transport = true; break;
                case RTType::Stop:
                    count_midi_clocks = 0;
                    midi_transport    = false;
                    break;
                case RTType::Reset:
                    count_midi_clocks = 0;
                    midi_transport    = false;
                    break;
                default: break;
            }
        }
        break;
        case MesgType::NoteOn:
        {
            const daisy::NoteOnEvent pp = event.AsNoteOn();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data == std::cend(note_to_osc_datas))
            {
                OscData data;
                data.osc.Init(sample_rate);
                // data.env.Init(sample_rate);

                bool is_inserted = false;
                std::tie(iter_osc_data, is_inserted)
                    = note_to_osc_datas.emplace(pp.note, data);
                assert(is_inserted);
            }
            assert(iter_osc_data != std::cend(note_to_osc_datas));
            auto& data = iter_osc_data->second;

            data.osc.Reset();
            data.osc.SetFreq(daisysp::mtof(pp.note));
            data.osc.SetAmp(std::max(pp.velocity / 127.f, .1f));
            // data.env.Retrigger(true);
            data.top_on  = top_now;
            data.top_off = 0;
            data.gate    = true;
        }
        break;
        case MesgType::NoteOff:
        {
            const daisy::NoteOffEvent pp = event.AsNoteOff();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data != std::cend(note_to_osc_datas))
            {
                auto& data   = iter_osc_data->second;
                data.top_off = top_now;
                data.gate    = false;
            }
        }
        break;
        case MesgType::ChannelMode:
        {
            // const AllNotesOffEvent pp = event.AsAllNotesOff();
            note_to_osc_datas.clear();
        }
        break;
        case MesgType::ControlChange:
        {
            const daisy::ControlChangeEvent pp = event.AsControlChange();

            switch(pp.control_number)
            {
                case 53: master_attack = pp.value / 127.f; break;
                case 54: master_decay = pp.value / 127.f; break;
                case 55: master_sustain = pp.value / 127.f; break;
                case 43: master_release = pp.value / 127.f; break;
            }
        }
        default: break;
    }
}

using Display = daisy::OledDisplay<daisy::SSD130xI2c128x64Driver>;

daisy::GPIO pin;

#if defined(WITH_MIDI_USB)
daisy::MidiUsbHandler midi_usb;
#endif

#if defined(WITH_DISPLAY)
Display display;

static std::array<char, 256> DSY_SDRAM_BSS format_buffer;

bool update_display = false;
#endif

static std::list<float> average_loads;

static const std::array<const char*, 8> waveform_names{
    "sin",
    "tri",
    "saw",
    "ramp",
    "square",
    "ptri",
    "psaw",
    "psquare",
};

int main(void)
{
    using Curve = daisy::Parameter::Curve;
    using daisy::GPIO;

    pod.Init();
    pod.SetAudioBlockSize(audio_block_size);
    pod.SetAudioSampleRate(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ);
    const float  sample_rate = pod.AudioSampleRate();
    const size_t block_size  = pod.AudioBlockSize();

    // pod.seed.usb_handle.Init(daisy::UsbHandle::FS_INTERNAL);

#if defined(WITH_MIDI_USB)
    { // init midi usb
        auto config = daisy::MidiUsbHandler::Config{};
        config.transport_config.periph
            = daisy::MidiUsbTransport::Config::Periph::INTERNAL;
        midi_usb.Init(config);
    }
#endif

    pin.Init(daisy::seed::D7, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    meter.Init(sample_rate, block_size);

#if defined(WITH_DISPLAY)
    { // init display
        auto config = Display::Config{};
        display.Init(config);
    }
#endif

    daisy::System::Delay(100);
    pod.seed.StartLog();
    daisy::System::Delay(100);

    pod.seed.PrintLine("[main] init");

    { // init vocoder
        // void*  buffer[2];
        // size_t buffer_size[2];
        // vocoder.Init(buffer,
        //              buffer_size,
        //              lut_sine_window_4096,
        //              4096,
        //              2,
        //              16,
        //              sample_rate);
        // vocoder.Buffer();
    }

    daisy::Parameter knob_volume;
    daisy::Parameter knob_waveform;
    knob_volume.Init(pod.knob1, 0, 1, Curve::LINEAR);
    knob_waveform.Init(pod.knob2, 0, 8, Curve::LINEAR);

    pod.StartAdc();
    pod.StartAudio(audio_callback);
    pod.midi.StartReceive();
#if defined(WITH_MIDI_USB)
    midi_usb.StartReceive();
#endif

    pod.seed.PrintLine("[main] loop");

    while(true)
    {
        const auto top_now = pod.seed.system.GetNow() * 1e-3f;

        { // controls
            pod.ProcessAllControls();

            master_waveform = std::round(knob_waveform.Process());
            assert(master_waveform >= 0);
            assert(master_waveform < 8);
            auto iter_name_data = std::begin(note_to_osc_datas);
            while(iter_name_data != std::end(note_to_osc_datas))
            {
                const auto gate    = iter_name_data->second.gate;
                const auto elapsed = top_now - iter_name_data->second.top_off;
                if(!gate && elapsed > master_release)
                {
                    iter_name_data = note_to_osc_datas.erase(iter_name_data);
                    continue;
                }
                assert(iter_name_data != std::end(note_to_osc_datas));
                auto& data = iter_name_data->second;
                data.osc.SetWaveform(master_waveform);
                // data.env.SetAttackTime(master_attack);
                // data.env.SetDecayTime(master_decay);
                // data.env.SetSustainLevel(master_sustain);
                // data.env.SetReleaseTime(master_release);
                iter_name_data++;
            }

            master_volume = knob_volume.Process();

#if defined(WITH_DISPLAY)
            if(pod.encoder.RisingEdge())
            {
                update_display ^= true;
                if(!update_display)
                {
                    display.Fill(false);
                    display.Update();
                }
            }
#endif

            if(pod.button1.RisingEdge())
            {
                pod.seed.PrintLine("[main] clear");
                note_to_osc_datas.clear();
            }

            if(pod.button2.RisingEdge())
            {
                pod.seed.PrintLine("[main] num_oscs %d",
                                   note_to_osc_datas.size());
                for(const auto& [note, data] : note_to_osc_datas)
                {
                    const auto elapsed_on  = top_now - data.top_on;
                    const auto elapsed_off = top_now - data.top_off;
                    pod.seed.PrintLine("** note %d on %dms ago off %dms ago %s",
                                       note,
                                       elapsed_on,
                                       elapsed_off,
                                       data.gate ? "down" : "up");
                }
            }
        }

        // trs midi
        pod.midi.Listen();
        if(pod.midi.HasEvents())
        {
            std::vector<daisy::MidiEvent> events;
            while(pod.midi.HasEvents())
            {
                const auto event = pod.midi.PopEvent();
                events.emplace_back(event);
            }

            std::sort(std::begin(events),
                      std::end(events),
                      [](const daisy::MidiEvent& aa,
                         const daisy::MidiEvent& bb) -> bool
                      { return aa.type < bb.type; });

            // pod.seed.PrintLine("[midi] **** TRS ****");
            for(const auto& event : events)
            {
                midi_dump(event, pod.seed);
                midi_callback(event, sample_rate, top_now);
            }
        }

#if defined(WITH_MIDI_USB)
        // usb midi
        midi_usb.Listen();
        if(midi_usb.HasEvents())
        {
            std::vector<daisy::MidiEvent> events;
            while(midi_usb.HasEvents())
            {
                const auto event = midi_usb.PopEvent();
                events.emplace_back(event);
            }
            std::sort(std::begin(events),
                      std::end(events),
                      [](const daisy::MidiEvent& aa,
                         const daisy::MidiEvent& bb) -> bool
                      { return aa.type < bb.type; });

            // pod.seed.PrintLine("[midi] **** USB ****");
            for(const auto& event : events)
            {
                midi_dump(event, pod.seed);
                midi_callback(event, sample_rate, top_now);
            }
        }
#endif

        { // leds
            const auto main_led_on
                = (top_now - std::floor(top_now)) < .1f ? 1.f : 0.f;
            pod.seed.SetLed(main_led_on);

            const auto midi_color = !midi_transport               ? 0.f
                                    : count_midi_clocks % 24 == 0 ? 0.f
                                    : count_midi_clocks % 24 < 13 ? 1.f
                                                                  : 0.f;
            pod.led1.Set(0, 0, midi_color);

            auto note_colors = std::array<float, 3>{0, 0, 0};
            if(!note_to_osc_datas.empty())
            {
                // clang-format off
                constexpr std::hash<uint8_t> hasher;
                size_t seed = 0x12ab45cd;
                for (const auto& [note, data] : note_to_osc_datas)
                    seed ^= hasher(note) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                std::mt19937 rng(seed);
                std::uniform_real_distribution<float> dist(0, 1);
                // clang-format on

                std::get<0>(note_colors) = dist(rng);
                std::get<1>(note_colors) = dist(rng);
                std::get<2>(note_colors) = dist(rng);
            }
            pod.led2.Set(std::get<0>(note_colors),
                         std::get<1>(note_colors),
                         std::get<2>(note_colors));

            pod.UpdateLeds();
        }

        { // load meter
            auto load = note_to_osc_datas.size() / 8.f;
            // auto load = master_volume;
            // auto load = meter.GetMaxCpuLoad();
            if(!std::isfinite(load))
                load = 0.f;
            load = std::max(0.f, std::min(1.f, load));
            average_loads.emplace_back(load);
            while(average_loads.size() > 16)
                average_loads.pop_front();
            assert(average_loads.size() <= 16);
        }

#if defined(WITH_DISPLAY)
        if(update_display)
        { // display
            display.Fill(true);

            display.SetCursor(0, 0);
            display.WriteString("Coucou", Font_11x18, false);
            display.DrawRect(0, 16, 128, 64, false, true);

            display.DrawRect(109, 1, 126, 14, false);
            uint_fast8_t xx = 110;
            for(const auto& load : average_loads)
            {
                const uint_fast8_t hh = std::round(14 - load * (14 - 2));
                display.DrawLine(xx, 14, xx, hh, false);
                xx += 1;
            }

            // display.DrawLine(0, 16, 128, 64, true);
            // display.DrawLine(0, 64, 128, 16, true);

            const uint_fast8_t hh = std::round(63 - master_volume * (63 - 16));
            display.DrawRect(0, 63, 6, hh, true, false);

            snprintf(format_buffer.data(),
                     format_buffer.size(),
                     "A%02d D%02d S%02d R%02d",
                     static_cast<int>(std::round(1e2 * master_attack)),
                     static_cast<int>(std::round(1e2 * master_decay)),
                     static_cast<int>(std::round(1e2 * master_sustain)),
                     static_cast<int>(std::round(1e2 * master_release)));
            display.SetCursor(8, 16);
            display.WriteString(format_buffer.data(), Font_7x10, true);

            snprintf(format_buffer.data(),
                     format_buffer.size(),
                     "%02d %s",
                     note_to_osc_datas.size(),
                     waveform_names[master_waveform]);
            display.SetCursor(8, 26);
            display.WriteString(format_buffer.data(), Font_7x10, true);

            const auto radius    = 16;
            const auto angle     = M_PI * 2 * (count_midi_clocks % 24) / 24.f;
            const auto center_xx = 128 - radius - 1;
            const auto center_yy = 64 - radius - 1;
            const auto hand_xx   = center_xx + (radius - 2) * sin(angle);
            const auto hand_yy   = center_yy - (radius - 2) * cos(angle);
            display.DrawCircle(center_xx, center_yy, radius, true);
            display.DrawLine(center_xx, center_yy, hand_xx, hand_yy, true);
            display.Update();
        }
#endif

        { // toogle pin & delay
            pin.Toggle();
            daisy::System::DelayUs(100);
        }
    }
}
