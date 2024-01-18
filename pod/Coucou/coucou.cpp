#include "pvoc/phase_vocoder.h"
#include "pvoc/parameters.h"

#include "daisy_pod.h"
#include "daisysp.h"

#include "hid/disp/oled_display.h"
#include "dev/oled_ssd130x.h"
#include "util/oled_fonts.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <list>

constexpr size_t audio_block_size = 32;

// #define WITH_MIDI_USB
#define WITH_DISPLAY

struct OscData
{
    uint32_t            top_on;
    uint32_t            top_off;
    daisysp::Oscillator osc;
    daisysp::Adsr       env;
    bool                gate;
};

using NoteToOscDatas = std::map<uint8_t, OscData>;

NoteToOscDatas note_to_osc_datas = {};
int32_t        note_balance      = 0;

using FloatFrameArray = std::array<FloatFrame, audio_block_size>;

FloatFrameArray DSY_SDRAM_BSS buffer_fft;

PhaseVocoder vocoder;

daisy::CpuLoadMeter meter;

int32_t count_midi_clocks = 0;
bool    midi_transport    = false;
uint8_t master_waveform   = 0;
float   master_volume     = .5f;
float   master_attack     = .5f;
float   master_decay      = .5f;
float   master_sustain    = .5f;
float   master_release    = .5f;

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

    for(size_t ii = 0; ii < size; ii += 2)
    {
        float ss = 0;
        for(auto& [note, data] : note_to_osc_datas)
            ss += data.osc.Process() * data.env.Process(data.gate);
        ss *= master_volume;
        out[ii + 0] = buffer_fft[ii / 2].l + ss;
        out[ii + 1] = buffer_fft[ii / 2].r + ss;
    }

    meter.OnBlockEnd();
}

void midi_dump(const daisy::MidiEvent& event, daisy::DaisySeed& seed)
{
    using MesgType = daisy::MidiMessageType;

    std::string label    = "unknown";
    int32_t     note     = -1;
    int32_t     velocity = -1;
    switch(event.type)
    {
        case MesgType::NoteOff:
            label    = "note_off";
            note     = event.data[0];
            velocity = event.data[1];
            break;
        case MesgType::NoteOn:
            label    = "note_on";
            note     = event.data[0];
            velocity = event.data[1];
            break;
        case MesgType::PolyphonicKeyPressure: label = "poly_pressure"; break;
        case MesgType::ControlChange:
            label    = "control_change";
            note     = event.data[0];
            velocity = event.data[1];
            break;
        case MesgType::ProgramChange: label = "program_change"; break;
        case MesgType::ChannelPressure: label = "channel_pressure"; break;
        case MesgType::PitchBend: label = "pitch_bend"; break;
        case MesgType::SystemCommon: label = "sys_common"; break;
        case MesgType::SystemRealTime:
            label = "sys_rt";
            note  = static_cast<uint8_t>(event.srt_type);
            break;
        case MesgType::ChannelMode: label = "channel_mode"; break;
        default: break;
    }

    seed.PrintLine("[midi] %s channel %d note %d vel %d balance %d",
                   label.c_str(),
                   event.channel,
                   note,
                   velocity,
                   note_balance);
}

void midi_callback(const daisy::MidiEvent& event,
                   const float             sample_rate,
                   const size_t            block_size,
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
            note_balance += 1;

            const daisy::NoteOnEvent pp = event.AsNoteOn();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data == std::cend(note_to_osc_datas))
            {
                OscData data;
                data.top_on  = top_now;
                data.top_off = 0;
                data.osc.Init(sample_rate);
                data.env.Init(sample_rate, block_size);
                data.gate = true;

                bool is_inserted = false;
                std::tie(iter_osc_data, is_inserted)
                    = note_to_osc_datas.emplace(pp.note, data);
                assert(is_inserted);
            }
            assert(iter_osc_data != std::cend(note_to_osc_datas));
            auto& data = iter_osc_data->second;

            // data.osc.Reset();
            data.osc.SetFreq(daisysp::mtof(pp.note));
            data.osc.SetAmp(std::max(pp.velocity / 127.f, .1f));
            data.gate = true;
        }
        break;
        case MesgType::NoteOff:
        {
            note_balance -= 1;

            const daisy::NoteOffEvent pp = event.AsNoteOff();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data != std::cend(note_to_osc_datas))
            {
                auto& data   = iter_osc_data->second;
                data.gate    = false;
                data.top_off = top_now;
            }
        }
        break;
        case MesgType::ChannelMode:
        {
            // const AllNotesOffEvent pp = event.AsAllNotesOff();
            note_to_osc_datas.clear();
            note_balance = 0;
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

daisy::DaisyPod pod;
daisy::GPIO     pin;

#if defined(WITH_MIDI_USB)
daisy::MidiUsbHandler midi_usb;
#endif

#if defined(WITH_DISPLAY)
Display display;

static std::array<char, 256> DSY_SDRAM_BSS format_buffer;
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
    const auto sample_rate = pod.AudioSampleRate();
    const auto block_size  = pod.AudioBlockSize();

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
        const auto top_now = pod.seed.system.GetNow();

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
                if(!gate && elapsed > 1000)
                {
                    iter_name_data = note_to_osc_datas.erase(iter_name_data);
                    continue;
                }
                assert(iter_name_data != std::end(note_to_osc_datas));
                auto& data = iter_name_data->second;
                data.osc.SetWaveform(master_waveform);
                data.env.SetAttackTime(master_attack * 2);
                data.env.SetDecayTime(master_decay * 2);
                data.env.SetSustainLevel(master_sustain);
                data.env.SetReleaseTime(master_release * 10);
                iter_name_data++;
            }

            master_volume = knob_volume.Process();

            // if(pod.encoder.RisingEdge())
            // {
            //     use_midi_usb ^= true;
            //     pod.seed.PrintLine("[main] using %s midi",
            //                        use_midi_usb ? "usb" : "trs");
            // }

            if(pod.button1.RisingEdge())
            {
                pod.seed.PrintLine("[main] clear");
                note_to_osc_datas.clear();
                note_balance = 0;
            }

            if(pod.button2.RisingEdge())
            {
                pod.seed.PrintLine("[main] num_oscs %d balance %d",
                                   note_to_osc_datas.size(),
                                   note_balance);
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

            pod.seed.PrintLine("[midi] **** TRS ****");
            std::sort(std::begin(events),
                      std::end(events),
                      [](const daisy::MidiEvent& aa,
                         const daisy::MidiEvent& bb) -> bool
                      { return aa.type < bb.type; });
            for(const auto& event : events)
            {
                midi_dump(event, pod.seed);
                midi_callback(event, sample_rate, block_size, top_now);
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

            pod.seed.PrintLine("[midi] **** USB ****");
            for(const auto& event : events)
            {
                midi_dump(event, pod.seed);
                midi_callback(event, sample_rate, block_size, top_now);
            }
        }
#endif

        { // leds
            const auto main_led_on = top_now % 1000 < 100 ? 1.f : 0.f;
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
