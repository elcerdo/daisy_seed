#include "daisy_pod.h"
#include "daisysp.h"
#include "fft.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <array>

// #define WITH_MIDI_USB

struct OscData
{
    uint32_t            top;
    daisysp::Oscillator osc;
};

using NoteToOscDatas = std::map<uint8_t, OscData>;

NoteToOscDatas note_to_osc_datas = {};
int32_t        note_balance      = 0;

constexpr size_t audio_block_size = 256;

using Patate = std::array<std::complex<double>, audio_block_size>;

Patate DSY_SDRAM_BSS left_channel;
Patate DSY_SDRAM_BSS right_channel;

int32_t count_midi_clocks = 0;
bool    midi_transport    = false;
float   master_volume     = .5f;
bool    use_midi_usb      = false;

void audio_callback(daisy::AudioHandle::InputBuffer  in,
                    daisy::AudioHandle::OutputBuffer out,
                    size_t                           size)
{
    assert(left_channel.size() == size);
    assert(right_channel.size() == size);
    for(size_t ii = 0; ii < size; ii++)
    {
        const std::complex<double> left  = in[0][ii];
        const std::complex<double> right = in[1][ii];

        left_channel[ii]  = left;
        right_channel[ii] = right;
    }

    // coucou::fast_fourier(left_channel.data(), left_channel.size());
    // coucou::fast_fourier(left_channel.data(), left_channel.size());
    // coucou::fast_fourier(foo.data(), foo.size());
    // coucou::fast_fourier(foo.data(), foo.size());

    for(size_t ii = 0; ii < size; ii++)
    {
        float ss = 0;
        for(auto& [note, data] : note_to_osc_datas)
            ss += data.osc.Process();
        ss *= master_volume;
        out[0][ii] = left_channel[ii].real() + ss;
        out[1][ii] = right_channel[ii].real() + ss;
    }
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
        case MesgType::ControlChange: label = "control_change"; break;
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
                   const float             samplerate,
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
                data.top = top_now;
                data.osc.Init(samplerate);

                bool is_inserted = false;
                std::tie(iter_osc_data, is_inserted)
                    = note_to_osc_datas.emplace(pp.note, data);
                assert(is_inserted);
                note_balance += 1;
            }
            assert(iter_osc_data != std::cend(note_to_osc_datas));
            auto& osc = iter_osc_data->second.osc;

            osc.Reset();
            osc.SetFreq(daisysp::mtof(pp.note));
            osc.SetAmp(1); // pp.velocity
            // osc.SetAmp(std::max(pp.velocity / 127, .1f));
        }
        break;
        case MesgType::NoteOff:
        {
            const daisy::NoteOffEvent pp = event.AsNoteOff();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data != std::cend(note_to_osc_datas))
            {
                iter_osc_data = note_to_osc_datas.erase(iter_osc_data);
                note_balance -= 1;
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
        default: break;
    }
}

daisy::DaisyPod pod;
#if defined(WITH_MIDI_USB)
daisy::MidiUsbHandler midi_usb;
#endif

int main(void)
{
    using Curve = daisy::Parameter::Curve;

    pod.Init();
#if defined(WITH_MIDI_USB)
    auto midi_usb_config = daisy::MidiUsbHandler::Config{};
    midi_usb.Init(midi_usb_config);
#endif
    pod.SetAudioBlockSize(
        audio_block_size); // number of samples handled per callback
    pod.SetAudioSampleRate(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ);
    pod.seed.usb_handle.Init(daisy::UsbHandle::FS_INTERNAL);
    daisy::System::Delay(200);

    pod.seed.StartLog();
    daisy::System::Delay(200);

    pod.seed.PrintLine("[main] init");

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

    const auto audio_samplerate = pod.AudioSampleRate();
    while(true)
    {
        const auto top_now = pod.seed.system.GetNow();

        // controls

        pod.ProcessAllControls();

        auto waveform = std::floor(knob_waveform.Process());
        assert(waveform >= 0);
        assert(waveform < 8);
        for(auto& [note, data] : note_to_osc_datas)
            data.osc.SetWaveform(waveform);

        master_volume = knob_volume.Process();

        if(pod.encoder.RisingEdge())
        {
            use_midi_usb ^= true;
            pod.seed.PrintLine("[main] using %s midi",
                               use_midi_usb ? "usb" : "trs");
        }

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
                const auto elapsed = top_now - data.top;
                pod.seed.PrintLine("       ** note %d %dms ago", note, elapsed);
            }
        }

        // midi events

        pod.midi.Listen();
        if(pod.midi.HasEvents())
        {
            std::vector<daisy::MidiEvent> events;
            while(pod.midi.HasEvents())
            {
                const auto event = pod.midi.PopEvent();
                events.emplace_back(event);
            }

            if(!use_midi_usb)
            {
                pod.seed.PrintLine("[midi] **** TRS ****");
                std::sort(std::begin(events),
                          std::end(events),
                          [](const daisy::MidiEvent& aa,
                             const daisy::MidiEvent& bb) -> bool
                          { return aa.type < bb.type; });
                for(const auto& event : events)
                {
                    midi_dump(event, pod.seed);
                    midi_callback(event, audio_samplerate, top_now);
                }
            }
        }

#if defined(WITH_MIDI_USB)
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

            if(use_midi_usb)
            {
                pod.seed.PrintLine("[midi] **** USB ****");
                for(const auto& event : events)
                {
                    midi_dump(event, pod.seed);
                    midi_callback(event, audio_samplerate, top_now);
                }
            }
        }
#endif

        // leds

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

        daisy::System::Delay(1);
    }
}
