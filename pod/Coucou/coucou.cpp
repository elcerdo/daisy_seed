#include "daisy_pod.h"
#include "daisysp.h"
#include "fft.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <array>

struct OscData
{
    uint32_t            top;
    daisysp::Oscillator osc;
};

using NoteToOscDatas = std::map<uint8_t, OscData>;

NoteToOscDatas note_to_osc_datas = {};
int32_t        note_balance      = 0;

using Patate = std::vector<std::complex<double>>;

Patate left_channel;
Patate right_channel;

int32_t count_midi_clocks = 0;
float   master_volume     = .5f;

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

    auto left_iter  = std::cbegin(left_channel);
    auto right_iter = std::cbegin(right_channel);
    for(size_t ii = 0; ii < size; ii++)
    {
        float ss = 0;
        for(auto& [note, data] : note_to_osc_datas)
            ss += data.osc.Process();
        ss *= master_volume;
        out[0][ii] = (*left_iter++).real() + ss;
        out[1][ii] = (*right_iter++).real() + ss;
    }
}

void midi_dump(const daisy::MidiEvent& event, daisy::DaisySeed& seed)
{
    using MesgType = daisy::MidiMessageType;

    std::string label    = "unknown";
    uint8_t     note     = 0;
    uint8_t     velocity = 0;
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
        case MesgType::SystemRealTime: label = "sys_rt"; break;
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
                case RTType::Stop: count_midi_clocks = 0; break;
                case RTType::Reset: count_midi_clocks = 0; break;
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
            }
            assert(iter_osc_data != std::cend(note_to_osc_datas));
            auto& osc = iter_osc_data->second.osc;

            osc.Reset();
            osc.SetFreq(daisysp::mtof(pp.note));
            osc.SetAmp(1); // pp.velocity
            // osc.SetAmp(std::max(pp.velocity / 127, .1f));

            note_balance += 1;
        }
        break;
        case MesgType::NoteOff:
        {
            const daisy::NoteOffEvent pp = event.AsNoteOff();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data != std::cend(note_to_osc_datas))
                iter_osc_data = note_to_osc_datas.erase(iter_osc_data);

            note_balance -= 1;
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

int main(void)
{
    using Curve = daisy::Parameter::Curve;

    pod.Init();
    pod.SetAudioBlockSize(256); // number of samples handled per callback
    pod.SetAudioSampleRate(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ);
    pod.seed.usb_handle.Init(daisy::UsbHandle::FS_INTERNAL);
    daisy::System::Delay(200);

    pod.seed.StartLog(true);
    daisy::System::Delay(200);

    pod.seed.PrintLine("[main] init");

    left_channel.resize(pod.AudioBlockSize());
    right_channel.resize(pod.AudioBlockSize());

    daisy::Parameter knob_volume;
    daisy::Parameter knob_waveform;
    knob_volume.Init(pod.knob1, 0, 1, Curve::LINEAR);
    knob_waveform.Init(pod.knob2, 0, 8, Curve::LINEAR);

    pod.StartAdc();
    pod.StartAudio(audio_callback);
    pod.midi.StartReceive();

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

        if(pod.button1.RisingEdge())
        {
            pod.seed.PrintLine("[main] clear");
            note_to_osc_datas.clear();
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
            pod.seed.PrintLine("[midi] *******");
        while(pod.midi.HasEvents())
        {
            const auto event = pod.midi.PopEvent();
            midi_dump(event, pod.seed);
            midi_callback(event, audio_samplerate, top_now);
        }

        // leds

        const auto main_clock_color = top_now % 1000 < 100 ? 1.f : 0.f;
        const auto midi_clock_color = count_midi_clocks % 24 == 0 ? 1.f : 0.f;
        pod.led1.Set(main_clock_color, 0, midi_clock_color);

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
