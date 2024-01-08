#include "daisy_pod.h"
#include "daisysp.h"
#include "fft.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <array>

// using namespace daisy;
// using namespace daisysp;

struct OscData
{
    uint32_t            top;
    daisysp::Oscillator osc;
};

using NoteToOscDatas = std::map<uint8_t, OscData>;

NoteToOscDatas note_to_osc_datas;

daisy::Parameter knob_volume;
daisy::Parameter knob_waveform;

void audio_callback(daisy::AudioHandle::InputBuffer  in,
                    daisy::AudioHandle::OutputBuffer out,
                    size_t                           size)
{
    auto waveform = std::floor(knob_waveform.Process());
    assert(waveform >= 0);
    assert(waveform < 8);
    for(auto& [note, data] : note_to_osc_datas)
        data.osc.SetWaveform(waveform);

    const auto master_volume = knob_volume.Process();

    using Patate = std::vector<std::complex<double>>;

    Patate left_channel;
    Patate right_channel;
    left_channel.reserve(size);
    right_channel.reserve(size);
    for(size_t ii = 0; ii < size; ii++)
    {
        const std::complex<double> left  = in[0][ii];
        const std::complex<double> right = in[1][ii];

        left_channel.emplace_back(left);
        right_channel.emplace_back(right);
    }

    coucou::fast_fourier(left_channel.data(), left_channel.size());
    coucou::fast_fourier(left_channel.data(), left_channel.size());
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

daisy::DaisyPod hardware;

void midi_dump(const daisy::MidiEvent& evt)
{
    using MesgType = daisy::MidiMessageType;

    std::string label = "unknown";
    switch(evt.type)
    {
        case MesgType::NoteOff: label = "note_off"; break;
        case MesgType::NoteOn: label = "note_on"; break;
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

    hardware.seed.PrintLine("[midi] %s channel %d", label.c_str(), evt.channel);
}

size_t count_midi_clocks;

void midi_callback(const daisy::MidiEvent& evt,
                   const float             samplerate,
                   const uint32_t          top_now)
{
    using MesgType = daisy::MidiMessageType;
    using RTType   = daisy::SystemRealTimeType;

    switch(evt.type)
    {
        case MesgType::SystemRealTime:
        {
            switch(evt.srt_type)
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
            const daisy::NoteOnEvent pp = evt.AsNoteOn();

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
        }
        break;
        case MesgType::NoteOff:
        {
            const daisy::NoteOffEvent pp = evt.AsNoteOff();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data != std::cend(note_to_osc_datas))
                iter_osc_data = note_to_osc_datas.erase(iter_osc_data);
        }
        break;
        case MesgType::ChannelMode:
        {
            // const AllNotesOffEvent pp = evt.AsAllNotesOff();
            note_to_osc_datas.clear();
        }
        break;
        default: break;
    }
}

int main(void)
{
    using Curve = daisy::Parameter::Curve;

    auto& hw = hardware;

    hw.Init();

    hw.SetAudioBlockSize(256); // number of samples handled per callback
    hw.SetAudioSampleRate(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.seed.usb_handle.Init(daisy::UsbHandle::FS_INTERNAL);
    daisy::System::Delay(100);

    knob_volume.Init(hw.knob1, 0, 1, Curve::LINEAR);
    knob_waveform.Init(hw.knob2, 0, 8, Curve::LINEAR);

    count_midi_clocks = 0;

    hw.seed.StartLog(true);
    hw.StartAdc();
    hw.StartAudio(audio_callback);
    hw.midi.StartReceive();

    hw.seed.PrintLine("[init] main loop");

    while(true)
    {
        const auto top_now = hw.seed.system.GetNow();

        // controls

        hw.ProcessAllControls();

        if(hw.button1.Pressed())
            note_to_osc_datas.clear();

        // midi events

        hw.midi.Listen();
        while(hw.midi.HasEvents())
        {
            const auto evt = hw.midi.PopEvent();
            midi_dump(evt);
            midi_callback(evt, hw.AudioSampleRate(), top_now);
        }

        // leds

        const auto main_clock_color = top_now % 1000 < 100 ? 1.f : 0.f;
        const auto midi_clock_color = count_midi_clocks % 24 == 0 ? 1.f : 0.f;
        hw.led1.Set(main_clock_color, 0, midi_clock_color);

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
        hw.led2.Set(std::get<0>(note_colors),
                    std::get<1>(note_colors),
                    std::get<2>(note_colors));

        hw.UpdateLeds();
    }
}
