#include "daisy_pod.h"
#include "daisysp.h"
#include "fft.h"

#include <array>
#include <random>
#include <chrono>
#include <map>
#include <vector>

using namespace daisy;
using namespace daisysp;

using Rng       = std::mt19937;
using DistColor = std::uniform_real_distribution<float>;
using Clock     = std::chrono::high_resolution_clock;

struct OscData
{
    Clock::time_point top;
    Oscillator        osc;
};

using NoteToOscDatas = std::map<uint8_t, OscData>;

DaisyPod       hw;
Parameter      knob_volume;
Parameter      knob_waveform;
NoteToOscDatas note_to_osc_datas;

Rng       rng;
DistColor dist_color;
size_t    count_midi_clocks;

void audio_callback(AudioHandle::InterleavingInputBuffer  in,
                    AudioHandle::InterleavingOutputBuffer out,
                    size_t                                size)
{
    using Patate = std::vector<std::complex<double>>;

    Patate foo;
    foo.reserve(size / 2);
    for(size_t ii = 0; ii < size; ii += 2)
        foo.emplace_back(in[ii]);

    coucou::fast_fourier(foo.data(), foo.size());


    hw.ProcessAllControls();

    auto waveform = std::floor(knob_waveform.Process());
    assert(waveform >= 0);
    assert(waveform < 8);
    for(auto& [note, data] : note_to_osc_datas)
        data.osc.SetWaveform(waveform);

    const auto master_volume = knob_volume.Process();
    for(size_t ii = 0; ii < size; ii += 2)
    {
        float ss = 0;
        for(auto& [note, data] : note_to_osc_datas)
            ss += data.osc.Process();
        ss *= master_volume;
        out[ii]     = in[ii] + ss;
        out[ii + 1] = in[ii + 1] + ss;
    }
}

void midi_callback(MidiEvent evt)
{
    // hw.seed.PrintLine("[midi_callback]");

    switch(evt.type)
    {
        case MidiMessageType::SystemRealTime:
        {
            switch(evt.srt_type)
            {
                case SystemRealTimeType::TimingClock:
                    count_midi_clocks += 1;
                    break;
                case SystemRealTimeType::Stop: count_midi_clocks = 0; break;
                case SystemRealTimeType::Reset: count_midi_clocks = 0; break;
                default: break;
            }
        }
        break;
        case MidiMessageType::NoteOn:
        {
            const NoteOnEvent pp = evt.AsNoteOn();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data == std::cend(note_to_osc_datas))
            {
                const auto samplerate = hw.AudioSampleRate();

                OscData data;
                data.top = Clock::now();
                data.osc.Init(samplerate);

                bool is_inserted = false;
                std::tie(iter_osc_data, is_inserted)
                    = note_to_osc_datas.emplace(pp.note, data);
                assert(is_inserted);
            }
            assert(iter_osc_data != std::cend(note_to_osc_datas));
            auto& osc = iter_osc_data->second.osc;

            // osc.Reset();
            // osc.SetAmp(0.0);
            osc.SetFreq(mtof(pp.note));
            osc.SetAmp(1); // pp.velocity
            // osc.SetAmp(std::max(pp.velocity, .1f));
            hw.led2.Set(dist_color(rng), dist_color(rng), dist_color(rng));
        }
        break;
        case MidiMessageType::NoteOff:
        {
            const NoteOffEvent pp = evt.AsNoteOff();

            auto iter_osc_data = note_to_osc_datas.find(pp.note);
            if(iter_osc_data != std::cend(note_to_osc_datas))
                iter_osc_data = note_to_osc_datas.erase(iter_osc_data);

            hw.led2.Set(0, 0, 0);
        }
        break;
        case MidiMessageType::ChannelMode:
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
    hw.Init();
    hw.SetAudioBlockSize(256); // number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.seed.usb_handle.Init(UsbHandle::FS_INTERNAL);
    System::Delay(250);

    // hw.seed.StartLog(true);
    // hw.seed.PrintLine("coucou");

    knob_volume.Init(hw.knob1, 0, 1, Parameter::LINEAR);
    knob_waveform.Init(hw.knob2, 0, 8, Parameter::LINEAR);

    rng               = Rng(0x12ab45cd);
    dist_color        = std::uniform_real_distribution<float>(0, 1);
    count_midi_clocks = 0;

    hw.StartAdc();
    hw.StartAudio(audio_callback);
    hw.midi.StartReceive();

    const auto top_start = Clock::now();
    while(true)
    {
        hw.midi.Listen();
        while(hw.midi.HasEvents())
            midi_callback(hw.midi.PopEvent());

        const std::chrono::duration<float> dur = (Clock::now() - top_start);

        const auto green = dur.count() - std::floor(dur.count());
        hw.led1.Set(0, green, count_midi_clocks % 24 == 0 ? 1 : 0);
        hw.UpdateLeds();

        if(hw.button1.Pressed())
            note_to_osc_datas.clear();
    }
}
