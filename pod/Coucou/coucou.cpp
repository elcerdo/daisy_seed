#include "daisy_pod.h"
#include "daisysp.h"

#include <array>
#include <random>

using namespace daisy;
using namespace daisysp;

DaisyPod   hw;
Parameter  knob_aa, knob_bb;
Oscillator osc;

void audio_callback(AudioHandle::InterleavingInputBuffer  in,
                    AudioHandle::InterleavingOutputBuffer out,
                    size_t                                size)
{
    hw.ProcessAllControls();


    for(size_t ii = 0; ii < size; ii += 2)
    {
        const auto ss = osc.Process();
        out[ii]       = in[ii] + ss;
        out[ii + 1]   = in[ii + 1] + ss;
    }
}

void midi_callback(MidiEvent evt)
{
    switch(evt.type)
    {
        case MidiMessageType::NoteOn:
        {
            const NoteOnEvent pp = evt.AsNoteOn();
            osc.SetFreq(mtof(pp.note));
            osc.SetAmp(1.0); // force amplitude to max
            osc.Reset();
        }
        break;

        case MidiMessageType::NoteOff:
        {
            osc.SetAmp(0.0);
        }
        break;

        default: break;
    }
}

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(4); // number of samples handled per callback
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.seed.usb_handle.Init(UsbHandle::FS_INTERNAL);
    System::Delay(250);

    const auto samplerate = hw.AudioSampleRate();
    osc.Init(samplerate);
    osc.SetWaveform(Oscillator::WAVE_SAW);
    osc.SetAmp(0.0);

    knob_aa.Init(hw.knob1, 0, 1, Parameter::LINEAR);
    knob_bb.Init(hw.knob2, 0, 1, Parameter::LINEAR);

    hw.StartAdc();
    hw.StartAudio(audio_callback);
    hw.midi.StartReceive();

    while(true)
    {
        hw.midi.Listen();
        while(hw.midi.HasEvents())
            midi_callback(hw.midi.PopEvent());

        const auto red   = knob_aa.Process();
        const auto green = knob_bb.Process();
        hw.led1.Set(red, green, 0);
        hw.UpdateLeds();
    }
}
