#include "fft.h"

#include <U8x8lib.h>

#include "daisy_pod.h"
#include "daisysp.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <array>

extern "C" uint8_t daisy_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
    assert(false);
    // uint8_t i;
    // switch(msg)
    // {
    //     case U8X8_MSG_GPIO_AND_DELAY_INIT:
        
    //     for( i = 0; i < U8X8_PIN_CNT; i++ )
    //     if ( u8x8->pins[i] != U8X8_PIN_NONE )
    //     {
    //     if ( i < U8X8_PIN_OUTPUT_CNT )
    //     {
    //         pinMode(u8x8->pins[i], OUTPUT);
    //     }
    //     else
    //     {
    // #ifdef INPUT_PULLUP
    //         pinMode(u8x8->pins[i], INPUT_PULLUP);
    // #else
    //         pinMode(u8x8->pins[i], OUTPUT);
    //         digitalWrite(u8x8->pins[i], 1);
    // #endif 
    //     }
    //     }
        
    //     break;

    // #ifndef __AVR__	
    //     /* this case is not compiled for any AVR, because AVR uC are so slow */
    //     /* that this delay does not matter */
    //     case U8X8_MSG_DELAY_NANO:
    //     delayMicroseconds(arg_int==0?0:1);
    //     break;
    // #endif
        
    //     case U8X8_MSG_DELAY_10MICRO:
    //     /* not used at the moment */
    //     break;
        
    //     case U8X8_MSG_DELAY_100NANO:
    //     /* not used at the moment */
    //     break;
    
    //     case U8X8_MSG_DELAY_MILLI:
    //     delay(arg_int);
    //     break;
    //     case U8X8_MSG_DELAY_I2C:
    //     /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
    //     delayMicroseconds(arg_int<=2?5:2);
    //     break;
    //     case U8X8_MSG_GPIO_I2C_CLOCK:
    //     case U8X8_MSG_GPIO_I2C_DATA:
    //     if ( arg_int == 0 )
    //     {
    //     pinMode(u8x8_GetPinValue(u8x8, msg), OUTPUT);
    //     digitalWrite(u8x8_GetPinValue(u8x8, msg), 0);
    //     }
    //     else
    //     {
    // #ifdef INPUT_PULLUP
    //     pinMode(u8x8_GetPinValue(u8x8, msg), INPUT_PULLUP);
    // #else
    //     pinMode(u8x8_GetPinValue(u8x8, msg), OUTPUT);
    //     digitalWrite(u8x8_GetPinValue(u8x8, msg), 1);
    // #endif 
    //     }
    //     break;
    //     default:
    //     if ( msg >= U8X8_MSG_GPIO(0) )
    //     {
    //     i = u8x8_GetPinValue(u8x8, msg);
    //     if ( i != U8X8_PIN_NONE )
    //     {
    //     if ( u8x8_GetPinIndex(u8x8, msg) < U8X8_PIN_OUTPUT_CNT )
    //     {
    //         digitalWrite(i, arg_int);
    //     }
    //     else
    //     {
    //         if ( u8x8_GetPinIndex(u8x8, msg) == U8X8_PIN_OUTPUT_CNT )
    //         {
    //         // call yield() for the first pin only, u8x8 will always request all the pins, so this should be ok
    //         yield();
    //         }
    //         u8x8_SetGPIOResult(u8x8, digitalRead(i) == 0 ? 0 : 1);
    //     }
    //     }
    //     break;
    //     }
        
    //     return 0;
    // }
    return 1;
}

extern "C" uint8_t daisy_hw_i2c(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
    assert(false);
// #ifdef U8X8_HAVE_HW_I2C
//   switch(msg)
//   {
//     case U8X8_MSG_BYTE_SEND:
//       Wire.write((uint8_t *)arg_ptr, (int)arg_int);
//       break;
//     case U8X8_MSG_BYTE_INIT:
//       if ( u8x8->bus_clock == 0 ) 	/* issue 769 */
// 	u8x8->bus_clock = u8x8->display_info->i2c_bus_clock_100kHz * 100000UL;
// #if defined(ESP8266) || defined(ARDUINO_ARCH_ESP8266) || defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
//       /* for ESP8266/ESP32, Wire.begin has two more arguments: clock and data */          
//       if ( u8x8->pins[U8X8_PIN_I2C_CLOCK] != U8X8_PIN_NONE && u8x8->pins[U8X8_PIN_I2C_DATA] != U8X8_PIN_NONE )
//       {
// 	// second argument for the wire lib is the clock pin. In u8g2, the first argument of the  clock pin in the clock/data pair
// 	Wire.begin((int)u8x8->pins[U8X8_PIN_I2C_DATA] , u8x8->pins[U8X8_PIN_I2C_CLOCK]);
//       }
//       else
//       {
// 	Wire.begin();
//       }
// #else
//       Wire.begin();
// #endif
//       break;
//     case U8X8_MSG_BYTE_SET_DC:
//       break;
//     case U8X8_MSG_BYTE_START_TRANSFER:
// #if ARDUINO >= 10600
//       /* not sure when the setClock function was introduced, but it is there since 1.6.0 */
//       /* if there is any error with Wire.setClock() just remove this function call by */
//       /* defining U8X8_DO_NOT_SET_WIRE_CLOCK */
// #ifndef U8X8_DO_NOT_SET_WIRE_CLOCK
//       Wire.setClock(u8x8->bus_clock);
// #endif 
// #endif
//       Wire.beginTransmission(u8x8_GetI2CAddress(u8x8)>>1);
//       break;
//     case U8X8_MSG_BYTE_END_TRANSFER:
//       Wire.endTransmission();
//       break;
//     default:
//       return 0;
//   }
// #endif
  return 1;
}


class Screen : public U8X8 {
  public: Screen() : U8X8() {
    u8x8_Setup(getU8x8(), u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, daisy_hw_i2c, daisy_gpio_and_delay);
    constexpr uint8_t reset = U8X8_PIN_NONE;
    constexpr uint8_t clock = U8X8_PIN_NONE;
    constexpr uint8_t data = U8X8_PIN_NONE;
    u8x8_SetPin_HW_I2C(getU8x8(), reset, clock, data);
  }
};

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
daisy::GPIO     pin;

#if defined(WITH_MIDI_USB)
daisy::MidiUsbHandler midi;
#endif

Screen                screen;
std::array<char, 256> format_buffer;

int main(void)
{
    using Curve = daisy::Parameter::Curve;
    using daisy::GPIO;

    pod.Init();
#if defined(WITH_MIDI_USB)
    { // init midi usb
        auto config = daisy::MidiUsbHandler::Config{};
        // config.transport_config.periph
        //     = daisy::MidiUsbTransport::Config::Periph::EXTERNAL;
        // midi.Init(config);
    }
#endif
    pod.SetAudioBlockSize(audio_block_size);
    pod.SetAudioSampleRate(daisy::SaiHandle::Config::SampleRate::SAI_48KHZ);
    // pod.seed.usb_handle.Init(daisy::UsbHandle::FS_INTERNAL);
    pod.seed.StartLog();
    pin.Init(daisy::seed::D7, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);
    screen.begin();
    screen.setPowerSave(0);
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

    // screen.Begin();

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
        midi.Listen();
        if(midi.HasEvents())
        {
            std::vector<daisy::MidiEvent> events;
            while(midi.HasEvents())
            {
                const auto event = midi.PopEvent();
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

        // screen

        screen.setFont(u8x8_font_8x13_1x2_f);
        screen.setInverseFont(0);
        screen.drawString(0, 0, "Hello World!");
        screen.drawString(0, 0, R"(AbCd{}[]*-/|\)");

        screen.setFont(u8x8_font_pxplusibmcgathin_f);
        snprintf(format_buffer.data(),
                 format_buffer.size(),
                 "foo %03d",
                 note_to_osc_datas.size());
        screen.setFont(u8x8_font_chroma48medium8_r);
        screen.drawString(0, 6, format_buffer.data());

        // pins

        pin.Toggle();

        daisy::System::DelayUs(100);
    }
}
