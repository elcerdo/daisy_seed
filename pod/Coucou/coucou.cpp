#include "fft.h"

#include <U8x8lib.h>

#include "daisy_pod.h"
#include "daisysp.h"

#include <array>
#include <random>
#include <map>
#include <vector>
#include <array>


/*
struct Screen
{
    // using Display = U8X8_SSD1306_128X64_NONAME_HW_I2C;

    static constexpr uint16_t address = 24;
    static constexpr uint8_t  width   = 128;
    static constexpr uint8_t  height  = 64;
    static constexpr uint32_t timeout = 10;

    daisy::I2CHandle handle;

    void Init();
    void Begin();
    void Update();
    void SetPower(const bool enable);

    void HelloWorld();
};

void Screen::Init()
{
    using daisy::I2CHandle;

    I2CHandle::Config config;
    config.mode   = I2CHandle::Config::Mode::I2C_MASTER;
    config.periph = I2CHandle::Config::Peripheral::I2C_1;
    // config.speed          = I2CHandle::Config::Speed::I2C_100KHZ;
    config.speed          = I2CHandle::Config::Speed::I2C_1MHZ;
    config.pin_config.scl = {DSY_GPIOB, 8}; // D11
    config.pin_config.sda = {DSY_GPIOB, 9}; // D12

    [[maybe_unused]] const auto ret = handle.Init(config);
    assert(ret == daisy::I2CHandle::Result::OK);
}

void Screen::Begin()
{
    const auto i2c_send = [this](uint8_t data)
    {
        const auto ret = handle.TransmitBlocking(address, &data, 1, timeout);
        assert(ret == daisy::I2CHandle::Result::OK);
    };

    //     #if defined OUTPUT_OPEN_DRAIN
    //         pinMode(sda_pin, OUTPUT_OPEN_DRAIN);
    //         digitalWrite(sda_pin, HIGH);
    //         pinMode(scl_pin, OUTPUT_OPEN_DRAIN);
    //         digitalWrite(scl_pin, HIGH);
    //     #else
    //         pinMode(sda_pin, INPUT);
    //         pinMode(scl_pin, INPUT);
    //     #endif
    //     if (reset_pin != NO_RESET_PIN)
    //     {
    //         pinMode(reset_pin, OUTPUT);
    //         digitalWrite(reset_pin, LOW);
    //         delay(10);
    //         digitalWrite(reset_pin, HIGH);
    //     }
    //     delay(100);

    // i2c_start();

    // i2c_send(i2c_address << 1); // address + write
    i2c_send(0x00); // command
    i2c_send(0xAE); // display off
    i2c_send(0xD5); // clock divider
    i2c_send(0x80);
    i2c_send(0xA8); // multiplex ratio
    i2c_send(height - 1);
    i2c_send(0xD3); // no display offset
    i2c_send(0x00);
    i2c_send(0x40); // start line address=0
    i2c_send(0x8D); // enable charge pump
    i2c_send(0x14);
    i2c_send(0x20); // memory adressing mode=horizontal
    i2c_send(0x00);
    i2c_send(0xA1); // segment remapping mode
    i2c_send(0xC8); // COM output scan direction

    if(width == 128 && height == 32)
    {
        i2c_send(0xDA); // com pins hardware configuration
        i2c_send(0x02);
    }
    else if(width == 128 && height == 64)
    {
        i2c_send(0xDA); // com pins hardware configuration
        i2c_send(0x12);
    }
    else if(width == 96 && height == 16)
    {
        i2c_send(0xDA); // com pins hardware configuration
        i2c_send(0x02);
    }

    i2c_send(0x81); // contrast control
    i2c_send(0x80);
    i2c_send(0xD9); // pre-charge period
    i2c_send(0x22);
    i2c_send(0xDB); // set vcomh deselect level
    i2c_send(0x20);
    i2c_send(0xA4); // output RAM to display
    i2c_send(0xA6); // display mode A6=normal, A7=inverse
    i2c_send(0x2E); // stop scrolling

    // i2c_stop();

    // daisy::System::Delay(100);

    // Update();
    SetPower(true);
}


void Screen::Update()
{
    SetPower(true);
    // auto data = std::array<uint8_t, 2>{
    //     0xff,
    //     0,
    // };

    // uint16_t index = 0;
    // for (uint_fast8_t page = 0; page < pages; page++)
    // {
    //     // Set memory address to fill
    //     i2c_start();
    //     i2c_send(i2c_address << 1); // address + write
    //     i2c_send(0x00); // command
    //     if (displayController == CTRL_SH1106)
    //     {
    //         i2c_send(0xB0 + page); // set page
    //         i2c_send(0x00); // lower columns address =0
    //         i2c_send(0x10); // upper columns address =0
    //     }
    //     else
    //     {
    //         i2c_send(0xB0 + page); // set page
    //         i2c_send(0x21); // column address
    //         i2c_send(0x00); // first column =0
    //         i2c_send(width - 1); // last column
    //     }
    //     i2c_stop();

    //     // send one page of buffer to the display
    //     i2c_start();
    //     i2c_send(i2c_address << 1); // address + write
    //     i2c_send(0x40); // data
    //     if(usingOffset){
    //         i2c_send(0);
    //         i2c_send(0);
    //     }
    //     for (uint_fast8_t column = 0; column < width; column++)
    //     {
    //         i2c_send(buffer[index++]);
    //     }
    //     i2c_stop();
    //     yield(); // to avoid that the watchdog triggers
    // }
}


#define U8X8_C(c0) (U8X8_MSG_CAD_SEND_CMD), (c0)
#define U8X8_A(a0) (U8X8_MSG_CAD_SEND_ARG), (a0)
#define U8X8_CA(c0, a0) \
    (U8X8_MSG_CAD_SEND_CMD), (c0), (U8X8_MSG_CAD_SEND_ARG), (a0)

#define U8X8_START_TRANSFER() (U8X8_MSG_CAD_START_TRANSFER)
#define U8X8_END_TRANSFER() (U8X8_MSG_CAD_END_TRANSFER)
// #define U8X8_END() (0xff)

void Screen::SetPower(const bool enable)
{
    static const std::array<uint8_t, 4> mesg_on = {
        U8X8_START_TRANSFER(), // enable chip, delay is part of the transfer start
        U8X8_C(0x0ae),         // display off
        U8X8_END_TRANSFER(),   // disable chip
        // U8X8_END(),
    };
    static const std::array<uint8_t, 4> mesg_off = {
        U8X8_START_TRANSFER(), // enable chip, delay is part of the transfer start
        U8X8_C(0x0af),         // display on
        U8X8_END_TRANSFER(),   // disable chip
        // U8X8_END(),
    };

    const auto& mesg = enable ? mesg_on : mesg_off;

    [[maybe_unused]] const auto ret = handle.TransmitBlocking(
        address, const_cast<uint8_t*>(mesg.data()), mesg.size(), timeout);
    assert(ret == daisy::I2CHandle::Result::OK);
}
*/

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

using Screen = U8X8_SSD1306_128X64_NONAME_HW_I2C;

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
