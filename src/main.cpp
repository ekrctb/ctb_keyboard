#include <Arduino.h>
#include "RingBuffer.hpp"

constexpr bool SERIAL_DEBUG = false;

struct KeyConfig
{
    char const *name;
    uint8_t pin;
    uint8_t scanCode;
};

// The host has to do an I/O during this delay and failing this timeout implies a communication failure
constexpr uint8_t DELAY_PRIMARY = 30;
// The host is expected to be consuming very cycles during this delay
constexpr uint8_t DELAY_SECONDARY = 10;

constexpr uint8_t PIN_DATA = 12, PIN_CLOCK = 13;
constexpr KeyConfig KEYS[] = {
    KeyConfig{
        "Left",
        2,
        0x75, // NumPad 8
    },
    KeyConfig{
        "Right",
        3,
        0x7d, // NumPad 9
    },
    KeyConfig{
        "Dash",
        4,
        0x24, // E
    },
};
constexpr int NUM_KEYS = (int)(sizeof(KEYS) / sizeof(*KEYS));

enum class State : uint8_t
{
    Idle,       // We can send data to the host
    CanReceive, // We should receive data from the host
    Inhibited,  // We should wait until host is ready
};

constexpr uint8_t ACK = 0xFA;
constexpr uint8_t BAT_SUCCESS = 0xAA;

inline int readPin(uint8_t pin)
{
    return (*portInputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin)) != 0 ? HIGH : LOW;
}

inline void outputHigh(uint8_t pin)
{
    *portOutputRegister(digitalPinToPort(pin)) |= digitalPinToBitMask(pin);
}

inline void outputLow(uint8_t pin)
{
    *portOutputRegister(digitalPinToPort(pin)) &= ~digitalPinToBitMask(pin);
}

State readState()
{
    auto data = readPin(PIN_DATA);
    auto clock = readPin(PIN_CLOCK);
    if (data == HIGH && clock == HIGH)
        return State::Idle;
    if (data == LOW && clock == HIGH)
        return State::CanReceive;
    return State::Inhibited;
}

bool oddParity(uint8_t x)
{
    return !__builtin_parity(x);
}

struct LogEntry
{
    unsigned long time;
    const char *message;
    uint8_t byte;
};

RingBuffer<LogEntry, 8> logEntries;

void logInner(const char *message, uint8_t byte)
{
    logEntries.push(LogEntry{
        micros(),
        message,
        byte,
    });
}

inline void log(const char *message, uint8_t byte = 0)
{
    if (SERIAL_DEBUG)
        logInner(message, byte);
}

void sendByte(uint8_t code)
{
    uint8_t bits[11];
    bits[0] = LOW;
    for (int i = 0; i < 8; ++i)
        bits[1 + i] = (code >> i & 1) != 0 ? HIGH : LOW;
    bits[9] = oddParity(code) ? HIGH : LOW;
    bits[10] = HIGH;

    while (readState() != State::Idle)
        ;

    pinMode(PIN_CLOCK, OUTPUT);
    pinMode(PIN_DATA, OUTPUT);

    switch (code)
    {
    case ACK:
        delayMicroseconds(1000);
        break;
    case BAT_SUCCESS:
        delayMicroseconds(10000);
        break;
    default:
        delayMicroseconds(DELAY_PRIMARY);
        break;
    }

    for (int i = 0; i < 11; ++i)
    {
        if (bits[i] == HIGH)
            outputHigh(PIN_DATA);
        else
            outputLow(PIN_DATA);
        delayMicroseconds(DELAY_SECONDARY);

        outputLow(PIN_CLOCK);
        delayMicroseconds(DELAY_PRIMARY);

        outputHigh(PIN_CLOCK);
        delayMicroseconds(DELAY_SECONDARY);
    }

    pinMode(PIN_CLOCK, INPUT_PULLUP);
    pinMode(PIN_DATA, INPUT_PULLUP);

    log("Sent byte", code);
}

uint8_t receiveByte()
{
    while (readState() != State::CanReceive)
        ;
    pinMode(PIN_CLOCK, OUTPUT);

    uint8_t bits[10];
    for (int i = 0; i < 10; ++i)
    {
        outputLow(PIN_CLOCK);
        delayMicroseconds(DELAY_PRIMARY);

        bits[i] = readPin(PIN_DATA);
        outputHigh(PIN_CLOCK);
        delayMicroseconds(DELAY_PRIMARY);
    }

    uint8_t command = 0;
    for (int i = 0; i < 8; ++i)
        command |= bits[i] == HIGH ? 1 << i : 0;

    //The device should bring the Data line low and generate one clock pulse

    pinMode(PIN_DATA, OUTPUT);
    outputLow(PIN_DATA);
    delayMicroseconds(DELAY_SECONDARY);

    outputLow(PIN_CLOCK);
    delayMicroseconds(DELAY_PRIMARY);

    outputHigh(PIN_CLOCK);
    delayMicroseconds(DELAY_SECONDARY);

    outputHigh(PIN_DATA);
    delayMicroseconds(DELAY_PRIMARY);

    pinMode(PIN_DATA, INPUT_PULLUP);
    pinMode(PIN_CLOCK, INPUT_PULLUP);

    if (bits[8] != (oddParity(command) ? HIGH : LOW))
        log("parity error", command);

    if (bits[9] != HIGH)
        log("stop bit error", command);

    log("Received byte", command);

    return command;
}

void executeHostCommand(uint8_t command)
{
    switch (command)
    {
    // 0xFF (Reset) - Respond with "ack" (0xFA) then BAT (Basic Assurance Test) is performed. States are resetted.
    case 0xff:
        sendByte(ACK);
        sendByte(BAT_SUCCESS);
        log("Reset completed");
        break;

    // 0xFE (Resend) - Resend the last sent byte
    // case 0xfe:
    //     sendByte(lastSentByte);
    //     break;

    // 0xF4 (Enable) - Enable sending keys
    case 0xf4:
        sendByte(ACK);
        // Currently ignored
        break;

    // 0xF5 (Disable) - Disable sending keys until enabled
    case 0xf5:
        sendByte(ACK);
        // Currently ignored
        break;

    // 0xF3 (Set Typematic Rate/Delay) - Receive an argument
    case 0xf3:
    {
        sendByte(ACK);
        uint8_t typematic = receiveByte();
        log("Typematic", typematic);
        // Currently ignored
        // Several typematic commands are sent at the startup probably to determine which typematic setting is supported.
        break;
    }

    // 0xF2 (Read ID) - Send 0xAB, 0x83
    case 0xf2:
        sendByte(ACK);
        sendByte(0xAB);
        sendByte(0x83);
        break;

    // 0xED (Set LED State) - Receive an argument
    case 0xed:
    {
        sendByte(ACK);
        uint8_t led = receiveByte();
        log("LED", led);
        break;
    }

    default:
        log("Unknown command", command);
        break;
    }
}

State scanKeysUntilStateChange()
{
    auto state = readState();

    static bool previousKeyStates[NUM_KEYS] = {};

    for (; state == State::Idle; state = readState())
    {
        for (int i = 0; i < NUM_KEYS; ++i)
        {
            bool prev = previousKeyStates[i];
            bool cur = readPin(KEYS[i].pin) == LOW;
            if (prev != cur)
            {
                if (cur)
                {
                    sendByte(KEYS[i].scanCode);
                    previousKeyStates[i] = true;
                    log(KEYS[i].name, 1);
                    return state;
                }
                else
                {
                    sendByte(0xF0);
                    sendByte(KEYS[i].scanCode);
                    previousKeyStates[i] = false;
                    log(KEYS[i].name, 0);
                    return state;
                }
            }
        }
    }

    return state;
}

void setup()
{
    if (SERIAL_DEBUG)
    {
        Serial.begin(9600);
        Serial.println("hello");
    }

    pinMode(PIN_DATA, INPUT_PULLUP);
    pinMode(PIN_CLOCK, INPUT_PULLUP);

    for (int i = 0; i < NUM_KEYS; ++i)
        pinMode(KEYS[i].pin, INPUT_PULLUP);
}

void loop()
{
    auto state = scanKeysUntilStateChange();

    if (state == State::CanReceive)
    {
        auto command = receiveByte();
        executeHostCommand(command);
    }

    while (!logEntries.empty())
    {
        LogEntry &entry = logEntries.pop();
        Serial.print(entry.time);
        Serial.print(" ");
        Serial.print(entry.message);
        if (entry.byte != 0)
        {
            Serial.print(" ");
            Serial.print(entry.byte, HEX);
        }
        Serial.println();
    }
}
