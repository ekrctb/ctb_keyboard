#include <Arduino.h>
#include "RingBuffer.hpp"

enum class LogLevel
{
    NONE = 0,
    NOTICE = 1,
    INFO = 2,
    DEBUG = 3,
};

constexpr LogLevel LOG_LEVEL = LogLevel::INFO;

constexpr uint8_t DELAY_SEND_CLOCK_LOW_START = 30;
constexpr uint8_t DELAY_SEND_CLOCK_LOW = 15;

constexpr uint8_t DELAY_RECEIVE = 30;

constexpr uint8_t PIN_DATA = 8, PIN_CLOCK = 9;

constexpr uint8_t NUM_KEYS = 3;
constexpr uint8_t KEYS_MASK = (2 << (NUM_KEYS - 1)) - 1;
constexpr uint8_t PIN_KEY_START = 2;

constexpr char const *KEY_NAMES[NUM_KEYS] = {
    "Dash",
    "Left",
    "Right",
};

constexpr uint8_t KEY_SCAN_CODES[NUM_KEYS] = {
    0x24, // E
    0x75, // NumPad 8
    0x7d, // NumPad 9
};

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

inline State readState()
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

inline void logDebug(const char *message, uint8_t byte = 0)
{
    if (LOG_LEVEL >= LogLevel::DEBUG)
        logInner(message, byte);
}

inline void logInfo(const char *message, uint8_t byte = 0)
{
    if (LOG_LEVEL >= LogLevel::INFO)
        logInner(message, byte);
}

inline void logNotice(const char *message, uint8_t byte = 0)
{
    if (LOG_LEVEL >= LogLevel::NOTICE)
        logInner(message, byte);
}

void sendByte(uint8_t code)
{
    // 1. Wait for idle state
    while (readState() != State::Idle)
        ;

    // 2. Set pins to output mode and calculate port addresses [8us]
    pinMode(PIN_CLOCK, OUTPUT);
    pinMode(PIN_DATA, OUTPUT);

    // Clock output is optimized but data output is not optimized
    // as some delay caused by the computation seems critical for the timing
    auto outClock = portOutputRegister(digitalPinToPort(PIN_CLOCK));
    auto maskClock = digitalPinToBitMask(PIN_CLOCK);

    // 3. Send the start bit [DELAY + 22 us]
    outputLow(PIN_DATA);
    *outClock &= ~maskClock;

    uint8_t bits[10];
    for (int i = 0; i < 8; ++i)
        bits[i] = (code >> i & 1) != 0 ? HIGH : LOW;
    bits[8] = oddParity(code) ? HIGH : LOW;
    bits[9] = HIGH;

    delayMicroseconds(DELAY_SEND_CLOCK_LOW_START);

    // 4. Send remaining bits [10 * DELAY + 30 us]
    for (int i = 0; i < 10; ++i)
    {
        *outClock |= maskClock;

        if (bits[i] == HIGH)
            outputHigh(PIN_DATA);
        else
            outputLow(PIN_DATA);

        *outClock &= ~maskClock;
        delayMicroseconds(DELAY_SEND_CLOCK_LOW);
    }

    // 5. Set pins to input mode [8us]

    pinMode(PIN_CLOCK, INPUT_PULLUP);
    pinMode(PIN_DATA, INPUT_PULLUP);

    logDebug("Sent byte", code);
}

bool receiveByte(uint8_t *outCommand)
{
    while (readState() != State::CanReceive)
        ;
    pinMode(PIN_CLOCK, OUTPUT);

    uint8_t bits[10];
    for (int i = 0; i < 10; ++i)
    {
        outputLow(PIN_CLOCK);
        delayMicroseconds(DELAY_RECEIVE);

        bits[i] = readPin(PIN_DATA);
        outputHigh(PIN_CLOCK);
        delayMicroseconds(DELAY_RECEIVE);
    }

    uint8_t command = 0;
    for (int i = 0; i < 8; ++i)
        command |= bits[i] == HIGH ? 1 << i : 0;

    //The device should bring the Data line low and generate one clock pulse

    pinMode(PIN_DATA, OUTPUT);
    outputLow(PIN_DATA);
    delayMicroseconds(DELAY_RECEIVE);

    outputLow(PIN_CLOCK);
    delayMicroseconds(DELAY_RECEIVE);

    outputHigh(PIN_CLOCK);
    delayMicroseconds(DELAY_RECEIVE);

    outputHigh(PIN_DATA);
    delayMicroseconds(DELAY_RECEIVE);

    pinMode(PIN_DATA, INPUT_PULLUP);
    pinMode(PIN_CLOCK, INPUT_PULLUP);

    *outCommand = command;

    if (bits[8] != (oddParity(command) ? HIGH : LOW))
    {
        logInfo("Parity error", command);
        return false;
    }

    if (bits[9] != HIGH)
    {
        logInfo("Stop bit error", command);
        return false;
    }

    logDebug("Received byte", command);

    return true;
}

void executeHostCommand(uint8_t command)
{
    switch (command)
    {
    // 0xFF (Reset) - Respond with "ack" (0xFA) then BAT (Basic Assurance Test) is performed. States are resetted.
    case 0xff:
        sendByte(ACK);
        sendByte(BAT_SUCCESS);
        logInfo("Reset completed");
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
        uint8_t typematic;
        if (!receiveByte(&typematic))
            break;
        logInfo("Typematic", typematic);
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
        uint8_t led;
        if (!receiveByte(&led))
            break;
        logInfo("LED", led);
        break;
    }

    default:
        logNotice("Unknown command", command);
        break;
    }
}

inline void pressKey(int i)
{
    sendByte(KEY_SCAN_CODES[i]);

    logDebug(KEY_NAMES[i], 1);
}

inline void releaseKey(int i)
{
    sendByte(0xf0);
    sendByte(KEY_SCAN_CODES[i]);

    logDebug(KEY_NAMES[i], 0);
}

inline uint8_t readAllKeys()
{
    static_assert(PIN_KEY_START + NUM_KEYS <= 8, "Rewrite key input logic");
    return ~PIND >> PIN_KEY_START & KEYS_MASK;
}

State scanKeysUntilStateChange()
{
    static uint8_t prev = 0;
    static uint8_t resend = 0;

    auto ps2State = readState();

    for (; ps2State == State::Idle; ps2State = readState())
    {
        uint8_t cur = readAllKeys();
        uint8_t diff = (cur ^ prev) | resend;
        if (diff == 0)
            continue;
        prev = cur;
        resend ^= diff; // Send a key press/release twice for a reliability

        for (int i = 0; i < NUM_KEYS; ++i, diff >>= 1, cur >>= 1)
        {
            if (!(diff & 1))
                continue;

            if (cur & 1)
                pressKey(i);
            else
                releaseKey(i);
        }

        break;
    }

    return ps2State;
}

void setup()
{
    if (LOG_LEVEL > LogLevel::NONE)
    {
        Serial.begin(9600);
        Serial.println("hello");
    }

    pinMode(PIN_DATA, INPUT_PULLUP);
    pinMode(PIN_CLOCK, INPUT_PULLUP);

    for (int i = 0; i < NUM_KEYS; ++i)
        pinMode(PIN_KEY_START + i, INPUT_PULLUP);
}

void loop()
{
    auto state = scanKeysUntilStateChange();

    if (state == State::CanReceive)
    {
        uint8_t command;
        if (receiveByte(&command))
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
