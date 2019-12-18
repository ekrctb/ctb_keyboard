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

constexpr uint8_t DELAY_SEND_CLOCK_LOW_START = 40;
constexpr uint8_t DELAY_SEND_CLOCK_LOW = 15;
constexpr uint8_t DELAY_SEND_CLOCK_HIGH = 4;

constexpr uint8_t DELAY_RECEIVE = 30;

constexpr uint8_t PIN_CLOCK = 8, PIN_DATA = 9;
constexpr uint8_t IDLE = 0b11;
constexpr uint8_t CAN_RECEIVE = 0b01;

constexpr uint8_t PINB_START = 8;
constexpr uint8_t PINB_END = 13;
inline uint8_t readClockData()
{
    static_assert(PINB_START <= PIN_CLOCK && PIN_DATA == PIN_CLOCK + 1 && PIN_DATA <= PINB_END, "Rewrite readClockData");
    return PINB >> (PIN_CLOCK - PINB_START) & 0b11;
}
static_assert(PINB_START <= PIN_CLOCK && PIN_CLOCK <= PINB_END && PINB_START <= PIN_DATA && PIN_DATA <= PINB_END, "Rewrite CLOCK_DATA_MASK");
constexpr uint8_t CLOCK_DATA_MASK = 1 << (PIN_CLOCK - PINB_START) | 1 << (PIN_DATA - PINB_START);
inline void writeClockData(uint8_t clock, uint8_t data)
{
    PORTB = (PORTB & ~CLOCK_DATA_MASK) | clock << (PIN_CLOCK - PINB_START) | data << (PIN_DATA - PINB_START);
}
inline void modeClockData(uint8_t clock, uint8_t data)
{
    DDRB = (DDRB & ~CLOCK_DATA_MASK) | clock << (PIN_CLOCK - PINB_START) | data << (PIN_DATA - PINB_START);
}

constexpr uint8_t NUM_KEYS = 4;
constexpr uint8_t KEYS_MASK = (2 << (NUM_KEYS - 1)) - 1;
constexpr uint8_t PIN_KEY_START = 2;

constexpr char const *KEY_NAMES[NUM_KEYS] = {
    "Dash",
    "Left",
    "Right",
    "Space",
};

constexpr uint8_t KEY_SCAN_CODES[NUM_KEYS] = {
    0x24, // E
    0x75, // NumPad 8
    0x7d, // NumPad 9
    0x29, // Space
};

inline uint8_t readAllKeys()
{
    return ~PIND >> PIN_KEY_START & KEYS_MASK; // pin 2, 3, 4, 5
}

constexpr uint8_t ACK = 0xFA;
constexpr uint8_t BAT_SUCCESS = 0xAA;

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
    if (!logEntries.push(LogEntry{
            micros(),
            message,
            byte,
        }))
    {
        Serial.println("Too many logs");
    }
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
    while (readClockData() != IDLE)
        ;

    // 2. Set pins to output mode and calculate port addresses
    modeClockData(OUTPUT, OUTPUT);

    // 3. Send the start bit
    writeClockData(HIGH, LOW);
    delayMicroseconds(DELAY_SEND_CLOCK_HIGH);
    writeClockData(LOW, LOW);

    // The computation here is added to the delay of DELAY_SEND_CLOCK_LOW_START
    uint8_t bits[10];
    for (int i = 0; i < 8; ++i)
        bits[i] = (code >> i & 1) != 0 ? HIGH : LOW;
    bits[8] = oddParity(code) ? HIGH : LOW;
    bits[9] = HIGH;

    delayMicroseconds(DELAY_SEND_CLOCK_LOW_START);

    // 4. Send remaining bits
    for (int i = 0; i < 10; ++i)
    {
        auto bit = bits[i];
        writeClockData(HIGH, bit);
        delayMicroseconds(DELAY_SEND_CLOCK_HIGH);
        writeClockData(LOW, bit);
        delayMicroseconds(DELAY_SEND_CLOCK_LOW);
    }

    // 5. Set pins to input pull-up mode
    writeClockData(HIGH, HIGH);
    modeClockData(INPUT, INPUT);

    logDebug("Sent byte", code);
}

bool receiveByte(uint8_t *outCommand)
{
    while (readClockData() != CAN_RECEIVE)
        ;
    modeClockData(OUTPUT, INPUT);

    uint8_t bits[10];
    for (int i = 0; i < 10; ++i)
    {
        writeClockData(LOW, HIGH);
        delayMicroseconds(DELAY_RECEIVE);

        bits[i] = readClockData() >> 1;
        writeClockData(HIGH, HIGH);
        delayMicroseconds(DELAY_RECEIVE);
    }

    uint8_t command = 0;
    for (int i = 0; i < 8; ++i)
        command |= bits[i] == HIGH ? 1 << i : 0;

    modeClockData(OUTPUT, OUTPUT);
    writeClockData(HIGH, LOW);
    delayMicroseconds(DELAY_RECEIVE);

    writeClockData(LOW, LOW);
    delayMicroseconds(DELAY_RECEIVE);

    writeClockData(HIGH, LOW);
    delayMicroseconds(DELAY_RECEIVE);

    writeClockData(HIGH, HIGH);
    delayMicroseconds(DELAY_RECEIVE);

    modeClockData(INPUT, INPUT);

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

RingBuffer<uint8_t, 16> scanCodeBuffer;

inline void pressKey(int i)
{
    scanCodeBuffer.push(KEY_SCAN_CODES[i]);

    logDebug(KEY_NAMES[i], 1);
}

inline void releaseKey(int i)
{
    scanCodeBuffer.push(0xf0);
    scanCodeBuffer.push(KEY_SCAN_CODES[i]);

    logDebug(KEY_NAMES[i], 0);
}

void sendKeys(uint8_t diff, uint8_t pressed)
{
    for (int i = 0; i < NUM_KEYS; ++i, diff >>= 1, pressed >>= 1)
    {
        if (diff & 1)
        {
            if (pressed & 1)
                pressKey(i);
            else
                releaseKey(i);
        }
    }
}

void scanKeys()
{
    static uint8_t prev = 0;

    uint8_t cur = readAllKeys();
    uint8_t diff = (cur ^ prev);
    if (diff != 0)
    {
        prev = cur;
        sendKeys(diff, cur);
    }
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
    switch (readClockData())
    {
    case CAN_RECEIVE:
    {
        uint8_t command;
        if (receiveByte(&command))
            executeHostCommand(command);
        break;
    }
    case IDLE:
    {
        scanKeys();
        uint8_t code;
        while (readClockData() == IDLE && scanCodeBuffer.pop(&code))
            sendByte(code);
        break;
    }
    }

    if (LOG_LEVEL > LogLevel::NONE)
    {
        LogEntry entry;
        while (logEntries.pop(&entry))
        {
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
}
