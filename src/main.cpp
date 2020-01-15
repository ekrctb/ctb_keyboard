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

// Send many key codes to see if keys are skipped
constexpr bool DEBUG_STRESS_TEST = false;
// Use one-byte "Set 1" representation of the release codes.
// Depending on the (undocumented?/undefined?) behavior of a 8042 controller
// for not translating such bytes.
constexpr bool USE_UNTRANSLATED_SET1_CODE_FOR_RELEASE = true;

constexpr uint8_t DELAY_SEND_STATE_CHECK = 4;
constexpr uint8_t DELAY_SEND_DATA_TO_FALLING = 5;
constexpr uint8_t DELAY_SEND_LOW = 12;
constexpr uint8_t DELAY_SEND_LOW_START_BIT = 30;

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
    PORTB = clock << (PIN_CLOCK - PINB_START) | data << (PIN_DATA - PINB_START);
}
inline void modeClockData(uint8_t clock, uint8_t data)
{
    DDRB = clock << (PIN_CLOCK - PINB_START) | data << (PIN_DATA - PINB_START);
}

constexpr uint8_t NUM_KEYS = 3;
constexpr uint8_t KEYS_MASK = (2 << (NUM_KEYS - 1)) - 1;
constexpr uint8_t PIN_KEY_START = 2;
constexpr uint8_t KEY_MINIMUM_MILLISECONDS = 10;
constexpr bool PIN_KEY_PULLUP = true;
constexpr bool PIN_KEY_COMPLEMENT = true;

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

constexpr uint8_t KEY_SET1_RELEASE_CODES[NUM_KEYS] = {
    0x92, // E
    0xc8, // NumPad 8
    0xc9, // NumPad 9
};

inline uint8_t readAllKeys()
{
    return (PIN_KEY_COMPLEMENT ? ~PIND : PIND) >> PIN_KEY_START & KEYS_MASK;
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

bool trySendByte(uint8_t code)
{
    uint8_t bits[10];
    auto computeBits = [code, &bits]() {
        for (int i = 0; i < 8; ++i)
            bits[i] = (code >> i & 1) != 0 ? HIGH : LOW;
        bits[8] = oddParity(code) ? HIGH : LOW;
        bits[9] = HIGH;
    };

    // 1. Assume it is an idle state

    // 2. Set pins to output mode
    modeClockData(OUTPUT, OUTPUT);

    // 3. Send the start bit
    writeClockData(HIGH, LOW);
    delayMicroseconds(DELAY_SEND_DATA_TO_FALLING);

    writeClockData(LOW, LOW);
    computeBits(); // Compute this during the waiting time of clock low
    delayMicroseconds(DELAY_SEND_LOW_START_BIT);

    writeClockData(HIGH, LOW);
    modeClockData(INPUT, OUTPUT);
    delayMicroseconds(DELAY_SEND_STATE_CHECK);

    // 4. Send the remaining bits
    for (int i = 0; i < 10; ++i)
    {
        // 5. If the communication is inhibited before sending 11th clock pulse,
        //    abort the current communication.
        if ((readClockData() & 1) == 0)
        {
            modeClockData(INPUT, INPUT);
            writeClockData(HIGH, HIGH);
            if (DEBUG_STRESS_TEST)
                Serial.print('i');
            logDebug("Inhibited during send", code);
            return false;
        }
        modeClockData(OUTPUT, OUTPUT);

        uint8_t bit = bits[i];

        writeClockData(HIGH, bit);
        delayMicroseconds(DELAY_SEND_DATA_TO_FALLING);

        writeClockData(LOW, bit);
        delayMicroseconds(DELAY_SEND_LOW);

        writeClockData(HIGH, bit);
        modeClockData(INPUT, OUTPUT);
        delayMicroseconds(DELAY_SEND_STATE_CHECK);
    }

    // 6. Set pins to input mode
    modeClockData(INPUT, INPUT);

    logDebug("Sent byte", code);
    return true;
}

// Loop until the byte chunk is sent completely
inline void sendByteChunk(uint8_t code0, uint8_t code1 = 0)
{
    while (1)
    {
        while (readClockData() != IDLE)
            ;
        if (!trySendByte(code0))
            continue;
        if (code1 != 0)
        {
            while (readClockData() != IDLE)
                ;
            if (!trySendByte(code1))
                continue;
        }
        break;
    }
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
        sendByteChunk(ACK);
        sendByteChunk(BAT_SUCCESS);
        logInfo("Reset completed");
        break;

    // 0xFE (Resend) - Resend the last sent byte
    // case 0xfe:
    //     sendByteChunk(lastSentByte);
    //     break;

    // 0xF4 (Enable) - Enable sending keys
    case 0xf4:
        sendByteChunk(ACK);
        // Currently ignored
        break;

    // 0xF5 (Disable) - Disable sending keys until enabled
    case 0xf5:
        sendByteChunk(ACK);
        // Currently ignored
        break;

    // 0xF3 (Set Typematic Rate/Delay) - Receive an argument
    case 0xf3:
    {
        sendByteChunk(ACK);
        uint8_t typematic;
        if (!receiveByte(&typematic))
            break;
        sendByteChunk(ACK);
        logInfo("Typematic", typematic);
        // Currently ignored
        // Several typematic commands are sent at the startup probably to determine which typematic setting is supported.
        break;
    }

    // 0xF2 (Read ID) - Send 0xAB, 0x83
    case 0xf2:
        sendByteChunk(ACK);
        sendByteChunk(0xAB, 0x83);
        break;

    // 0xED (Set LED State) - Receive an argument
    case 0xed:
    {
        sendByteChunk(ACK);
        uint8_t led;
        if (!receiveByte(&led))
            break;
        sendByteChunk(ACK);
        logInfo("LED", led);
        break;
    }

    default:
        logNotice("Unknown command", command);
        break;
    }
}

uint32_t timestamps[NUM_KEYS];
uint8_t currentKeyState = 0;
uint8_t sendingKeys = 0;

inline bool checkTimestamp(int i)
{
    auto time = millis();
    if (time - timestamps[i] < KEY_MINIMUM_MILLISECONDS)
    {
        logDebug("Bounced key state change ignored", i);
        return false;
    }
    else
    {
        timestamps[i] = time;
        return true;
    }
}

inline void onKeyPressed(int i)
{
    if (!checkTimestamp(i))
        return;

    sendingKeys |= 1 << i;
    currentKeyState |= 1 << i;

    logDebug(KEY_NAMES[i], 1);
}

inline void onKeyReleased(int i)
{
    if (!checkTimestamp(i))
        return;

    sendingKeys |= 1 << i;
    currentKeyState &= ~(1 << i);

    logDebug(KEY_NAMES[i], 0);
}

void processKeyStateChanges(uint8_t diff, uint8_t pressed)
{
    for (int i = 0; i < NUM_KEYS; ++i, diff >>= 1, pressed >>= 1)
    {
        if (diff & 1)
        {
            if (pressed & 1)
                onKeyPressed(i);
            else
                onKeyReleased(i);
        }
    }
}

void scanKeys()
{
    uint8_t cur = readAllKeys();
    uint8_t diff = (cur ^ currentKeyState);
    if (diff != 0)
        processKeyStateChanges(diff, cur);
}

bool sendScanCode()
{
    if (sendingKeys == 0)
        return false;

    static uint8_t lastIndex = NUM_KEYS - 1;
    uint8_t i = lastIndex + 1;
    uint8_t mask;
    while (1)
    {
        if (i == NUM_KEYS)
            i = 0;
        mask = 1 << i;
        if (sendingKeys & mask)
            break;
        ++i;
    }
    lastIndex = i;

    if ((currentKeyState & mask) != 0)
        sendByteChunk(KEY_SCAN_CODES[i]);
    else if (USE_UNTRANSLATED_SET1_CODE_FOR_RELEASE)
        sendByteChunk(KEY_SET1_RELEASE_CODES[i]);
    else
        sendByteChunk(0xf0, KEY_SCAN_CODES[i]);

    sendingKeys &= ~mask;

    return true;
}

void debugStressTest()
{
    const uint8_t CODES[] = {
        0x45,
        0x16,
        0x1e,
        0x26,
        0x25,
        0x2e,
        0x36,
        0x3d,
        0x3e,
        0x46,
        0x1c,
        0x32,
        0x21,
        0x23,
        0x24,
        0x2b,
        0x34,
        0x33,
        0x43,
        0x3b,
        0x42,
        0x4b,
        0x3a,
        0x31,
        0x44,
        0x4d,
        0x15,
        0x2d,
        0x1b,
        0x2c,
        0x3c,
        0x2a,
        0x1d,
        0x22,
        0x35,
        0x1a,
        0x5a,
    };

    auto keys = readAllKeys();
    if ((keys & 0b10) == 0)
        return;

    static uint8_t nextIndex = 0;
    uint8_t i = nextIndex;

    if (keys & 0b100)
    {
        sendByteChunk(0xf0, CODES[i]);
    }
    else
    {
        sendByteChunk(CODES[i]);
    }

    if (random() >= RANDOM_MAX / 2)
    {
        delayMicroseconds(1000 + random() % 1000);
    }

    if (++nextIndex == sizeof(CODES))
    {
        nextIndex = 0;
        if (keys & 0b001)
        {
            for (auto code : CODES)
            {
                sendByteChunk(0xf0, code);
            }
        }
    }
}

void setup()
{
    if (LOG_LEVEL > LogLevel::NONE)
    {
        Serial.begin(1000000);
        Serial.println("ctb_keyboard");
        Serial.println("Built at " __DATE__ " " __TIME__);
        Serial.println();

#define PRINT_VAR(var) Serial.print(#var " = "), Serial.println(var)
        PRINT_VAR((int)LOG_LEVEL);
        PRINT_VAR(DEBUG_STRESS_TEST);
        PRINT_VAR(USE_UNTRANSLATED_SET1_CODE_FOR_RELEASE);
        Serial.println();
        PRINT_VAR(PIN_CLOCK);
        PRINT_VAR(PIN_DATA);
        PRINT_VAR(PIN_KEY_START);
        PRINT_VAR(NUM_KEYS);
        Serial.println();
#undef PRINT_VAR
    }

    pinMode(PIN_DATA, INPUT_PULLUP);
    pinMode(PIN_CLOCK, INPUT_PULLUP);

    for (int i = 0; i < NUM_KEYS; ++i)
        pinMode(PIN_KEY_START + i, PIN_KEY_PULLUP ? INPUT_PULLUP : INPUT);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    scanKeys();

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
        if (DEBUG_STRESS_TEST)
        {
            debugStressTest();
            break;
        }

        if (sendScanCode())
        {
            // ~ 550us until become idle again
        }
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
