#pragma once
#include <Arduino.h>

class Debouncer
{
    using History = uint8_t;
    static constexpr History ALL_ONE = History(-1);

public:
    enum class State : uint8_t
    {
        RELEASED,
        PRESSED_BOUNCING,
        PRESSED,
        RELEASED_BOUNCING,
    };

    enum class Result
    {
        None,
        Released,
        Pressed,
    };

    void init(bool pressed)
    {
        if (!pressed)
        {
            state = State::RELEASED;
            history = 0;
        }
        else
        {
            state = State::PRESSED;
            history = ALL_ONE;
        }
    }

    Result push(bool pressed)
    {
        history <<= 1;
        history |= pressed ? 1 : 0;
        Result result = Result::None;
        switch (state)
        {
        case State::RELEASED:
            if (pressed)
            {
                state = State::PRESSED_BOUNCING;
                result = Result::Pressed;
            }
            break;
        case State::PRESSED_BOUNCING:
            if (history == ALL_ONE)
            {
                state = State::PRESSED;
            }
            break;
        case State::PRESSED:
            if (!pressed)
            {
                state = State::RELEASED_BOUNCING;
                result = Result::Released;
            }
            break;
        case State::RELEASED_BOUNCING:
            if (history == 0)
            {
                state = State::RELEASED;
            }
            break;
        }
        return result;
    }

private:
    State state;
    uint8_t history;
};
