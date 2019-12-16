template <typename T, int CAPACITY>
class RingBuffer
{
    T buf[CAPACITY];
    uint8_t next_write = 0;
    uint8_t next_read = 0;

public:
    void push(T x)
    {
        buf[next_write] = x;
        if (++next_write == CAPACITY)
            next_write = 0;

        if (next_read == next_write)
        {
            Serial.println("Ring buffer capacity overflow");
            abort();
        }
    }

    bool empty() const
    {
        return next_read == next_write;
    }

    T &pop()
    {
        if (next_read == next_write)
        {
            Serial.println("Popped an empty ring buffer");
            abort();
        }

        T &x = buf[next_read];
        if (++next_read == CAPACITY)
            next_read = 0;
        return x;
    }
};
