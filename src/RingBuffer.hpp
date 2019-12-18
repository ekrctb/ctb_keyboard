template <typename T, int CAPACITY>
class RingBuffer
{
    T buf[CAPACITY];
    uint8_t next_write = 0;
    uint8_t next_read = 0;

public:
    bool push(T x)
    {
        buf[next_write] = x;
        if (++next_write == CAPACITY)
            next_write = 0;

        if (next_read == next_write)
        {
            if (next_write-- == 0)
                next_write = CAPACITY - 1;
            return false;
        }
        else
        {
            return true;
        }
    }

    bool pop(T *out)
    {
        if (next_read == next_write)
        {
            return false;
        }
        else
        {
            *out = buf[next_read];
            if (++next_read == CAPACITY)
                next_read = 0;
            return true;
        }
    }
};
