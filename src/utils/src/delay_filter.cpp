#include <utils/delay_filter.h>
#include <utils/utils.h>


DelayFilt::DelayFilt()
{
    init(0, 0);
}

void DelayFilt::init(int64_t const& delay_usec, int buf_size)
{
    _delay = delay_usec;
    _pointer = 0;
    _arr.resize(buf_size);
}

DelayFilt::DelayFilt(int64_t const& delay_usec, int buf_size)
{
    init(delay_usec, buf_size);
}

double DelayFilt::process(int64_t const& t_usec, double const& x)
{
    int last = modulo(_pointer + 1, _arr.size());
    int delayed = last;

    for (int i = _pointer; i != last; i = modulo(i - 1, _arr.size()))
    {
        if (_arr[i].t + _delay < t_usec)
        {
            delayed = i;
            break;
        }
    }

    double res = _arr[delayed].x;

    _pointer = modulo(_pointer + 1, _arr.size());
    _arr[_pointer].t = t_usec;
    _arr[_pointer].x = x;

    return res;
}
