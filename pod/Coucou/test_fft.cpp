#include <iostream>
#include <random>

#include "fft.h"

int main()
{
    std::vector<std::complex<double>> xxs;
    xxs.resize(2048);

    auto   rng   = std::mt19937(0x12457896);
    auto   dist  = std::normal_distribution<double>(0, 1e-2);
    size_t count = 0;
    for(auto& xx : xxs)
    {
        const auto tt = static_cast<double>(count++);

        xx = 0;
        xx += std::complex<double>(dist(rng), dist(rng));
        xx += sin(2 * M_PI * tt / 128);
        xx += 1;
    }

    auto yys = xxs;
    coucou::fast_fourier(yys.data(), xxs.size());

    using std::cout;
    using std::endl;

    cout << "{" << endl;

    {
        bool first = true;
        cout << "\"xxs\":[" << endl;
        for(const auto& xx : xxs)
        {
            if(!first)
                cout << "," << endl;
            cout << "[" << xx.real() << "," << xx.imag() << "]";
            first = false;
        }
        cout << "]," << endl;
    }

    {
        bool first = true;
        cout << "\"yys\":[" << endl;
        for(const auto& yy : yys)
        {
            if(!first)
                cout << "," << endl;
            cout << "[" << yy.real() << "," << yy.imag() << "]";
            first = false;
        }
        cout << "]" << endl;
    }

    cout << "}" << endl;

    return 0;
}