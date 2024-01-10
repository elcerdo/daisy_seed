#include "fft.h"

#include <cstring>
#include <cmath>

/* ------ RECURSIVE IMPLEMENTATION ------- */

namespace coucou::detail
{

void split(std::complex<double>* inputs, int N);

}

/* Moves all even indices to 1st half
   odd indices to 2nd half of inputs. 
   inputs : an array of complex numbers
   N : length of inputs array  */
void coucou::detail::split(std::complex<double>* inputs, int N)
{
    using std::complex;

    complex<double>* even = new complex<double>[N / 2];
    complex<double>* odd  = new complex<double>[N / 2];
    int              ei   = 0;
    int              oi   = 0;

    for(int j = 0; j < N; j++)
    {
        if((j % 2) == 0)
            even[ei++] = inputs[j];
        else
            odd[oi++] = inputs[j];
    }

    int size = N / 2;
    memcpy(inputs, even, sizeof(complex<double>) * size);
    memcpy(inputs + size, odd, sizeof(complex<double>) * size);

    delete[] even;
    delete[] odd;
}

/* This function computes the fast fourier 
   transform of a list of complex numbers 
   of length N.
   x : input array of complex number that represent
       a sampled function amplitude 
   N : the length of x must be a power of 2 
*/
void coucou::fast_fourier(std::complex<double>* x, double N)
{
    using std::complex;
    using namespace std::complex_literals;

    /* base of recursion */
    if(N == 1)
        return;

    /* no rounding needed if N is base 2 */
    int n = N / 2;

    /* set primitive root of unity */
    // complex<double> w  = 1;

    /* move odd and evened indexed to each half
	   of array x */
    detail::split(x, 2 * n);

    /* even and odd */
    fast_fourier(x, n);
    /* pass pointer starting
	at the n/2th element */
    fast_fourier(x + n, n);

    complex<double> even(0, 0);
    complex<double> odd(0, 0);

    for(int k = 0; k < n; k++)
    {
        /* code */
        even = x[k];
        odd  = x[k + n]; /* k + N/2 */

        complex<double> w = exp(2 * M_PI * k * 1i / N);
        x[k]              = even + w * odd;
        x[k + n]          = even - w * odd;
    }
}