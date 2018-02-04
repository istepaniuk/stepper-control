static const long HIGHEST_SQRT_RESULT_BIT = 0x40000000L;

static unsigned int sqrt(unsigned int x)
{
    register unsigned int result;
    register unsigned int q2scan_bit;
    register unsigned char flag;

    result = 0;
    q2scan_bit = HIGHEST_SQRT_RESULT_BIT;

    do {
        if ((result + q2scan_bit) <= x) {
            x -= result + q2scan_bit;
            flag = 1;
        }
        else {
            flag = 0;
        }

        result >>= 1;

        if (flag) {
            result += q2scan_bit;
        }

    }
    while (q2scan_bit >>= 2);

    if (result < x) {
        return result + 1;
    }

    return result;
}