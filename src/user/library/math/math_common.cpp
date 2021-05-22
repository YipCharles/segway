#include "math_common.h"

template <typename T>
T constrain_value(const T amt, const T low, const T high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (__ARM_isnan(amt)) {
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
        return high;
    }

    return amt;
}

template int constrain_value<int>(const int amt, const int low, const int high);
template long constrain_value<long>(const long amt, const long low, const long high);
template long long constrain_value<long long>(const long long amt, const long long low, const long long high);
template short constrain_value<short>(const short amt, const short low, const short high);
template float constrain_value<float>(const float amt, const float low, const float high);
template double constrain_value<double>(const double amt, const double low, const double high);

