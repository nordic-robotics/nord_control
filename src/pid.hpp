#pragma once
 
#include <algorithm>

namespace kontroll
{
    template<class T>
    class pid
    {
        T last_error = 0;
        T last_input = 0;
        T error_sum = 0;
     
        T clamp(T X, T min, T max)
        {
            return std::min(std::max(X, min), max);
        }
     
    public:
        pid(T k_p = 0, T k_i = 0, T k_d = 0)
            : k_p(k_p), k_i(k_i), k_d(k_d) { };
         
        T k_p, k_i, k_d;
        T min = 0;
        T max = 0;

        T operator()(T input, T target, T delta_time)
        {
            return compute(input, target, delta_time);
        }
     
        T compute(T input, T target, T delta_time)
        {
            auto error = target - input;
            error_sum += error * delta_time;
            auto error_derivative = (error - last_error) / delta_time;
           // auto input_derivative = (input - last_input) / delta_time;
     
            last_error = error;
            last_input = input;
     
            T output = k_p * error + k_i * error_sum + k_d * error_derivative;
            if (min != max)
            {
                error_sum = clamp(error_sum, min, max);
                output = clamp(output, min, max);
            }
     
            return output;
        }
    };
}