/**
 * Authors:
 *      Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Maintained by: Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Last Update: 08/02/2018
 * Github: https://github.com/jimmyberg/DigitalFilters
 * License: GNU
 * Brief: Class for a moving average filter, inherited from the DigitalFilter class of moving filters. 
 *        A moving average filter is operates by averaging a number of points from the input signal to
 *        produce each point in the output signal.  
 */
#pragma once

#include "digital_filter.hpp"

template<size_t size>
class MovingAvarageFilter{

    public:

        /**
         * @brief      Update function to push new value into the moving average filter.
         *
         * @param[in]  input The new value after dt time
         *
         * @return     The new output value
         */
        double update(double input) {
            input *= 1000;
            output += int64_t(input) - *buffer.rbegin();
            buffer.push(input);
            return double(output) / (1000);
        }

    private:

        int64_t output = 0;
        CircularDelay<int64_t, size> buffer;

};