/**
 * Authors:
 *      Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Maintained by: Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Last Update: 08/02/2018
 * Github: https://github.com/jimmyberg/DigitalFilters
 * License: GNU
 * Brief: Class for a differentiator, inherited from the DigitalFilter class of moving filters. 
 *        A differentiator is a filter that is designed such that the output is approximately
 *        directly proportionalto the rate of change (the time derivative) of the input. 
 */

#pragma once

#include "digital_filter.hpp"

/**
 * @brief      Class for the differentiator.
 */
template<typename T>
class Differentiator : public DigitalFilter<T> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  sampleTime     Sample time for the low pass filter
         */
        Differentiator(T sampleTime): sampleTime(sampleTime) {}

        /**
         * @brief      Update function to push new value into the differentiator.
         *
         * @param[in]  input The new value after dt time
         *
         * @return     The new output value
         */
        T update(T input){
            y = (input - x1) / sampleTime;
            x1 = input;
            
            return y;
        }

        /**
         * @brief      Gets the output.
         *
         * @return     The output.
         */
        T getOutput(){ return y; }

    private:

        const T sampleTime;
        T y = 0;
        T x1 = 0;
};