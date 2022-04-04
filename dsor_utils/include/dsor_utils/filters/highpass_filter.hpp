/**
 * Authors:
 *      Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Maintained by: Jimmy van den Berg (vandenberg.jimmy@gmail.com)
 * Last Update: 08/02/2018
 * Github: https://github.com/jimmyberg/DigitalFilters
 * License: GNU
 * Brief: Multiple classes for low pass filters of different orders, inherited from the DigitalFilter class of moving filters. 
 *        A low-pass filter is a filter that passes signals with a frequency lower than a selected cutoff frequency and
 *        attenuates signals with frequencies higher than the cutoff frequency.
 */

#pragma once

#include "digital_filter.hpp"

/**
 * @brief      Class for high pass filter using bilinear transform.
 */
class HighPassFilter : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  idt     Sample time for the low pass filter
         * @param[in]  omega_c  Cutoff angular frequency @f$ \omega_c = 2 pi f_c@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        HighPassFilter(float idt, float omega_c):
            amplFac(1/((idt * omega_c / 2) + 1)),
            y1c((idt * omega_c / 2) - 1),
            dt(idt) {
                if(omega_c < idt){
                    throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
                }
            }

        /**
         * @brief      Update function to push new value into the low pass filter
         *
         * @param[in]  newValue  The new value after dt time
         *
         * @return     The new output value
         */
        float update(float newValue) final{
            // Note that output before assignment equals y1 being y[n-1]
            output = amplFac * (newValue - x1 - output * y1c);
            x1 = newValue;
            return output;
        }

        /**
         * @brief      Gets the output.
         *
         * @return     The output.
         */
        float getOutput() final{ return output; }

        /**
         * @brief      Force the output to a desired value
         *
         *             This can be useful when the output needs to be forced in case
         *             of extreme inputs or such
         *
         * @param[in]  newOutput  The new output
         */
        void configOutput(float newOutput) { output = newOutput; }

        const float* outputPointer() { return &output; }

    private:

        const float amplFac; // one time calculation constant
        const float y1c; // one time calculation constant
        const float dt;
        float x1 = 0;
        float output = 0;

};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the bilinear transform.
 */
class HighPassFilter3 : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  idt     Sample time for the low pass filter
         * @param[in]  omega_c  Cutoff angular frequency @f$ \omega_c = 2 pi f_c@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        HighPassFilter3(float sampleTime, float omega_c, float ioutput = 0):
            xc { 8, -24, 24, -8 },
            yc {
                1 / (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8),
                    3 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c - 24,
                    3 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c + 24,
                    1 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c - 8
            }
            {
                if(omega_c < sampleTime){
                    throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
                }
            }

        /**
         * @brief      Update function to push new value into the low pass filter
         *
         * @param[in]  newValue  The new value after dt time
         *
         * @return     The new output value
         */
        float update(float newValue) final{

            x.push(newValue);
            float y0 = 0;
            const float* floatP = xc;
            for (auto it = x.rbegin(); it != x.rend(); it++) {
                y0 += *it * *floatP++;
            }

            floatP = yc + 1;
            
            for (auto it = y.rbegin(); it != y.rend(); it++) {
                y0 -= *it * *floatP++;
            }

            return y.push(yc[0] * y0);

        }

        /**
         * @brief      Gets the output.
         *
         * @return     The output.
         */
        float getOutput() final{ return y.get(0); }

    private:

        const float xc[4];
        const float yc[4];

        CircularDelay<float, 3> y;
        CircularDelay<float, 4> x;

};