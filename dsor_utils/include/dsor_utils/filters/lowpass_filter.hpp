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
 * @brief      Class for a low pass filter.
 *
 *             Design to be a first order Butterworth low pass filter.
 *             Transformation done using the matched-Z-transform method
 */
class LowPassFilter : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  idt     Sample time for the low pass filter
         * @param[in]  omega_c  Cutoff angular frequency @f$ \omega_c = 2 pi f_c@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        LowPassFilter(float idt, float omega_c, float ioutput = 0):
            epow(exp(-idt * omega_c)), dt(idt), output(ioutput) {

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
        float update(float newValue) final{ return output = (output-newValue) * epow + newValue; }

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
        void configOutput(float newOutput){ output = newOutput; }

        const float* outputPointer(){return &output;}

    private:

        const float epow; /// one time calculation constant
        const float dt;
        float output;

};

/**
 * @brief      Class for a 2nd order low pass filter.
	*
 *             Design to be a 2nd order Butterworth low pass filter.
 *             Transformation done using the bilinear transform method
 */
class LowPassFilter2 : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  dt     Sample time for the low pass filter
         * @param[in]  tau_c  The time constant for the filter @f$ \tau_c = \frac{1}{2 pi f_c}@f$, where @f$ f_c @f$ is the cutoff frequency
         */
        LowPassFilter2(float dt, float tau_c, float ioutput = 0):
            yc {
                -2 * (tps::pow(dt, 2) - 4 * tps::pow(tau_c, 2)) / (tps::pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * tps::pow(tau_c, 2)),
                (-tps::pow(dt, 2) + 2 * sqrt(2) * tau_c * dt - 4 * tps::pow(tau_c, 2)) / (tps::pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * tps::pow(tau_c, 2))
            },
            xc {
                tps::pow(dt, 2) / (tps::pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * tps::pow(tau_c, 2)),
                2 * tps::pow(dt, 2) / (tps::pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * tps::pow(tau_c, 2)),
                tps::pow(dt, 2) / (tps::pow(dt, 2) + 2 * sqrt(2) * tau_c * dt + 4 * tps::pow(tau_c, 2))
            }
            {
                if(tau_c < M_PI * dt) {
                    throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
                }
            }

        /**
         * @brief      Update function to push new value into the filter
         *
         * @param[in]  newValue  The new value after dt time
         *
         * @return     The new output value
         */
        float update(float newValue) final {
            x.push(newValue);
            float output = 0;

            for (int i = 0; i < 2; ++i) {
                output += y.get(i) * yc[i];
            }

            for (int i = 0; i < 3; ++i) {
                output += x.get(i) * xc[i];
            }

            return y.push(output);
        }

        /**
         * @brief      Gets the output.
         *
         * @return     The output.
         */
        float getOutput() final{ return y.get(0); }

        /**
         * @brief      Force the output to a desired value
         *
         *             This can be useful when the output needs to be forced in case
         *             of extreme inputs or such
         *
         * @param[in]  newOutput  The new output
         */
        void configOutput(float newOutput) {
            for(auto& it : x) {
                it = newOutput;
            }

            for(auto& it : y) {
                it = newOutput;
            }
        }

    private:

        const float yc[2];
        const float xc[3];

        CircularDelay<float, 2> y;
        CircularDelay<float, 3> x;

};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the bilinear transform.
 */
class LowPassFilter3 : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  sampleTime   Sample time for the low pass filter
         * @param[in]  omega_c      Cutoff angular frequency @f$ \omega_c = 2 pi f_c@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        LowPassFilter3(float sampleTime, float omega_c, float ioutput = 0):
            yc {
                1
                ,
                (float)((3 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c - 24)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
                ,
                (float)((3 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) - 8 * sampleTime * omega_c + 24)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
                ,
                (float)((1 * tps::pow(sampleTime * omega_c, 3) - 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c - 8)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
            },
            xc {
                (float)(1 * tps::pow(sampleTime * omega_c, 3)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
                ,
                (float)(3 * tps::pow(sampleTime * omega_c, 3)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
                ,
                (float)(3 * tps::pow(sampleTime * omega_c, 3)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
                ,
                (float)(1 * tps::pow(sampleTime * omega_c, 3)
                /
                (1 * tps::pow(sampleTime * omega_c, 3) + 4 * tps::pow(sampleTime * omega_c, 2) + 8 * sampleTime * omega_c + 8))
            }
            {
                if(omega_c < sampleTime){
                    throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
                }
            }

        /**
         * @brief      Update function to push new value into the filter
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

        const float yc[4];
        const float xc[4];

        CircularDelay<float, 3> y;
        CircularDelay<float, 4> x;

};

/**
 * @brief      Class for third order low pass filter. This is designed using
 *             the matched Z transform.
 */
class LowPassFilter3MatchedZ : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  sampleTime   Sample time for the low pass filter
         * @param[in]  omega_c      Cutoff angular frequency @f$ \omega_c = 2 pi f_c@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        LowPassFilter3MatchedZ(float sampleTime, float omega_c):
            amplFac(-(2*(expl(3 * omega_c * sampleTime) - expl(2 * omega_c * sampleTime))*cosl(sqrtl(3) * omega_c * sampleTime / 2) - expl(7 * omega_c * sampleTime / 2) + expl(3 * omega_c * sampleTime / 2))*expl(-7 * omega_c * sampleTime / 2)),
            yc {
                (float)(-(2 * cosl(sqrtl(3) * omega_c * sampleTime / 2) * expl(omega_c * sampleTime * 5 / 2) + expl(2 * omega_c * sampleTime)) * expl(-3 * omega_c * sampleTime))
                ,
                (float)((2 * cosl(sqrtl(3) * omega_c * sampleTime / 2) * expl(omega_c * sampleTime * 3 / 2) + expl(2 * omega_c * sampleTime)) * expl(-3 * omega_c * sampleTime))
                ,
                (float)(-expl(-2 * omega_c * sampleTime))
            }
            {
                if(omega_c / (2 * M_PI) < sampleTime){
                    throw std::domain_error("LowPassFilter3MatchedZ constructor error: tua_c is smaller than the sample time dt.");
                }
            }

        /**
         * @brief      Update function to push new value into the filter.
         *
         * @param[in]  newValue  The new value after dt time
         *
         * @return     The new output value
         */
        float update(float newValue) final {
            float y0 = newValue * amplFac;
            const float* floatP = yc;
            for (auto it = y.rbegin(); it != y.rend(); it++)
            {
                y0 -= *it * *floatP++;
            }
            return y.push(y0);
        }

        /**
         * @brief      Gets the output.
         *
         * @return     The output.
         */
        float getOutput() final{ return y.get(0); }

    private:

        const float amplFac;
        const float yc[3];

        CircularDelay<float, 3> y;
};

/**
 * @brief      Class for third order high pass filter. This is designed using
 *             the approximated differtial approuch where s=(Z-1)/(Z*T).
 */
class LowPassFilter3DiffApprox : public DigitalFilter<float> {

    public:

        /**
         * @brief      Constructor to set sample time and the tau constant
         *
         * @param[in]  sampleTime   Sample time for the low pass filter
         * @param[in]  omega_c      Cutoff angular frequency @f$ \omega_c = 2 pi f_c@f$  where @f$ f_c @f$ is the cutoff frequency
         */
        LowPassFilter3DiffApprox(float sampleTime, float omega_c, float ioutput = 0):
            xc {
                1 * tps::pow(sampleTime * omega_c, 3)
                ,
                0
                ,
                0
                ,
                0
            },
            yc {
                1 / (1 * tps::pow(sampleTime * omega_c, 3) + 2 * tps::pow(sampleTime * omega_c, 2) + 2 * sampleTime * omega_c + 1),
                    0 * tps::pow(sampleTime * omega_c, 3) - 2 * tps::pow(sampleTime * omega_c, 2) - 4 * sampleTime * omega_c - 3,
                    0 * tps::pow(sampleTime * omega_c, 3) + 0 * tps::pow(sampleTime * omega_c, 2) + 2 * sampleTime * omega_c + 3,
                    0 * tps::pow(sampleTime * omega_c, 3) - 0 * tps::pow(sampleTime * omega_c, 2) + 0 * sampleTime * omega_c - 1
            }
            {
                if(omega_c < sampleTime){
                    throw std::domain_error("LowPassFilter constructor error: tua_c is smaller than the sample time dt.");
                }
            }

        /**
         * @brief      Update function to push new value into the filter.
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