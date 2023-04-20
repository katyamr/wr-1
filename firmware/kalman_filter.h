#pragma once

template<typename T>
struct kalman_filter_t {
    using value_type = T;

    kalman_filter_t(value_type e,
        value_type em, value_type ee, value_type q_):
        err_measure(em), q(q_), err_estimate(ee), last_estimate(e)
    { }

    value_type operator()(value_type v) {
        value_type gain = err_estimate / (err_estimate + err_measure);
        value_type estimate = last_estimate + gain * (v - last_estimate);
        err_estimate = (1.0 - gain) * err_estimate
            + fabs(last_estimate - estimate) * q;
        last_estimate = estimate;
        return estimate;
    }

    inline operator value_type() const { return last_estimate; }

    const value_type err_measure;
    const value_type q;

    value_type err_estimate;
    value_type last_estimate;
};
