#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <matrix/matrix/math.hpp>
#include <cmath>

// --- CUSTOM SELF-CONTAINED SECOND-ORDER LOW-PASS FILTER ---
//
// Discrete-time biquad realized in Direct Form II:
//     w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
//     y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
//
// Coefficients are the bilinear-transform (with frequency pre-warping) of the
// analog Butterworth prototype H(s) = wc^2 / (s^2 + 2*zeta*wc*s + wc^2), with
// the damping ratio FIXED at zeta = 1/sqrt(2) ~= 0.7071 (maximally-flat
// magnitude response) -- this is not a tunable parameter; it is implicit in
// the "1.41421356f" (= sqrt(2) = 2*zeta) term in set_cutoff_frequency() below.
// This is the same coefficient design as PX4's own math::LowPassFilter2p.
class CustomLowPassFilter2p
{
public:
    CustomLowPassFilter2p() = default;

    void set_cutoff_frequency(float sample_freq, float cutoff_freq)
    {
        if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
            _enabled = false;
            return;
        }
        _enabled = true;

        float theta = 2.0f * M_PIf * cutoff_freq / sample_freq;
        float c = 1.0f / std::tan(theta * 0.5f);
        float sq_c = c * c;
        float sqrt2_c = 1.41421356f * c; // sqrt(2)*c == 2*zeta*c, zeta = 1/sqrt(2) (Butterworth)

        _b0 = 1.0f / (sq_c + sqrt2_c + 1.0f);
        _b1 = 2.0f * _b0;
        _b2 = _b0;

        _a1 = 2.0f * (1.0f - sq_c) * _b0;
        _a2 = (sq_c - sqrt2_c + 1.0f) * _b0;
    }

    float apply(float sample)
    {
        if (!_enabled) {
            return sample;
        }

        float delay_element = sample - _a1 * _delay1 - _a2 * _delay2;
        float output = _b0 * delay_element + _b1 * _delay1 + _b2 * _delay2;

        _delay2 = _delay1;
        _delay1 = delay_element;

        return output;
    }

    // Explicitly (re)initializes both delay states to `value`. Called once,
    // deliberately, on the first valid control cycle (see TailsitterINDI::Run())
    // rather than relying on the implicit zero-initialization of _delay1/_delay2
    // below, so the cold-start assumption is visible and easy to revisit if the
    // module is ever started while NOT at rest (e.g. in-air module restart).
    void reset(float value = 0.0f)
    {
        _delay1 = value;
        _delay2 = value;
    }

private:
    bool _enabled{false};
    float _b0{1.0f}, _b1{0.0f}, _b2{0.0f};
    float _a1{0.0f}, _a2{0.0f};
    float _delay1{0.0f}, _delay2{0.0f};
};

// --- MAIN MODULE CLASS ---
class TailsitterINDI : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    TailsitterINDI();
    ~TailsitterINDI() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

    // Required module descriptor for modern PX4
    static ModuleBase::Descriptor desc;

private:
    void Run() override;

    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

    // Outer-loop quaternion attitude-P gains (rad/s of commanded body rate per rad
    // of attitude error). Values are in the same ballpark as PX4's stock MC_ROLL_P /
    // MC_PITCH_P / MC_YAW_P defaults -- retune for the Swan-K1 once flight-tested.
    matrix::Vector3f _Kp_att{6.5f, 6.5f, 2.8f};

    // If no fresh vehicle_attitude_setpoint has been received within this window,
    // the outer loop fails safe to omega_d = 0 (pure rate damping, the previous
    // behavior) rather than tracking a stale/invalid setpoint.
    static constexpr uint64_t ATTITUDE_SETPOINT_TIMEOUT_US = 500000; // 0.5 s

    uint64_t _timestamp_prev{0};
    matrix::Vector3f _omega_prev{0.f, 0.f, 0.f};

    // Set once, on the first valid Run() cycle, to explicitly (re-)zero every
    // filter's internal state. See the reset() call site in Run() for why this
    // is done explicitly rather than left to the default member initializers.
    bool _filters_initialized{false};

    CustomLowPassFilter2p _accel_filter_x;
    CustomLowPassFilter2p _accel_filter_y;
    CustomLowPassFilter2p _accel_filter_z;

    CustomLowPassFilter2p _actuator_filter_x;
    CustomLowPassFilter2p _actuator_filter_y;
    CustomLowPassFilter2p _actuator_filter_z;

    matrix::Vector3f _u_cmd_prev{0.f, 0.f, 0.f};
    matrix::Vector3f _Kp{10.f, 10.f, 10.f};
};
