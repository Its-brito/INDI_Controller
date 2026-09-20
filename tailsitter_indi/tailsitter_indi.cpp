#include "tailsitter_indi.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>

TailsitterINDI::TailsitterINDI() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
    _accel_filter_x.set_cutoff_frequency(250.0f, 20.0f);
    _accel_filter_y.set_cutoff_frequency(250.0f, 20.0f);
    _accel_filter_z.set_cutoff_frequency(250.0f, 20.0f);

    _actuator_filter_x.set_cutoff_frequency(250.0f, 20.0f);
    _actuator_filter_y.set_cutoff_frequency(250.0f, 20.0f);
    _actuator_filter_z.set_cutoff_frequency(250.0f, 20.0f);
}

bool TailsitterINDI::init() {
    ScheduleOnInterval(4000); // 4ms = 250 Hz loop interval
    return true;
}

void TailsitterINDI::Run() {
    if (should_exit()) {
        ScheduleClear();
        return;
    }

    vehicle_angular_velocity_s angular_velocity{};
    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

        matrix::Vector3f omega{angular_velocity.xyz};
        uint64_t timestamp_now = angular_velocity.timestamp_sample;

        float dt = 0.0f;
        if (_timestamp_prev != 0) {
            dt = (timestamp_now - _timestamp_prev) / 1e6f;
        }

        matrix::Vector3f omega_dot_filtered{0.f, 0.f, 0.f};

        if (dt > 0.0001f) {
            matrix::Vector3f omega_dot_raw = (omega - _omega_prev) / dt;

            // Explicitly (re-)initialize both the acceleration and actuator
            // filters' internal state on the first valid control cycle, rather
            // than relying on the implicit zero-initialization of their delay
            // elements. The acceleration filters are seeded with the first raw
            // derivative sample (avoiding a spurious ramp-up transient), and
            // the actuator filters are seeded with the current (still-zero)
            // _u_cmd_prev, which is the correct assumption for a module that
            // starts pre-arm at rest with zero commanded torque. If this module
            // is ever restarted in-flight, this initialization assumption must
            // be revisited (see Chapter 4, Sec. 4.6).
            if (!_filters_initialized) {
                _accel_filter_x.reset(omega_dot_raw(0));
                _accel_filter_y.reset(omega_dot_raw(1));
                _accel_filter_z.reset(omega_dot_raw(2));

                _actuator_filter_x.reset(_u_cmd_prev(0));
                _actuator_filter_y.reset(_u_cmd_prev(1));
                _actuator_filter_z.reset(_u_cmd_prev(2));

                _filters_initialized = true;
            }

            omega_dot_filtered(0) = _accel_filter_x.apply(omega_dot_raw(0));
            omega_dot_filtered(1) = _accel_filter_y.apply(omega_dot_raw(1));
            omega_dot_filtered(2) = _accel_filter_z.apply(omega_dot_raw(2));
        }

        _omega_prev = omega;
        _timestamp_prev = timestamp_now;

        // --- Outer-loop attitude control: turn the commanded quaternion into a
        //     desired body rate omega_d for the INDI rate loop below. -------------
        matrix::Vector3f omega_d{0.f, 0.f, 0.f};
        matrix::Vector3f omega_dot_d{0.f, 0.f, 0.f}; // no rate feedforward (yet)

        vehicle_attitude_s attitude{};
        vehicle_attitude_setpoint_s attitude_sp{};
        const bool have_attitude = _vehicle_attitude_sub.copy(&attitude);
        const bool have_attitude_sp = _vehicle_attitude_setpoint_sub.copy(&attitude_sp);

        const bool attitude_sp_fresh = have_attitude_sp &&
            (hrt_absolute_time() - attitude_sp.timestamp) < ATTITUDE_SETPOINT_TIMEOUT_US;

        const bool attitude_sp_valid = attitude_sp_fresh &&
            !(fabsf(attitude_sp.q_d[0]) < 1e-6f && fabsf(attitude_sp.q_d[1]) < 1e-6f &&
              fabsf(attitude_sp.q_d[2]) < 1e-6f && fabsf(attitude_sp.q_d[3]) < 1e-6f);

        if (have_attitude && attitude_sp_valid) {
            const matrix::Quatf q(attitude.q);          // current estimated attitude
            matrix::Quatf qd(attitude_sp.q_d);           // commanded attitude

            matrix::Quatf qe = qd * q.inversed();        // error quaternion
            if (qe(0) < 0.f) {
                qe = -qe; // shortest-path rotation
            }
            const matrix::Vector3f qe_vec{qe(1), qe(2), qe(3)};

            // Standard quaternion-error proportional attitude law (cf. PX4
            // mc_att_control / Lee et al.): omega_d = 2 * Kp_att (.) qe_vec.
            omega_d = _Kp_att.emult(qe_vec) * 2.f;
        }
        // else: no fresh/valid attitude setpoint (e.g. not in Offboard/Position
        // attitude control) -> omega_d stays zero, i.e. fail-safe pure rate damping.

        matrix::Vector3f error = omega_d - omega;
        matrix::Vector3f nu = omega_dot_d + _Kp.emult(error);

        // Synchronized filtering (Chapter 4, Sec. 4.6): the SAME 250 Hz/20 Hz
        // filter applied to the raw angular-acceleration derivative above must
        // also be applied to the actuator state that anchors the incremental
        // update below, so that both terms entering the INDI law carry an
        // equivalent group delay. u_filtered is therefore the quantity that the
        // control increment is added to -- NOT the raw, unfiltered _u_cmd_prev.
        matrix::Vector3f u_filtered;
        u_filtered(0) = _actuator_filter_x.apply(_u_cmd_prev(0));
        u_filtered(1) = _actuator_filter_y.apply(_u_cmd_prev(1));
        u_filtered(2) = _actuator_filter_z.apply(_u_cmd_prev(2));

        matrix::Matrix3f G_inv;
        G_inv.zero();

        // Diagonal terms = Ixx, Iyy, Izz from the SDF
        G_inv(0, 0) = 0.113333f; // Ixx [kg*m^2] -> Maps roll angular acceleration error to torque
        G_inv(1, 1) = 0.030208f; // Iyy [kg*m^2] -> Maps pitch angular acceleration error to torque
        G_inv(2, 2) = 0.083542f; // Izz [kg*m^2] -> Maps yaw angular acceleration error to torque

        matrix::Vector3f delta_u = G_inv * (nu - omega_dot_filtered);

        // NOTE (fixed): previously this accumulated onto the raw, unfiltered
        // _u_cmd_prev, which silently discarded u_filtered and broke the
        // synchronized-filtering property described in Chapter 4, Sec. 4.6.
        // The increment must instead be added to the filtered previous command
        // so both terms of the INDI law share the same group delay.
        _u_cmd_prev = u_filtered + delta_u;

        for (int i = 0; i < 3; i++) {
            _u_cmd_prev(i) = math::constrain(_u_cmd_prev(i), -5.0f, 5.0f);
        }

        vehicle_torque_setpoint_s torque_sp{};
        torque_sp.timestamp = hrt_absolute_time();
        torque_sp.xyz[0] = _u_cmd_prev(0);
        torque_sp.xyz[1] = _u_cmd_prev(1);
        torque_sp.xyz[2] = _u_cmd_prev(2);
        _vehicle_torque_setpoint_pub.publish(torque_sp);
    }
}

// Define the static descriptor object
ModuleBase::Descriptor TailsitterINDI::desc{task_spawn, custom_command, print_usage};

int TailsitterINDI::task_spawn(int argc, char *argv[]) {
    TailsitterINDI *instance = new TailsitterINDI();

    if (instance) {
        desc.object.store(instance);
        desc.task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

        delete instance;
        desc.object.store(nullptr);
        desc.task_id = -1;
    }

    return PX4_ERROR;
}

int TailsitterINDI::custom_command(int argc, char *argv[]) {
    return print_usage("Unrecognized command");
}

int TailsitterINDI::print_usage(const char *reason) {
    if (reason) { PX4_WARN("%s\n", reason); }
    PRINT_MODULE_DESCRIPTION("Tailsitter INDI Attitude Controller");
    PRINT_MODULE_USAGE_NAME("tailsitter_indi", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int tailsitter_indi_main(int argc, char *argv[]) {
    return TailsitterINDI::main(TailsitterINDI::desc, argc, argv);
}
