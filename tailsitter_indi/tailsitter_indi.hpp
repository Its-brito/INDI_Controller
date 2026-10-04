#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <matrix/matrix/math.hpp>
#include <cmath>

using namespace time_literals;

// --- SELF-CONTAINED SECOND-ORDER LOW-PASS FILTER ---------------------------
//
// Discrete-time biquad, Direct Form II:
//     w[n] = x[n] - a1*w[n-1] - a2*w[n-2]
//     y[n] = b0*w[n] + b1*w[n-1] + b2*w[n-2]
// Bilinear transform (pre-warped) of the Butterworth prototype
// H(s) = wc^2 / (s^2 + sqrt(2) wc s + wc^2)  (zeta = 1/sqrt(2), fixed).
// Same design as PX4's math::LowPassFilter2p.
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
		const float theta = 2.0f * M_PIf * cutoff_freq / sample_freq;
		const float c = 1.0f / std::tan(theta * 0.5f);
		const float sq_c = c * c;
		const float sqrt2_c = 1.41421356f * c;

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

		const float w = sample - _a1 * _delay1 - _a2 * _delay2;
		const float y = _b0 * w + _b1 * _delay1 + _b2 * _delay2;
		_delay2 = _delay1;
		_delay1 = w;
		return y;
	}

	// Initialise the internal state so that a constant input `value` gives a
	// constant output `value` immediately (steady-state initialisation).
	// Steady state of DF-II: w = x / (1 + a1 + a2), and y = (b0+b1+b2) w = x (unity DC gain).
	void reset(float value = 0.0f)
	{
		const float den = 1.f + _a1 + _a2;
		const float w = (_enabled && den > 1e-6f) ? value / den : value;
		_delay1 = w;
		_delay2 = w;
	}

private:
	bool _enabled{false};
	float _b0{1.0f}, _b1{0.0f}, _b2{0.0f};
	float _a1{0.0f}, _a2{0.0f};
	float _delay1{0.0f}, _delay2{0.0f};
};

// --- MAIN MODULE CLASS -----------------------------------------------------
//
// Incremental Nonlinear Dynamic Inversion (INDI) attitude/rate controller for
// the gz quadtailsitter. Works at the NORMALIZED-TORQUE level: it replaces
// mc_rate_control / fw_rate_control and publishes hover-frame torque to
// vehicle_torque_setpoint_virtual_mc AND _virtual_fw, so vtol_att_control keeps
// handling transitions and the control allocator keeps mixing the motors.
//
// All vectors are in the HOVER body frame (FRD), i.e. the frame of
// vehicle_attitude / vehicle_angular_velocity / the allocator:
//   hover x = FW yaw axis, hover y = FW pitch axis, hover z = -FW roll axis.
//
// Start with the stock rate controllers stopped - see rc.vtol_apps (INDI_EN).
class TailsitterINDI : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TailsitterINDI();
	~TailsitterINDI() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

	static ModuleBase::Descriptor desc;

private:
	void Run() override;
	void parameters_update();
	void update_filter_design();
	void reset_state(const matrix::Vector3f &u_init);

	// Attitude outer loop: returns desired hover-frame body rates.
	matrix::Vector3f attitude_control(bool fixed_wing);

	// Torque actually applied by the allocator last cycle, in INDI's
	// (pre-DIFTHR-scaling) domain. Returns false if not available.
	bool allocated_torque(matrix::Vector3f &u_alloc, const matrix::Vector3f &difthr_scale);

	// Subscriptions
	// Run() is triggered by every new gyro sample (like mc_rate_control), so the
	// motor outputs are produced in the same lockstep cycle as the simulator step.
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status), 0}; // matrix 0 = motors
	uORB::Subscription _vehicle_torque_setpoint0_sub{ORB_ID(vehicle_torque_setpoint), 0};  // allocator input
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Publications: same virtual topics as the stock rate controllers
	uORB::Publication<vehicle_torque_setpoint_s> _torque_mc_pub{ORB_ID(vehicle_torque_setpoint_virtual_mc)};
	uORB::Publication<vehicle_torque_setpoint_s> _torque_fw_pub{ORB_ID(vehicle_torque_setpoint_virtual_fw)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_mc_pub{ORB_ID(vehicle_thrust_setpoint_virtual_mc)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_fw_pub{ORB_ID(vehicle_thrust_setpoint_virtual_fw)};

	float _loop_rate_hz{250.f};      // measured gyro rate; filters are designed for it
	float _dt_avg{0.004f};
	static constexpr uint64_t ATTITUDE_SETPOINT_TIMEOUT_US = 500000;

	// Cached state
	vehicle_status_s _vehicle_status{};
	vehicle_control_mode_s _vehicle_control_mode{};
	bool _landed{true};
	float _airspeed{NAN};
	vehicle_torque_setpoint_s _torque_sp0{};
	control_allocator_status_s _ca_status{};
	matrix::Vector3f _thrust_body{};

	uint64_t _timestamp_prev{0};
	matrix::Vector3f _omega_prev{};
	bool _filters_initialized{false};

	CustomLowPassFilter2p _accel_filter[3];
	CustomLowPassFilter2p _actuator_filter[3];

	matrix::Vector3f _u_act{};        // actuator-model state (allocated torque through motor lag)
	matrix::Vector3f _u_cmd{};        // last INDI command (allocator domain)
	matrix::Vector3f _u_f_prev{};     // previous filtered actuator state (for the G2 term)
	matrix::Vector3f _u_cmd_prev{};   // previous command (for the G2 term)

	// Diagnostics for print_status
	matrix::Vector3f _last_att_err_deg{};
	bool _last_fixed_wing{false};
	bool _alloc_feedback_ok{false};

	// hover -> FW frame rotation: q_fw = q_hover * _q_mc_to_fw^-1 (same as fw_att_control)
	const matrix::Quatf _q_mc_to_fw{matrix::Eulerf(0.f, -M_PI_2_F, 0.f)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::INDI_MC_ATT_X>) _param_mc_att_x,
		(ParamFloat<px4::params::INDI_MC_ATT_Y>) _param_mc_att_y,
		(ParamFloat<px4::params::INDI_MC_ATT_Z>) _param_mc_att_z,
		(ParamFloat<px4::params::INDI_FW_ATT_R>) _param_fw_att_r,
		(ParamFloat<px4::params::INDI_FW_ATT_P>) _param_fw_att_p,
		(ParamFloat<px4::params::INDI_FW_ATT_Y>) _param_fw_att_y,
		(ParamFloat<px4::params::INDI_RATE_K_X>) _param_rate_k_x,
		(ParamFloat<px4::params::INDI_RATE_K_Y>) _param_rate_k_y,
		(ParamFloat<px4::params::INDI_RATE_K_Z>) _param_rate_k_z,
		(ParamFloat<px4::params::INDI_GI_MC_X>) _param_gi_mc_x,
		(ParamFloat<px4::params::INDI_GI_MC_Y>) _param_gi_mc_y,
		(ParamFloat<px4::params::INDI_GI_MC_Z>) _param_gi_mc_z,
		(ParamFloat<px4::params::INDI_GI_FW_X>) _param_gi_fw_x,
		(ParamFloat<px4::params::INDI_GI_FW_Y>) _param_gi_fw_y,
		(ParamFloat<px4::params::INDI_GI_FW_Z>) _param_gi_fw_z,
		(ParamFloat<px4::params::INDI_G2_Z>) _param_g2_z,
		(ParamFloat<px4::params::INDI_ACT_TAU>) _param_act_tau,
		(ParamFloat<px4::params::INDI_LPF_HZ>) _param_lpf_hz,
		(ParamInt<px4::params::INDI_FW_TC_EN>) _param_fw_tc_en,
		(ParamInt<px4::params::VT_FW_DIFTHR_EN>) _param_vt_fw_difthr_en,
		(ParamFloat<px4::params::VT_FW_DIFTHR_S_R>) _param_vt_fw_difthr_s_r,
		(ParamFloat<px4::params::VT_FW_DIFTHR_S_P>) _param_vt_fw_difthr_s_p,
		(ParamFloat<px4::params::VT_FW_DIFTHR_S_Y>) _param_vt_fw_difthr_s_y,
		(ParamFloat<px4::params::FW_AIRSPD_TRIM>) _param_fw_airspd_trim,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall
	)
};
