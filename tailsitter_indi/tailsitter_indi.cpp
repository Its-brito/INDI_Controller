#include "tailsitter_indi.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

static constexpr float ONE_G = 9.80665f; // m/s^2

/*
 * INDI control law (torque level, per hover-frame axis i, diagonal G):
 *
 *   nu      = K_rate (omega_d - omega)                       virtual control [rad/s^2]
 *   u_c     = u_f + (nu - omega_dot_f + G2d (u_c[k-1] - u_f[k-1])) / (G1 + G2d)
 *
 *   u_f        = LPF( actuator_model( allocated torque ) )  - what the motors are
 *                ACTUALLY producing, after allocator saturation and motor lag,
 *                filtered with the SAME filter as omega_dot (synchronisation).
 *   omega_dot_f= LPF( d omega / dt )
 *   G1         = 1 / G_inv  [rad/s^2 per unit normalized torque]
 *   G2d        = G2 / dt    rotor spin-up term (hover z only), Smeur et al. 2016.
 *
 * G_inv defaults come from Test_script/g_matrix_quadtailsitter.py (gz model.sdf:
 * motorConstant, momentConstant, rotor positions AND the +-0.17 rad rotor cant,
 * inertia, SIM_GZ_EC 10..1500 rad/s, PX4 allocator normalization).
 */

TailsitterINDI::TailsitterINDI() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	parameters_update();
}

bool TailsitterINDI::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void TailsitterINDI::parameters_update()
{
	updateParams();
	update_filter_design();
}

void TailsitterINDI::update_filter_design()
{
	// Both filters MUST be identical (synchronisation), and designed for the
	// actual sample rate of this loop (= gyro publication rate).
	for (int i = 0; i < 3; i++) {
		_accel_filter[i].set_cutoff_frequency(_loop_rate_hz, _param_lpf_hz.get());
		_actuator_filter[i].set_cutoff_frequency(_loop_rate_hz, _param_lpf_hz.get());
	}

	_filters_initialized = false;
}

void TailsitterINDI::reset_state(const Vector3f &u_init)
{
	_u_act = u_init;
	_u_cmd = u_init;
	_u_cmd_prev = u_init;
	_u_f_prev = u_init;

	for (int i = 0; i < 3; i++) {
		_actuator_filter[i].reset(u_init(i));
	}
}

bool TailsitterINDI::allocated_torque(Vector3f &u_alloc, const Vector3f &difthr_scale)
{
	// allocated = (torque the allocator was asked for) - (what it could not allocate).
	// vehicle_torque_setpoint instance 0 is written by vtol_att_control AFTER the
	// VT_FW_DIFTHR_S_* scaling, so divide it back out to get INDI's own domain.
	_vehicle_torque_setpoint0_sub.update(&_torque_sp0);
	_control_allocator_status_sub.update(&_ca_status);

	const hrt_abstime now = hrt_absolute_time();

	if (_torque_sp0.timestamp == 0 || _ca_status.timestamp == 0
	    || now - _torque_sp0.timestamp > 50_ms || now - _ca_status.timestamp > 50_ms) {
		return false;
	}

	for (int i = 0; i < 3; i++) {
		const float applied = _torque_sp0.xyz[i] - _ca_status.unallocated_torque[i];

		if (difthr_scale(i) < 1e-3f) {
			return false; // axis disabled in FW (VT_FW_DIFTHR_EN) - no feedback possible
		}

		u_alloc(i) = applied / difthr_scale(i);
	}

	return u_alloc.isAllFinite();
}

Vector3f TailsitterINDI::attitude_control(bool fixed_wing)
{
	Vector3f omega_d{};

	vehicle_attitude_s att{};
	vehicle_attitude_setpoint_s att_sp{};
	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_sp = _vehicle_attitude_setpoint_sub.copy(&att_sp);

	const bool sp_valid = have_att && have_sp
			      && (hrt_absolute_time() - att_sp.timestamp) < ATTITUDE_SETPOINT_TIMEOUT_US
			      && Quatf(att_sp.q_d).isAllFinite()
			      && Quatf(att_sp.q_d).norm() > 0.5f;

	if (!sp_valid) {
		_last_att_err_deg.zero();
		return omega_d; // fail-safe: pure rate damping
	}

	const Quatf q(att.q);        // hover frame (always)
	Quatf qd(att_sp.q_d);

	// (3) In FW, vehicle_attitude_setpoint comes from fw_virtual_attitude_setpoint and is
	// expressed in the FIXED-WING frame. Convert it to the hover frame the attitude uses:
	//   q_fw = q_hover * q_mc_to_fw^-1   =>   q_hover = q_fw * q_mc_to_fw
	// During transitions vtol_att_control publishes hover-frame setpoints and
	// vehicle_type is still ROTARY_WING, so no conversion there.
	if (fixed_wing) {
		qd = qd * _q_mc_to_fw;
	}

	// (2) BODY-frame attitude error: q_e = q^-1 * q_d  (was q_d * q^-1 = world frame).
	Quatf qe = q.inversed() * qd;

	if (qe(0) < 0.f) {
		qe = -qe; // shortest rotation
	}

	const Vector3f qe_vec{qe(1), qe(2), qe(3)};
	_last_att_err_deg = qe_vec * (2.f * 57.2958f);

	Vector3f k_att;

	if (fixed_wing) {
		// Gains are given per FW axis; map to hover axes:
		// hover x = FW yaw, hover y = FW pitch, hover z = -FW roll.
		k_att = Vector3f(_param_fw_att_y.get(), _param_fw_att_p.get(), _param_fw_att_r.get());

	} else {
		k_att = Vector3f(_param_mc_att_x.get(), _param_mc_att_y.get(), _param_mc_att_z.get());
	}

	omega_d = k_att.emult(qe_vec) * 2.f;

	// Coordinated-turn feed-forward in FW: steady banked turn needs FW body yaw rate
	// r = g sin(phi) cos(theta) / V. Without it INDI would damp the turn rate and fly
	// uncoordinated (PX4's fw_att_control does the equivalent). FW yaw = hover x.
	if (fixed_wing && _param_fw_tc_en.get() > 0) {
		const Eulerf e_fw(q * _q_mc_to_fw.inversed());
		const float v_min = math::max(_param_fw_airspd_stall.get(), 5.f);
		const float v = PX4_ISFINITE(_airspeed) ? math::max(_airspeed, v_min) : _param_fw_airspd_trim.get();
		omega_d(0) += ONE_G * sinf(e_fw.phi()) * cosf(e_fw.theta()) / v;
	}

	return omega_d;
}

void TailsitterINDI::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		parameters_update();
	}

	_vehicle_status_sub.update(&_vehicle_status);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	vehicle_land_detected_s land{};

	if (_vehicle_land_detected_sub.update(&land)) {
		_landed = land.landed || land.maybe_landed;
	}

	airspeed_validated_s aspd{};

	if (_airspeed_validated_sub.update(&aspd)) {
		_airspeed = aspd.true_airspeed_m_s;
	}

	vehicle_rates_setpoint_s rates_sp{};

	if (_vehicle_rates_setpoint_sub.update(&rates_sp)) {
		// thrust comes from mc_att_control (MC/transition: [0,0,-T]) or
		// fw_att_control (FW: [T,0,0]); both are published by the attitude controllers,
		// which keep running.
		_thrust_body = Vector3f(rates_sp.thrust_body);
	}

	vehicle_angular_velocity_s angular_velocity{};

	if (!_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		return;
	}

	const Vector3f omega{angular_velocity.xyz};
	const uint64_t t_now = angular_velocity.timestamp_sample;
	const float dt = (_timestamp_prev != 0) ? math::constrain((t_now - _timestamp_prev) * 1e-6f, 0.001f, 0.02f) : 0.f;
	const Vector3f omega_dot_raw = (dt > 0.f) ? Vector3f((omega - _omega_prev) / dt) : Vector3f{};
	_omega_prev = omega;
	_timestamp_prev = t_now;

	// Track the real loop rate; redesign both filters if it moves by >10 %.
	if (dt > 0.f) {
		_dt_avg += 0.01f * (dt - _dt_avg);
		const float rate = 1.f / _dt_avg;

		if (fabsf(rate - _loop_rate_hz) > 0.1f * _loop_rate_hz) {
			_loop_rate_hz = rate;
			update_filter_design();
		}
	}

	const bool fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING
				&& !_vehicle_status.in_transition_mode;
	_last_fixed_wing = fixed_wing;

	// Torque scaling vtol_att_control applies to our virtual_fw torque in FW (hover axes).
	Vector3f difthr_scale{1.f, 1.f, 1.f};

	if (fixed_wing) {
		const int en = _param_vt_fw_difthr_en.get();
		difthr_scale(0) = (en & 4) ? _param_vt_fw_difthr_s_y.get() : 0.f; // hover x = FW yaw
		difthr_scale(1) = (en & 2) ? _param_vt_fw_difthr_s_p.get() : 0.f; // hover y = FW pitch
		difthr_scale(2) = (en & 1) ? _param_vt_fw_difthr_s_r.get() : 0.f; // hover z = FW roll
	}

	// (4) u0 = what the allocator really applied (after saturation), passed through a
	// first-order motor model. Falls back to our own clipped command if unavailable.
	Vector3f u_alloc;
	_alloc_feedback_ok = allocated_torque(u_alloc, difthr_scale);

	if (!_alloc_feedback_ok) {
		u_alloc = _u_cmd;
	}

	const bool active = _vehicle_control_mode.flag_armed && !_landed && dt > 0.f;

	if (!active) {
		// Disarmed / on the ground: no increments (the ground holds the airframe, so
		// omega_dot = 0 and an INDI loop would otherwise wind the torque up).
		reset_state(Vector3f{});
		_filters_initialized = false;

	} else {
		if (!_filters_initialized) {
			for (int i = 0; i < 3; i++) {
				_accel_filter[i].reset(omega_dot_raw(i));
			}

			reset_state(u_alloc);
			_filters_initialized = true;
		}

		// motor lag (gz timeConstantUp 0.0125 s / Down 0.025 s)
		const float alpha = dt / (math::max(_param_act_tau.get(), 1e-4f) + dt);
		_u_act += (u_alloc - _u_act) * alpha;

		Vector3f omega_dot_f, u_f;

		for (int i = 0; i < 3; i++) {
			omega_dot_f(i) = _accel_filter[i].apply(omega_dot_raw(i));
			u_f(i) = _actuator_filter[i].apply(_u_act(i));
		}

		const Vector3f omega_d = attitude_control(fixed_wing);
		const Vector3f k_rate(_param_rate_k_x.get(), _param_rate_k_y.get(), _param_rate_k_z.get());
		const Vector3f nu = k_rate.emult(omega_d - omega);

		const Vector3f g_inv = fixed_wing
				       ? Vector3f(_param_gi_fw_x.get(), _param_gi_fw_y.get(), _param_gi_fw_z.get())
				       : Vector3f(_param_gi_mc_x.get(), _param_gi_mc_y.get(), _param_gi_mc_z.get());

		// (5) diagonal INDI with the G2 (rotor spin-up) term on hover z (= FW roll)
		const Vector3f g2d(0.f, 0.f, _param_g2_z.get() / dt);
		Vector3f u_c;

		for (int i = 0; i < 3; i++) {
			const float g1 = (g_inv(i) > 1e-6f) ? 1.f / g_inv(i) : 0.f;
			const float den = g1 + g2d(i);

			if (den < 1e-6f) {
				u_c(i) = u_f(i);
				continue;
			}

			u_c(i) = u_f(i) + (nu(i) - omega_dot_f(i) + g2d(i) * (_u_cmd_prev(i) - _u_f_prev(i))) / den;
		}

		// normalized torque limits (was +-5)
		u_c = matrix::constrain(u_c, -1.f, 1.f);

		_u_f_prev = u_f;
		_u_cmd_prev = u_c;
		_u_cmd = u_c;
	}

	// Publish (hover frame). Undo the DIFTHR scaling vtol_att_control will apply in FW.
	vehicle_torque_setpoint_s torque{};
	torque.timestamp_sample = angular_velocity.timestamp_sample;

	for (int i = 0; i < 3; i++) {
		torque.xyz[i] = (difthr_scale(i) > 1e-3f) ? _u_cmd(i) / difthr_scale(i) : 0.f;
	}

	// Collective thrust magnitude, routed to whichever virtual topic vtol_att_control reads.
	const float thrust = math::constrain(math::max(_thrust_body(0), -_thrust_body(2)), 0.f, 1.f);

	vehicle_thrust_setpoint_s thrust_mc{};
	thrust_mc.timestamp_sample = angular_velocity.timestamp_sample;
	thrust_mc.xyz[2] = -thrust;

	vehicle_thrust_setpoint_s thrust_fw{};
	thrust_fw.timestamp_sample = angular_velocity.timestamp_sample;
	thrust_fw.xyz[0] = thrust;

	const hrt_abstime now = hrt_absolute_time();
	torque.timestamp = now;
	thrust_mc.timestamp = now;
	thrust_fw.timestamp = now;

	_torque_mc_pub.publish(torque);
	_torque_fw_pub.publish(torque);
	_thrust_mc_pub.publish(thrust_mc);
	_thrust_fw_pub.publish(thrust_fw);
}

int TailsitterINDI::print_status()
{
	PX4_INFO("mode: %s | allocator feedback: %s",
		 _last_fixed_wing ? "FW" : "MC/transition", _alloc_feedback_ok ? "ok" : "MISSING (using own command)");
	PX4_INFO("attitude error (hover axes, deg): %.2f %.2f %.2f",
		 (double)_last_att_err_deg(0), (double)_last_att_err_deg(1), (double)_last_att_err_deg(2));
	PX4_INFO("torque cmd (normalized): %.3f %.3f %.3f",
		 (double)_u_cmd(0), (double)_u_cmd(1), (double)_u_cmd(2));
	return 0;
}

ModuleBase::Descriptor TailsitterINDI::desc{task_spawn, custom_command, print_usage};

int TailsitterINDI::task_spawn(int argc, char *argv[])
{
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

int TailsitterINDI::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command");
}

int TailsitterINDI::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s\n", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Tailsitter INDI attitude + rate controller (normalized-torque level).
Replaces mc_rate_control and fw_rate_control: publishes hover-frame torque and
thrust to the *_virtual_mc / *_virtual_fw topics consumed by vtol_att_control.
The stock rate controllers must NOT run at the same time (see INDI_EN in rc.vtol_apps).
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("tailsitter_indi", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int tailsitter_indi_main(int argc, char *argv[])
{
	return TailsitterINDI::main(TailsitterINDI::desc, argc, argv);
}
