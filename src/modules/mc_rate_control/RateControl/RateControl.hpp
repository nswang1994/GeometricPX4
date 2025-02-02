/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file RateControl.hpp
 *
 * PID 3 axis angular rate / angular velocity control.
 */

#pragma once

#include <matrix/matrix/math.hpp>
//#include <ctime>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <uORB/topics/rate_ctrl_status.h>

class RateControl
{
public:
	RateControl() = default;
	~RateControl() = default;

	/**
	 * Set the rate control gains
	 * @param P 3D vector of proportional gains for body x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);
	void setESO(const bool &ESO) { ThereIsESO = ESO; };
	void setESOGains(const float &K1, const float &K2, const float &K3, const float &Kappa);

	/**
	 * Set the mximum absolute value of the integrator for all axes
	 * @param integrator_limit limit value for all axes x, y, z
	 */
	void setIntegratorLimit(const matrix::Vector3f &integrator_limit) { _lim_int = integrator_limit; };

	/**
	 * Set direct rate to torque feed forward gain
	 * @see _gain_ff
	 * @param FF 3D vector of feed forward gains for body x,y,z axis
	 */
	void setFeedForwardGain(const matrix::Vector3f &FF) { _gain_ff = FF; };

	/**
	 * Set saturation status
	 * @param control saturation vector from control allocator
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	/**
	 * Run one control loop cycle calculation
	 * @param rate estimation of the current vehicle angular rate
	 * @param rate_sp desired vehicle angular rate setpoint
	 * @param dt desired vehicle angular rate setpoint
	 * @return [-1,1] normalized torque vector to apply to the vehicle
	 */
	//matrix::Vector3f update(const matrix::Vector3f &rate, const matrix::Vector3f &rate_sp,
	//			const matrix::Vector3f &angular_accel, const float dt, const bool landed);

	matrix::Vector3f update(const matrix::Vector3f &Omega, const matrix::Vector3f &Omegad,
				const matrix::MatrixfSO3 &R, const matrix::Vector3f &angular_accel,
				const float dt, const bool landed, const float time);
	/**
	 * Set the integral term to 0 to prevent windup
	 * @see _rate_int
	 */
	//void resetIntegral() { psi_I={0.001f,0.0f,0.001f};; }
	void resetIntegral() { _rate_int.zero(); }
	/**
	 * Get status message of controller for logging/debugging
	 * @param rate_ctrl_status status message to fill with internal states
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status);

private:
	void updateIntegral(matrix::Vector3f &rate_error, const float dt);

	// ESO
	void AttitudeESO(matrix::Vector3f tau, float dt);
	float takeoff_time;
	bool ESOflag=1;
	bool ThereIsESO;
	matrix::Vector3f phi1(matrix::Vector3f e1);
	matrix::Vector3f phi2(matrix::Vector3f e1);

	// Gains
	matrix::Vector3f _gain_p; ///< rate control proportional gain for all axes x, y, z
	matrix::Vector3f _gain_i; ///< rate control integral gain
	matrix::Vector3f _gain_d; ///< rate control derivative gain
	matrix::Vector3f _lim_int; ///< integrator term maximum absolute value
	matrix::Vector3f _gain_ff; ///< direct rate to torque feed forward gain only useful for helicopters


	// States
	//matrix::Vector3f psi_I; ///< integral term of the rate controller
	matrix::Vector3f _rate_int; ///< integral term of the rate controller

	// Feedback from control allocation
	matrix::Vector<bool, 3> _control_allocator_saturation_negative;
	matrix::Vector<bool, 3> _control_allocator_saturation_positive;



	//The following variables are for geometric control

	float J[9] = { 0.03f, 0.0f, 0.0f, 0.0f, 0.03f, 0.0f, 0.0f, 0.0f, 0.06f };
	float L[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2.0f };
	float Id[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	matrix::MatrixfSO3 L_{L};
	matrix::MatrixfSO3 J_{J};


	//float kappa_A = 0.8f;
	float _p = 1.2f;
	// ESO

	matrix::MatrixfSO3 R_hatnext;
	matrix::MatrixfSO3 R_hat{ Id };

	matrix::Vector3f Omega_;
	matrix::Vector3f Omega_hatnext;
	matrix::Vector3f Omega_hat{0,0,0};
	matrix::Vector3f tauD_hatnext;
	matrix::Vector3f tauD_hat{0,0,0};
	matrix::Vector3f tauD_rejection{0,0,0};


	matrix::MatrixfSO3 R_;
	float k_a1;
	float k_a2;
	float k_a3;
	float kappa_a;
	//float k_a1=8.0f;
	//float k_a2=4.0f;
	//float k_a3=2.0f;
	//float kappa_a=0.6f;
	//matrix::Vector3f Omega;
	matrix::Vector3f Omegad_Prev;
};
