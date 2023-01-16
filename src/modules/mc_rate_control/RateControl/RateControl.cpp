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
 * @file RateControl.cpp
 */

#include <RateControl.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

void RateControl::setGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_p = P;
	_gain_i = I;
	_gain_d = D;
}

void RateControl::setGeoGains(const float &kA_, const float &kappaA_){
	k_A = 0.01f*kA_;
	kappa_A = 0.1f*kappaA_;
	return;
}

void RateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
				      const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}
/*
Vector3f RateControl::update(const Vector3f &rate, const Vector3f &rate_sp, const Vector3f &angular_accel,
			     const float dt, const bool landed)
{
	// angular rates error
	Vector3f rate_error = rate_sp - rate;

	// PID control with feed forward
	const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(rate_sp);

	// update integral only if we are not landed
	if (!landed) {
		updateIntegral(rate_error, dt);
	}

	return torque;
}
*/
Vector3f RateControl::update(	const matrix::Vector3f &Omega, const matrix::Vector3f &Omegad,
				const matrix::MatrixfSO3 &R, const matrix::MatrixfSO3 &Rd,
				const matrix::Vector3f &angular_accel, const float dt, const bool landed, const float time)
{
	// angular rates error
	//Vector3f rate_error = Omegad - Omega;
	Q = R.transpose()*Rd;
	Vector3f omega = Omega -Q.transpose()*Omegad;
	Vector3f psi_A = omega + kappa_A*L_*(Q.sKgenerator() + pow(Vector3f(Q.sKgenerator()).norm_squared(),(1.0f/_p-1.0f))* Q.sKgenerator());
	Vector3f w = Q.wGenerator(omega);
	Matrix3f H;


	H.HouseHolder(Q.sKgenerator(),1.0f - 1.0f/_p);
	//Vector3f sK = Q.sKgenerator();
	//Vector3f dOmegad = (Omegad-OmegadPrev)/0.01f;
	const Vector3f torque = -k_A * L_* (psi_A+ pow(psi_A.norm_squared(),(1.0f/_p-1.0f))*psi_A)
	- kappa_A * J_*(w + pow(Vector3f(Q.sKgenerator()).norm_squared(),(1.0f/_p-1.0f))*H*w)
	+ J_ * (Q.transpose() * Vector3f((Omegad-OmegadPrev)/0.01f)  - omega.skew() * Q.transpose() * Omegad)
	- Vector3f(J_ * Omega )% Omega - tauD_rejection;


	// PID control with feed forward
	//const Vector3f torque = _gain_p.emult(rate_error) + _rate_int - _gain_d.emult(angular_accel) + _gain_ff.emult(Omegad);
	//clock_t clock();
	// update integral only if we are not landed
	if (!landed) {
		if (ESOflag == 1){
			takeoff_time = time;
			ESOflag = 0;
		}
		R_ = R;
		AttitudeESO(torque, dt);
		tauD_hat = tauD_hatnext;
		Omega_hat = Omega_hatnext;
		R_hat = R_hatnext;

	}
	if ((time-takeoff_time)/1000000>5){
		tauD_rejection = tauD_hat;

	}

	OmegadPrev = Omegad;
	//if (!landed) {
	//	updateIntegral(rate_error, dt);
	//}

	return torque;
}

void RateControl::updateIntegral(Vector3f &rate_error, const float dt)
{
	for (int i = 0; i < 3; i++) {
		// prevent further positive control saturation
		if (_control_allocator_saturation_positive(i)) {
			rate_error(i) = math::min(rate_error(i), 0.f);
		}

		// prevent further negative control saturation
		if (_control_allocator_saturation_negative(i)) {
			rate_error(i) = math::max(rate_error(i), 0.f);
		}

		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		float i_factor = rate_error(i) / math::radians(400.f);
		i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

		// Perform the integration using a first order method
		float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

		// do not propagate the result if out of range or invalid
		if (PX4_ISFINITE(rate_i)) {
			_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
		}
	}
}

void RateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status)
{
	rate_ctrl_status.rollspeed_integ = _rate_int(0);
	rate_ctrl_status.pitchspeed_integ = _rate_int(1);
	rate_ctrl_status.yawspeed_integ = _rate_int(2);
}



void RateControl::AttitudeESO(matrix::Vector3f tau, float dt){
	Vector3f eR = MatrixfSO3(R_hat.transpose()*R_).sKgenerator();
	MatrixfSO3 eQ;
	Matrix3f H;
	H.HouseHolder(eR,1.0f-1.0f/_p);
	eQ = R_hat.transpose()*R_;
	Vector3f eOmega = Omega_ - eQ.transpose()*Omega_hat;
	//Vector3f ew = eQ.wGenerator(eOmega);
	Vector3f psia = eOmega + kappa_a*(eR+pow(eR.norm_squared(),1.0f/_p - 1.0f)*eR);
	R_hatnext = R_hat.expmso3(Vector3f(Omega_hat*dt));
	//Vector3f Omegahatlaw = eQ*inv(J_)*(Vector3f(J_*Omega_).cross(Omega_) +k_a1*J_*psi_a+ tauD_hat + kappa_a*J_* Vector3f (eQ.wGenerator(eOmega))  )+eQ*(eOmega.skew())*eQ.transpose()*Omega_hat;
	//Vector3f tauDhatlaw = k_a2*J_*psi_a;
	/***************************************************/
	Omega_hatnext = Omega_hat
	+ dt*Vector3f(eQ*inv(J_)*(Vector3f(J_*Omega_).cross(Omega_) +k_a1*J_*phi1(psia)+ tau + tauD_hat
	+ kappa_a*J_* Vector3f (eQ.wGenerator(eOmega)) + kappa_a*J_*pow(eR.norm_squared(),1.0f/_p-1.0f)* Vector3f(eQ.wGenerator(eOmega)) )
	+ eQ*(eOmega.skew())*eQ.transpose()*Omega_hat) ;
	/***************************************************/
	tauD_hatnext = tauD_hat
	+ dt*Vector3f(k_a2*J_*phi2(psia));
	//PX4_INFO("tauD_hat:\t%8.4f\t%8.4f\t%8.4f",
	//				(double)tauD_hat(0),
	//				(double)tauD_hat(1),
	//				(double)tauD_hat(2));
	return;
}


matrix::Vector3f RateControl::phi1(matrix::Vector3f e1){
	Vector3f output;
	output = k_a3 * e1 + pow(e1.norm_squared(), (1.0f-_p)/(3.0f*_p -2.0f))*e1;
	return output;
}

matrix::Vector3f RateControl::phi2(matrix::Vector3f e1){
	Vector3f output;
	output = pow(k_a3, 2.0f) * e1
	+ ((2.0f*k_a3*(2.0f*_p-1.0f))/(3.0f*_p-2.0f))*pow(e1.norm_squared(), (1.0f-_p)/(3.0f*_p -2.0f))*e1
	+ (_p/(3.0f*_p-2.0f))*pow(e1.norm_squared(), 2*(1.0f-_p)/(3.0f*_p -2.0f))*e1;
	return output;
}
