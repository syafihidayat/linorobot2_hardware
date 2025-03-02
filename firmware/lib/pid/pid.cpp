// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Arduino.h"
#include "pid.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd)
 : min_val_(min_val), max_val_(max_val), KP(kp),KI(ki),KD(kd)
{
}

void PID::paramater(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PID::paramaterT(float kp_, float ki_, float kd_)
{
    kpT = kp_;
    kiT = ki_;
    kdT = kd_;
}

float PID::control_angle(float target, float enc,float pwm, float deltaT)
{
    deg2target = target / 360 * 3840;

    err.propostional = deg2target - enc;
    
    err.integral += err.propostional * deltaT;
    err.derivative = (err.propostional - err.previous) / deltaT;
    err.previous = err.propostional;
    err.u = KP * err.propostional + KI * err.integral + KD * err.derivative;
    return fmax(-1 * pwm, fmin(err.u, pwm));
}

float PID::control_angle_speed(float target_angle,float target_speed,float enc,float deltaT){

    deg2target = target_angle / 360 *3840;

    err.propostional = deg2target - enc;
    err.integral += err.propostional * deltaT;

    err.derivative = (err.propostional - err.previous);
    err.previous = err.propostional;

    err.u = KP * err.propostional + KI * err.integral + KD * err.derivative;
    return control_speed(target_speed, enc, deltaT);

}

float PID::control_base(float error, float speed, int condition, float deltaT)
{
    if (condition)
    {
        eProportional = error;
        if (eProportional > 180)
        {
            eProportional -= 360;
        }
        else if (eProportional < -180)
        {
            eProportional += 360;
        }
    }
    else
    {
        eProportional = error;
    }
    eIntegral += eProportional * deltaT;
    float eDerivative = (eProportional - prevError) / deltaT;
    prevError = eProportional;
    float u = kp * eProportional + ki * eIntegral + kd * eDerivative;
    float uT = kpT * eProportional + kiT * eIntegral + kdT * eDerivative;
    return condition ? uT : u;
}

float PID::control_speed(float target, float enc, float deltaT)
  {
    /*convert nilai*/
    radian = (enc - encPrev) / deltaT; // encoder count menjadi encoder count/second
    encPrev = enc;
    angular_vel = radian / (total_gear_ratio * enc_ppr); // convert encoder count/second jadi radian/second

    // masukkan ke low pass filter untuk memperbagus hasil
    angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
    angular_vel_Prev = angular_vel;

    /*hitung pid*/
    // ubah angular_vel_Filt menjadi angular_vel jika ingin menghitung tanpa filter
    float error = target - angular_vel_Filt;

    error_integral += error * deltaT;

    float error_derivative = (error - error_previous) / deltaT;
    error_previous = error;

    float u = KP * error + KI * error_integral + KD * error_derivative;
    return fmax(min_val_, fmin(u, max_val_));
  }

float PID::control_default(float target, float curr, float deltaT)
{
    float error = target - curr;

    error_integral += error * deltaT;

    float error_derivative = (error - error_previous) / deltaT;
    error_previous = error;

    float u = KP * error + KI * error_integral + KD * error_derivative;
    return fmax(min_val_, fmin(u, max_val_));
}
float PID::get_lowPass(float lowpass_input)
{
    lowpass_filt = 0.854 * lowpass_filt + 0.0728 * lowpass_input + 0.0728 * lowpass_prev;
    lowpass_prev = lowpass_input;

    return lowpass_filt;
}
float PID::get_deg2Target()const
{
    return deg2target;
}
float PID::get_error() const
{
    return err.propostional;
}
 float PID::get_error_int() const
  {
    return err.integral;
  }

  float PID::get_error_der() const
  {
    return err.derivative;
  }

  float PID::get_pid_out() const
  {
    return err.u;
  }
float PID::get_filt_vel() const
{
    return angular_vel_Filt;
}
float PID::get_vel() const
{
    return angular_vel;
}
float PID::get_enc_vel() const
{
    return radian;
}
