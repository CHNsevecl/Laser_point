#include "PID.hpp"

// 默认构造函数
PIDController::PIDController() 
    : kp_(1.0), ki_(0.0), kd_(0.0),
      integral_(0.0), prev_error_(0.0), prev_output_(0.0),
      min_output_(-INFINITY), max_output_(INFINITY),
      min_integral_(-INFINITY), max_integral_(INFINITY),
      first_run_(true), enable_integral_limit_(true) {
}

// 带参数的构造函数
PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd),
      integral_(0.0), prev_error_(0.0), prev_output_(0.0),
      min_output_(-INFINITY), max_output_(INFINITY),
      min_integral_(-INFINITY), max_integral_(INFINITY),
      first_run_(true), enable_integral_limit_(true) {
}

// 设置PID增益
void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setKp(double kp) {
    kp_ = kp;
}

void PIDController::setKi(double ki) {
    ki_ = ki;
}

void PIDController::setKd(double kd) {
    kd_ = kd;
}

// 设置输出限制
void PIDController::setOutputLimits(double min, double max) {
    if (min > max) {
        std::swap(min, max);
    }
    min_output_ = min;
    max_output_ = max;
}

// 设置积分限制
void PIDController::setIntegralLimits(double min, double max) {
    if (min > max) {
        std::swap(min, max);
    }
    min_integral_ = min;
    max_integral_ = max;
}

// 设置积分限制使能
void PIDController::enableIntegralLimit(bool enable) {
    enable_integral_limit_ = enable;
}

// 重置PID状态
void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    prev_output_ = 0.0;
    first_run_ = true;
}

// 计算PID输出（固定时间步长）
double PIDController::calculate(double setpoint, double measurement) {
    double error = setpoint - measurement;
    
    // 比例项
    double proportional = kp_ * error;
    
    // 积分项
    if (ki_ != 0.0) {
        integral_ += error * dt_;
        
        // 应用积分限制
        if (enable_integral_limit_) {
            integral_ = std::clamp(integral_, min_integral_, max_integral_);
        }
    }
    double integral = ki_ * integral_;
    
    // 微分项
    double derivative = 0.0;
    if (kd_ != 0.0 && !first_run_) {
        derivative = kd_ * (error - prev_error_) / dt_;
    }
    
    // 计算输出
    double output = proportional + integral + derivative;
    
    // 应用输出限制和积分抗饱和
    if (output > max_output_) {
        output = max_output_;
        // 抗饱和：如果输出被限制，停止积分增长
        if (ki_ != 0.0 && error > 0) {
            integral_ -= error * dt_; // 回退积分
        }
    } else if (output < min_output_) {
        output = min_output_;
        // 抗饱和：如果输出被限制，停止积分减小
        if (ki_ != 0.0 && error < 0) {
            integral_ -= error * dt_; // 回退积分
        }
    }
    
    // 更新状态
    prev_error_ = error;
    prev_output_ = output;
    first_run_ = false;
    
    return output;
}

// 计算PID输出（自动计算时间步长）
double PIDController::calculateAuto(double setpoint, double measurement) {
    auto now = std::chrono::steady_clock::now();
    
    if (first_run_) {
        last_time_ = now;
        dt_ = 0.001; // 默认时间步长
    } else {
        dt_ = std::chrono::duration<double>(now - last_time_).count();
        // 限制最大时间步长
        if (dt_ > 0.1) {
            dt_ = 0.1;
        }
    }
    
    last_time_ = now;
    
    return calculate(setpoint, measurement);
}

// 设置积分项
void PIDController::setIntegral(double integral) {
    integral_ = integral;
    if (enable_integral_limit_) {
        integral_ = std::clamp(integral_, min_integral_, max_integral_);
    }
}