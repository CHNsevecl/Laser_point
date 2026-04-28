#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <chrono>
#include <cmath>
#include <algorithm>

class PIDController {
private:
    // PID参数
    double kp_;      // 比例增益
    double ki_;      // 积分增益
    double kd_;      // 微分增益
    
    // PID状态变量
    double integral_;      // 积分项
    double prev_error_;    // 上一次误差
    double prev_output_;   // 上一次输出
    
    // 限制参数
    double min_output_;    // 输出最小值
    double max_output_;    // 输出最大值
    double min_integral_;  // 积分项最小值
    double max_integral_;  // 积分项最大值
    
    // 时间相关
    std::chrono::steady_clock::time_point last_time_;
    double dt_;            // 时间差
    
    // 控制选项
    bool first_run_;       // 是否第一次运行
    bool enable_integral_limit_;  // 是否启用积分限制
    
public:
    // 构造函数
    PIDController();
    PIDController(double kp, double ki, double kd);
    
    // 设置PID参数
    void setGains(double kp, double ki, double kd);
    void setKp(double kp);
    void setKi(double ki);
    void setKd(double kd);
    
    // 获取PID参数
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }
    
    // 设置输出限制
    void setOutputLimits(double min, double max);
    void setIntegralLimits(double min, double max);
    
    // 设置积分限制使能
    void enableIntegralLimit(bool enable);
    
    // 重置PID状态
    void reset();
    
    // 计算PID输出（基于固定时间步长）
    double calculate(double setpoint, double measurement);
    
    // 计算PID输出（自动计算时间步长）
    double calculateAuto(double setpoint, double measurement);
    
    // 获取当前状态
    double getIntegral() const { return integral_; }
    double getPrevError() const { return prev_error_; }
    
    // 设置积分项（用于手动干预）
    void setIntegral(double integral);
};

#endif // PIDCONTROLLER_H