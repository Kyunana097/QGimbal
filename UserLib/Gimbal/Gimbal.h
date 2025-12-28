/**
 * @brief 		Gimbal.h库文件
 * @detail
 * @author 	    Haoqi Liu
 * @date        2025/10/3
 * @version 	V1.0.0
 * @note
 * @warning
 * @par 		历史版本
                V1.0.0创建于2025/10/3
 * */

#ifndef QGIMBAL_GIMBAL_H
#define QGIMBAL_GIMBAL_H

#include "QD4310.h"
#include "PID.h"

class Gimbal {
public:
    Gimbal(QD4310& yawMotor, QD4310& pitchMotor, const float yaw_center, const float pitch_center,
           const PID& PID_yaw_imu, const PID& PID_pitch_imu, const float ctrl_ts) :
        yawMotor(yawMotor), pitchMotor(pitchMotor), PID_yaw_imu(PID_yaw_imu), PID_pitch_imu(PID_pitch_imu), Ts(ctrl_ts),
        yaw_center(yaw_center), pitch_center(pitch_center) {}

    void enable() {
        PID_yaw_imu.target = yaw_imu_angle;
        PID_pitch_imu.target = pitch_imu_angle + pitchMotor.angle / std::numbers::pi_v<float> * 180;
        yawMotor.enable();
        pitchMotor.enable();
        enabled = true;
    }

    void disable() {
        yawMotor.setCurrent(0);
        pitchMotor.setCurrent(0);
        yawMotor.disable();
        pitchMotor.disable();
        enabled = false;
    }

    /**
     * @param yaw_speed yaw轴速度,单位:rpm
     * @param pitch_speed pitch轴速度,单位:rpm
     */
    void Ctrl(const float yaw_speed, const float pitch_speed) {
        target_yaw_speed = yaw_speed;
        target_pitch_speed = pitch_speed;
    }

    /**
     * @param yaw_imu_angle_ imu测量的偏航角度,单位:度
     */
    void Ctrl_ISR(const float yaw_imu_angle_, const float pitch_imu_angle_) {
        static float speed_to_ctrl = 0;
        yaw_imu_angle = yaw_imu_angle_;
        pitch_imu_angle = pitch_imu_angle_ + pitchMotor.angle / std::numbers::pi_v<float> * 180;
        if (!enabled) return;

        PID_yaw_imu.target += target_yaw_speed * Ts * 360 / 60;
        if (PID_yaw_imu.target > 360.0f) {
            PID_yaw_imu.target -= 360.0f;
        } else if (PID_yaw_imu.target < 0.0f) {
            PID_yaw_imu.target += 360.0f;
        }

        // 过零点处理
        if (PID_yaw_imu.target - yaw_imu_angle > 180.0f) {
            speed_to_ctrl = PID_yaw_imu.calc(yaw_imu_angle + 360.0f);
        } else if (PID_yaw_imu.target - yaw_imu_angle < -180.0f) {
            speed_to_ctrl = PID_yaw_imu.calc(yaw_imu_angle - 360.0f);
        } else {
            speed_to_ctrl = PID_yaw_imu.calc(yaw_imu_angle);
        }

        yawMotor.setCurrent(speed_to_ctrl);
        // /*==============pitch轴限位===============*/
        // if ((target_pitch_speed > 0 && pitchMotor.angle > pitch_center + 0.2f) ||
        //     (target_pitch_speed < 0 && pitchMotor.angle < pitch_center - 0.2f))
        //     pitchMotor.setSpeed(0);
        // else pitchMotor.setSpeed(target_pitch_speed);

        if ((target_pitch_speed > 0 && pitchMotor.angle > pitch_center + 0.2f) ||
            (target_pitch_speed < 0 && pitchMotor.angle < pitch_center - 0.2f))
            PID_pitch_imu.target += 0;
        else
            PID_pitch_imu.target += target_pitch_speed * Ts * 360 / 60;
        if (PID_pitch_imu.target > 360.0f) {
            PID_pitch_imu.target -= 360.0f;
        } else if (PID_pitch_imu.target < 0.0f) {
            PID_pitch_imu.target += 360.0f;
        }

        // 过零点处理
        if (PID_pitch_imu.target - pitch_imu_angle > 180.0f) {
            speed_to_ctrl = PID_pitch_imu.calc(pitch_imu_angle + 360.0f);
        } else if (PID_pitch_imu.target - pitch_imu_angle < -180.0f) {
            speed_to_ctrl = PID_pitch_imu.calc(pitch_imu_angle - 360.0f);
        } else {
            speed_to_ctrl = PID_pitch_imu.calc(pitch_imu_angle);
        }

        pitchMotor.setCurrent(speed_to_ctrl);
    }

private:
    QD4310& yawMotor;
    QD4310& pitchMotor;
    PID PID_yaw_imu;
    PID PID_pitch_imu;

    bool enabled{false};
    float Ts;
    float yaw_imu_angle{0};
    float pitch_imu_angle{0};
    float target_yaw_speed{0};
    float target_pitch_speed{0};

    float yaw_center;
    float pitch_center;
};

#endif //QGIMBAL_GIMBAL_H
