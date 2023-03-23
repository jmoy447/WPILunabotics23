
#include <frc/Joystick.h>

#include <frc/TimedRobot.h>

#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include <frc/Timer.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <iostream>

#include "ctre/Phoenix.h"

using namespace std;

class Helen_Test : public frc::TimedRobot {

  NT_Subscriber ySub;

  static const int leftLeadDeviceID = 4, leftFollowDeviceID = 3, rightLeadDeviceID = 2, rightFollowDeviceID = 1;

  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  TalonSRX linear_slider = {1};
  
  rev::SparkMaxRelativeEncoder leftLead_encoder = m_leftLeadMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder rightLead_encoder = m_rightLeadMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder leftFollower_encoder = m_leftFollowMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder rightFollower_encoder = m_rightFollowMotor.GetEncoder();

  float rightSpeed = 0.6;

  float leftSpeed = 0.6;

  bool hardStop = false;

  vector<int> speeds;
  
  

  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
// frc::DifferentialDrive::TankDrive m_robotDrive;
 private:

    int enc_res = 42;
    int wheelbase = 20; // in
    int wheel_diam = 10; // in
    float right_speed = 0.0;
    float left_speed = 0.0;

    // Helen_Test() {}

    void RobotInit() override{
        m_rightFollowMotor.Follow(m_rightLeadMotor, false);
        m_leftFollowMotor.Follow(m_leftLeadMotor, false);
        m_rightLeadMotor.SetInverted(true);
        // m_robotDrive.SetSafetyEnabled(false);
        // units::unit_t<units::time::second, double, units::linear_scale> experation;
        units::second_t experation(1.5);
        m_robotDrive.SetExpiration(experation);
        

    }
    //pi per second at top speed
    //pi/2 per second at half speed
    //linear

    //encoder resolution: 42
    //wheelbase: 20inches 
    //diameter: 10inches 

   
    void AutonomousPeriodic() override {
        // drive(-20);
        // m_robotDrive.TankDrive(0.5, 0.0);
        // m_leftLeadMotor.Set(0.5);
        // turn(90);
        // drive(-50);
        // turn(45);
        // linear_slider.Set(ControlMode::PercentOutput, 10);
        localization();
    //    print_encoders();
    }

    void localization() {
        double dist_goal = 2;
        double x_val = frc::SmartDashboard::GetNumber("X", 0);
        double y_val = frc::SmartDashboard::GetNumber("Y", 0);

        while((x_val >= dist_goal + 0.2 || x_val >= dist_goal - 0.2) && (y_val >= dist_goal + 0.2 || y_val >= dist_goal - 0.2)) {
            m_robotDrive.TankDrive(0.5, -0.5);
        }
        m_robotDrive.TankDrive(0,0);

        std::cout << "X " << x_val << std::endl;

    }

    void clearEncoders(){
        leftLead_encoder.SetPosition(0);
        leftFollower_encoder.SetPosition(0);
        rightLead_encoder.SetPosition(0);
        rightFollower_encoder.SetPosition(0);
    }

    void turn(float angle) {
        float encoder_goal = abs(((angle/360)*(3.14*wheel_diam) * enc_res))/2;
        float speed = 0.4;
        if(angle > 0) {
            while((leftLead_encoder.GetPosition()<= encoder_goal + 5|| leftLead_encoder.GetPosition() <= encoder_goal - 5)  && (abs(rightLead_encoder.GetPosition())<= encoder_goal -5 || abs(rightLead_encoder.GetPosition())<= encoder_goal + 5)) {
                int errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                int errorRight = encoder_goal - abs(rightLead_encoder.GetPosition());
                int k = 0.5;

                // m_leftLeadMotor.Set(speed - (errorLeft * k));
                float speedL = speed - (errorLeft * k);
                // m_rightLeadMotor.Set(-speed - (errorRight * k));
                float speedR = -speed - (errorRight * k);
                m_robotDrive.TankDrive(speedL, speedR);
            }
        } else if (angle < 0) {
            while(leftLead_encoder.GetPosition()>= encoder_goal || rightLead_encoder.GetPosition()<= abs(encoder_goal)) {
                int errorLeft = abs(encoder_goal) - abs(leftLead_encoder.GetPosition());
                int errorRight = abs(encoder_goal) - abs(rightLead_encoder.GetPosition());
                int k = 0.2;

                // m_leftLeadMotor.Set(-speed - (errorLeft * k));
                float speedL = -speed - (errorLeft * k);
                // m_rightLeadMotor.Set(speed - (errorRight * k));
                float speedR = speed - (errorRight * k);
                m_robotDrive.TankDrive(speedL,speedR);
            }
        } else {
            // m_leftLeadMotor.Set(0);
            // m_rightLeadMotor.Set(0);
            m_robotDrive.TankDrive(0.0,0.0);
        }
    } 

    void arc_turn_left(float radius, float time, int theta) {

        float radius_left = radius - (wheelbase/2);
        float radius_right = radius + (wheelbase/2);

        float dist_left = theta * radius_left;
        float dist_right = theta * radius_right;

        float speed_left = dist_left/(2*3.14*(wheel_diam/2)*time); //rad/sec
        float speed_right = dist_right/(2*3.14*(wheel_diam/2)*time); //rad/sec

        auto prev_time = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::now();
        auto elapsed_seconds = now_time - prev_time;
        while(elapsed_seconds.count() < time){
            float speedL = speed_left/3.14;
            float speedR = speed_right/3.14;
            std::cout << "SpeedL: " << speedL << std::endl;
            std::cout << "SpeedR: " << speedR << std::endl;
            // m_leftLeadMotor.Set(speedL);
            // m_rightLeadMotor.Set(speedR);
            m_robotDrive.TankDrive(speedL,speedR);
            auto now_time = std::chrono::system_clock::now();
            auto elapsed_seconds = now_time - prev_time;
        }
        // m_leftLeadMotor.Set(0);
        // m_rightLeadMotor.Set(0);
        m_robotDrive.TankDrive(0.0,0.0);
    }

    void arc_turn_right(float radius, float time, chrono::milliseconds time_ms, float theta) {
        float radius_left = radius + (wheelbase/2);
        float radius_right = radius - (wheelbase/2);

        float dist_left = theta * radius_left;
        float dist_right = theta * radius_right;

        float speed_left = dist_left/(2*3.14*(wheel_diam/2)*time); //rad/sec
        float speed_right = dist_right/(2*3.14*(wheel_diam/2)*time); //rad/sec

        auto now_time = std::chrono::high_resolution_clock::now();
        auto set_time = now_time + time_ms;
        bool should_turn = now_time < set_time;
        std::cout << "duration: " << std::chrono::duration_cast<chrono::milliseconds>(set_time - now_time).count() << std::endl;
        std::cout << "should turn: " << should_turn << std::endl;
        while(should_turn){
            float speedL = speed_left/3.14;
            float speedR = speed_right/3.14;
            // std::cout << "time: " << now_time << std::endl;
            // std::cout << "duration: " << duration << std::endl;
            // m_leftLeadMotor.Set(speedL);
            // m_rightLeadMotor.Set(speedR);
            m_robotDrive.TankDrive(speedL,speedR);
            now_time = std::chrono::high_resolution_clock::now();
            // double duration = (double)(set_time - now_time) / (double) CLOCKS_PER_SEC;
            should_turn = now_time < set_time;
        }
        // m_leftLeadMotor.Set(0);
        // m_rightLeadMotor.Set(0);

        m_robotDrive.TankDrive(0.0,0.0);
        sleep(1);
    }

    bool drive (float distance) {
        
        double encoder_goal = leftLead_encoder.GetPosition() + (distance/(3.14*wheel_diam) * enc_res);
        
        if(encoder_goal > 0) {
            float speed = 0.5;
            std::cout << "GETPOSITION " << leftLead_encoder.GetPosition() << " ENCODERGOAL " << encoder_goal << std::endl;
            while((leftLead_encoder.GetPosition() <= encoder_goal + 50 || leftLead_encoder.GetPosition() <=encoder_goal - 50) && (rightLead_encoder.GetPosition() <= encoder_goal + 50 || rightLead_encoder.GetPosition() <= encoder_goal- 50)) {
                double errorLeft = encoder_goal - leftLead_encoder.GetPosition();
                double errorRight = encoder_goal - rightLead_encoder.GetPosition();
                int k = 0.5;
                // std::cout << "GETPOSITION " << leftLead_encoder.GetPosition() << " ENCODERGOAL " << encoder_goal << std::endl;
                // m_leftLeadMotor.Set(speed - (errorLeft * k));
                float speedL = speed - (errorLeft * k);
                // m_rightLeadMotor.Set(speed - (errorRight * k));
                float speedR = speed - (errorRight * k);
                // std::cout << "GETPOSITION " << leftLead_encoder.GetPosition() << " ENCODERGOAL " << encoder_goal << std::endl;
                // std::cout << "SPEED " << speedL << std::endl;
                m_robotDrive.TankDrive(speedL,speedR);
            }
            // m_leftLeadMotor.Set(0);
            // m_rightLeadMotor.Set(0);
            m_robotDrive.TankDrive(0.0,0.0);
            sleep(3);
            std::cout << "RETURNING" << std::endl;

            return true;
        }
        else {
            float speed = 0.3;
            while((leftLead_encoder.GetPosition() >= encoder_goal + 50 || leftLead_encoder.GetPosition() >= encoder_goal - 50) && (rightLead_encoder.GetPosition() >= encoder_goal + 50 || rightLead_encoder.GetPosition() >= encoder_goal - 50)) {
                int errorLeft = abs(encoder_goal) - abs(leftLead_encoder.GetPosition());
                int errorRight = abs(encoder_goal) - abs(rightLead_encoder.GetPosition());
                int k = 0.2;

                // m_leftLeadMotor.Set(-speed - (errorLeft * k));
                float speedL = -speed - (errorLeft * k);
                // m_rightLeadMotor.Set(-speed - (errorRight * k));
                float speedR = -speed - (errorRight * k);
                m_robotDrive.TankDrive(speedL, speedR);

            }
            // m_leftLeadMotor.Set(0);
            // m_rightLeadMotor.Set(0);
            m_robotDrive.TankDrive(0.0,0.0);
            return true;

        }
    }

    void print_encoders() {
        frc::SmartDashboard::PutNumber("Left Lead Encoder Position", leftLead_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("Right Lead Position", rightLead_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("Left Follower Position", leftFollower_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("Right Follower Position", rightLead_encoder.GetPosition());
    }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Helen_Test>(); }
#endif



