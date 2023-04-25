#include "main.h"

#define PI 3.1415926535
#define PID_INTEGRAL_LIMIT 50

Drivetrain::Drivetrain(
    pros::MotorGroup* LeftMotors,
    pros::MotorGroup* RightMotors,
    double DriveGearRatio,
    double WheelDiameter,
    double WheelDistance,
    
    pros::ADIEncoder* LeftEncoder, 
    pros::ADIEncoder* RightEncoder, 
    pros::ADIEncoder* BackEncoder, 
    double LeftTrackingDistance,
    double RightTrackingDistance,
    double BackTrackingDistance,
    double LeftEncoderScale, 
    double RightEncoderScale, 
    double BackEncoderScale,
    double LeftEncoderDiameter,
    double RightEncoderDiameter,
    double BackEncoderDiameter,
    double StartOrientation
) {
    this -> LeftMotors = LeftMotors;
    this -> RightMotors = RightMotors;
    this -> DriveGearRatio = DriveGearRatio;
    this -> WheelDistance = WheelDistance;
    this -> LeftEncoder = LeftEncoder;
    this -> RightEncoder = RightEncoder;
    this -> BackEncoder = BackEncoder;
    this -> LeftTrackingDistance = LeftTrackingDistance;
    this -> RightTrackingDistance = RightTrackingDistance;
    this -> BackTrackingDistance = BackTrackingDistance;
    this -> LeftEncoderScale = LeftEncoderScale;
    this -> RightEncoderScale = RightEncoderScale;
    this -> BackEncoderScale = BackEncoderScale;
    this -> LeftEncoderDiameter = LeftEncoderDiameter;
    this -> RightEncoderDiameter = RightEncoderDiameter;
    this -> BackEncoderDiameter = BackEncoderDiameter;

    this -> lastResetOrientation = StartOrientation;
    this -> orientation = StartOrientation;

    pros::Task SensorUpdateTask([this]() -> void {
        this -> update();
    }, "SensorUpdateTask");
}

void Drivetrain::initPID(double kP, double kI, double kD) {
    this -> kP = kP;
    this -> kI = kI;
    this -> kP = kD;

    PIDActive = true;
}

void Drivetrain::initOdometry() {
    OdometryActive = true;
}

void Drivetrain::move(double distance, double speed, bool waitForCompletion = true) {
    LeftPID.targetPID += distance / DriveGearRatio;
    LeftPID.maxSpeed = speed;
    RightPID.targetPID += distance / DriveGearRatio;
    RightPID.maxSpeed = speed;

    if (waitForCompletion) {
        do {
            pros::delay(1);
        } while (LeftPID.pidActive && RightPID.pidActive);
    }
}

void Drivetrain::turn(double angle, double speed, bool waitForCompletion = true) {
    LeftPID.targetPID += sin(angle) * WheelDistance;
    LeftPID.maxSpeed = speed;
    RightPID.targetPID += -sin(angle) * WheelDistance;
    RightPID.maxSpeed = speed;

    if (waitForCompletion) {
        do {
            pros::delay(1);
        } while (LeftPID.pidActive && RightPID.pidActive);
    }
}

void Drivetrain::turnToPoint(vector2 target, double speed, bool waitForCompletion = true) {
    LeftPID.targetPID += atan((target.y - position.y)/(target.x - position.x)) * WheelDistance;
    LeftPID.maxSpeed = speed;
    RightPID.targetPID += -atan(((target.y - position.y)/(target.x - position.x))) * WheelDistance;
    RightPID.maxSpeed = speed;

    if (waitForCompletion) {
        do {
            pros::delay(1);
        } while (LeftPID.pidActive && RightPID.pidActive);
    }
}

void Drivetrain::moveToPoint(vector2 target, double speed) {
    turnToPoint(target, speed);
    move(sqrt((target.y - position.y)+(target.x - position.x)), speed);
}

void Drivetrain::updateSensorReadings() {
    currentLeftReading = (LeftEncoder->get_value()) * LeftEncoderScale;
    currentRightReading = (RightEncoder->get_value()) * RightEncoderScale;
    currentBackReading = (BackEncoder->get_value()) * BackEncoderScale;
}

void Drivetrain::updatePID(double kP, double kI, double kD) {
    while (true) {
        if (LeftPID.pidActive) {
            double error = currentLeftReading - LeftPID.targetPID;

            if (kI != 0) {
                if( abs(error) < PID_INTEGRAL_LIMIT )
                    LeftPID.pidIntegral += error;
                else
                    LeftPID.pidIntegral = 0;
            } else 
                LeftPID.pidIntegral = 0;

            LeftPID.pidDerivative = LeftPID.pidError - LeftPID.pidLastError;
            LeftPID.pidLastError  = LeftPID.pidError;

            LeftPID.pidDrive = (kP * LeftPID.pidError) + (kI * LeftPID.pidIntegral) + (kD * LeftPID.pidDerivative);

            if (LeftPID.pidDrive > 127)
                LeftPID.pidDrive = 127;
            if (LeftPID.pidDrive < -127)
                LeftPID.pidDrive = -127;

            LeftMotors->move(LeftPID.pidDrive);
        }

        if (RightPID.pidActive) {
            double error = currentRightReading - RightPID.targetPID;

            if (kI != 0) {
                if( abs(error) < PID_INTEGRAL_LIMIT )
                    RightPID.pidIntegral += error;
                else
                    RightPID.pidIntegral = 0;
            } else 
                RightPID.pidIntegral = 0;

            RightPID.pidDerivative = RightPID.pidError - RightPID.pidLastError;
            RightPID.pidLastError  = RightPID.pidError;

            RightPID.pidDrive = (kP * RightPID.pidError) + (kI * RightPID.pidIntegral) + (kD * RightPID.pidDerivative);

            if (RightPID.pidDrive > 127)
                RightPID.pidDrive = 127;
            if (RightPID.pidDrive < -127)
                RightPID.pidDrive = -127;

            RightMotors->move(RightPID.pidDrive);
        }
    }
}

void Drivetrain::updateOdometry() {
    // https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

    double changeInLeft = ((currentLeftReading - oldLeftReading)/360)*LeftEncoderDiameter*PI;
    double changeInRight = ((currentRightReading - oldRightReading)/360)*RightEncoderDiameter*PI;
    double changeInBack = ((currentBackReading - oldBackReading)/360)*BackEncoderDiameter*PI;

    double changeInOrientation = (changeInLeft-changeInRight) / (LeftTrackingDistance+RightTrackingDistance);
    orientation += changeInOrientation;

    double radius = changeInRight/changeInOrientation + RightTrackingDistance;
    double arcChordLength = 2 * sin(changeInOrientation/2);

    double x = arcChordLength * changeInBack / changeInOrientation + BackTrackingDistance;
    double y = arcChordLength * changeInRight / changeInOrientation + RightTrackingDistance;
    double r = sqrt(pow(x, 2.0) + pow(y, 2.0));
    double angle = atan2(y, x)-changeInOrientation;

    position.x += r*cos(angle);
    position.y += r*sin(angle);

    // double averageOrientation = lastResetOrientation + (changeInOrientation/2);
}

void Drivetrain::update() {
    updateSensorReadings();

    if (PIDActive) { updatePID(kP, kI, kD); }
    if (OdometryActive) { updateOdometry(); }
}
