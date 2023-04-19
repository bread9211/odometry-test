#include "main.h"

#define PID_INTEGRAL_LIMIT 50

typedef struct pidVars  {
    bool pidActive = true;
    double pidError;
    double pidDerivative;
    double pidDrive;

    double pidLastError = 0;
    double pidIntegral = 0;

    double targetPID;

    double maxSpeed = 127;
} pidVars;

typedef struct vector2 {
    double x = 0;
    double y = 0;
} vector2;

class Drivetrain {
    public:
        // Drivetrain

        pros::MotorGroup* LeftMotors;
        pros::MotorGroup* RightMotors;
        double DriveGearRatio;
        double WheelDistance;

        // PID
            
        pidVars LeftPID;
        pidVars RightPID;

        double PIDSpeed;

        // Odometry

        pros::ADIEncoder* LeftEncoder;
        double LeftTrackingDistance;
        double LeftEncoderScale;

        pros::ADIEncoder* RightEncoder;
        double RightTrackingDistance;
        double RightEncoderScale;

        pros::ADIEncoder* BackEncoder;
        double BackTrackingDistance;
        double BackEncoderScale;

        vector2 position;
        double orientation;

        int32_t oldLeftReading;
        int32_t oldRightReading;
        int32_t oldBackReading;

        int32_t currentLeftReading;
        int32_t currentRightReading;
        int32_t currentBackReading;

        double lastResetOrientation;

        bool PIDActive = false;
        double kP, kI, kD;

        bool OdometryActive = false;

        Drivetrain(
            pros::MotorGroup* LeftMotors,
            pros::MotorGroup* RightMotors,
            double DriveGearRatio,
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
            double StartOrientation
        ) {}

        void initPID(double kP, double kI, double kD) {}

        void initOdometry() {}

        void move(double distance, double speed, bool waitForCompletion = true) {}

        void turn(double angle, double speed, bool waitForCompletion = true) {}

        void turnToPoint(vector2 target, double speed, bool waitForCompletion = true) {}

        void moveToPoint(vector2 target, double speed) {}

    private:
        void updateSensorReadings() {}

        void updatePID(double kP, double kI, double kD) {}

        void updateOdometry() {}

        void update() {}
};