#include "main.h"

#define vector2 std::vector<double>

#define PID_INTEGRAL_LIMIT 50

typedef struct {
    double kP;
    double kI;
    double kD;
} pidArgs;

class Drivetrain {
    public:
        // Drivetrain

        pros::MotorGroup* LeftMotors;
        pros::MotorGroup* RightMotors;
        double DriveGearRatio;

        // PID

        double targetPID;
        double targetAnglePID;

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

        std::vector<double> position;
        double orientation;

        int32_t oldLeftReading;
        int32_t oldRightReading;
        int32_t oldBackReading;

        int32_t currentLeftReading;
        int32_t currentRightReading;
        int32_t currentBackReading;

        double lastResetOrientation;

        Drivetrain(
            pros::MotorGroup* LeftMotors,
            pros::MotorGroup* RightMotors,
            double DriveGearRatio,
            
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
        ) {
            this -> LeftMotors = LeftMotors;
            this -> RightMotors = RightMotors;
            this -> DriveGearRatio = DriveGearRatio;
            this -> LeftEncoder = LeftEncoder;
            this -> RightEncoder = RightEncoder;
            this -> BackEncoder = BackEncoder;
            this -> LeftTrackingDistance = LeftTrackingDistance;
            this -> RightTrackingDistance = RightTrackingDistance;
            this -> BackTrackingDistance = BackTrackingDistance;
            this -> LeftEncoderScale = LeftEncoderScale;
            this -> RightEncoderScale = RightEncoderScale;
            this -> BackEncoderScale = BackEncoderScale;

            this -> lastResetOrientation = StartOrientation;
            this -> orientation = StartOrientation;
        }

        void initPID(double kP, double kI, double kD) {
            pros::Task PIDTask([this, kP, kI, kD] {
                this -> updatePID(kP, kI, kD);
            }, "PIDTask");
        }

        void initOdometry() {
            pros::Task OdometryTask([this]() -> void {
                this -> updateOdometry();
            }, "OdometryTask");
        }

        void turnToPoint(vector2 target) {

        }

    private:
        void updatePID(double kP, double kI, double kD) {
            typedef struct pidVars  {
                bool pidActive = false;
                double pidError;
                double pidDerivative;
                double pidDrive;

                double pidLastError = 0;
                double pidIntegral = 0;

                double targetPID;
            } pidVars;
            
            pidVars LeftPID;
            pidVars RightPID;

            while (true) {
                if (LeftPID.pidActive) {
                    int32_t avgEncoderReading = (currentLeftReading+currentRightReading) / 2;

                    double error = avgEncoderReading - LeftPID.targetPID;

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
                    int32_t avgEncoderReading = (currentLeftReading+currentRightReading) / 2;

                    double error = avgEncoderReading - RightPID.targetPID;

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

        void updateOdometry() {
            // https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

            currentLeftReading = (LeftEncoder -> get_value()) * LeftEncoderScale;
            currentRightReading = (RightEncoder -> get_value()) * RightEncoderScale;
            currentBackReading = (BackEncoder -> get_value()) * BackEncoderScale;

            double changeInLeft = currentLeftReading - oldLeftReading;
            double changeInRight = currentRightReading - oldRightReading;
            double changeInBack = currentBackReading - oldBackReading;

            double changeInOrientation = (changeInLeft-changeInRight) / (LeftTrackingDistance+RightTrackingDistance);
            orientation += changeInOrientation;

            double radius = changeInRight/changeInOrientation + RightTrackingDistance;
            double arcChordLength = 2 * sin(changeInOrientation/2);

            double x = arcChordLength * changeInBack / changeInOrientation + BackTrackingDistance;
            double y = arcChordLength * changeInRight / changeInOrientation + RightTrackingDistance;
            double r = sqrt(pow(x, 2.0) + pow(y, 2.0));
            double angle = atan2(y, x)-changeInOrientation;

            position[0] += r*cos(angle);
            position[1] += r*sin(angle);

            double averageOrientation = lastResetOrientation + (changeInOrientation/2);
        }
};