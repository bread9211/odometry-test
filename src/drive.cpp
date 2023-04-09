#include <main.h>

#define vector2 std::vector<double>

#define PID_INTEGRAL_LIMIT 50

typedef struct {
    double kP;
    double kI;
    double kD;
} pidArgs;

class Drive {
    public:
        // Drivetrain

        pros::MotorGroup* LeftMotors;
        pros::MotorGroup* RightMotors;
        double DriveGearRatio;

        // PID

        double targetPID;

        bool pidActive;

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

        Drive(
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
            pros::Task OdometryTask(updateOdometry, "OdomTask");
        }

        void turnToPoint(vector2 target) {

        }

    private:
        void updatePID(double kP, double kI, double kD) {
            double  pidError;
            double  pidDerivative;
            double  pidDrive;

            double pidLastError = 0;
            double pidIntegral = 0;

            while (true) {
                if (pidActive) {
                    int32_t avgEncoderReading = (currentLeftReading+currentRightReading) / 2;

                    double error = avgEncoderReading - targetPID;

                    if (kI != 0) {
                        if( abs(error) < PID_INTEGRAL_LIMIT )
                            pidIntegral += error;
                        else
                            pidIntegral = 0;
                    } else 
                        pidIntegral = 0;

                    pidDerivative = pidError - pidLastError;
                    pidLastError  = pidError;

                    pidDrive = (kP * pidError) + (kI * pidIntegral) + (kD * pidDerivative);

                    if( pidDrive > 127 )
                        pidDrive = 127;
                    if( pidDrive < -127 )
                        pidDrive = -127;
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