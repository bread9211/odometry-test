#include <main.h>

class Odometry {
    public:
        double DriveGearRatio;

        pros::ADIEncoder LeftEncoder;
        double LeftTrackingDistance;
        double LeftEncoderRatio;

        pros::ADIEncoder RightEncoder;
        double RightTrackingDistance;
        double RightEncoderRatio;

        pros::ADIEncoder BackEncoder;
        double BackTrackingDistance;
        double BackEncoderRatio;

        std::vector<double> position;
        double orientation;

        int32_t oldLeftReading;
        int32_t oldRightReading;
        int32_t oldBackReading;

        int32_t currentLeftReading;
        int32_t currentRightReading;
        int32_t currentBackReading;

        double lastResetOrientation;

        Odometry(
            double driveGearRatio, 
            pros::ADIEncoder leftEncoder, 
            pros::ADIEncoder rightEncoder, 
            pros::ADIEncoder backEncoder, 
            double leftTrackingDistance,
            double rightTrackingDistance,
            double backTrackingDistance,
            double leftEncoderRatio, 
            double rightEncoderRatio, 
            double backEncoderRatio,
            double startOrientation
        ) {
            DriveGearRatio = driveGearRatio;
            LeftEncoder = leftEncoder;
            RightEncoder = rightEncoder;
            BackEncoder = backEncoder;
            LeftTrackingDistance = leftTrackingDistance;
            RightTrackingDistance = rightTrackingDistance;
            BackTrackingDistance = backTrackingDistance;
            LeftEncoderRatio = LeftEncoderRatio;
            RightEncoderRatio = rightEncoderRatio;
            BackEncoderRatio = backEncoderRatio;

            lastResetOrientation = startOrientation;
            orientation = startOrientation;
        }

        void update() {
            // https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

            currentLeftReading = LeftEncoder.get_value();
            currentRightReading = RightEncoder.get_value();
            currentBackReading = BackEncoder.get_value();

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