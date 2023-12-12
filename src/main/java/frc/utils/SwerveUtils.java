package frc.utils;

public class SwerveUtils {
    public static double StepTowards(double current, double target, double stepSize) {
        if (Math.abs(current - target) <= stepSize) {
            return target;
        }
        else if (target < current) {
            return current - stepSize;
        }
        else {
            return current + stepSize;
        }
    }

    public static double StepTowardsCircular(double current, double target, double stepSize) {
        current = WrapAngle(current);
        target = WrapAngle(target);

        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(current - target);

        if (difference <= stepSize) {
            return target;
        }
        else if (difference > Math.PI) {
            if (current + 2*Math.PI - target < stepSize || target + 2*Math.PI - current < stepSize) {
                return target;
            }
            else {
                return WrapAngle(current - stepDirection * stepSize);
            }
        }
        else {
            return current + stepDirection * stepSize;
        }
    }

    public static double AngleDifference(double angleA, double angleB) {
        double difference = Math.abs(angleA - angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    
    }

    public static double WrapAngle(double angle) {
        double twoPI = 2 * Math.PI;

        if (angle == twoPI) {
            return 0.0;
        }
        else if (angle > twoPI) {
            return angle - twoPI * Math.floor(angle / twoPI);
        }
        else if (angle < 0.0) {
            return angle + twoPI * (Math.floor((-angle / twoPI) + 1));
        }
        else {
            return angle;
        }
    }
}
