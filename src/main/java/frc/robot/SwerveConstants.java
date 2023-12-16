package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    //TODO make these classes not exist or not bad
    public static class CAN {
        public static final int FRONT_LEFT_DRIVE = 5;
        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int BACK_LEFT_DRIVE = 9;
        public static final int BACK_RIGHT_DRIVE = 3;
        public static final int FRONT_LEFT_TURN = 6;
        public static final int FRONT_RIGHT_TURN = 8;
        public static final int BACK_LEFT_TURN = 2;
        public static final int BACK_RIGHT_TURN = 4;
        public static final int GYRO = 10;
        public static final int CANDLE = 11;
    }

    public static class PID {
        public static final double DRIVE_P = 0.04;
        public static final double DRIVE_I = 0.0;
        public static final double DRIVE_D = 0.0;
        public static final double DRIVE_FF = 1.0;
        public static final double DRIVE_MIN_OUTPUT = -1.0;
        public static final double DRIVE_MAX_OUTPUT = 1.0;

        public static final double TURN_P = 1.0;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.0;
        public static final double TURN_FF = 0.0;
        public static final double TURN_MIN_OUTPUT = -1;
        public static final double TURN_MAX_OUTPUT = 1;

        public static final boolean TURN_IS_INVERTED = false;
        public static final double TURN_MIN_PID_INPUT = 0.0;
        public static final double TURN_MAX_PID_INPUT = 0.0; //set to clicks to radian rate

        public static final double FRONT_LEFT_ANGLE_OFFSET = -90.0;
        public static final double FRONT_RIGHT_ANGLE_OFFSET = 0.0;
        public static final double BACK_LEFT_ANGLE_OFFSET = 0.0;
        public static final double BACK_RIGHT_ANGLE_OFFSET = 0.0;
    }

    public static class CONVERSION {
        //TODO Find whatever these are for the wheels
        public static final double DRIVE_ROTATIONS_TO_METERS = 1.0;
        public static final double DRIVE_RPM_TO_MPS = 1.0;
        public static final double TURN_CLICKS_TO_RADIANS = (2 * Math.PI);
        public static final double TURN_RPM_TO_RPS = (2 * Math.PI) / 60;

    }

    public static class MISC {
        public static final int DRIVE_AMP_LIMIT = 50;
        public static final int TURN_AMP_LIMIT = 20;

        public static final double DIRECTION_SLEW_RATE = 1.2;
        public static final double MAGNITUDE_SLEW_RATE = 1.8;
        public static final double ROTATIONAL_SLEW_RATE = 2.0;

        //Distance between centers of right and left wheels
        public static final double TRACK_WIDTH = Units.inchesToMeters(27.0);
        //Distance between front and back wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(27.0);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
        public static final boolean GYRO_REVERSED = false;
        public static final double MAX_SPEED = 4.8;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
        public static final double DRIVE_DEADBAND = 0.05;
    }
}
