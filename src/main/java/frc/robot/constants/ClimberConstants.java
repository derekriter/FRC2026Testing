package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.units.ClimbClampAngle;
import frc.robot.units.ClimbLeverAngle;
import java.util.Optional;

public class ClimberConstants {

    private ClimberConstants() {}

    public static class Lever {
        // motor defintion
        public static final int motorID = 2;
        public static final double gearRatio = 80d / 1d;

        // motor config
        public static final double maxCurrent = 80;
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
        public static final double maxDutyCycle = 1;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final Optional<ClimbLeverAngle> forwardSoftLimit = Optional.empty();
        public static final Optional<ClimbLeverAngle> reverseSoftLimit = Optional.empty();
        public static final double maxVoltage = 16;
        public static final boolean forwardHardLimitEnabled = true;
        public static final Optional<ClimbLeverAngle> forwardHardLimitResetValue = Optional.empty();
        public static final boolean reverseHardLimitEnabled = false;
        public static final Optional<ClimbLeverAngle> reverseHardLimitResetValue = Optional.empty();

        // setpoints
        public static final ClimbLeverAngle targetTolerance = ClimbLeverAngle.fromLeverDegrees(5); // +/- 5 degrees
        public static final ClimbLeverAngle open = ClimbLeverAngle.fromLeverDegrees(3);
        public static final ClimbLeverAngle closed = ClimbLeverAngle.fromMotorRotations(46);
    }

    public static class Clamp {
        // motor defintion
        public static final int motorID = 3;
        public static final double gearRatio = 40d / 1d;

        // motor config
        public static final double maxCurrent = 40;
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
        public static final double maxDutyCycle = 0.5;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final Optional<ClimbClampAngle> forwardSoftLimit = Optional.empty();
        public static final Optional<ClimbClampAngle> reverseSoftLimit = Optional.empty();
        public static final double maxVoltage = 16;
        public static final boolean forwardHardLimitEnabled = false;
        public static final Optional<ClimbClampAngle> forwardHardLimitResetValue = Optional.empty();
        public static final boolean reverseHardLimitEnabled = false;
        public static final Optional<ClimbClampAngle> reverseHardLimitResetValue = Optional.empty();

        // setpoints
        public static final ClimbClampAngle targetTolerance = ClimbClampAngle.fromClampDegrees(5); // +/- 5 degrees
        public static final ClimbClampAngle open = ClimbClampAngle.fromMotorRotations(0);
        public static final ClimbClampAngle closed = ClimbClampAngle.fromMotorRotations(7.5);
    }
}
