package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.units.ElevPosition;
import java.util.Optional;

public class ElevConstants {

    private ElevConstants() {}

    // motor defintion
    public static final int motorID = 3;
    public static final double rotsPerInch = 101.244 / 61;

    // motor config
    public static final double maxCurrent = 30;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final double maxDutyCycle = 1;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0.02;
    public static final double kG = 0.04;
    public static final Optional<ElevPosition> forwardSoftLimit = Optional.of(ElevPosition.fromInches(61));
    public static final Optional<ElevPosition> reverseSoftLimit = Optional.of(ElevPosition.fromRotations(0));
    public static final double maxVoltage = 16;
    public static final boolean forwardHardLimitEnabled = false;
    public static final Optional<ElevPosition> forwardHardLimitResetValue = Optional.empty();
    public static final boolean reverseHardLimitEnabled = true;
    public static final Optional<ElevPosition> reverseHardLimitResetValue = Optional.of(ElevPosition.fromRotations(0));

    // setpoints
    public static final ElevPosition targetTolerance = ElevPosition.fromInches(0.5); // +/- 0.5 inches
    public static final ElevPosition home = ElevPosition.fromRotations(0);
    public static final ElevPosition l1 = ElevPosition.fromInches(12);
    public static final ElevPosition l2 = ElevPosition.fromInches(18);
    public static final ElevPosition l3 = ElevPosition.fromInches(33);
    public static final ElevPosition l4 = ElevPosition.fromInches(58);
}
