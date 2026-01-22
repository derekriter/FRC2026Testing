package frc.robot.units;

import frc.robot.constants.ClimberConstants;
import frc.robot.utils.Logger;
import java.util.Optional;

public class ClimbClampAngle extends SubsytemUnit {

    private ClimbClampAngle(double universal) {
        super(universal);
    }

    public static ClimbClampAngle fromMotorRotations(double rots) {
        return new ClimbClampAngle(rots);
    }

    public static ClimbClampAngle fromClampRotations(double rots) {
        return new ClimbClampAngle(rots * ClimberConstants.Clamp.gearRatio);
    }

    public static ClimbClampAngle fromClampDegrees(double deg) {
        return fromClampRotations(deg / 360d);
    }

    public static ClimbClampAngle fromState(ClimbState state) {
        switch (state) {
            default:
            case OPEN: {
                return ClimberConstants.Clamp.open;
            }
            case CLOSED: {
                return ClimberConstants.Clamp.closed;
            }
        }
    }

    public double getAsMotorRotations() {
        return universalUnits;
    }

    public double getAsClampRotations() {
        return getAsMotorRotations() / ClimberConstants.Clamp.gearRatio;
    }

    public double getAsClampDegrees() {
        return getAsClampRotations() * 360;
    }

    public Optional<ClimbState> getAsState() {
        final double rots = getAsMotorRotations();
        final double tolerance = ClimberConstants.Clamp.targetTolerance.getAsMotorRotations();
        final double open = ClimberConstants.Clamp.open.getAsMotorRotations();
        final double closed = ClimberConstants.Clamp.closed.getAsMotorRotations();

        if (rots >= open - tolerance && rots <= open + tolerance) {
            return Optional.of(ClimbState.OPEN);
        } else if (rots >= closed - tolerance && rots <= closed + tolerance) {
            return Optional.of(ClimbState.CLOSED);
        } else {
            return Optional.empty();
        }
    }

    public ClimbClampAngle add(ClimbClampAngle b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null ClimbClampAngle", true);
            return fromMotorRotations(getAsMotorRotations()); // return copy of self
        }

        return fromMotorRotations(getAsMotorRotations() + b.getAsMotorRotations());
    }

    public ClimbClampAngle subtract(ClimbClampAngle b) {
        if (b == null) {
            Logger.reportWarning("Cannot subtract a null ClimbClampAngle", true);
            return fromMotorRotations(getAsMotorRotations()); // return copy of self
        }

        return fromMotorRotations(getAsMotorRotations() - b.getAsMotorRotations());
    }
}
