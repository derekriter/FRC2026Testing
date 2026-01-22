package frc.robot.units;

import frc.robot.constants.ClimberConstants;
import frc.robot.utils.Logger;
import java.util.Optional;

public class ClimbLeverAngle extends SubsytemUnit {

    private ClimbLeverAngle(double universal) {
        super(universal);
    }

    public static ClimbLeverAngle fromMotorRotations(double rots) {
        return new ClimbLeverAngle(rots);
    }

    public static ClimbLeverAngle fromLeverRotations(double rots) {
        return new ClimbLeverAngle(rots * ClimberConstants.Lever.gearRatio);
    }

    public static ClimbLeverAngle fromLeverDegrees(double deg) {
        return fromLeverRotations(deg / 360d);
    }

    public static ClimbLeverAngle fromState(ClimbState state) {
        switch (state) {
            default:
            case OPEN: {
                return ClimberConstants.Lever.open;
            }
            case CLOSED: {
                return ClimberConstants.Lever.closed;
            }
        }
    }

    public double getAsMotorRotations() {
        return universalUnits;
    }

    public double getAsLeverRotations() {
        return getAsMotorRotations() / ClimberConstants.Lever.gearRatio;
    }

    public double getAsLeverDegrees() {
        return getAsLeverRotations() * 360;
    }

    public Optional<ClimbState> getAsState() {
        final double rots = getAsMotorRotations();
        final double tolerance = ClimberConstants.Lever.targetTolerance.getAsMotorRotations();
        final double open = ClimberConstants.Lever.open.getAsMotorRotations();
        final double closed = ClimberConstants.Lever.closed.getAsMotorRotations();

        if (rots >= open - tolerance && rots <= open + tolerance) {
            return Optional.of(ClimbState.OPEN);
        } else if (rots >= closed - tolerance && rots <= closed + tolerance) {
            return Optional.of(ClimbState.CLOSED);
        } else {
            return Optional.empty();
        }
    }

    public ClimbLeverAngle add(ClimbLeverAngle b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null ClimbLeverAngle", true);
            return fromMotorRotations(getAsMotorRotations()); // return copy of self
        }

        return fromMotorRotations(getAsMotorRotations() + b.getAsMotorRotations());
    }

    public ClimbLeverAngle subtract(ClimbLeverAngle b) {
        if (b == null) {
            Logger.reportWarning("Cannot subtract a null ClimbLeverAngle", true);
            return fromMotorRotations(getAsMotorRotations()); // return copy of self
        }

        return fromMotorRotations(getAsMotorRotations() - b.getAsMotorRotations());
    }
}
