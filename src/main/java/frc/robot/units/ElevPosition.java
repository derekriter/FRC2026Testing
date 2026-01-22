package frc.robot.units;

import frc.robot.constants.ElevConstants;
import frc.robot.utils.Logger;
import java.util.Optional;

public class ElevPosition extends SubsytemUnit {

    private ElevPosition(double universal) {
        super(universal);
    }

    public static ElevPosition fromRotations(double rots) {
        return new ElevPosition(rots);
    }

    public static ElevPosition fromInches(double inches) {
        return new ElevPosition(inches * ElevConstants.rotsPerInch);
    }

    public static ElevPosition fromLevel(ElevLevel level) {
        switch (level) {
            default:
            case HOME: {
                return ElevConstants.home;
            }
            case L1: {
                return ElevConstants.l1;
            }
            case L2: {
                return ElevConstants.l2;
            }
            case L3: {
                return ElevConstants.l3;
            }
            case L4: {
                return ElevConstants.l4;
            }
        }
    }

    public double getAsRotations() {
        return universalUnits;
    }

    public double getAsInches() {
        return getAsRotations() / ElevConstants.rotsPerInch;
    }

    public Optional<ElevLevel> getAsElevLevel() {
        final double rots = getAsRotations();
        final double tolerance = ElevConstants.targetTolerance.getAsRotations();
        final double home = ElevConstants.home.getAsRotations();
        final double l1 = ElevConstants.l1.getAsRotations();
        final double l2 = ElevConstants.l2.getAsRotations();
        final double l3 = ElevConstants.l3.getAsRotations();
        final double l4 = ElevConstants.l4.getAsRotations();

        if (rots >= home - tolerance && rots <= home + tolerance) {
            return Optional.of(ElevLevel.HOME);
        } else if (rots >= l1 - tolerance && rots <= l1 + tolerance) {
            return Optional.of(ElevLevel.L1);
        } else if (rots >= l2 - tolerance && rots <= l2 + tolerance) {
            return Optional.of(ElevLevel.L2);
        } else if (rots >= l3 - tolerance && rots <= l3 + tolerance) {
            return Optional.of(ElevLevel.L3);
        } else if (rots >= l4 - tolerance && rots <= l4 + tolerance) {
            return Optional.of(ElevLevel.L4);
        } else {
            return Optional.empty();
        }
    }

    public ElevPosition add(ElevPosition b) {
        if (b == null) {
            Logger.reportWarning("Cannot add a null ElevPosition", true);
            return fromRotations(getAsRotations()); // return copy of self
        }

        return fromRotations(getAsRotations() + b.getAsRotations());
    }

    public ElevPosition subtract(ElevPosition b) {
        if (b == null) {
            Logger.reportWarning("Cannot subtract a null ElevPosition", true);
            return fromRotations(getAsRotations()); // return copy of self
        }

        return fromRotations(getAsRotations() - b.getAsRotations());
    }
}
