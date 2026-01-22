package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import java.util.Optional;

public class ShooterConstants {

    private ShooterConstants() {}

    public static class LeftShooter {
        // motor definitions
        public static final int motorID = 12;

        // motor config
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = false;
        public static final int maxCurrent = 20;
        public static final Optional<Double> voltageCompensation = Optional.of(12d);
        public static final double maxDutyCycle = 1;
    }

    public static class RightShooter {
        // motor definitions
        public static final int motorID = 11;

        // motor config
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final boolean inverted = true;
        public static final int maxCurrent = 20;
        public static final Optional<Double> voltageCompensation = Optional.of(12d);
        public static final double maxDutyCycle = 1;
    }
}
