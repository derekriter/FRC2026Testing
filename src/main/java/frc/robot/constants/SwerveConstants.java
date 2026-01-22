package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Optional;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveConstants {

    private SwerveConstants() {}

    public static final TelemetryVerbosity verbosity = TelemetryVerbosity.HIGH;
    public static final File configDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

    // swerve config
    public static final Pose2d initialPose =
            new Pose2d(new Translation2d(Units.Meter.of(1), Units.Meter.of(4)), Rotation2d.kZero);
    public static final LinearVelocity maxSpeed = LinearVelocity.ofBaseUnits(14.5, Units.FeetPerSecond);
    public static final Optional<Time> chassisDescretization = Optional.of(Time.ofBaseUnits(0.02, Units.Seconds));
    public static final boolean headingCorrection = false;
    public static final boolean cosineCompensator = true;
    public static final boolean angularVelocityCompensationInTeleop = true;
    public static final boolean angularVelocityCompensationInAuto = true;
    public static final double angularVelocityCompensationCoeff = -0.1;
    public static final boolean moduleFeedForward = false;

    // pathplanner config
    public static final PIDConstants pathPlannerTranslationPID = new PIDConstants(5, 0, 0);
    public static final PIDConstants pathPlannerRotationPID = new PIDConstants(5, 0, 0);
    public static final LinearAcceleration maxPathfindingLinearAccel =
            LinearAcceleration.ofBaseUnits(4, Units.MetersPerSecondPerSecond);
    public static final AngularAcceleration maxPathfidingAngularAccel =
            AngularAcceleration.ofBaseUnits(720, Units.DegreesPerSecondPerSecond);
}
