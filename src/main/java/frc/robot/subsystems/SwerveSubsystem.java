package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants;
import frc.robot.utils.Alerts;
import frc.robot.utils.Logger;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive drive;
    private final boolean usingVision;

    public SwerveSubsystem(boolean allowVisionOdo) throws RuntimeException {
        SwerveDriveTelemetry.verbosity = SwerveConstants.verbosity;
        try {
            drive = new SwerveParser(SwerveConstants.configDirectory)
                    .createSwerveDrive(
                            SwerveConstants.maxSpeed.abs(Units.MetersPerSecond), SwerveConstants.initialPose);
        } catch (Exception e) {
            Alerts.swerveLoadConfig.set(true);
            throw new RuntimeException(e);
        }
        usingVision = allowVisionOdo;

        drive.setChassisDiscretization(
                SwerveConstants.chassisDescretization.isPresent(),
                SwerveConstants.chassisDescretization
                        .map((val) -> val.abs(Units.Seconds))
                        .orElse(0.02));
        drive.setHeadingCorrection(SwerveConstants.headingCorrection);
        drive.setCosineCompensator(SwerveConstants.cosineCompensator
                && !SwerveDriveTelemetry
                        .isSimulation); // using compensator in simulation can result in unexpected behaviour
        drive.setAngularVelocityCompensation(
                SwerveConstants.angularVelocityCompensationInTeleop,
                SwerveConstants.angularVelocityCompensationInAuto,
                SwerveConstants.angularVelocityCompensationCoeff);

        if (usingVision) {
            drive.stopOdometryThread(); // stop the odo thread to allow for better synchronizing of odo updates
        }
        initPathplanner();
    }

    private void initPathplanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getCurrentPose, // robot pose supplier
                    this::resetOdometry, // function to reset odo
                    this::getCurrentRobotRelativeVelocity, // ChassisSpeeds supplier, must return robot relative speeds
                    (robotRelativeSpeeds, moduleFF) -> { // function to drive the robot with robot relative speeds
                        if (SwerveConstants.moduleFeedForward) {
                            drive.drive(
                                    robotRelativeSpeeds,
                                    drive.kinematics.toSwerveModuleStates(robotRelativeSpeeds),
                                    moduleFF.linearForces());
                        } else {
                            drive.drive(robotRelativeSpeeds);
                        }
                    },
                    new PPHolonomicDriveController(
                            SwerveConstants.pathPlannerTranslationPID,
                            SwerveConstants.pathPlannerRotationPID), // PathPlanner PID controllers
                    config, // robot config, loaded from PathPlanner GUI
                    () -> { // boolean supplier, controls when path is flipped to the red alliance
                        return DriverStation.getAlliance()
                                .map((val) -> val == DriverStation.Alliance.Red)
                                .orElse(false);
                    },
                    this // reference to subsystem in order to set requirements
                    );
        } catch (Exception e) {
            Alerts.swerveInitPathPlanner.set(true);
            Logger.reportError(e);
        }

        // warmup path finding
        PathfindingCommand.warmupCommand().schedule();
    }

    public void resetOdometry(Pose2d newPose) {
        if (newPose == null) {
            Logger.reportWarning("Cannot reset swerve odometry to a null pose, using fallback pose", true);
            newPose = SwerveConstants.initialPose;
        }
        drive.resetOdometry(newPose);
    }

    public Command genPathfindToPoseCommand(Pose2d goalEndPose, LinearVelocity goalEndVelocity) {
        PathConstraints constraints = new PathConstraints(
                drive.getMaximumChassisVelocity(),
                SwerveConstants.maxPathfindingLinearAccel.abs(Units.MetersPerSecondPerSecond),
                drive.getMaximumChassisAngularVelocity(),
                SwerveConstants.maxPathfidingAngularAccel.abs(Units.RadiansPerSecondPerSecond));

        return AutoBuilder.pathfindToPose(goalEndPose, constraints, goalEndVelocity);
    }

    public Command genCenterModulesCommand() {
        return run(() -> {
            for (SwerveModule module : drive.getModules()) {
                module.setAngle(0);
            }
        });
    }

    public Pose2d getCurrentPose() {
        return drive.getPose();
    }

    public ChassisSpeeds getCurrentRobotRelativeVelocity() {
        return drive.getRobotVelocity();
    }
}
