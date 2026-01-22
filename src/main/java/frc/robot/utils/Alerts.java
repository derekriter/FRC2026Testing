package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ElevConstants;
import frc.robot.constants.ShooterConstants;

public class Alerts {

    public static final Alert driver1Missing,
            driver2Missing,
            fmsConnected,
            lowBattery,
            criticalBattery,
            elevMotorDisconnected,
            elevMotorOverheating,
            elevMotorFaults,
            elevMotorConfigFail,
            shooterLMotorConfigFail,
            shooterRMotorConfigFail,
            shooterLMotorDisconnected,
            shooterRMotorDisconnected,
            shooterLMotorOverheating,
            shooterRMotorOverheating,
            shooterLMotorFaults,
            shooterRMotorFaults,
            shooterLMotorWarnings,
            shooterRMotorWarnings,
            climbLeverConfigFail,
            climbClampConfigFail,
            climbLeverDisconnected,
            climbClampDisconnected,
            climbLeverOverheating,
            climbClampOverheating,
            climbLeverFaults,
            climbClampFaults,
            swerveLoadConfig,
            swerveInitPathPlanner;

    static {
        driver1Missing = new Alert("Driver 1 controller is not plugged in to port 0", AlertType.kWarning);
        driver1Missing.set(false);

        driver2Missing = new Alert("Driver 2 controller is not plugged in to port 1", AlertType.kWarning);
        driver2Missing.set(false);

        fmsConnected = new Alert("FMS connected", AlertType.kInfo);
        fmsConnected.set(false);

        lowBattery = new Alert("Low battery", AlertType.kWarning);
        lowBattery.set(false);

        criticalBattery = new Alert("Very low battery", AlertType.kError);
        criticalBattery.set(false);

        elevMotorDisconnected = new Alert(
                String.format("Missing connection to elevator motor (CAN %d)", ElevConstants.motorID),
                AlertType.kError);
        elevMotorDisconnected.set(false);

        elevMotorOverheating = new Alert(
                String.format("Elevator motor (CAN %d) is overheating", ElevConstants.motorID), AlertType.kWarning);
        elevMotorOverheating.set(false);

        elevMotorFaults = new Alert(
                String.format("Potentially critical faults active on elevator motor (CAN %d)", ElevConstants.motorID),
                AlertType.kWarning);
        elevMotorFaults.set(false);

        elevMotorConfigFail = new Alert(
                String.format("Failed to update elevator motor (CAN %d) config", ElevConstants.motorID),
                AlertType.kError);
        elevMotorConfigFail.set(false);

        shooterLMotorConfigFail = new Alert(
                String.format(
                        "Failed to update left shooter motor (CAN %d) config", ShooterConstants.LeftShooter.motorID),
                AlertType.kError);
        shooterLMotorConfigFail.set(false);

        shooterRMotorConfigFail = new Alert(
                String.format(
                        "Failed to update right shooter motor (CAN %d) config", ShooterConstants.RightShooter.motorID),
                AlertType.kError);
        shooterRMotorConfigFail.set(false);

        shooterLMotorDisconnected = new Alert(
                String.format(
                        "Missing connection to left shooter motor (CAN %d)", ShooterConstants.LeftShooter.motorID),
                AlertType.kError);
        shooterLMotorDisconnected.set(false);

        shooterRMotorDisconnected = new Alert(
                String.format(
                        "Missing connection to right shooter motor (CAN %d)", ShooterConstants.RightShooter.motorID),
                AlertType.kError);
        shooterRMotorDisconnected.set(false);

        shooterLMotorOverheating = new Alert(
                String.format("Left shooter motor (CAN %d) is overheating", ShooterConstants.LeftShooter.motorID),
                AlertType.kWarning);
        shooterLMotorOverheating.set(false);

        shooterRMotorOverheating = new Alert(
                String.format("Right shooter motor (CAN %d) is overheating", ShooterConstants.RightShooter.motorID),
                AlertType.kWarning);
        shooterRMotorOverheating.set(false);

        shooterLMotorFaults = new Alert(
                String.format(
                        "Potentially critical faults active on left shooter motor (CAN %d)",
                        ShooterConstants.LeftShooter.motorID),
                AlertType.kWarning);
        shooterLMotorFaults.set(false);

        shooterRMotorFaults = new Alert(
                String.format(
                        "Potentially critical faults active on right shooter motor (CAN %d)",
                        ShooterConstants.RightShooter.motorID),
                AlertType.kWarning);
        shooterRMotorFaults.set(false);

        shooterLMotorWarnings = new Alert(
                String.format(
                        "Potentially critical warnings active on left shooter motor (CAN %d)",
                        ShooterConstants.LeftShooter.motorID),
                AlertType.kWarning);
        shooterLMotorWarnings.set(false);

        shooterRMotorWarnings = new Alert(
                String.format(
                        "Potentially critical warnings active on right shooter motor (CAN %d)",
                        ShooterConstants.RightShooter.motorID),
                AlertType.kWarning);
        shooterRMotorWarnings.set(false);

        climbLeverConfigFail = new Alert(
                String.format("Failed to update climber lever motor (CAN %d) config", ClimberConstants.Lever.motorID),
                AlertType.kError);
        climbLeverConfigFail.set(false);

        climbClampConfigFail = new Alert(
                String.format("Failed to update climber clamp motor (CAN %d) config", ClimberConstants.Clamp.motorID),
                AlertType.kError);
        climbClampConfigFail.set(false);

        climbLeverDisconnected = new Alert(
                String.format("Missing connection to climber lever motor (CAN %d)", ClimberConstants.Lever.motorID),
                AlertType.kError);
        climbLeverDisconnected.set(false);

        climbClampDisconnected = new Alert(
                String.format("Missing connection to climber clamp motor (CAN %d)", ClimberConstants.Clamp.motorID),
                AlertType.kError);
        climbClampDisconnected.set(false);

        climbLeverOverheating = new Alert(
                String.format("Climb lever motor (CAN %d) is overheating", ClimberConstants.Lever.motorID),
                AlertType.kWarning);
        climbLeverOverheating.set(false);

        climbClampOverheating = new Alert(
                String.format("Climb clamp motor (CAN %d) is overheating", ClimberConstants.Clamp.motorID),
                AlertType.kWarning);
        climbClampOverheating.set(false);

        climbLeverFaults = new Alert(
                String.format(
                        "Potentially critical faults active on climb lever motor (CAN %d)",
                        ClimberConstants.Lever.motorID),
                AlertType.kWarning);
        climbLeverFaults.set(false);

        climbClampFaults = new Alert(
                String.format(
                        "Potentially critical faults active on climb clamp motor (CAN %d)",
                        ClimberConstants.Clamp.motorID),
                AlertType.kWarning);
        climbClampFaults.set(false);

        swerveLoadConfig = new Alert("Swerve subsytem faile to load config file", AlertType.kError);
        swerveLoadConfig.set(false);

        swerveInitPathPlanner = new Alert("Swerve subsystem failed to initialize PathPlanner", AlertType.kError);
        swerveInitPathPlanner.set(false);
    }

    private Alerts() {}
}
