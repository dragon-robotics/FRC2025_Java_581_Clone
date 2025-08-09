// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveMaintainHeadingCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  /* Joysticks */
  private final CommandXboxController joystick;

  /* Subsystems */
  public final CommandSwerveDrivetrain drivetrain;
  public final VisionSubsystem vision;
  public final Superstructure superstructure;

  /* Commands */

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Initialize the Joysticks
    joystick = new CommandXboxController(0);

    // Initialize the Subsystems
    drivetrain = TunerConstants.createDrivetrain(
        250,
        SwerveConstants.ODOMETRY_STD,
        VisionConstants.DEFAULT_TAG_STDDEV);
    vision = new VisionSubsystem(
        drivetrain,
        drivetrain::addVisionMeasurement,
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[0],
            VisionConstants.APTAG_POSE_EST_CAM_FL_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[1],
            VisionConstants.APTAG_POSE_EST_CAM_F_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[2],
            VisionConstants.APTAG_POSE_EST_CAM_FR_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[3],
            VisionConstants.APTAG_POSE_EST_CAM_R_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[4],
            VisionConstants.APTAG_POSE_EST_CAM_BR_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[5],
            VisionConstants.APTAG_POSE_EST_CAM_B_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[6],
            VisionConstants.APTAG_POSE_EST_CAM_BL_POS,
            () -> drivetrain.getState()),
        new VisionIOPhotonVisionSim(
            VisionConstants.APTAG_CAMERA_NAMES[7],
            VisionConstants.APTAG_POSE_EST_CAM_L_POS,
            () -> drivetrain.getState()));
    superstructure = new Superstructure(
        drivetrain,
        vision);

    // Initialize all the Namedcommands

    // Initialize the auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure Joystick Bindings
    configureBindings();

    drivetrain.resetPose(new Pose2d(3, 3, new Rotation2d()));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        superstructure.DriveMaintainHeading(
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX()));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
}
