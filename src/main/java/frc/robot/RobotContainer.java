// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  /* Joysticks */
  private final CommandXboxController joystick;

  /* Subsystems */
  public final CommandSwerveDrivetrain m_swerve;
  public final VisionSubsystem m_vision;
  public final Superstructure m_superstructure;

  /* Commands */
  private final Command m_driveMaintainHeadingCommand;

  /* Path follower */
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // Initialize the Joysticks
    joystick = new CommandXboxController(0);

    // Initialize the Subsystems
    m_swerve = TunerConstants.createDrivetrain(
        250,
        SwerveConstants.ODOMETRY_STD,
        VisionConstants.DEFAULT_TAG_STDDEV);

    switch (Constants.CURRENT_MODE) {
      case REAL:
        m_vision = new VisionSubsystem(
            m_swerve,
            m_swerve::addVisionMeasurement,
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[0],
                VisionConstants.APTAG_POSE_EST_CAM_FL_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[1],
                VisionConstants.APTAG_POSE_EST_CAM_F_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[2],
                VisionConstants.APTAG_POSE_EST_CAM_FR_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[3],
                VisionConstants.APTAG_POSE_EST_CAM_R_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[4],
                VisionConstants.APTAG_POSE_EST_CAM_BR_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[5],
                VisionConstants.APTAG_POSE_EST_CAM_B_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[6],
                VisionConstants.APTAG_POSE_EST_CAM_BL_POS,
                () -> m_swerve.getState().Pose),
            new VisionIOPhotonVision(
                VisionConstants.APTAG_CAMERA_NAMES[7],
                VisionConstants.APTAG_POSE_EST_CAM_L_POS,
                () -> m_swerve.getState().Pose));
        break;
      case SIM:
        m_vision = new VisionSubsystem(
            m_swerve,
            m_swerve::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[0],
                VisionConstants.APTAG_POSE_EST_CAM_FL_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[1],
                VisionConstants.APTAG_POSE_EST_CAM_F_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[2],
                VisionConstants.APTAG_POSE_EST_CAM_FR_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[3],
                VisionConstants.APTAG_POSE_EST_CAM_R_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[4],
                VisionConstants.APTAG_POSE_EST_CAM_BR_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[5],
                VisionConstants.APTAG_POSE_EST_CAM_B_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[6],
                VisionConstants.APTAG_POSE_EST_CAM_BL_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.APTAG_CAMERA_NAMES[7],
                VisionConstants.APTAG_POSE_EST_CAM_L_POS,
                m_swerve.mapleSimSwerveDrivetrain.mapleSimDrive::getSimulatedDriveTrainPose));
        break;
      default:
        m_vision = new VisionSubsystem(m_swerve, m_swerve::addVisionMeasurement, new VisionIO() {});
        break;
    }
    m_superstructure = new Superstructure(
        m_swerve,
        m_vision);

    // Initialize all the NamedCommands
    m_driveMaintainHeadingCommand = m_superstructure.DriveMaintainHeading(
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX(),
        () -> -joystick.getRightX());

    // Initialize the auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    // Configure Joystick Bindings
    configureBindings();

    // Reset the swerve pose to a known position if we are in sim
    if (!RobotBase.isReal())
      m_swerve.resetPose(new Pose2d(3, 3, new Rotation2d()));
  }

  private void configureBindings() {

    // Default swerve command is to drive while maintaining heading.
    m_swerve.setDefaultCommand(m_driveMaintainHeadingCommand);

    // Intaking Coral auto-aligns to the 1 of 6 closest coral station locations
    // Left Bumper

    // Intaking Algae on the reef auto-aligns to the closest algae location
    // M1 (AKA Middle Left Bumper)

    // Intaking Algae on the floor auto-aligns to the closest algae
    // Left Trigger acting as a button

    // Scoring Coral auto-aligns to the closest scoring branch
    // Right Bumper

    // Scoring Processor auto-aligns to the closest scoring processor
    // M2 (AKA Right Middle Bumper)

    // Scoring Barge auto-aligns to 1 of 4 closest barge locations
    // Right Trigger acting as a button

    // // Idle while the robot is disabled. This ensures the configured
    // // neutral mode is applied to the drive motors while disabled.
    // final var idle = new SwerveRequest.Idle();
    // RobotModeTriggers.disabled().whileTrue(
    // drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    // .applyRequest(() -> point.withModuleDirection(new
    // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // // reset the field-centric heading on left bumper press
    // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));
  }

  public Command getAutonomousCommand() {
    /* Run the path selected from the auto chooser */
    return autoChooser.getSelected();
  }
}
