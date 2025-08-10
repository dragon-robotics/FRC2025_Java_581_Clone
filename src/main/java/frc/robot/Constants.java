package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class Constants {

    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM
    }

  public static final class FieldConstants {
    public static AprilTagFieldLayout APTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final double FIELD_LENGTH = APTAG_FIELD_LAYOUT.getFieldLength();
    public static final double FIELD_WIDTH = APTAG_FIELD_LAYOUT.getFieldWidth();

    // Load a custom AprilTag field layout if available //
    // The different layouts of the AprilTags on the field
    public static final AprilTagFieldLayout DEFAULT_APTAG_FIELD_LAYOUT;
    public static final AprilTagFieldLayout WELDED_RED_APTAG_FIELD_LAYOUT;
    public static final AprilTagFieldLayout WELDED_BLUE_APTAG_FIELD_LAYOUT;

    // Static initializer block
    static {
      // Load default layout - this does NOT throw IOException
      // It might return null if the resource is missing, though kDefaultField should
      // be safe.
      // Construct paths for welded layouts
      Path defaultPath = Path.of(
          Filesystem.getDeployDirectory().getPath(),
          "apriltags",
          "welded",
          "2025-reef-only.json");
      AprilTagFieldLayout defaultLayout = null;
      try {
        defaultLayout = new AprilTagFieldLayout(defaultPath);
      } catch (IOException e) {
        System.err.println("!!! CRITICAL: Failed to load default AprilTag field resource!");
        DriverStation.reportError("CRITICAL: Failed to load default AprilTag field resource: " + e.getMessage(), true);

        // If loading from file fails, we will use the static kDefaultField layout
        // as a fallback.
        defaultLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      }

      // Initialize temporary variables for welded layouts
      AprilTagFieldLayout redLayout = null;
      AprilTagFieldLayout blueLayout = null;

      // Construct paths for welded layouts
      Path redPath = Path.of(
          Filesystem.getDeployDirectory().getPath(),
          "apriltags",
          "welded",
          "2025-red-reef.json");
      Path bluePath = Path.of(
          Filesystem.getDeployDirectory().getPath(),
          "apriltags",
          "welded",
          "2025-blue-reef.json");

      // Try loading layouts from file paths - THESE can throw IOException
      try {
        redLayout = new AprilTagFieldLayout(redPath);
        blueLayout = new AprilTagFieldLayout(bluePath);
      } catch (IOException e) {
        // Handle the error if loading from files fails
        System.err.println("!!! Failed to load welded AprilTag field layout files!");
        e.printStackTrace();
        DriverStation.reportError("Failed to load welded AprilTag layouts: " + e.getMessage(), true);
        // redLayout and blueLayout will remain null if they failed
      } finally {
        // Assign the loaded layouts to the final fields
        // Use default layout as fallback if welded layouts are still null
        DEFAULT_APTAG_FIELD_LAYOUT = defaultLayout;
        WELDED_RED_APTAG_FIELD_LAYOUT = (redLayout != null) ? redLayout : defaultLayout;
        WELDED_BLUE_APTAG_FIELD_LAYOUT = (blueLayout != null) ? blueLayout : defaultLayout;

        // Now initialize the main layout based on alliance
        AprilTagFieldLayout selectedLayout = DEFAULT_APTAG_FIELD_LAYOUT; // Start with default
        if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            // Prefer loaded red layout, fallback to default
            selectedLayout = WELDED_RED_APTAG_FIELD_LAYOUT;
          } else { // Blue Alliance
            // Prefer loaded blue layout, fallback to default
            selectedLayout = WELDED_BLUE_APTAG_FIELD_LAYOUT;
          }
        } else {
          // Default if no alliance is set (e.g., practice mode) - use Blue or Default
          selectedLayout = WELDED_BLUE_APTAG_FIELD_LAYOUT;
        }

        // Assign the final selected layout
        APTAG_FIELD_LAYOUT = selectedLayout;

        // Final check if the main layout is still null (only if default also failed)
        if (APTAG_FIELD_LAYOUT == null) {
          // This is a critical state
          DriverStation.reportError("CRITICAL: No AprilTag field layout could be assigned!", true);
        }
      }
    }

    public static class CoralStation {
      // public static final Pose2d[] BLUE_CORAL_STATION_TAGS = new Pose2d[2];
      // public static final Pose2d[] BLUE_CORAL_STATION_LOCS = new Pose2d[6];
      // public static final Pose2d[] RED_CORAL_STATION_LOCS = new Pose2d[6]; // Red coral station locations can be initialized from Blue coral station locations
    }

    public static class Reef {
      // Get the pose of the reef AprilTag on the blue side //
      public static final Pose2d[] BLUE_REEF_TAGS = new Pose2d[6];
      public static final Pose2d[] BLUE_REEF_BRANCHES = new Pose2d[12];
      public static final Pose2d[] BLUE_REEF_ALGAE = new Pose2d[6];
      public static final Pose2d[] RED_REEF_BRANCHES = new Pose2d[12]; // Red reef branches can be initialized from Blue reef branches
      public static final Pose2d[] RED_REEF_ALGAE = new Pose2d[6]; // Red reef algae can be initialized from Blue reef algae locations
      
      static {
        // Get the Apriltag layout //
        AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        double adjustX = Units.inchesToMeters(30.738); // Center of robot + bumper
        // double adjustX = Units.inchesToMeters(17); // Center of robot + bumper
        double adjustY = Units.inchesToMeters(6.468); // Positive adjustment for Left and Negative for Right

        // Get the blue reef tags from the layout //
        for(int tag = 17; tag < 23; tag++) {
          BLUE_REEF_TAGS[tag - 17] = aprilTagLayout.getTagPose(tag).get().toPose2d();

          // Get the rotation for the reef tag. Even (0, 2, 4, etc.) is left and odd (1, 3, 5) is right //
          // Left branch //
          BLUE_REEF_BRANCHES[2*(tag - 17)] = new Pose2d(
            BLUE_REEF_TAGS[tag - 17].transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero)).getX(),
            BLUE_REEF_TAGS[tag - 17].transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero)).getY(),
            Rotation2d.fromDegrees(BLUE_REEF_TAGS[tag - 17].getRotation().getDegrees()))
            .transformBy(new Transform2d(0.0,0.0,Rotation2d.kZero)); // Fudge Factor
          // Right branch //
          BLUE_REEF_BRANCHES[2*(tag - 17) + 1] = new Pose2d(
            BLUE_REEF_TAGS[tag - 17].transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero)).getX(),
            BLUE_REEF_TAGS[tag - 17].transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero)).getY(),
            Rotation2d.fromDegrees(BLUE_REEF_TAGS[tag - 17].getRotation().getDegrees()))
            .transformBy(new Transform2d(0.0,0.0,Rotation2d.kZero)); // Fudge Factor
          // Algae locations //
          BLUE_REEF_ALGAE[tag - 17] = new Pose2d(
            BLUE_REEF_TAGS[tag - 17].transformBy(new Transform2d(adjustX, 0, Rotation2d.kZero)).getX(),
            BLUE_REEF_TAGS[tag - 17].transformBy(new Transform2d(adjustX, 0, Rotation2d.kZero)).getY(),
            Rotation2d.fromDegrees(BLUE_REEF_TAGS[tag - 17].getRotation().getDegrees()))
            .transformBy(new Transform2d(0.0,0.0,Rotation2d.kZero)); // Fudge Factor
        }

        // Initialize the red reef branches //
        for(int branch = 0; branch < 12; branch++) {
          // Get the rotation for the reef tag. Even (0, 2, 4, etc.) is left and odd (1, 3, 5) is right //
          RED_REEF_BRANCHES[branch] = new Pose2d(
            FIELD_LENGTH - BLUE_REEF_BRANCHES[branch].getX(),
            FIELD_WIDTH - BLUE_REEF_BRANCHES[branch].getY(),
            BLUE_REEF_BRANCHES[branch].getRotation().rotateBy(Rotation2d.kPi));
          
          // Initialize the 6 red algae locations //
          if (branch < 6) {
            RED_REEF_ALGAE[branch] = new Pose2d(
              FIELD_LENGTH - BLUE_REEF_ALGAE[branch].getX(),
              FIELD_WIDTH - BLUE_REEF_ALGAE[branch].getY(),
              BLUE_REEF_ALGAE[branch].getRotation().rotateBy(Rotation2d.kPi));
          }
        }
      }
    }

    public static class Processor {
      // public static final Pose2d BLUE_PROCESSOR_TAG;
      // public static final Pose2d RED_PROCESSOR_TAG;

      // static {

      // }
    }

    public static class Barge {

    }
  }

  public static class VisionConstants {
    // Set up 8 pose estimation cameras with their respective names and positions
    public static final String[] APTAG_CAMERA_NAMES = {
        "AprilTagPoseEstCameraFL",
        "AprilTagPoseEstCameraF",
        "AprilTagPoseEstCameraFR",
        "AprilTagPoseEstCameraR",
        "AprilTagPoseEstCameraBR",
        "AprilTagPoseEstCameraB",
        "AprilTagPoseEstCameraBL",
        "AprilTagPoseEstCameraL"
    };

    // Front-Left Camera: Mounted at front-left corner, pointing outward at 45 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FL_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(45)));

    // Front Camera: Mounted at front face, pointing outward at 0 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_F_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(0),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(0)));
            
    // Front-Right Camera: Mounted at front-right corner, pointing outward at -45 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_FR_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-45)));

    // Right Camera: Mounted at right face, pointing outward at -90 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_R_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-90)));

    // Back-Right Camera: Mounted at back-right corner, pointing outward at -135 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BR_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-135)));

    // Back Camera: Mounted at back face, pointing outward at -180 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_B_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(0),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(-180)));

    // Back-Left Camera: Mounted at back-left corner, pointing outward at 135 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_BL_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-17.125),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(135)));

    // Left Camera: Mounted at left face, pointing outward at 90 degrees
    public static final Transform3d APTAG_POSE_EST_CAM_L_POS = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(0),
            Units.inchesToMeters(17.125),
            Units.inchesToMeters(6.825)),
        new Rotation3d(
            0,
            Units.degreesToRadians(-15),
            Units.degreesToRadians(90)));

    public static final Transform3d[] APTAG_POSE_EST_CAM_POSITIONS = {
        APTAG_POSE_EST_CAM_FL_POS,
        APTAG_POSE_EST_CAM_F_POS,
        APTAG_POSE_EST_CAM_FR_POS,
        APTAG_POSE_EST_CAM_R_POS,
        APTAG_POSE_EST_CAM_BR_POS,
        APTAG_POSE_EST_CAM_B_POS,
        APTAG_POSE_EST_CAM_BL_POS,
        APTAG_POSE_EST_CAM_L_POS
    };

    // Vision standard deviation for pose estimation
    public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> DEFAULT_TAG_STDDEV = VecBuilder.fill(0.9, 0.9, 0.9);

    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.1;
    public static double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LINEAR_STDDEV_BASELINE = 0.02; // Meters
    public static double ANGULAR_STDDEV_BASELINE = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] CAMERA_STDDEV_FACTORS = new double[] {
        1.0, // FL Camera
        1.0, // F Camera
        1.0, // FR Camera
        1.0, // R Camera
        1.0, // BR Camera
        1.0, // B Camera
        1.0, // BL Camera
        1.0 // L Camera
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double LINEAR_STDDEV_MEGATAG2_FACTOR = 0.5; // More stable than full 3D solve
    public static double ANGULAR_STDDEV_MEGATAG2_ANGLE_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available    
  }

  public static class SwerveConstants {
    // General constants for swerve drive //
    public static final double HEADING_KP = 3.5;
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0.5;
    public static final double HEADING_TOLERANCE = 0.01;

    public static final double MAX_SPEED_METERS_PER_SECOND = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double SWERVE_DEADBAND = 0.1;

    // SWERVE MODULE ODOMETRY STANDARD DEVIATIONS //
    public static final Matrix<N3, N1> ODOMETRY_STD = VecBuilder.fill(0.1, 0.1, 0.1);
  }

  public static class OperatorConstants {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final int OPERATOR_BUTTON_PORT = 2;
    public static final int TEST_PORT = 3;
  }
  
  public static class JoystickConstants {
    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;
  }  
}
