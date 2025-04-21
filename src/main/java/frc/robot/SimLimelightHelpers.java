package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * A simulation-specific implementation of LimelightHelpers that provides
 * simulated vision data during simulation.
 */
public class SimLimelightHelpers {
    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable table = inst.getTable("limelight");
    
    // Publishers for simulated Limelight data
    private static final DoubleArrayPublisher botposePub = table.getDoubleArrayTopic("botpose").publish();
    private static final DoubleArrayPublisher botpose_wpiredPub = table.getDoubleArrayTopic("botpose_wpired").publish();
    private static final DoubleArrayPublisher botpose_wpibluePub = table.getDoubleArrayTopic("botpose_wpiblue").publish();
    private static final DoubleArrayPublisher camerapose_robotspacePub = table.getDoubleArrayTopic("camerapose_robotspace").publish();
    private static final StringPublisher jsonPub = table.getStringTopic("json").publish();
    
    // Simulated target visibility
    private static boolean targetVisible = true;
    
    // Simulated target ID
    private static int targetId = 1;

    // Camera position relative to robot center (in meters)
    private static final double CAMERA_X = 0.0; // Centered on robot
    private static final double CAMERA_Y = 0.0; // Centered on robot
    private static final double CAMERA_Z = 0.5; // 0.5 meters above robot center
    
    // AprilTag positions on the field (in meters)
    private static final Pose3d LEFT_REEF_TAG = new Pose3d(2.0, 2.0, 0.0, new Rotation3d());
    private static final Pose3d RIGHT_REEF_TAG = new Pose3d(2.0, -2.0, 0.0, new Rotation3d());
    private static final Pose3d HIGHER_TAG = new Pose3d(3.0, 0.0, 1.0, new Rotation3d());
    
    // Field of view parameters (in degrees)
    private static final double HORIZONTAL_FOV = 59.6;
    private static final double VERTICAL_FOV = 45.7;
    
    // Reference to drivetrain for getting robot pose
    private static CommandSwerveDrivetrain drivetrain;
    
    /**
     * Sets the drivetrain reference for getting robot pose
     */
    public static void setDrivetrain(CommandSwerveDrivetrain dt) {
        drivetrain = dt;
    }
    
    /**
     * Sets whether a target is visible in the simulated Limelight
     * @param visible Whether a target should be visible
     */
    public static void setTargetVisible(boolean visible) {
        targetVisible = visible;
    }
    
    /**
     * Sets the simulated target ID
     * @param id The target ID to simulate
     */
    public static void setTargetId(int id) {
        targetId = id;
    }
    
    /**
     * Checks if a target is within the camera's field of view
     * @param targetPose The target's pose in field space
     * @param robotPose The robot's pose in field space
     * @return Whether the target is visible
     */
    private static boolean isTargetInFOV(Pose3d targetPose, Pose2d robotPose) {
        // Calculate relative position of target to robot
        Translation3d relativePos = targetPose.getTranslation().minus(
            new Translation3d(robotPose.getX(), robotPose.getY(), CAMERA_Z)
        );
        
        // Convert to camera space (accounting for robot rotation)
        double robotAngle = robotPose.getRotation().getRadians();
        double dx = relativePos.getX() * Math.cos(-robotAngle) - relativePos.getY() * Math.sin(-robotAngle);
        double dy = relativePos.getX() * Math.sin(-robotAngle) + relativePos.getY() * Math.cos(-robotAngle);
        double dz = relativePos.getZ();
        
        // Calculate angles from camera to target
        double horizontalAngle = Math.toDegrees(Math.atan2(dx, Math.sqrt(dy * dy + dz * dz)));
        double verticalAngle = Math.toDegrees(Math.atan2(dy, Math.sqrt(dx * dx + dz * dz)));
        
        // Check if target is within FOV
        return Math.abs(horizontalAngle) <= HORIZONTAL_FOV / 2 && 
               Math.abs(verticalAngle) <= VERTICAL_FOV / 2;
    }
    
    /**
     * Gets the target pose based on the current target ID
     * @return The target's pose in field space
     */
    private static Pose3d getTargetPose() {
        switch (targetId) {
            case 1: // Left reef tag
                return LEFT_REEF_TAG;
            case 2: // Right reef tag
                return RIGHT_REEF_TAG;
            case 3: // Higher tag
                return HIGHER_TAG;
            default:
                return LEFT_REEF_TAG; // Default to left reef tag
        }
    }
    
    /**
     * Simulates the getTV() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return Whether a target is visible
     */
    public static boolean getTV(String limelightName) {
        if (!targetVisible) {
            return false;
        }
        
        // Get the robot's current pose
        Pose2d robotPose = getRobotPose();
        
        // Get the target pose
        Pose3d targetPose = getTargetPose();
        
        // Check if target is in field of view
        return isTargetInFOV(targetPose, robotPose);
    }
    
    /**
     * Gets the robot's current pose from the drivetrain
     * @return The robot's current pose
     */
    private static Pose2d getRobotPose() {
        if (drivetrain != null) {
            return drivetrain.getState().Pose;
        }
        return new Pose2d(); // Default pose if drivetrain not set
    }
    
    /**
     * Simulates the getBotPose() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's pose in field space [x, y, z, roll, pitch, yaw]
     */
    public static double[] getBotPose(String limelightName) {
        // Default pose if no target is visible
        if (!getTV(limelightName)) {
            return new double[]{0, 0, 0, 0, 0, 0};
        }
        
        // Get the robot's current pose
        Pose2d robotPose = getRobotPose();
        
        // Convert to the format expected by Limelight
        double[] pose = new double[6];
        pose[0] = robotPose.getX(); // X position
        pose[1] = robotPose.getY(); // Y position
        pose[2] = CAMERA_Z; // Z position (height of camera)
        pose[3] = 0; // Roll (assuming no roll)
        pose[4] = 0; // Pitch (assuming no pitch)
        pose[5] = robotPose.getRotation().getDegrees(); // Yaw
        
        // Publish to NetworkTables
        botposePub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getBotPose_wpiRed() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's pose in field space for the red alliance [x, y, z, roll, pitch, yaw]
     */
    public static double[] getBotPose_wpiRed(String limelightName) {
        double[] pose = getBotPose(limelightName);
        botpose_wpiredPub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getBotPose_wpiBlue() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's pose in field space for the blue alliance [x, y, z, roll, pitch, yaw]
     */
    public static double[] getBotPose_wpiBlue(String limelightName) {
        double[] pose = getBotPose(limelightName);
        botpose_wpibluePub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getCameraPose_RobotSpace() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The camera's pose in robot space [x, y, z, roll, pitch, yaw]
     */
    public static double[] getCameraPose_RobotSpace(String limelightName) {
        // Simulate camera position relative to robot
        double[] pose = {CAMERA_X, CAMERA_Y, CAMERA_Z, 0, 0, 0};
        camerapose_robotspacePub.set(pose);
        return pose;
    }
    
    /**
     * Simulates the getBotPose2d() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 2D pose in field space
     */
    public static Pose2d getBotPose2d(String limelightName) {
        return getRobotPose();
    }
    
    /**
     * Simulates the getBotPose2d_wpiBlue() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 2D pose in field space for the blue alliance
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        return getRobotPose();
    }
    
    /**
     * Simulates the getBotPose2d_wpiRed() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 2D pose in field space for the red alliance
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        return getRobotPose();
    }
    
    /**
     * Simulates the getBotPose3d() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return The robot's 3D pose in field space
     */
    public static Pose3d getBotPose3d(String limelightName) {
        Pose2d pose2d = getRobotPose();
        return new Pose3d(
            new Translation3d(pose2d.getX(), pose2d.getY(), CAMERA_Z),
            new Rotation3d(0, 0, Units.degreesToRadians(pose2d.getRotation().getDegrees()))
        );
    }
    
    /**
     * Simulates the getBotPoseEstimate_wpiBlue() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return A simulated pose estimate for the blue alliance
     */
    public static LimelightHelpers.PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        if (!targetVisible) {
            return new LimelightHelpers.PoseEstimate();
        }
        
        Pose2d pose = getRobotPose();
        
        // Create a simulated pose estimate
        return new LimelightHelpers.PoseEstimate(
            pose,
            System.currentTimeMillis() / 1000.0, // Current time in seconds
            0.02, // 20ms latency
            1, // Tag count
            0.5, // Tag span
            1.0, // Average tag distance
            0.1, // Average tag area
            new LimelightHelpers.RawFiducial[0], // No raw fiducials
            false // Not MegaTag2
        );
    }
    
    /**
     * Simulates the getBotPoseEstimate_wpiBlue_MegaTag2() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @return A simulated pose estimate for the blue alliance using MegaTag2
     */
    public static LimelightHelpers.PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
        if (!targetVisible) {
            return new LimelightHelpers.PoseEstimate();
        }
        
        Pose2d pose = getRobotPose();
        
        // Create a simulated pose estimate
        return new LimelightHelpers.PoseEstimate(
            pose,
            System.currentTimeMillis() / 1000.0, // Current time in seconds
            0.02, // 20ms latency
            1, // Tag count
            0.5, // Tag span
            1.0, // Average tag distance
            0.1, // Average tag area
            new LimelightHelpers.RawFiducial[0], // No raw fiducials
            true // MegaTag2
        );
    }
    
    /**
     * Simulates the SetRobotOrientation() function from LimelightHelpers
     * @param limelightName The name of the Limelight (ignored in simulation)
     * @param yaw Robot yaw in degrees
     * @param yawRate Angular velocity of robot yaw in degrees per second
     * @param pitch Robot pitch in degrees
     * @param pitchRate Angular velocity of robot pitch in degrees per second
     * @param roll Robot roll in degrees
     * @param rollRate Angular velocity of robot roll in degrees per second
     */
    public static void SetRobotOrientation(String limelightName, double yaw, double yawRate, 
        double pitch, double pitchRate, double roll, double rollRate) {
        // In simulation, we don't need to do anything with this data
        // But we'll publish it to NetworkTables for consistency
        double[] orientation = {yaw, yawRate, pitch, pitchRate, roll, rollRate};
        table.getDoubleArrayTopic("robot_orientation_set").publish().set(orientation);
    }
} 