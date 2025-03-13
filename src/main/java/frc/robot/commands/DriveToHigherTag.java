package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToHigherTag extends Command {
    private PIDController xController, yController, rotController;
    private Timer dontSeeTagTimer, stopTimer, extraForwardTimer;
    private CommandSwerveDrivetrain drivebase;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private boolean isInExtraForwardMode = false;
    
    // PID constants
    private final double X_P = 2.0;  // Forward/back movement - lower than reef since we're moving further
    private final double Y_P = 4.0;  // Left/right movement for centering
    private final double ROT_P = 0.05;  // Rotation control
    
    // Target setpoints and tolerances
    private final double X_SETPOINT = -1.5;  // Further back from tag (in meters)
    private final double Y_SETPOINT = 0.0;   // Centered with tag
    private final double ROT_SETPOINT = 0.0; // Facing straight at tag
    
    private final double X_TOLERANCE = 0.05;  // Meters
    private final double Y_TOLERANCE = 0.05;  // Meters
    private final double ROT_TOLERANCE = 1.0; // Degrees
    
    // Timing constants
    private final double DONT_SEE_TAG_WAIT_TIME = 3.0;
    private final double POSE_VALIDATION_TIME = 0.3;
    private final double EXTRA_FORWARD_TIME = 4.0;  // Extra time to move forward
    private final double FORWARD_SPEED = 0.7;  // Speed for forward movement

    public DriveToHigherTag(CommandSwerveDrivetrain drivebase) {
        this.drivebase = drivebase;
        
        xController = new PIDController(X_P, 0, 0);
        yController = new PIDController(Y_P, 0, 0);
        rotController = new PIDController(ROT_P, 0, 0);
        
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();
        this.extraForwardTimer = new Timer();
        this.isInExtraForwardMode = false;

        rotController.setSetpoint(ROT_SETPOINT);
        rotController.setTolerance(ROT_TOLERANCE);

        xController.setSetpoint(X_SETPOINT);
        xController.setTolerance(X_TOLERANCE);

        yController.setSetpoint(Y_SETPOINT);
        yController.setTolerance(Y_TOLERANCE);
    }

    @Override
    public void execute() {
        // If we're in extra forward mode, just keep moving forward
        if (isInExtraForwardMode) {
            drivebase.setControl(robotCentric.withVelocityX(FORWARD_SPEED)
                                          .withVelocityY(0)
                                          .withRotationalRate(0));
            return;
        }

        if (LimelightHelpers.getTV("")) {
            this.dontSeeTagTimer.reset();

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            SmartDashboard.putNumber("x_to_tag", positions[2]);
            SmartDashboard.putNumber("y_to_tag", positions[0]);
            SmartDashboard.putNumber("rot_to_tag", positions[4]);

            // Calculate control outputs
            double xSpeed = xController.calculate(positions[2]);
            double ySpeed = -yController.calculate(positions[0]);
            double rotValue = -rotController.calculate(positions[4]);

            // Limit maximum speeds
            xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), 2.0), xSpeed);
            ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), 2.0), ySpeed);
            rotValue = Math.copySign(Math.min(Math.abs(rotValue), 1.0), rotValue);

            // Always maintain some forward movement while not at X setpoint
            if (!xController.atSetpoint()) {
                xSpeed = Math.max(xSpeed, FORWARD_SPEED);
            }

            drivebase.setControl(robotCentric.withVelocityX(xSpeed)
                                          .withVelocityY(ySpeed)
                                          .withRotationalRate(rotValue));

            // Check if we're centered enough to start extra forward movement
            if (Math.abs(positions[0]) < Y_TOLERANCE && 
                Math.abs(positions[4]) < ROT_TOLERANCE) {
                isInExtraForwardMode = true;
                extraForwardTimer.reset();
                extraForwardTimer.start();
            }
        } else {
            // If we lose the tag but we're already centered, start extra forward movement
            if (stopTimer.get() > POSE_VALIDATION_TIME) {
                isInExtraForwardMode = true;
                extraForwardTimer.reset();
                extraForwardTimer.start();
            } else {
                drivebase.setControl(robotCentric.withVelocityX(0)
                                              .withVelocityY(0)
                                              .withRotationalRate(0));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(robotCentric.withVelocityX(0)
                                      .withVelocityY(0)
                                      .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Only finish if we've completed the extra forward movement or haven't seen the tag for too long
        return (isInExtraForwardMode && extraForwardTimer.hasElapsed(EXTRA_FORWARD_TIME)) ||
               (!isInExtraForwardMode && dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME));
    }
} 