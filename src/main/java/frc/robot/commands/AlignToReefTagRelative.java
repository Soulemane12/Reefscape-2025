package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ReefAlignmentConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private CommandSwerveDrivetrain drivebase;
  private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

  public AlignToReefTagRelative(boolean isRightScore, CommandSwerveDrivetrain drivebase) {
    xController = new PIDController(ReefAlignmentConstants.X_REEF_ALIGNMENT_P, 0, 0);  // Vertical movement
    yController = new PIDController(ReefAlignmentConstants.Y_REEF_ALIGNMENT_P, 0, 0);  // Horizontal movement
    rotController = new PIDController(ReefAlignmentConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(ReefAlignmentConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(ReefAlignmentConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(ReefAlignmentConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(ReefAlignmentConstants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? ReefAlignmentConstants.Y_SETPOINT_REEF_ALIGNMENT : -ReefAlignmentConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(ReefAlignmentConstants.Y_TOLERANCE_REEF_ALIGNMENT);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);

      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), 2.0), xSpeed);
      ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), 2.0), ySpeed);
      rotValue = Math.copySign(Math.min(Math.abs(rotValue), 1.0), rotValue);

      drivebase.setControl(robotCentric.withVelocityX(xSpeed)
                                    .withVelocityY(ySpeed)
                                    .withRotationalRate(rotValue));

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.setControl(robotCentric.withVelocityX(0)
                                    .withVelocityY(0)
                                    .withRotationalRate(0));
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
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(ReefAlignmentConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(ReefAlignmentConstants.POSE_VALIDATION_TIME);
  }
} 