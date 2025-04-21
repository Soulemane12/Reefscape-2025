package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.elevator.ElevatorJoystickCommand;
import frc.robot.commands.position.PivotSetPositionCommand;
import frc.robot.commands.position.PositionJoystickCommand;
import frc.robot.commands.elevator.ElevatorToL2Position;
import frc.robot.commands.elevator.ElevatorToL3Position;
import frc.robot.commands.elevator.ElevatorToL4Position;
import frc.robot.commands.elevator.ElevatorTo0Position;
import frc.robot.commands.elevator.ElevatorToPoint0Position;
import frc.robot.commands.AlignToReefTagRelative;

import frc.robot.commands.elevator.ElevatorPositionCommandBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.DriveToHigherTag;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.1) // Reduce deadband to 5%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController driver = new CommandXboxController(0); //driver
    private final CommandXboxController operator = new CommandXboxController(1); //operator

    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Shooter shooter = new Shooter();
    private final Pivot m_Pivot = new Pivot();
    private final Elevator m_elevator = new Elevator();
    private final Climber m_climber = new Climber();

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private final double kElevatorGravityCompensation = 0.04;
    private final double kPositionGravityCompensation = -0.6; // Increased gravity compensation

    // Add adjusted position variables as class fields
    private final double adjustedL2;
    private final double adjustedL3;
    private final double adjustedL0;
    private final double adjustedL4;
    private final double adjustedParallel;
    private final double adjustedin;

    private final SendableChooser<Command> autoChooser;

    private final ElevatorToL2Position m_elevatorToL2Position;
    private final ElevatorToL3Position m_elevatorToL3Position;
    private final ElevatorToL4Position m_elevatorToL4Position;
    private final ElevatorTo0Position m_elevatorTo0Position;
    private final ElevatorToPoint0Position m_elevatorToPoint0Position;


    private final PivotSetPositionCommand m_pivotToL2;
    private final PivotSetPositionCommand m_pivotToL3;

    private final PivotSetPositionCommand m_pivotTo0;
    private final PivotSetPositionCommand m_pivotToL4;
    private final PivotSetPositionCommand m_pivotToIntake;
    private final PivotSetPositionCommand m_pivotToIN;

    private final MotionMagicVoltage pivotRequest;
    private final MotionMagicVoltage elevatorRequest;

    public final edu.wpi.first.wpilibj.smartdashboard.Field2d m_field = new edu.wpi.first.wpilibj.smartdashboard.Field2d();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Auto");
        
        // Publish Field2d widget to SmartDashboard
        SmartDashboard.putData("Field", m_field);
        
        // Add AprilTag targets to the field visualization
        m_field.getObject("LeftReefTag").setPose(2.0, 2.0, Rotation2d.fromDegrees(0));
        m_field.getObject("RightReefTag").setPose(2.0, -2.0, Rotation2d.fromDegrees(0));
        m_field.getObject("HigherTag").setPose(3.0, 0.0, Rotation2d.fromDegrees(0));
        
        // Set the drivetrain reference in SimLimelightHelpers for simulation
        if (Robot.isSimulation()) {
            SimLimelightHelpers.setDrivetrain(drivetrain);
        }
        
        // Initialize the request objects
        pivotRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);
        
        elevatorRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);
        
        // Initialize the elevator command base
        ElevatorPositionCommandBase.initialize(m_elevator, elevatorRequest);

        // Create command instances
        m_elevatorToL2Position = new ElevatorToL2Position();
        m_elevatorToL3Position = new ElevatorToL3Position();
        m_elevatorToL4Position = new ElevatorToL4Position();
        m_elevatorTo0Position = new ElevatorTo0Position();
        m_elevatorToPoint0Position = new ElevatorToPoint0Position();

        // Set elevator to "fake" zero position on robot init
        m_elevatorToPoint0Position.schedule();
        new PivotSetPositionCommand(m_Pivot, pivotRequest, PivotConstants.kPivotInPosition).schedule();

        // Compute the adjusted setpoints with gravity compensation
        adjustedL2 = PivotConstants.kPivotL2Position + 
            ((kPositionGravityCompensation+1.7) * Math.sin(PivotConstants.kPivotL2Position));
        
        adjustedL3 = PivotConstants.kPivotL3Position + 
            ((kPositionGravityCompensation+1.7) * Math.sin(PivotConstants.kPivotL3Position));
        
        adjustedL0 = PivotConstants.kPivotInPosition + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotInPosition));
        
        adjustedL4 = PivotConstants.kPivotL4Position + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotL4Position));
        
        adjustedParallel = PivotConstants.kPivotIntakePosition + 
            ((kPositionGravityCompensation +1.7)* Math.sin(PivotConstants.kPivotIntakePosition));
        
        adjustedin = PivotConstants.kPivotInPosition + 
            (kPositionGravityCompensation * Math.sin(PivotConstants.kPivotInPosition));

        // Now create your pivot commands with the adjusted setpoints
        m_pivotToL2 = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL2);
        m_pivotToL3 = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL3);

        m_pivotTo0   = new PivotSetPositionCommand(m_Pivot, m_request, adjustedL0);
        m_pivotToL4   = new PivotSetPositionCommand(m_Pivot, pivotRequest, adjustedL4);
        m_pivotToIntake = new PivotSetPositionCommand(m_Pivot, m_request, adjustedParallel);
     
        m_pivotToIN = new PivotSetPositionCommand(m_Pivot, m_request, adjustedin);

        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("moveToL2", Commands.sequence(
            new ElevatorToL2Position().withTimeout(.2),
            new PivotSetPositionCommand(m_Pivot, m_request, adjustedL2),
            Commands.waitSeconds(0.01)
        ));

        NamedCommands.registerCommand("moveToL3", Commands.sequence(
            new ElevatorToL3Position().withTimeout(.33),
            new PivotSetPositionCommand(m_Pivot, m_request, adjustedL3),
            Commands.waitSeconds(0.01)
        ));

        NamedCommands.registerCommand("moveToL4", Commands.sequence(
            new ElevatorToL4Position().withTimeout(.95),
            new PivotSetPositionCommand(m_Pivot, pivotRequest, adjustedL4),
            Commands.waitSeconds(0.01)
        ));

        NamedCommands.registerCommand("pivotTo0", Commands.sequence(
            new PivotSetPositionCommand(m_Pivot, pivotRequest, PivotConstants.kPivotInPosition),
            Commands.waitSeconds(0.01)
            
        ));

        NamedCommands.registerCommand("Zero_Pos", Commands.sequence(
            new PivotSetPositionCommand(m_Pivot, pivotRequest,PivotConstants.kPivotInPosition ).withTimeout(.95),
            new ElevatorTo0Position(),
            Commands.waitSeconds(0.01)
        ));

        NamedCommands.registerCommand("shooterIntake", Commands.sequence(
            shooter.shooterIntakeControl().withTimeout(.75)

            
        ) );

        NamedCommands.registerCommand("shooterOutake", Commands.sequence(
            shooter.shooterOutakeControl(),            
            Commands.waitSeconds(0.01)
            
        ) );
        
        NamedCommands.registerCommand("alignToRightReef", new AlignToReefTagRelative(true, drivetrain).withTimeout(.75));
        
        NamedCommands.registerCommand("alignToLeftReef", new AlignToReefTagRelative(false, drivetrain).withTimeout(.75));

        

        

        // Configure button bindings
        configureBindings();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }


    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Add simulation-specific bindings for testing Limelight targets
        if (Robot.isSimulation()) {
            // Left bumper + A = show left reef tag
            driver.leftBumper().and(driver.a()).onTrue(Commands.runOnce(() -> setSimulatedTarget(1)));
            
            // Left bumper + B = show right reef tag
            driver.leftBumper().and(driver.b()).onTrue(Commands.runOnce(() -> setSimulatedTarget(2)));
            
            // Left bumper + Y = show higher tag
            driver.leftBumper().and(driver.y()).onTrue(Commands.runOnce(() -> setSimulatedTarget(3)));
            
            // Left bumper + X = hide all tags
            driver.leftBumper().and(driver.x()).onTrue(Commands.runOnce(() -> hideSimulatedTargets()));
        }

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.30).withVelocityY(0))
        );
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.30).withVelocityY(0))
        );
        driver.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.30))
        );
        driver.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.30))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Parallel-Elevator-Pivot sequences for teleop
        operator.pov(180).onTrue(
            Commands.sequence(
                new ElevatorTo0Position()
            )
        );

        operator.pov(0).onTrue(
            Commands.sequence(
                new PivotSetPositionCommand(m_Pivot, pivotRequest, PivotConstants.kPivotInPosition)
            )
        );

        operator.pov(270).onTrue(
            Commands.sequence(
                new ElevatorToL4Position().withTimeout(.95),
                new PivotSetPositionCommand(m_Pivot, m_request, adjustedParallel),
                Commands.waitSeconds(999)            
            )
        );

        operator.a().onTrue(Commands.sequence(
            new ElevatorToL2Position().withTimeout(.2),
            new PivotSetPositionCommand(m_Pivot, m_request, adjustedL2),
            Commands.waitSeconds(999)
        ));

        operator.b().onTrue(Commands.sequence(
            new ElevatorToL3Position().withTimeout(.33),
            new PivotSetPositionCommand(m_Pivot, m_request, adjustedL3),
            Commands.waitSeconds(999)
        ));

        operator.y().onTrue(Commands.sequence(
            new ElevatorToL4Position().withTimeout(.95),
            new PivotSetPositionCommand(m_Pivot, pivotRequest, adjustedL4),
            Commands.waitSeconds(999)
        ));

        operator.x().onTrue(Commands.sequence(
            new ElevatorTo0Position().withTimeout(0.01),
            new PivotSetPositionCommand(m_Pivot, m_request, adjustedParallel),
            Commands.waitSeconds(999)
        ));
        
     
        // Right bumper for right reef alignment, left bumper for left reef alignment
        driver.rightBumper().whileTrue(new AlignToReefTagRelative(true, drivetrain));
        driver.leftBumper().whileTrue(new AlignToReefTagRelative(false, drivetrain));
        
        // Y button for driving to and centering with a higher AprilTag
        driver.y().whileTrue(new DriveToHigherTag(drivetrain));

        //Shooter Control
        operator.rightTrigger().whileTrue(shooter.shooterIntakeControl());
        operator.leftTrigger().whileTrue(shooter.shooterOutakeControl());

        //Climber Control
        operator.rightBumper().whileTrue(m_climber.climberForwardControl());
        operator.leftBumper().whileTrue(m_climber.climberReverseControl());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Middle");
    }
    
    /**
     * Sets which target is visible in the simulated Limelight.
     * This only works in simulation.
     * 
     * @param targetId The ID of the target to make visible (1 = left reef, 2 = right reef, 3 = higher tag)
     */
    public void setSimulatedTarget(int targetId) {
        if (Robot.isSimulation()) {
            SimLimelightHelpers.setTargetId(targetId);
            SimLimelightHelpers.setTargetVisible(true);
        }
    }
    
    /**
     * Makes all targets invisible in the simulated Limelight.
     * This only works in simulation.
     */
    public void hideSimulatedTargets() {
        if (Robot.isSimulation()) {
            SimLimelightHelpers.setTargetVisible(false);
        }
    }
}