package frc.robot.commands.position;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Pivot;

public class PositionJoystickCommand extends Command {
    private final Pivot m_position;
    private final CommandXboxController m_joystick;
    private final double kPositionGravityCompensation;

    public PositionJoystickCommand(Pivot position, CommandXboxController joystick, double gravityCompensation) {
        m_position = position;
        m_joystick = joystick;
        kPositionGravityCompensation = gravityCompensation;
        addRequirements(position);
    }

    @Override
    public void execute() {
        double rawSpeed = m_joystick.getRightY();
        double speed = applyDeadband(rawSpeed, 0.05);
        
        if (speed == 0) {
            m_position.setShooterPosition(kPositionGravityCompensation);
        } else if (speed < 0) {
            m_position.setShooterPosition(-speed + kPositionGravityCompensation);
        } else {
            m_position.setShooterPosition(-speed);
        }
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }
} 