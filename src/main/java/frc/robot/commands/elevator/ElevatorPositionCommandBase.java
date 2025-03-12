package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public abstract class ElevatorPositionCommandBase extends Command {
    protected static Elevator m_elevator;
    protected static MotionMagicVoltage m_request;

    public static void initialize(Elevator elevator, MotionMagicVoltage request) {
        m_elevator = elevator;
        m_request = request;
    }

    public ElevatorPositionCommandBase() {
        addRequirements(m_elevator);
    }

    @Override
    public boolean isFinished() {
        // Consider the command finished when we're within 0.1 rotations of the target
        double targetPosition = m_request.Position;
        double currentPosition = m_elevator.getPosition();
        return Math.abs(currentPosition - targetPosition) < 0.1;
    }
}