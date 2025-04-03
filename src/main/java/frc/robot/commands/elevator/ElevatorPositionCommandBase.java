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

   
}