package frc.robot.commands.elevator;

public class ElevatorToL4Position extends ElevatorPositionCommandBase {
    
    public ElevatorToL4Position() {
        // The parent class (ElevatorPositionCommandBase) already handles the elevator requirements
        // No need to add any other requirements since this command only moves the elevator
    }

    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(4.5));
    }
}