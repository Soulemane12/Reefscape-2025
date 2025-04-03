package frc.robot.commands.elevator;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorToL2Position extends ElevatorPositionCommandBase {
    @Override
    public void execute() {
        m_elevator.setPositionWithRequest(m_request.withPosition(ElevatorConstants.kElevatorL2Position));
    }
}