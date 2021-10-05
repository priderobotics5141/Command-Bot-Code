package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TankDriveCommand extends Command {
    public TankDriveCommand() {
        // the requires(Subsystem) method must be called for each subsystem used by the command
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        //gets the values of controller axis'
        double leftStickY = RobotContainer.getControllerRawAxis(Constants.LEFT_STICK_Y);
        double rightStickY = RobotContainer.getControllerRawAxis(Constants.RIGHT_STICK_Y);

        //sends the value of the sticks to the motors multiplied by the sensitivity
        Robot.driveTrain.setLeftMotors(leftStickY * Constants.SENSITIVITY);
        Robot.driveTrain.setRightMotors(rightStickY * Constants.SENSITIVITY);

    }

    @Override
    protected boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    protected void end() { //stops motors
        Robot.driveTrain.setRightMotors(0);
        Robot.driveTrain.setLeftMotors(0);

    }

    @Override
    protected void interrupted() {
        this.end(); //Makes sure to run end
    }
}
