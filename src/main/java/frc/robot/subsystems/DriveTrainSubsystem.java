package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TankDriveCommand;

public class DriveTrainSubsystem extends Subsystem {
    private VictorSP leftMotor1 = new VictorSP(Constants.MOTOR_LEFT_0_ID);
    private VictorSP leftMotor2 = new VictorSP(Constants.MOTOR_LEFT_1_ID);
    private VictorSP rightMotor1 = new VictorSP(Constants.MOTOR_RIGHT_0_ID);
    private VictorSP rightMotor2 = new VictorSP(Constants.MOTOR_RIGHT_1_ID);

    private final static DriveTrainSubsystem INSTANCE = new DriveTrainSubsystem();

    public static DriveTrainSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this DriveTrainSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    public DriveTrainSubsystem() {

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new TankDriveCommand());
    }

    public void setLeftMotors(double speed) {
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);

        leftMotor1.set(speed);
        leftMotor2.set(speed);
    }
    public void setRightMotors(double speed) {
        rightMotor1.setInverted(false);
        rightMotor2.setInverted(false);

        rightMotor1.set(speed);
        rightMotor2.set(speed);
    }

}

