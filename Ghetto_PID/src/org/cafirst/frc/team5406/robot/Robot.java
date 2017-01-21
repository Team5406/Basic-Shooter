package org.cafirst.frc.team5406.robot;

import org.cafirst.frc.team5406.parts.XboxController;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//Strings used for auto and Smartdashboard values
	final String leftAuto = "Left";
	final String rightAuto = "Right";
	final String bothAuto = "Both";
	final String revolution = "Revolution";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	final String desiredRPMKey = "Desired RPM";

	//IDs of Motors
	final int LEFT_DRIVE_MOTOR = 10;
	final int LEFT_BACK_DRIVE_MOTOR = 11;
	final int RIGHT_DRIVE_MOTOR = 12;
	final int RIGHT_BACK_DRIVE_MOTOR = 13;
	
	//Ticks Per Revolution
	private int rev = 1900;
	
	//Differences value
	private double timeLast = 0;
	private double tickLast = 0;
	
	//Starting power
	private double power = 0.8;
	
	//Drive Motors
	private CANTalon leftDriveMotor = new CANTalon(LEFT_DRIVE_MOTOR);
	private CANTalon leftBackDriveMotor = new CANTalon(LEFT_BACK_DRIVE_MOTOR);
	private CANTalon rightDriveMotor = new CANTalon(RIGHT_DRIVE_MOTOR);
	private CANTalon rightBackDriveMotor = new CANTalon(RIGHT_BACK_DRIVE_MOTOR);
	
	//Encoders
	private Encoder leftEncoder = new Encoder(1, 2);//5, 6 These are for Squirt
	private Encoder rightEncoder = new Encoder(3, 4);//7, 8 These are for Squirt
	
	//Robot Drive (Controls Driving)
	private RobotDrive drive = new RobotDrive(leftDriveMotor, rightDriveMotor);
	
	//Controller
	private XboxController driverPad;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		//Auto Options
		chooser.addDefault("Left Auto", leftAuto);
		chooser.addObject("Right Auto", rightAuto);
		chooser.addObject("Both Auto", bothAuto);
		chooser.addObject("Revolution", revolution);
		
		//Dashboards
		SmartDashboard.putData("Auto choices", chooser);
		SmartDashboard.putNumber("Ticks", 0);
		SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.get() * -1);
		SmartDashboard.putNumber("Desired RPM", 0);

		//Encoder reset
		leftEncoder.reset();
		rightEncoder.reset();
		
		//Driver Controller
		driverPad = new XboxController(0);
		
		//Voltage ramp rate for the wheels to 50v/s. Higher risks belt skipper. Lower lowers responsiveness.
		leftDriveMotor.setVoltageRampRate(50);
		rightDriveMotor.setVoltageRampRate(50);
		
		//Sets the masters and the slaves for the motors
		leftDriveMotor.changeControlMode(TalonControlMode.PercentVbus);
		leftBackDriveMotor.changeControlMode(TalonControlMode.Follower);
		leftBackDriveMotor.set(LEFT_DRIVE_MOTOR);
		rightDriveMotor.changeControlMode(TalonControlMode.PercentVbus);
		rightBackDriveMotor.changeControlMode(TalonControlMode.Follower);
		rightBackDriveMotor.set(RIGHT_DRIVE_MOTOR);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	
	@Override
	public void teleopPeriodic() {
		
		//Allows you to change desiredRPM with direction pad. Left plus and right minus.
		if(Math.round(driverPad.getDirectionDegrees()) == 90)
			SmartDashboard.putNumber(desiredRPMKey, SmartDashboard.getNumber(desiredRPMKey, 0) + 1);
		if(Math.round(driverPad.getDirectionDegrees()) == -90)
			SmartDashboard.putNumber(desiredRPMKey, SmartDashboard.getNumber(desiredRPMKey, 0) - 1);
		
		//Puts values on Smart Dashboard
		SmartDashboard.putNumber("Direction Pad", driverPad.getDirectionDegrees());
		SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.get() * -1);
		
		//Power Shooter
		powerShooter(leftDriveMotor, (int) SmartDashboard.getNumber(desiredRPMKey, 0));
	}
	
	@Override
	public void autonomousInit() {
		
		SmartDashboard.putData("Auto choices", chooser);
		
		autoSelected = chooser.getSelected();
		
		leftEncoder.reset();
		rightEncoder.reset();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		
		/*SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.get() * -1);
		
		double left = 0;
		double right = 0;
		
		SmartDashboard.putNumber("Speed", left);
		
		switch (autoSelected) {
		
		case revolution:
			if(leftEncoder.get() < SmartDashboard.getNumber("Ticks", 0))
				left = -0.1 - 0.8 * (SmartDashboard.getNumber("Ticks", 0) - leftEncoder.get() - 15) / SmartDashboard.getNumber("Ticks", 0);
			else if(leftEncoder.get() > SmartDashboard.getNumber("Ticks", 0) + 25)
				left = 0.1;
			
			drive.drive(left, 0);
			break;
			
		case rightAuto:
			if(rightEncoder.get() * -1 < SmartDashboard.getNumber("Ticks", 0))
				right = -0.1 - 0.8 * (SmartDashboard.getNumber("Ticks", 0) + rightEncoder.get() - 15) / SmartDashboard.getNumber("Ticks", 0);
			else if(rightEncoder.get() * -1 > SmartDashboard.getNumber("Ticks", 0) + 25)
				right = 0.1;
			
			SmartDashboard.putNumber("Speed", right);
			break;
			
		case leftAuto:
			if(leftEncoder.get() < SmartDashboard.getNumber("Ticks", 0))
				left = -0.1 - 0.8 * (SmartDashboard.getNumber("Ticks", 0) - leftEncoder.get() - 15) / SmartDashboard.getNumber("Ticks", 0);
			else if(leftEncoder.get() > SmartDashboard.getNumber("Ticks", 0) + 25)
				left = 0.1;
			
			SmartDashboard.putNumber("Speed", left);
			break;
			
			
			
		case bothAuto:
		default:
			if(leftEncoder.get() < SmartDashboard.getNumber("Ticks", 0))
				left = -0.1 - 0.9 * (SmartDashboard.getNumber("Ticks", 0) - leftEncoder.get() - 15) / SmartDashboard.getNumber("Ticks", 0);
			else if(leftEncoder.get() > SmartDashboard.getNumber("Ticks", 0) + 25)
				left = 0.1;
			
			if(rightEncoder.get() * -1 < SmartDashboard.getNumber("Ticks", 0))
				right = -0.1 - 0.9 * (SmartDashboard.getNumber("Ticks", 0) - rightEncoder.get() * -1 - 15) / SmartDashboard.getNumber("Ticks", 0);
			else if(rightEncoder.get() * -1 > SmartDashboard.getNumber("Ticks", 0) + 25)
				right = 0.1;
			
			SmartDashboard.putNumber("Speed", left);
			break;
		}
		
		drive.setLeftRightMotorOutputs(left, right);*/
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	/**
	 * Powers the shooter and maintains the desired RPM.
	 * @param shooterMotor Motor of the shooter.
	 * @param desiredRPM Your desired RPM. Max is around 390
	 */
	public void powerShooter(CANTalon shooterMotor, int desiredRPM)
	{
		//Gets the time for time delta
		double timeCurrent = System.nanoTime() / (1E6);
		double timeDifference = timeCurrent - timeLast;
		timeLast = timeCurrent;
		
		//Gets the ticks for tick delta
		double tickCurrent = leftEncoder.get() * -1;
		double tickDifference = tickCurrent - tickLast;
		tickLast = tickCurrent;
		
		//Calculates the RPM
		double RPM = ((tickDifference / timeDifference) * 60000) / rev;
		
		//If RPM is within the range power difference shrinks
		if(RPM < desiredRPM + 7 && RPM > desiredRPM - 7)
		{
			if(RPM > desiredRPM && power != 1) power -= 0.0005 / (desiredRPM / RPM);
			if(RPM < desiredRPM && power != 1) power += 0.0005 / (RPM / desiredRPM);
		}
		
		//If RPM is far away from the desired RPM then large increase/decrease in power
		else
		{
			if(RPM > desiredRPM && power != 1) power -= 0.005 / (RPM / desiredRPM);
			if(RPM < desiredRPM && power != 1) power += 0.005 / (desiredRPM / RPM);
		}
		
		//Keeps power within 0% and 100%
		if(power > 1) power = 1;
		if(power < -1) power = -1;
		if(desiredRPM == 0) power = 0;
		
		//Puts values on Dashboard
		SmartDashboard.putNumber("Time Difference", timeDifference);
		SmartDashboard.putNumber("Tick Difference", tickDifference);
		SmartDashboard.putNumber("Rev", RPM);
		SmartDashboard.putNumber("MyRPM", RPM);
		SmartDashboard.putNumber("Speed", power);
		
		//Puts power to the motor
		shooterMotor.set(power);
	}
}

