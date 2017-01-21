package org.cafirst.frc.team5406.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.cafirst.frc.team5406.parts.XboxController;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Encoder;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//Strings used to indicate auto and Smartdashboard values
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	final String desiredRPMKey = "Desired RPM";
	
	//Talon SRX IDs - Drive Motors (on CAN bus)
	public static int LEFT_DRIVE_MOTOR_A_PORT = 10; 
	public static int LEFT_DRIVE_MOTOR_B_PORT = 11;
	public static int RIGHT_DRIVE_MOTOR_A_PORT = 12;
	public static int RIGHT_DRIVE_MOTOR_B_PORT = 13;
	public static int LEFT_DRIVE_ENC_A = 1;
	public static int LEFT_DRIVE_ENC_B = 2;
	int leftDriveEncOffset = 0;
	public static int RIGHT_DRIVE_ENC_A = 3;
	public static int RIGHT_DRIVE_ENC_B = 4;
	public static int rightDriveEncOffset = 0;
	public static int encoderCPR = 2048;
	public static int wheelDiam = 8; //inches
	
	//Used to find delta time and delta ticks
	public long lastEnc = 0;
	public long lastTime = 0;
	public double wheelSpeed = 0;
	
	//Value used for PID
	double previousError = 0.0;
	double accumI = 0.0;
	
	//Controller
	private XboxController driverPad;

	
    //Setup Drive Motors
    CANTalon leftDriveMotorA = new CANTalon(LEFT_DRIVE_MOTOR_A_PORT);
    CANTalon leftDriveMotorB = new CANTalon(LEFT_DRIVE_MOTOR_B_PORT);
    CANTalon rightDriveMotorA = new CANTalon(RIGHT_DRIVE_MOTOR_A_PORT);
    CANTalon rightDriveMotorB = new CANTalon(RIGHT_DRIVE_MOTOR_B_PORT); 
    Encoder rightDriveEnc = new Encoder(RIGHT_DRIVE_ENC_A, RIGHT_DRIVE_ENC_B);
    Encoder leftDriveEnc = new Encoder(LEFT_DRIVE_ENC_A, LEFT_DRIVE_ENC_B);



	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//Puts data on Smart Dashboard
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
        //Left motors have to turn in opposite direction
        leftDriveMotorA.setInverted(true);
        leftDriveMotorB.setInverted(true);
        
        //Controller
        driverPad = new XboxController(0);
        
        //Using 50V/s as a ramp up rate to minimize belt skipping. Slower rates decrease responsiveness.
        leftDriveMotorA.setVoltageRampRate(50);
        rightDriveMotorA.setVoltageRampRate(50);
        
        //Setup primary motors to be power driven and secondary motors as followers
        leftDriveMotorA.changeControlMode(TalonControlMode.PercentVbus);
        leftDriveMotorB.changeControlMode(TalonControlMode.Follower);
        leftDriveMotorB.set(LEFT_DRIVE_MOTOR_A_PORT); //pass ID of primary motor
        rightDriveMotorA.changeControlMode(TalonControlMode.PercentVbus);
        rightDriveMotorB.changeControlMode(TalonControlMode.Follower);
        rightDriveMotorB.set(RIGHT_DRIVE_MOTOR_A_PORT); //pass ID of primary motor
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
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		lastEnc = leftDriveEnc.get();
		lastTime = System.nanoTime();
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		/*switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			
			powerShooter(leftDriveMotorA, (int) SmartDashboard.getNumber(desiredRPMKey, 0));
			
			break;
		}*/
	}
	
	

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		//Allows you to change desired RPM using Controller direction pad. Left is plus and right is minus.
		if(Math.round(driverPad.getDirectionDegrees()) == 90)
			SmartDashboard.putNumber(desiredRPMKey, SmartDashboard.getNumber(desiredRPMKey, 0) + 1);
		
		if(Math.round(driverPad.getDirectionDegrees()) == -90)
			SmartDashboard.putNumber(desiredRPMKey, SmartDashboard.getNumber(desiredRPMKey, 0) - 1);
		
		//Powers shooter
		powerShooter(leftDriveMotorA,(int) SmartDashboard.getNumber(desiredRPMKey, 0));
		
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
		int currentEnc = leftDriveEnc.get();
		//change in encoder value
		int dEnc = (int)(currentEnc-lastEnc);
		lastEnc = currentEnc;
		//current time saved to a variable so that the same number can be reused
		long currentTime = System.nanoTime();
		//change in time from last iteration
		double dTime = currentTime - lastTime;
		lastTime = currentTime;
		double currentRPM = (dEnc/dTime)*(60*1e9/encoderCPR);

		//Calculates speed
		long targetDEnc = (long)(desiredRPM*(encoderCPR/(60*1e9))*dTime);
		double speedChangeMultiplier = calcSpeed(targetDEnc - dEnc);
		double speed = wheelSpeed +0.01*speedChangeMultiplier;
	    if(speed > 1){speed = 1;}
	    if (speed < -1){speed=-1;}
	    wheelSpeed = speed;
		
	    //Put Values on Smart Dashboard
    	SmartDashboard.putNumber("RPM", Math.abs(currentRPM));
    	SmartDashboard.putNumber("dt", dTime);
    	SmartDashboard.putNumber("dEnc", dEnc);
    	SmartDashboard.putNumber("tEnc", targetDEnc);
	    SmartDashboard.putNumber("Speed", wheelSpeed);
	    
	    //Sends power to the shooter motor
	    shooterMotor.set(wheelSpeed);
	}
	
	public double calcSpeed(double currentError){
		
		//PID Values
		double kP = 0.1;
		double kI = 0.008;
		double kD = 0.01;
		
		//PID Math
		double valP = kP * currentError;
		double valI = accumI;
		double valD = kD * (previousError - currentError);
		if(Math.abs(valD) > Math.abs(valP)) valD = valP; // Limit so that D isn't the driving number
		accumI += kI;
		
		//If we overshoot, reset the I
		if(Math.signum(previousError) != Math.signum(currentError)){ 
			accumI = 0; 
			valI = 0;
		}
		
		double speed = valP + (valI * (currentError > 0 ? 1.0 : -1.0)) - valD;

		previousError = currentError;
		
		return speed;
	}
}

