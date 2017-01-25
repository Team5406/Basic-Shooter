package org.usfirst.frc.team5406.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Encoder;
import java.util.Timer;
import java.util.TimerTask;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	//Talon SRX IDs - Drive Motors (on CAN bus)
	public static int SHOOTER_MOTOR_PORT = 20; 
	public static int LEFT_DRIVE_ENC_A = 1;
	public static int LEFT_DRIVE_ENC_B = 2;
	public static int encoderCPR = 1024;
	public static int wheelDiam = 8; //inches
	
	public long lastEnc = 0;
	public long lastTime = 0;
	public double wheelSpeed = 0;
	Timer PIDTimer = new Timer();
	
	
    //Setup Drive Motors
    CANTalon shooterMotor = new CANTalon(SHOOTER_MOTOR_PORT);
    Encoder shooterMotorEnc = new Encoder(LEFT_DRIVE_ENC_A, LEFT_DRIVE_ENC_B);


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
        
        
        //Setup primary motors to be power driven and secondary motors as followers
		shooterMotor.changeControlMode(TalonControlMode.Voltage);
		

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
		lastEnc = shooterMotorEnc.get();
		lastTime = System.nanoTime();
		PIDTimer = new Timer();
	    PIDTimer.schedule(new PIDLoop(), 0L, 10L); //time in milliseconds
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {


	}
	
	class PIDLoop extends TimerTask {
		private double kP = 0.001;
		private double kI = 0.05;
		private double kD = 0.5;
		private double accumI = 0.0;
		
		private double previousError = 0.0;
		private int targetRPM = 2700;
		
        public void run() {
        	long currentEnc = shooterMotorEnc.get();
			//change in encoder value
			long dEnc = (int)(currentEnc-lastEnc);
			lastEnc = currentEnc;
			//current time saved to a variable so that the same number can be reused
			long currentTime = System.nanoTime();
			//change in time from last iteration
			double dTime = currentTime - lastTime;
			lastTime = currentTime;
			double currentRPM = (dEnc/dTime)*(60*1e9/encoderCPR);

		    	    		
			long targetDEnc = (long)(targetRPM*(encoderCPR/(60*1e9))*dTime);
			double speedChangeMultiplier = calcSpeed(targetDEnc - dEnc);
	    	SmartDashboard.putNumber("speedChangeMultiplier", speedChangeMultiplier);
			
			double speed = wheelSpeed +0.05*speedChangeMultiplier;
		    if(speed > 12){speed = 12;}
		    if (speed < -12){speed=-12;}
		    wheelSpeed = speed;
			
	    	SmartDashboard.putNumber("RPM", currentRPM);
	    	SmartDashboard.putNumber("Time Change", dTime);
	    	SmartDashboard.putNumber("Encoder Change", dEnc);
	    	SmartDashboard.putNumber("Encoder Target", targetDEnc);
		    SmartDashboard.putNumber("Voltage", wheelSpeed);
		    SmartDashboard.putNumber("Error", targetDEnc - dEnc);
		    shooterMotor.set(wheelSpeed);
        }
        
        public double calcSpeed(double currentError){
			
    		double valP = kP * currentError;
    		double valI = accumI;
    		double valD = kD * (previousError - currentError);
    		//if(Math.abs(valD) > Math.abs(valP)) valD = valP; // Limit so that D isn't the driving number
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
	

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void disabledInit() {
		PIDTimer.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}



