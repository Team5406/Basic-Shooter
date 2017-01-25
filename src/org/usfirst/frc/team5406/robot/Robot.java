package org.usfirst.frc.team5406.robot;

/**
 * Example demonstrating the velocity closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 * 
 * Be sure to select the correct feedback sensor using SetFeedbackDevice() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolena input to the SetSensorDirection() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target velocity.  
 *
 * Tweak the PID gains accordingly.
 */

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
  
	CANTalon _talon = new CANTalon(20);	
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	long time_offset = 0;
	
	public void robotInit() {
        /* first choose the sensor */
        _talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        _talon.reverseSensor(false);
        //_talon.configEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
        //_talon.configPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

        /* set the peak and nominal outputs, 12V means full */
        _talon.configNominalOutputVoltage(+0.0f, -0.0f);
        _talon.configPeakOutputVoltage(+12.0f, 0.0f);
        /* set closed loop gains in slot0 */
        _talon.setProfile(0);
        _talon.setF(0.0296);
        _talon.setP(0.08);
        _talon.setI(0.001); 
        _talon.setD(3.5);
        
	}
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
 
    	double motorOutput = _talon.getOutputVoltage() / _talon.getBusVoltage();
  
    	SmartDashboard.putNumber("RPM", _talon.getSpeed());
	    SmartDashboard.putNumber("%VBus", motorOutput);
	    SmartDashboard.putNumber("Error", _talon.getClosedLoopError());


        if(++_loops >= 10) {
        	/* prepare line to print */
        	_sb.append("\ttime:");
    		_sb.append(Math.round((System.nanoTime()-time_offset)/1e6) + "ms");
        	_sb.append("\tout:");
    		_sb.append(motorOutput);
            _sb.append("\tspd:");
            _sb.append(_talon.getSpeed() );
            _sb.append("\terr:");
            _sb.append(_talon.getClosedLoopError());
            
        	_loops = 0;
        	System.out.println(_sb.toString());
        }
        _sb.setLength(0);
    }
    
    public void teleopInit(){
    	/* Speed mode */
    	double targetSpeed = 2700;
    	_talon.changeControlMode(TalonControlMode.Speed);
    	_talon.set(targetSpeed); /* 1500 RPM in either direction */

    	/* append more signals to print when in speed mode. */
    	time_offset = System.nanoTime();
        _sb.append("\terr:");
        _sb.append(_talon.getClosedLoopError());
        _sb.append("\ttrg:");
        _sb.append(targetSpeed);
    
    }
}