// skeleton of Robot.java
package org.texastorque;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.EncoderType;
import com.kauailabs.navx.frc.AHRS;

import org.texastorque.torquelib.base.TorqueIterative;
import org.texastorque.torquelib.util.GenericController;
import org.texastorque.util.VectorUtils;
import org.texastorque.torquelib.component.TorqueMotor;
import org.texastorque.torquelib.component.TorqueEncoder;
import org.texastorque.torquelib.controlLoop.ScheduledPID;


public class Robot extends TorqueIterative {

  	GenericController driver = new GenericController(0, 0.1);
  
	private AHRS NX_gyro;
	private double pitch = 0;
	private double roll = 0;
	private double yaw = 0;
	  
	private ScheduledPID rotationalPID; // MAKE THIS FINAL IN THE ACTUAL THING
 	private double rotSpeed = 0;
 	private double currentPos = 0;
	private double setpoint = 0;
	private double prevSetpoint = 0;
 
 	private TorqueMotor rotMot;
	private CANSparkMax transMot;
//   private Spark transMot;
//   private TorqueMotor transMot;
	private TorqueEncoder DB_rot;
	private CANEncoder DB_trans;

	private double encoderRotAngle = 0;
	private double encoderTransSpeed = 0;

	private double transX = 0;
	private double transY = 0;
	private double transMag = 0;
	private double transThetaRaw = 0;
	private double transTheta = 0;

	private double resultMag = 0;
	private double resultAngle = 0;

	private double rotMag = 0;

	private double transSpeed = 0;

	public void robotInit() {
		
		rotMot = new TorqueMotor(new VictorSP(1), false);
		transMot = new CANSparkMax(1, MotorType.kBrushless);
		
		DB_trans = transMot.getEncoder(EncoderType.kHallSensor, 4096);
		DB_rot = new TorqueEncoder(0,1,false,EncodingType.k4X);

		NX_gyro = new AHRS(SPI.Port.kMXP);

		// initialize reset
		DB_rot.reset();
		NX_gyro.reset();

		this.rotationalPID = new ScheduledPID.Builder(setpoint, -.8, .8, 1)
			.setPGains(0.017)
			// .setIGains(0)
			// .setDGains(0)
			.build();

	} // robot is enabled

	private void initSubsystems() {
	} // initialize subsystems 

	@Override
	public void autoInit() {
	} // auton start
	
	@Override
	public void teleopInit() {
		rotMot.set(0);
		transMot.set(0);
	} // teleop start

	@Override
	public void disabledInit() {
	} // disabled robot

	@Override
	public void autoContinuous() {
	} // run at all times in state auto 

	@Override
	public void teleopContinuous() {
		// ---- input -----
		transX = driver.getLeftXAxis();
		transY = -driver.getLeftYAxis();
		transMag = Math.hypot(transX, transY); // square value 
		transThetaRaw = Math.toDegrees(Math.atan2(transY, transX));
		transTheta = toBearing(transThetaRaw);
		
		rotMag = driver.getRightXAxis();

		// ----- feedback -------
		runEncoders();

		// ----- calculations -----
		resultMag = VectorUtils.vectorAddition2DMagnitude(transX, transY, Math.abs(rotMag), 0);
		transTheta -= yaw;
		if ((Math.abs(transTheta-encoderRotAngle) > 90) && (Math.abs(encoderRotAngle+transTheta) > 90)){
			if(encoderRotAngle < 0){
				transTheta+= 180;
			}
			else {
				transTheta -= 180;
			}
			resultMag *= -1;
		}

		// ------ output --------
		runRotationalPID();
		rotMot.set(rotSpeed);
		transMot.set(-resultMag);

	} // run at all times in state teleop

  public void runEncoders(){
    DB_rot.calc();
    encoderRotAngle = DB_rot.get()/8.5; // EXPERIMENTALLY DERIVED!!! NEED TO GET A MORE ACCURATE ONE
	encoderTransSpeed = DB_trans.getVelocity()*0.225; // NEED TO CHANGE (POTENTIALLY) COULD BE WRONG
	pitch = NX_gyro.getPitch();
	roll = NX_gyro.getRoll();
	yaw = NX_gyro.getYaw();
  } // calculate encoder turn 

	@Override
	public void disabledContinuous() {
	} // run at all times when disabled

	@Override
	public void alwaysContinuous() {
		smartDashboard();
	} // run at all times when robot has power

	public void smartDashboard(){
		SmartDashboard.putNumber("Rotation Angle", encoderRotAngle);
		SmartDashboard.putNumber("Trans Speed", encoderTransSpeed);
		SmartDashboard.putNumber("Translation Theta", transTheta); // 1_comment
		SmartDashboard.putNumber("TransMag", transMag);
		SmartDashboard.putNumber("RotMag", rotMag);
		SmartDashboard.putNumber("ResultMag", resultMag);
		SmartDashboard.putNumber("RotSpeed", rotSpeed);
		SmartDashboard.putNumber("Pitch", pitch);
		SmartDashboard.putNumber("Roll", roll);
		SmartDashboard.putNumber("Yaw", yaw);
	} // put stuff to smart dashboard

	// ------- METHODS TO USE IN STUFF ----------
	public double toBearing(double angle) { // to use with input from controller // 1_comment
        double bearing = 0;
		angle = angle % 360;
		if (transX == 0 && transY == 0){
			return 666;
		} 
		if ((angle <= 180 && angle >= 0 ) || (angle < 0 && angle >= -90)) {
			bearing = 90 - angle;
		} else {
			bearing = 90 - angle -360;
		}
        return bearing;
	} // change value to a bearing

	public void runRotationalPID(){
		if (transTheta <= 180 && transTheta >= -180){
			setpoint = transTheta;
		}
		currentPos = encoderRotAngle;
		if (setpoint != prevSetpoint){
			SmartDashboard.putNumber("difference", Math.abs(setpoint-currentPos));
			rotationalPID.changeSetpoint(setpoint);
			prevSetpoint = setpoint;
		}
		rotSpeed = rotationalPID.calculate(currentPos);
	}
	
	
} // Robot