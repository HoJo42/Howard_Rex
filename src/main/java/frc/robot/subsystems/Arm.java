// this file holds the subystem for lifting and lowering the arm
package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ArmPIDConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPIDConstants.ArmPositions;

public class Arm extends SubsystemBase {
    CANSparkMax m_arm;
    CANSparkMax m_arm2;
    RelativeEncoder m_encoder;

    /**
     * I'm unfamiliar with the PID Subsystem so I'm just switching to the ways I know how to do it.
     */
    PIDController m_controller;

    /**
     * The current target position of the Arm. Will be updated by instant commands
     * as needed.
     */
    ArmPositions m_currentPosition;

    // constructor
    public Arm() {
        m_currentPosition = ArmPositions.REST; // Intialize to a safe value
        this.m_controller = new PIDController(kP, kI, kD);
        m_arm = new CANSparkMax(kArmID, MotorType.kBrushless);
        m_arm2 = new CANSparkMax(kSecondArmID, MotorType.kBrushless);

        m_encoder = m_arm.getEncoder();
        m_encoder.setPositionConversionFactor(0.21875);
        m_arm2.follow(m_arm);
    }

    /**
     * Runs the Arm to its desired position (m_currentPosition). The default command
     * of the Arm subsystem should be set to run this method.
     */
    public void runToPosition() {
        setArmVoltage(m_controller.calculate(getMeasurement(), m_currentPosition.position));
    }

    public double getMeasurement() {
        return m_encoder.getPosition();
    }

    public void useOutput(double output, double setpoint) {
        m_arm.setVoltage(output);
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void setArmVoltage(double voltage) {
        //I would use some if statements to check that motor are only operating in safe ways.
        //EX: if the arm is at rest (encoder == 0) and trying to go further into the floor (likely negative voltage) then set the motors to stop. 
        // and make sure the regular set voltage is within an else statement so the if statement is effective.
        m_arm.setVoltage(voltage);
    }

    // defining method to stop arm motors
    public void armStop() {
        m_arm.set(0);
        m_arm2.set(0);
    }

    /**
     * Sets the desired position of the Arm.
     * 
     * @param desiredPosition The position you want the PID to target.
     */
    public void setDesiredPosition(ArmPositions desiredPosition) {
        this.m_currentPosition = desiredPosition;
    }
}
