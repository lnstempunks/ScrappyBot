#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) 2017-2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
# ----------------------------------------------------------------------------

import rev
import wpilib
from wpilib.drive import DifferentialDrive


class Robot(wpilib.TimedRobot):
    def robotInit(self):
        # Create motors
        self.lf_motor = rev.CANSparkMax(4, rev.MotorType.kBrushless)
        self.lb_motor = rev.CANSparkMax(1, rev.MotorType.kBrushless)
        self.rf_motor = rev.CANSparkMax(2, rev.MotorType.kBrushless)
        self.rb_motor = rev.CANSparkMax(3, rev.MotorType.kBrushless)

        # You must call getPIDController() on an existing CANSparkMax or
        # SparkMax object to fully use PID functionality
        # self.l_pidController = self.lf_motor.getPIDController()
        # self.r_pidController = self.rf_motor.getPIDController()

        # Instantiate built-in encoder to display position
        # self.l_encoder = self.lf_motor.getEncoder()
        # self.r_encoder = self.rf_motor.getEncoder()

        self.joystick = wpilib.Joystick(0)

        # PID Coefficents and Controller Output Range
        self.coeff = {"p": 0.22, "i": 1.188, "d": 0, "iz": 0, "ff": 0}
        self.kMinOutput = -0.5
        self.kMaxOutput = 0.5

        # Motor max RPM
        self.maxRPM = 5700

        # The restoreFactoryDefaults() method can be used to reset the
        # configuration parameters in the SPARK MAX to their factory default
        # state. If no argument is passed, these parameters will not persist
        # between power cycles

        # Set PID Coefficents
        # self.l_pidController.setP(self.coeff["p"])
        # self.l_pidController.setI(self.coeff["i"])
        # self.l_pidController.setD(self.coeff["d"])
        # self.l_pidController.setIZone(self.coeff["iz"])
        # self.l_pidController.setFF(self.coeff["ff"])
        # self.l_pidController.setOutputRange(self.kMinOutput, self.kMaxOutput)

        # self.r_pidController.setP(self.coeff["p"])
        # self.r_pidController.setI(self.coeff["i"])
        # self.r_pidController.setD(self.coeff["d"])
        # self.r_pidController.setIZone(self.coeff["iz"])
        # self.r_pidController.setFF(self.coeff["ff"])
        # self.r_pidController.setOutputRange(self.kMinOutput, self.kMaxOutput)
        # Push PID Coefficients to SmartDashboard
        wpilib.SmartDashboard.putNumber("P Gain", self.coeff["p"])
        wpilib.SmartDashboard.putNumber("I Gain", self.coeff["i"])
        wpilib.SmartDashboard.putNumber("D Gain", self.coeff["d"])
        wpilib.SmartDashboard.putNumber("I Zone", self.coeff["iz"])
        wpilib.SmartDashboard.putNumber("Feed Forward", self.coeff["ff"])
        wpilib.SmartDashboard.putNumber("Min Output", self.kMinOutput)
        wpilib.SmartDashboard.putNumber("Max Output", self.kMaxOutput)
        # self.lb_motor.follow(self.lf_motor, invert=True)
        # self.rb_motor.follow(self.rf_motor, invert=True)
        # Max P = 0.22
        # 2 Oscilations / second

    def teleopPeriodic(self):
        p = wpilib.SmartDashboard.getNumber("P Gain", 0)
        i = wpilib.SmartDashboard.getNumber("I Gain", 0)
        d = wpilib.SmartDashboard.getNumber("D Gain", 0)
        iz = wpilib.SmartDashboard.getNumber("I Zone", 0)
        ff = wpilib.SmartDashboard.getNumber("Feed Forward", 0)
        min_out = wpilib.SmartDashboard.getNumber("Min Output", 0)
        max_out = wpilib.SmartDashboard.getNumber("Max Output", 0)

        # Update PIDController datapoints with the latest from SmartDashboard
        if i != self.coeff["i"]:
            # self.l_pidController.setI(i)
            self.kI = i
        if d != self.coeff["d"]:
            # self.l_pidController.setD(d)
            self.kD = d
        if iz != self.coeff["iz"]:
            # self.l_pidController.setIZone(iz)
            self.kIz = iz
        if ff != self.coeff["ff"]:
            # self.l_pidController.setFF(ff)
            self.kFF = ff
        if (min_out != self.kMinOutput) or (max_out != self.kMaxOutput):
            # self.l_pidController.setOutputRange(min_out, max_out)
            self.kMinOutput = min_out
            self.kMaxOutput = max_out

        self.setpoint = self.joystick.getY()
        # PIDController objects are commanded to a set point using the
        # setReference() method.
        #
        # The first parameter is the value of the set point, whose units vary
        # depending on the control type set in the second parameter.
        #
        # The second parameter is the control type can be set to one of four
        # parameters:
        # rev.ControlType.kDutyCycle
        # rev.ControlType.kPosition
        # rev.ControlType.kVelocity
        # rev.ControlType.kVoltage
        #
        # For more information on what these types are, refer to the Spark Max
        # documentation.
        # self.l_pidController.setReference(self.setpoint, rev.ControlType.kVelocity)
        # self.r_pidController.setReference(self.setpoint, rev.ControlType.kVelocity)
        self.lf_motor.set(self.setpoint)
        self.rf_motor.set(self.setpoint)
        # Push Setpoint and the motor's current position to SmartDashboard.
        wpilib.SmartDashboard.putNumber("Setpoint", self.setpoint)
        # wpilib.SmartDashboard.putNumber("Process Variable", self.l_encoder.getPosition())


if __name__ == "__main__":
    wpilib.run(Robot)
