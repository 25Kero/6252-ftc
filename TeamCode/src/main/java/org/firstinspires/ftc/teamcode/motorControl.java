package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class motorControl {
    private double kP;
    private double kI;
    private double kD;
    private double kV;
    private double kA;
    private double kStatic;
    private PIDFController controller;
    private DcMotorEx assignedMotor;
    public motorControl(DcMotorEx motor, double kP, double kI, double kD, double kV, double kA) {
        assignedMotor = motor;

        PIDCoefficients coefficents = new PIDCoefficients(kP, kI, kD);
        controller = new PIDFController(coefficents, kV, kA, kStatic);
    }

    public void updateControl(double targetPos, double targetVel, double targetAccel) {
        controller.setTargetPosition(targetPos);
        controller.setTargetVelocity(targetVel);
        controller.setTargetAcceleration(targetAccel);
    }
    public void updateController() {
        double error = controller.update(assignedMotor.getCurrentPosition());
        assignedMotor.setPower(error);
    }
}
