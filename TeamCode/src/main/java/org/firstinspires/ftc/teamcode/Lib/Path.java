package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teamRobot;

public class Path extends teamRobot {
    double runTime;
   double[] MotorPowers;
    Pose2D targetPos;

    double clawRaise;

    boolean isComplete;

    public Path(double runTime, double targetX, double targetY, double heading, double clawRaise) {
        this.runTime = runTime;
        this.targetPos = new Pose2D(DistanceUnit.MM, targetX, targetY, AngleUnit.RADIANS, heading);
        this.MotorPowers = new double[]{0, 0, 0, 0};
        this.clawRaise = clawRaise;
        this.isComplete = false;
    }
    public void tick() {
    }
    public void endCall(){
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
    public void startCall(){
        xController.setTargetPosition(this.targetPos.getX(DistanceUnit.MM));
        yController.setTargetPosition(this.targetPos.getY(DistanceUnit.MM));
        headingController.setTargetPosition(this.targetPos.getHeading(AngleUnit.RADIANS));
        powerClaw();
    }
    public void powerMotors() {
        frontRightMotor.setPower(powerLimit(this.MotorPowers[0]));
        frontLeftMotor.setPower(powerLimit(this.MotorPowers[1]));
        backRightMotor.setPower(powerLimit(this.MotorPowers[2]));
        backLeftMotor.setPower(powerLimit(this.MotorPowers[3]));
    }

    public void powerClaw() {
        liftMotor.setTargetPosition((int) (-1120 * this.clawRaise));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
    }
    public double powerLimit(double power) {
        if(power > 1.0) {
            power = 1.0;
            return power;
        } else if (power < -1.0) {
            power = -1.0;
            return power;
        } else {
            return power;
        }
    }
}
