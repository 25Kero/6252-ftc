package org.firstinspires.ftc.teamcode.Lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class rotationPath extends Path {


    public rotationPath(double runTime, double targetX, double targetY, double heading, double clawRaise) {
        super(runTime, targetX, targetY, heading, clawRaise);
    }
    @Override
    public void tick() {
        odometryUnit.update();
        updateOdoRotation();
        powerMotors();
        if(Math.abs(odometryUnit.getPosition().getHeading(AngleUnit.RADIANS) - this.targetPos.getHeading(AngleUnit.RADIANS)) < 0.025) {this.isComplete = true;} // complete task when robot is within certain range
    }

    @Override
    public void endCall() {
        odometryUnit.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
    private void updateOdoRotation() {
        double headingCorrection = headingController.update(odometryUnit.getPosition().getHeading(AngleUnit.RADIANS));
        this.MotorPowers[0] = headingCorrection;
        this.MotorPowers[1] = headingCorrection;
        this.MotorPowers[2] = headingCorrection;
        this.MotorPowers[3] = headingCorrection;
    }
}
