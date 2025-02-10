package org.firstinspires.ftc.teamcode.Lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class translationPath extends Path {

    public translationPath(double runTime, double targetX, double targetY, double heading, double clawRaise) {
        super(runTime, targetX, targetY, heading, clawRaise);
    }
    @Override
    public void tick() {
        odometryUnit.update();
        xController.setTargetPosition(this.targetPos.getX(DistanceUnit.MM));
        yController.setTargetPosition(this.targetPos.getY(DistanceUnit.MM));
        headingController.setTargetPosition(this.targetPos.getHeading(AngleUnit.RADIANS));
        updateOdoTranslation();
        powerMotors();

        if(Math.abs(odometryUnit.getPosition().getX(DistanceUnit.MM ) - this.targetPos.getY(DistanceUnit.MM)) < 5 && Math.abs(odometryUnit.getPosition().getY(DistanceUnit.MM ) - this.targetPos.getX(DistanceUnit.MM)) < 5) {this.isComplete = true;} // complete task when robot is within certain range
    }
    private void updateOdoTranslation() {
        double xCorrection = xController.update(odometryUnit.getPosition().getX(DistanceUnit.MM));
        double yCorrection = yController.update(odometryUnit.getPosition().getY(DistanceUnit.MM));
        double headingCorrection = headingController.update(odometryUnit.getPosition().getHeading(AngleUnit.RADIANS));

        this.MotorPowers[0] = xCorrection - yCorrection + headingCorrection;
        this.MotorPowers[1] = -xCorrection - yCorrection + headingCorrection;
        this.MotorPowers[2] = xCorrection + yCorrection + headingCorrection;
        this.MotorPowers[3] = -xCorrection + yCorrection + headingCorrection;
    }
}
