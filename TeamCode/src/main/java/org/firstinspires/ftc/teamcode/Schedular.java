package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Schedular extends teamRobot{
    private List<Pose2D> tasks = new ArrayList<Pose2D>();
    private List<Double> timings = new ArrayList<Double>();
    private ElapsedTime taskTimer = new ElapsedTime();

    private double[] MotorPowers = {0, 0, 0, 0};
    public void createTask(Pose2D pose2D, double timing) {
        tasks.add(pose2D);
        timings.add(timing);
    }

    public void removeTask(int index) {
        tasks.remove(index);
        timings.remove(index);
    }

    public void runRoute() {
        int index = 0;
        while (index < tasks.size()) {
            taskTimer.reset();
            setTarget(index);

            while (taskTimer.time(TimeUnit.MILLISECONDS) < timings.get(index)) {
                updateOdo();
                powerMotors();
            }
            index++;
        }

    }

    private void setTarget(int index) {
        Pose2D targetPos = tasks.get(index);
        xController.setTargetPosition(targetPos.getX(DistanceUnit.MM));
        yController.setTargetPosition(targetPos.getY(DistanceUnit.MM));
        headingController.setTargetPosition(targetPos.getHeading(AngleUnit.RADIANS));
    }
    private void updateOdo() {
        odometryUnit.update();
        double xCorrection = xController.update(odometryUnit.getPosition().getX(DistanceUnit.MM));
        double yCorrection = yController.update(odometryUnit.getPosition().getY(DistanceUnit.MM));
        double headingCorrection = headingController.update(odometryUnit.getPosition().getHeading(AngleUnit.RADIANS));

        MotorPowers[0] = xCorrection - yCorrection + headingCorrection;
        MotorPowers[1] = -xCorrection - yCorrection + headingCorrection;
        MotorPowers[2] = xCorrection + yCorrection + headingCorrection;
        MotorPowers[3] = -xCorrection + yCorrection + headingCorrection;
//        frontRightMotor.setPower(xCorrection - yCorrection + headingCorrection);
//        frontLeftMotor.setPower(-xCorrection - yCorrection + headingCorrection);
//        backRightMotor.setPower(xCorrection + yCorrection + headingCorrection);
//        backLeftMotor.setPower(-xCorrection + yCorrection + headingCorrection);
    }

    private void powerMotors() {
        frontRightMotor.setPower(powerLimit(MotorPowers[0]));
        frontLeftMotor.setPower(powerLimit(MotorPowers[1]));
        backRightMotor.setPower(powerLimit(MotorPowers[2]));
        backLeftMotor.setPower(powerLimit(MotorPowers[3]));
    }

    private double powerLimit(double power) {
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
