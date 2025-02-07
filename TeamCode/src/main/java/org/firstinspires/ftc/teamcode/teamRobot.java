package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.jetbrains.annotations.Nullable;

public class teamRobot {
    public ElapsedTime timer = new ElapsedTime();
    public enum ClawState {
        OPEN_CLAW,
        CLOSE_CLAW
    }
    public DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, clawMotor, liftMotor, hangMotor;
    public GoBildaPinpointDriver odometryUnit;
    public Servo clawRightServo, clawLeftServo, clipServo, hangServo;
    public final String FRONT_RIGHT = "front right";
    public final String FRONT_LEFT = "front left";
    public final String BACK_RIGHT = "back right";
    public final String BACK_LEFT = "back left";
    public final String ODOMETRY_UNIT = "odo";
    public final String CLAW_MOTOR = "motor to replace murdered super servo";
    public final String LIFT = "Rapunzel Dogg";
    public final String CLAW_RIGHT = "Bryce1";
    public final String CLAW_LEFT = "Eden3";
    public final String CLIP = "Emily4";
    public final String HANG_SERVO = "Hooked on a Feeling";
    public final String HANG_MOTOR = "crank that soulja boy";
    public final PIDFController xController = new PIDFController(new PIDCoefficients(PID.kP, PID.kI, PID.kD));
    public final PIDFController yController = new PIDFController(new PIDCoefficients(PID.kP, PID.kI, PID.kD));
    public final PIDFController headingController = new PIDFController(new PIDCoefficients(PID.kPh, PID.kIh, PID.kDh));
    /**
     * Opens or closes the claw
     * @param clawState value if claw should be opened or closed
     */
    public void setClawState(ClawState clawState) {
        if (clawState == ClawState.OPEN_CLAW) {
            clawRightServo.setPosition(0.5);
            clawLeftServo.setPosition(0.4);
        } else {
            clawRightServo.setPosition(0.15);
            clawLeftServo.setPosition(0.75);
        }
    }

    /**
     * opens or closes the clip servo
     * @param clawState value to determine if the claw should open or close
     */
    public void setClipServoState(ClawState clawState) {
        if (clawState == ClawState.OPEN_CLAW) {
            clipServo.setPosition(0);
        } else {
            clipServo.setPosition(0.45);
        }
    }
    public void liftClaw(double motorRevolutions, double power) {
        double targetPosition = motorRevolutions * 1120;
        liftMotor.setTargetPosition((int) -targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
        while (liftMotor.isBusy()) {
        }
        liftMotor.setPower(0);
    }
    public void runTrajectory(double x, double y, double heading, double endTime, double clawLift, double scalar) {
        timer.reset();
        Pose2D pos = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.DEGREES, heading);
        xController.setTargetPosition(pos.getX(DistanceUnit.MM));
        //xController.setTargetVelocity(0);
        yController.setTargetPosition(pos.getY(DistanceUnit.MM));
        //yController.setTargetVelocity(0);
        headingController.setInputBounds(-180, 180);
        headingController.setTargetPosition(pos.getHeading(AngleUnit.RADIANS));

        liftMotor.setTargetPosition((int) (-1120 * clawLift));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);

        while (timer.milliseconds() < endTime) {
            odometryUnit.update();
            double xCorrection = scalar * xController.update(odometryUnit.getPosition().getX(DistanceUnit.MM));
            double yCorrection = scalar * yController.update(odometryUnit.getPosition().getY(DistanceUnit.MM));
            double headingCorrection = headingController.update(odometryUnit.getPosition().getHeading(AngleUnit.RADIANS));
            frontRightMotor.setPower(xCorrection - yCorrection + headingCorrection);
            frontLeftMotor.setPower(-xCorrection - yCorrection + headingCorrection);
            backRightMotor.setPower(xCorrection + yCorrection + headingCorrection);
            backLeftMotor.setPower(-xCorrection + yCorrection + headingCorrection);
        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        liftMotor.setPower(0);
        odometryUnit.update();
    }

    public void raiseClaw(double clawLower) {
        clawMotor.setTargetPosition((int) (clawLower * 1120));
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setPower(0.9);
        while (!!clawMotor.isBusy()) {
        }
        clawMotor.setPower(0);
    }

    public void turnRobot(double heading, double endTime, double clawLift) {
        headingController.setTargetPosition(Math.toRadians(heading));
        headingController.setTargetVelocity(0);
        liftMotor.setTargetPosition((int) (-1120 * clawLift));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);

        timer.reset();
        while (timer.milliseconds() < endTime) {
            odometryUnit.update();
            double headingCorrection = headingController.update(odometryUnit.getPosition().getHeading(AngleUnit.RADIANS));
            frontRightMotor.setPower(headingCorrection);
            frontLeftMotor.setPower(headingCorrection);
            backRightMotor.setPower(headingCorrection);
            backLeftMotor.setPower(headingCorrection);
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        liftMotor.setPower(0);
        odometryUnit.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
        odometryUnit.update();
    }
}
