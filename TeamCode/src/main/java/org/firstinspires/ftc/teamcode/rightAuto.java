package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class rightAuto extends LinearOpMode {

    private teamRobot Robot = new teamRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.frontRightMotor = hardwareMap.get(DcMotorEx.class, Robot.FRONT_RIGHT);
        Robot.frontLeftMotor = hardwareMap.get(DcMotorEx.class, Robot.FRONT_LEFT);
        Robot.backRightMotor = hardwareMap.get(DcMotorEx.class, Robot.BACK_RIGHT);
        Robot.backLeftMotor = hardwareMap.get(DcMotorEx.class, Robot.BACK_LEFT);
        Robot.clawMotor = hardwareMap.get(DcMotorEx.class, Robot.CLAW_MOTOR);
        Robot.liftMotor = hardwareMap.get(DcMotorEx.class, Robot.LIFT);
        Robot.odometryUnit = hardwareMap.get(GoBildaPinpointDriver.class, Robot.ODOMETRY_UNIT);

        Robot.clawRightServo = hardwareMap.get(Servo.class, Robot.CLAW_RIGHT);
        Robot.clawLeftServo = hardwareMap.get(Servo.class, Robot.CLAW_LEFT);
        Robot.clipServo = hardwareMap.get(Servo.class, Robot.CLIP);

        Robot.odometryUnit.setOffsets(pos.xOff, pos.yOff);
        Robot.odometryUnit.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Robot.odometryUnit.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        Robot.odometryUnit.resetPosAndIMU();
        Robot.odometryUnit.doInitialize();

        //start claw configuration, raised and closed
        Robot.clawMotor.setPower(0.3);
        Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);

        Robot.clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            Robot.runTrajectory(pos.clipStart[0], 0, 0, timings.startClip, 0.97, 0.2);
            Robot.liftClaw(0.5, 0.825);
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
            Robot.runTrajectory(pos.clipEnd[0], 0, 0, timings.endClip, 0.25, 1);

            Robot.runTrajectory(700, 700, 0, 1750, 0, 1);
            Robot.runTrajectory(850, 700, 0, 1750, 0, 1);
        }
    }
}
