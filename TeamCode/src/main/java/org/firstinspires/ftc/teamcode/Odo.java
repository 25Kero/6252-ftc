package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "New Left Auto", preselectTeleOp = "Driver Teleop 24-25")
public class Odo extends LinearOpMode {
    private teamRobot Robot = new teamRobot();

    @Override
    public void runOpMode() {
        // initialize motors and servos with configuration
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
        Robot.clawMotor.setPower(0.5);
        Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);

        Robot.clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //runs after play is pressed
        waitForStart();
        while (opModeIsActive()) {
            Robot.runTrajectory(pos.clipStart[0], pos.clipStart[1], 0, timings.startClip, 0.97, 0.2);
            Robot.liftClaw(0.5, 0.825);
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
            Robot.runTrajectory(pos.clipEnd[0], pos.clipEnd[1], 0, timings.endClip, 0.25, 1);

            //grab first block
            Robot.runTrajectory(pos.firstBlockGrab[0], pos.firstBlockGrab[1], 0, timings.firstBlockGrab, 0.25, 1);
            Robot.raiseClaw(-pos.clawLower);
            Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);
            sleep(500);
            Robot.raiseClaw(0);
            Robot.runTrajectory(pos.strafeToBucket1[0], pos.strafeToBucket1[1], 0, timings.strafeToBucket1, 0.20, 1);
            Robot.turnRobot(135, timings.turnTime, 3.4);

            // second block in bucket
            Robot.runTrajectory(pos.firstBucketCorrection[0], pos.firstBucketCorrection[1], 0, timings.firstBucketCorrection, 3.4, 1);
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
            sleep(500);
            Robot.runTrajectory(0,0, 0, timings.firstBucketCorrection,0, 1);
            Robot.turnRobot(-135, timings.turnTime, 0);

            //grab second block
            Robot.runTrajectory(pos.secondBlockGrab[0], pos.secondBlockGrab[1], 0, timings.secondBlockGrab, 0.20, 1);
            Robot.raiseClaw(-pos.clawLower);
            Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);
            sleep(500);
            Robot.raiseClaw(0.2);
            Robot.runTrajectory(pos.strafeToBucket2[0], pos.strafeToBucket2[1], 0, timings.strafeToBucket2, 0.20, 1);
            Robot.turnRobot(135, timings.turnTime, 3.4);

            //second block in bucket
            Robot.runTrajectory(pos.secondBucketCorrection[0], pos.secondBucketCorrection[1], 0, timings.secondBucketCorrection, 3.4, 1);
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
            sleep(500);
            Robot.runTrajectory(0,0, 0, timings.secondBucketCorrection,0, 1);
            Robot.turnRobot(-135, timings.turnTime, 0.20);
            //Robot.runTrajectory(1150, 650, 0,  timings.finalHang, 2.5, 1);
            requestOpModeStop();
        }
    }
}
