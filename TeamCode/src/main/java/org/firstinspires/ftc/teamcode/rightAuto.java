package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "New Right Auto", preselectTeleOp = "Driver Teleop 24-25")
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
        Robot.clawMotor.setPower(0.5);
        Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);

        Robot.clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        Robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            Robot.runTrajectory(pos.clipStart[0], 0, 0, timings.startClip, 0.97, 0.2);
            Robot.liftClaw(0.5, 0.825);
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
            Robot.runTrajectory(400, 0, 0, timings.endClip, 0, 1);
            Robot.runTrajectory(rightValues.firstStrafe[0], rightValues.firstStrafe[1], 0, 1800, 0, 0.9);
            Robot.runTrajectory(rightValues.firstBlockAlign[0], rightValues.firstBlockAlign[1], 0, 1800, 0, 0.9);
            Robot.runTrajectory(rightValues.goBack[0], rightValues.goBack[1], 0, 1800, 0, 1);
            Robot.runTrajectory(rightValues.secondBlockAlign[0], rightValues.secondBlockAlign[1], 0, 1800, 0, 0.9);
            sleep(1800);
            Robot.runTrajectory(rightValues.goBack[0], rightValues.goBack[1], 0, 1800, 0, 0.9);
            Robot.turnRobot(180, 1800, 0);

            //grab block from human player
            Robot.setClipServoState(teamRobot.ClawState.OPEN_CLAW);
            Robot.runTrajectory(265, 0, 0, 1800, 0, 0.9);
            Robot.setClipServoState(teamRobot.ClawState.CLOSE_CLAW);
            sleep(1500);
            Robot.runTrajectory(rightValues.returnToStation[0], rightValues.returnToStation[1], 0, 2000, rightValues.clipRaise, 1);
            Robot.turnRobot(-180, 2000, rightValues.clipRaise);

            Robot.runTrajectory(250, 0, 0, 1800, rightValues.clipRaise, 1);
            Robot.liftClaw(rightValues.clipLower, 0.825);
            Robot.setClipServoState(teamRobot.ClawState.OPEN_CLAW);
            sleep(800);
            Robot.runTrajectory(-550, 1200, 0, 2000, 0, 1);
            sleep(8000);
            Robot.runTrajectory(rightValues.firstStrafe[0], rightValues.firstStrafe[1], 0, 1800, 0, 1);
            Robot.runTrajectory(rightValues.firstBlockAlign[0], rightValues.firstBlockAlign[1], 0, 1800, 0, 1);
            Robot.runTrajectory(rightValues.goBack[0], rightValues.goBack[1], 0, 2250, 0, 1);
            Robot.runTrajectory(rightValues.secondBlockAlign[0], rightValues.secondBlockAlign[1], 0, 1800, 0, 1);
            Robot.runTrajectory(rightValues.goBack[0], rightValues.goBack[1], 0, 2250, 0, 1);
            Robot.turnRobot(180, 1800, 0);

            Robot.setClipServoState(teamRobot.ClawState.OPEN_CLAW);
            Robot.runTrajectory(300, 0, 0, 2000, 0.25, 1);
            Robot.setClipServoState(teamRobot.ClawState.CLOSE_CLAW);
            Robot.runTrajectory(-200, 1100, 0, 2000, 0.25, 1);
            requestOpModeStop();
        }
    }
}
