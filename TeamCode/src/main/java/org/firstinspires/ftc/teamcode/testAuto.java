package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Lib.trajectory;
import org.firstinspires.ftc.teamcode.Lib.rotationPath;
import org.firstinspires.ftc.teamcode.Lib.translationPath;
import org.firstinspires.ftc.teamcode.dashConfig.pos;

@Autonomous(name = "testAuto", preselectTeleOp = "Driver Teleop 24-25")

public class testAuto extends LinearOpMode {
    private teamRobot Robot = new teamRobot();
    private trajectory trajectory = new trajectory();

    @Override
    public void runOpMode(){
        Robot.frontRightMotor = hardwareMap.get(DcMotorEx .class, Robot.FRONT_RIGHT);
        Robot.frontLeftMotor = hardwareMap.get(DcMotorEx.class, Robot.FRONT_LEFT);
        Robot.backRightMotor = hardwareMap.get(DcMotorEx.class, Robot.BACK_RIGHT);
        Robot.backLeftMotor = hardwareMap.get(DcMotorEx.class, Robot.BACK_LEFT);
        Robot.clawMotor = hardwareMap.get(DcMotorEx.class, Robot.CLAW_MOTOR);
        Robot.liftMotor = hardwareMap.get(DcMotorEx.class, Robot.LIFT);
        Robot.odometryUnit = hardwareMap.get(GoBildaPinpointDriver.class, Robot.ODOMETRY_UNIT);

        Robot.clawRightServo = hardwareMap.get(Servo .class, Robot.CLAW_RIGHT);
        Robot.clawLeftServo = hardwareMap.get(Servo.class, Robot.CLAW_LEFT);
        Robot.clipServo = hardwareMap.get(Servo.class, Robot.CLIP);

        Robot.odometryUnit.setOffsets(0, 0);
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

        // create odo trajectory
        trajectory.addPath(new translationPath(2000, 500, 0, 0, 0));
        waitForStart();
        while (opModeIsActive()) {
            trajectory.runRoute();
            sleep(10000);
            requestOpModeStop();
        }
    }
}
