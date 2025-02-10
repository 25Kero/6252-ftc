package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dashConfig.pos;

@TeleOp(name = "Driver Teleop 24-25")
public class Master extends LinearOpMode {
    double rightX;
    double leftX;
    double rightY;
    double leftY;

    private teamRobot Robot = new teamRobot();
    private boolean precisionDrive = false;

    @Override
    public void runOpMode() {
        // initialize motors with configuration
        Robot.frontRightMotor = hardwareMap.get(DcMotorEx.class, Robot.FRONT_RIGHT);
        Robot.frontLeftMotor = hardwareMap.get(DcMotorEx.class, Robot.FRONT_LEFT);
        Robot.backRightMotor = hardwareMap.get(DcMotorEx.class, Robot.BACK_RIGHT);
        Robot.backLeftMotor = hardwareMap.get(DcMotorEx.class, Robot.BACK_LEFT);
        Robot.clawMotor = hardwareMap.get(DcMotorEx.class, Robot.CLAW_MOTOR);
        Robot.liftMotor = hardwareMap.get(DcMotorEx.class, Robot.LIFT);
        Robot.hangMotor = hardwareMap.get(DcMotorEx.class, Robot.HANG_MOTOR);

        Robot.clawRightServo = hardwareMap.get(Servo.class, Robot.CLAW_RIGHT);
        Robot.clawLeftServo = hardwareMap.get(Servo.class, Robot.CLAW_LEFT);
        Robot.clipServo = hardwareMap.get(Servo.class, Robot.CLIP);
        Robot.hangServo = hardwareMap.get(Servo.class, Robot.HANG_SERVO);
        Robot.odometryUnit = hardwareMap.get(GoBildaPinpointDriver.class, Robot.ODOMETRY_UNIT);
        Robot.odometryUnit.setOffsets(0, 0);
        Robot.odometryUnit.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Robot.odometryUnit.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        Robot.odometryUnit.resetPosAndIMU();

        Robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // everything after this runs after driver presses play
        waitForStart();

        // runs until driver hits stop
        while (opModeIsActive()) {
            driveControl();
            liftControl();
            clawControl();
            hangControl();
        }
    }

    /**
     * controls wheels for driving
     */
    private void driveControl() {

        if (gamepad1.left_bumper) {
            precisionDrive = false;
        }
        if (gamepad1.right_bumper) {
            precisionDrive = true;
        }
        rightX = gamepad1.right_stick_x;
        rightY = gamepad1.right_stick_y;
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y;
        // if precsisonDrive, then set values to 1/4 their value for the robot to move 1/4 the speed
        if (precisionDrive) {
            rightX = rightX / 3;
            rightY = rightY / 3;
            leftX = leftX / 3;
            leftY = leftY / 3;
        }
        // set wheel power to designated values
        Robot.frontRightMotor.setPower(-(rightY + rightX));
        Robot.frontLeftMotor.setPower(leftY - leftX);
        Robot.backRightMotor.setPower(-(rightY - rightX));
        Robot.backLeftMotor.setPower(leftY + leftX);
    }

    /**
     * controls the viper slide lifting system
     * dpad up raises, dpad down lowers
     */
    private void liftControl() {
        if (gamepad2.dpad_up) {
            Robot.liftMotor.setPower(-0.9);
        } else if (gamepad2.dpad_down) {
            Robot.liftMotor.setPower(0.9);
        } else {
            Robot.liftMotor.setPower(-0.05);
        }
    }

    private void hangControl() {
        if(gamepad1.y) {
            Robot.hangServo.setPosition(pos.servohang2);
        }
        if(gamepad1.x) {
            Robot.hangServo.setPosition(pos.servohang1);
        }

        if(gamepad1.b) {
            Robot.hangServo.setPosition(0.1);
        }

        if (gamepad1.dpad_up) {
            Robot.hangMotor.setPower(-0.9);
        } else if (gamepad1.dpad_down) {
            Robot.hangMotor.setPower(0.9);
        } else {
            Robot.hangMotor.setPower(0);
        }
    }
    /**
     * controls the claw servos and the motor that raises them
     * right bumper opens, left bumper closes
     * right > 0, lower claw. right < 0, raise claw
     * x opens clipping servo, b closes clipping servo
     */
    private void clawControl() {
        if (gamepad2.right_bumper) {
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
        }
        if (gamepad2.left_bumper) {
            Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);
        }
        if (gamepad2.right_stick_y > 0) {
            Robot.clawMotor.setPower(-0.5);
        } else if (gamepad2.right_stick_y < 0) {
            Robot.clawMotor.setPower(0.6);
        } else {
            Robot.clawMotor.setPower(0.1);
        }
        if (gamepad2.x) {
            Robot.setClipServoState(teamRobot.ClawState.OPEN_CLAW);
        }
        if (gamepad2.b) {
            Robot.setClipServoState(teamRobot.ClawState.CLOSE_CLAW);
        }
    }
}
