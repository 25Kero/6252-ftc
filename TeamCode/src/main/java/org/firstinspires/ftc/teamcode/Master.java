package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Driver Teleop")
public class Master extends LinearOpMode {
    private teamRobot Robot = new teamRobot();
    private boolean precisionDrive = false;

    @Override
    public void runOpMode() {
        // initialize motors with configuration
        Robot.frontRightMotor = hardwareMap.get(DcMotor.class, Robot.FRONT_RIGHT);
        Robot.frontLeftMotor = hardwareMap.get(DcMotor.class, Robot.FRONT_LEFT);
        Robot.backRightMotor = hardwareMap.get(DcMotor.class, Robot.BACK_RIGHT);
        Robot.backLeftMotor = hardwareMap.get(DcMotor.class, Robot.BACK_LEFT);
        Robot.clawMotor = hardwareMap.get(DcMotor.class, Robot.CLAW_MOTOR);
        Robot.liftMotor = hardwareMap.get(DcMotor.class, Robot.LIFT);

        Robot.clawRightServo = hardwareMap.get(Servo.class, Robot.CLAW_RIGHT);
        Robot.clawLeftServo = hardwareMap.get(Servo.class, Robot.CLAW_LEFT);
        Robot.clipServo = hardwareMap.get(Servo.class, Robot.CLIP);
        // everything after this runs after driver presses play
        waitForStart();

        // runs until driver hits stop
        while (opModeIsActive()) {
            driveControl();
            liftControl();
            clawControl();
        }
    }
    // controls the wheels for driving
    private void driveControl() {
         double rightX;
         double leftX;
         double rightY;
         double leftY;

        if (gamepad1.left_bumper) {
            precisionDrive = false;
        }
        if (gamepad1.right_bumper) {
            precisionDrive = true;
        }

        // if precsisonDrive, then set values to 1/4 their value for the robot to move 1/4 the speed
        if (precisionDrive) {
            rightX = 0.25 * gamepad1.right_stick_x;
            rightY = 0.25 * gamepad1.right_stick_y;
            leftX = gamepad1.left_stick_x / 4;
            leftY = gamepad1.left_stick_y / 4;
        } else {
            rightX = gamepad1.right_stick_x;
            rightY = gamepad1.right_stick_y;
            leftX = gamepad1.left_stick_x;
            leftY = gamepad1.left_stick_y;
        }

        // set wheel power to designated values
        Robot.frontRightMotor.setPower(-(rightY + rightX));
        Robot.frontLeftMotor.setPower(leftY - leftX);
        Robot.backRightMotor.setPower(-(rightY - rightX));
        Robot.backLeftMotor.setPower(leftY + leftX);
    }

    // controls the viper slide lifting system
    // dpad up raises, dpad down lowers
    private void liftControl() {
        if (gamepad2.dpad_up) {
            Robot.liftMotor.setPower(-0.9);
        } else if (gamepad2.dpad_down) {
            Robot.liftMotor.setPower(0.9);
        } else {
            Robot.liftMotor.setPower(-0.05);
        }
    }
    // controls the claw servos and the motor that raises them
    private void clawControl() {
        //open and close the claw
        // right bumper opens, left bumper closes
        if (gamepad2.right_bumper) {
            Robot.setClawState(teamRobot.ClawState.OPEN_CLAW);
        }
        if (gamepad2.left_bumper) {
            Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);
        }

        // lower and raise claw
        // values for sticks greater than zero mean user is pulling the stick down, vice versa
        if (gamepad2.right_stick_y > 0) {
            Robot.clawMotor.setPower(0);
        } else if (gamepad2.right_stick_y < 0) {
            Robot.clawMotor.setPower(0.6);
        } else {
            Robot.clawMotor.setPower(0.2);
        }

        //open and close clipping servo
        // x opens. b closes
        if (gamepad2.x) {
            Robot.setClipServoState(teamRobot.ClawState.OPEN_CLAW);
        }
        if (gamepad2.b) {
            Robot.setClipServoState(teamRobot.ClawState.CLOSE_CLAW);
        }
    }
}
