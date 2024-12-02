package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous", preselectTeleOp = "Driver Teleop")
public class Auto extends LinearOpMode {
    private teamRobot Robot = new teamRobot();

    @Override
    public void runOpMode() {
        // initialize motors and servos with configuration
        Robot.frontRightMotor = hardwareMap.get(DcMotor.class, Robot.FRONT_RIGHT);
        Robot.frontLeftMotor = hardwareMap.get(DcMotor.class, Robot.FRONT_LEFT);
        Robot.backRightMotor = hardwareMap.get(DcMotor.class, Robot.BACK_RIGHT);
        Robot.backLeftMotor = hardwareMap.get(DcMotor.class, Robot.BACK_LEFT);
        Robot.clawMotor = hardwareMap.get(DcMotor.class, Robot.CLAW_MOTOR);
        Robot.liftMotor = hardwareMap.get(DcMotor.class, Robot.LIFT);

        Robot.clawRightServo = hardwareMap.get(Servo.class, Robot.CLAW_RIGHT);
        Robot.clawLeftServo = hardwareMap.get(Servo.class, Robot.CLAW_LEFT);
        Robot.clipServo = hardwareMap.get(Servo.class, Robot.CLIP);

        //start claw configuration, raised and closed
        Robot.clawMotor.setPower(0.2);
        Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);
        Robot.resetEncoders();

        //runs after play is pressed
        waitForStart();

        //runs until stop is pressed
        while (opModeIsActive()) {
        }
    }
    // grabs a block b
    private void grab_block() {
        Robot.raiseClaw(0.075);
        Robot.raiseClaw(0.024);
        sleep(500);
        Robot.setClawState(teamRobot.ClawState.CLOSE_CLAW);
        sleep(300);
        Robot.raiseClaw(-0.09);
    }
}
