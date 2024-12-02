package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class teamRobot {
     public enum ClawState {
        OPEN_CLAW,
        CLOSE_CLAW
    }
    public DcMotor frontRightMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor clawMotor;
    public DcMotor liftMotor;
    public Servo clawRightServo;
    public Servo clawLeftServo;
    public Servo clipServo;
    private final double MOTOR_POWER = 0.7;
    public final String FRONT_RIGHT = "front right";
    public final String FRONT_LEFT = "front left";
    public final String BACK_RIGHT = "back right";
    public final String BACK_LEFT = "back left";
    public final String CLAW_MOTOR = "motor to replace murdered super servo";
    public final String LIFT = "Rapunzel Dogg";
    public final String CLAW_RIGHT = "Bryce1";
    public final String CLAW_LEFT = "Eden3";
    public final String CLIP = "Emily4";

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

    /**
     * raises the claw up or down based on how many times the motor should turn
     * @param motorRevolutions number of times the target motor should rotate. Uses Encoders
     */
    public void raiseClaw(double motorRevolutions) {
        double targetPosition = motorRevolutions * 1120;

        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setTargetPosition((int) targetPosition);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setPower(MOTOR_POWER);
        while (clawMotor.isBusy()) {
        }
        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawMotor.setPower(0.2);
    }

    /**
     * lifts the robot claw based on how much the motor should turn
     * @param motorRevolutions number of times the target motor should rotate. Uses Encoders
     */
    public void liftClaw(double motorRevolutions) {
        double targetPosition = motorRevolutions * 1120;

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition((int) -targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(MOTOR_POWER);
        while (liftMotor.isBusy()) {
        }
        liftMotor.setPower(0.01);
    }

    /**
     * moves the robot in a staright line based on motorRevolutions
     * @param motorRevolutions number of times the target motor should rotate. Uses Encoders
     */
    public void straightMove(double motorRevolutions) {
        // get posistion from revolutions needed
        double targetPosition = motorRevolutions * 1120;

        // reset motor encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set target posistion for all motors based on wheel placement
        frontLeftMotor.setTargetPosition((int) -targetPosition);
        frontRightMotor.setTargetPosition((int) targetPosition);
        backLeftMotor.setTargetPosition((int) -targetPosition);
        backRightMotor.setTargetPosition((int) targetPosition);

        // set motors to run to pos
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // sets speed at which the motors move to the target posistion
        frontLeftMotor.setPower(MOTOR_POWER);
        frontRightMotor.setPower(MOTOR_POWER);
        backLeftMotor.setPower(MOTOR_POWER);
        backRightMotor.setPower(MOTOR_POWER);

        // wait until all motors have reached the posistion before stopping
        while (!(!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() && !backLeftMotor.isBusy() && !backRightMotor.isBusy())) {
        }

        //sets power to 0, indicating a stop after the pos is rached
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * Strafes the robot both to right and to the left, based on motorRevolutions
     * @param motorRevolutions number of times the target motor should rotate. Uses Encoders
     */
    public void Strafe(double motorRevolutions) {
        //get posistion from revolutions needed
        double targetPosition = motorRevolutions * 1120;

        // reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // target position are set based on configuration for a strafe
        frontLeftMotor.setTargetPosition((int) -targetPosition);
        frontRightMotor.setTargetPosition((int) -targetPosition);
        backLeftMotor.setTargetPosition((int) targetPosition);
        backRightMotor.setTargetPosition((int) targetPosition);

        // motors are set to run to position
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // power is given to set motor speed
        frontLeftMotor.setPower(MOTOR_POWER);
        frontRightMotor.setPower(MOTOR_POWER);
        backLeftMotor.setPower(MOTOR_POWER);
        backRightMotor.setPower(MOTOR_POWER);

        // wait until everything is done
        while (!(!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() && !backLeftMotor.isBusy() && !backRightMotor.isBusy())) {
        }

        //set power to 0 after motors are at the desired posistion
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * turns the robot right or left by revolutions
     * @param right true if the robot should turn to the right. false if left
     * @param motorRevolutions number of times the target motor should rotate. Uses Encoders
     */
    public void turnRobot(boolean right, double motorRevolutions) {
        double targetPosition;

        // reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // check which direction the robot should turns and adjusts values
        if (right) {
            targetPosition = -motorRevolutions * 1120;
        } else {
            targetPosition = motorRevolutions * 1120;
        }
        //set target positions
        frontLeftMotor.setTargetPosition((int) targetPosition);
        frontRightMotor.setTargetPosition((int) targetPosition);
        backLeftMotor.setTargetPosition((int) targetPosition);
        backRightMotor.setTargetPosition((int) targetPosition);

        // prepare for travel
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set motor speed
        frontLeftMotor.setPower(MOTOR_POWER);
        frontRightMotor.setPower(MOTOR_POWER);
        backLeftMotor.setPower(MOTOR_POWER);
        backRightMotor.setPower(MOTOR_POWER);

        //wait until everything is done
        while (!(!frontLeftMotor.isBusy() && !frontRightMotor.isBusy() && !backLeftMotor.isBusy() && !backRightMotor.isBusy())) {
        }
        //reset power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    /**
     * resets all the encoders, used at auto start
     */
    public void resetEncoders() {
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setTargetPosition(0);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(0);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setTargetPosition(0);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setTargetPosition(0);

        //run to pos, should not move
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //reset work for lift motor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
