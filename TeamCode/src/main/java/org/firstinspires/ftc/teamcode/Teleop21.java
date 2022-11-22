package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="2023 Teleop", group="Iterative OpMode")

public class Teleop21 extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor liftMotor;

    private Servo gripper;

    // TODO: add extra wheel braking when elevator is above certain position.

    private final double maxSpeed = 0.625;

    private final double maxLiftSpeed = 0.5;

    //  int[] elevatorPos = {-2900, -2130, -1255, -180};

    @Override
    public void init(){

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        gripper = hardwareMap.get(Servo.class, "gripper");

        // Maps motors to direction of rotation (Left motors are always reversed)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        gripper.setDirection(Servo.Direction.REVERSE);

        // Maps lift motor to proper direction (STILL NEEDS TESTING) Note: 2022-11-17: it doesn't work
        //  lift.setDirection(DcMotor.Direction.FORWARD);
        //    liftDown.setDirection(DcMotor.Direction.REVERSE);

        // When no power is set on a motor, brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //    liftMotor.setPower(0.2);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start(){
    }

    private void moveToPos(double pow, int pos) {
        // BEN A -- while(liftMotor.getCurrentPosition() != liftMotor.getTargetPosition()) {
        liftMotor.setPower(pow);
        liftMotor.setTargetPosition(pos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // BEN A -- }
    }

    private void servoMove(double servoPos) {
        gripper.setPosition(servoPos);
    }

    private void setElevatorPos(int floorHeight) {
        liftMotor.setTargetPosition(floorHeight);
    }

    private void driveMotorPower() {
        liftMotor.getPower();
    }


    @Override
    public void loop(){

        // Position of elevator encoder
        telemetry.addData("Lift Position:", liftMotor.getCurrentPosition());
        // Position of drive motor encoders
        telemetry.addData("Left Front Motor Encoder:", frontLeft.getCurrentPosition());

        telemetry.addData("Left Back Motor Encoder:", rearLeft.getCurrentPosition());

        telemetry.addData("Right Front Motor Encoder:", frontRight.getCurrentPosition());

        telemetry.addData("Right Back Motor Encoder:", rearRight.getCurrentPosition());

        double drive = (gamepad1.left_stick_y);//inverted
        double strafe = (-gamepad1.left_stick_x);//inverted
        double turn = (gamepad1.right_stick_x);

        drive = drive * drive * Math.signum(drive);
        strafe = strafe * strafe * Math.signum(strafe);
        turn = turn * turn * Math.signum(turn);

        // Allows for multiple functions to happen at once (eg. drive & strafe, turn & drive, etc)
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
        frontLeft.setPower(maxSpeed*(drive - strafe + turn)/denominator);
        frontRight.setPower(maxSpeed*(drive - strafe - turn)/denominator);
        rearLeft.setPower(maxSpeed*(drive + strafe + turn)/denominator);
        rearRight.setPower(maxSpeed*(drive + strafe - turn)/denominator);
        //  liftMotor.setPower(maxLiftSpeed)

        // AUTOMATIC DOWN MOVEMENT
        // Moves down from high to medium
     /*   if(gamepad1.left_bumper && liftMotor.getCurrentPosition() < -2130) {
            moveToPos(-0.4, -2130);
        }*/

        if(gamepad2.left_bumper && liftMotor.getCurrentPosition() < -2130) {
            moveToPos(-0.4, -2130);
        }

        // Moves down from medium to low
     /*   if(gamepad1.left_bumper && liftMotor.getCurrentPosition() < -1255 && liftMotor.getCurrentPosition() > -2130) {
            moveToPos(-0.4, -1255);
        }*/

        if(gamepad2.left_bumper && liftMotor.getCurrentPosition() < -1255 && liftMotor.getCurrentPosition() > -2130) {
            moveToPos(-0.4, -1255);
        }

        // Moves down from low to travel
    /*    if(gamepad1.left_bumper && liftMotor.getCurrentPosition() < -180 && liftMotor.getCurrentPosition() > -1255) {
            moveToPos(-0.4, -180);
        }*/
        if(gamepad2.left_bumper && liftMotor.getCurrentPosition() < -180 && liftMotor.getCurrentPosition() > -1255) {
            moveToPos(-0.4, -180);
        }

        // Moves down from travel to bottom
      /*  if(gamepad1.left_bumper && liftMotor.getCurrentPosition() < -1 && liftMotor.getCurrentPosition() > -180) {
            moveToPos(-0.4, 0);

        }*/

        if(gamepad2.left_bumper && liftMotor.getCurrentPosition() < -1 && liftMotor.getCurrentPosition() > -180) {
            moveToPos(-0.4, 0);
        }

        // AUTOMATIC UP MOVEMENT
        // Moves up from medium to high
       /* if(gamepad1.right_bumper && liftMotor.getCurrentPosition() < -2130) {
            moveToPos(-0.6, -2900);
        }*/

        if(gamepad2.right_bumper && liftMotor.getCurrentPosition() < -2130) {
            moveToPos(-0.6, -2900);
        }

        // Moves up from low to medium
       /* if(gamepad1.right_bumper && liftMotor.getCurrentPosition() < -1255 && liftMotor.getCurrentPosition() > -2130) {
            moveToPos(-0.6, -2130);
            }*/

        if(gamepad2.right_bumper && liftMotor.getCurrentPosition() < -1255 && liftMotor.getCurrentPosition() > -2130) {
            moveToPos(-0.6, -2130);
        }

        // Moves up from travel to low
  /*      if(gamepad1.right_bumper && liftMotor.getCurrentPosition() < -180 && liftMotor.getCurrentPosition() > -1255) {
            moveToPos(-0.6, -1255);
        }*/

        if(gamepad2.right_bumper && liftMotor.getCurrentPosition() < -180 && liftMotor.getCurrentPosition() > -1255) {
            moveToPos(-0.6, -1255);
        }

        // Moves up from bottom to travel
     /*   if(gamepad1.right_bumper && liftMotor.getCurrentPosition() < 0 && liftMotor.getCurrentPosition() > -180) {
            moveToPos(-0.6, -180);
        }*/
        if(gamepad2.right_bumper && liftMotor.getCurrentPosition() < 0 && liftMotor.getCurrentPosition() > -180) {
            moveToPos(-0.6, -180);
        }
        // Servo open
     /*   if(gamepad1.x || liftMotor.getCurrentPosition() > -50) {
            servoMove(0.30);

        }*/
        if(gamepad2.x || liftMotor.getCurrentPosition() > -50) {
            servoMove(0.55);
        }

        // Servo close
       /* if(gamepad1.y) {
            servoMove(0.08);
        }*/
        if(gamepad2.y) {
            servoMove(0);
        }

        // Travel height
        if(gamepad2.dpad_down){
            moveToPos(-0.4, -180); //THIS WORKS FOR NOW, DON'T CHANGE IT
        }
        // Low pole
        if(gamepad2.dpad_right) {
            moveToPos(-0.6, -1255);
        }
        // Medium pole
        if(gamepad2.dpad_left) {
            moveToPos(-0.6, -2130);
        }
        // High pole
        if(gamepad2.dpad_up) {
            moveToPos(-0.7, -2900);
        }

        // Servo position in degrees
        telemetry.addData("Servo position specified", gripper.getPosition());
        telemetry.update();

     /*   if(gamepad1.a) {
            moveToPos(-0.6,-2900);
        }
        else if(gamepad1.b) {
            liftMotor.setPower(0.45);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // BEN A -- } else {
            // BEN A -- liftMotor.setPower(0);
        }*/
        if(gamepad2.a) {
            moveToPos(-0.6,-2900);
        }
        else if(gamepad2.b) {
            liftMotor.setPower(0.45);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}