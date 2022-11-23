package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name="2023: Autonomous", group="Robot")

public class Autonomous2023 extends LinearOpMode {

    // declare OpMode members
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor liftMotor;
    private Servo gripper;

    //  int[] elevatorPos = {-2900, -2130, -1255, -180};

    private ElapsedTime     runtime = new ElapsedTime();

    // measurements used to determine the distance travelled by the robot for every encoder count
    static final double     COUNTS_PER_INCH         = 51;
    static final double     DRIVE_SPEED             = 0.4039;
    static final double     TURN_SPEED              = 0.5; //not used in this year's autonomou
    
    private void servoMove(double servoPos) {
        gripper.setPosition(servoPos);
    }
    
    private void moveToPos(double pow, int pos) {
        liftMotor.setPower(pow);
        liftMotor.setTargetPosition(pos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 18;
    int MIDDLE = 4;
    int RIGHT = 33;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        // initialize the drive, lift and gripper variables
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        gripper = hardwareMap.get(Servo.class, "gripper");
        
        // map motors to direction of rotation (Left motors are always reversed)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        gripper.setDirection(Servo.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        
        // set behavior of motors when power is removed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // reset encoders for drive and lift motors
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        int randomizationTarget = tagOfInterest.id;
        // wait for the game to start (driver presses PLAY)
        //waitForStart();
        
        
        // put AprilTag determination code in here


        // progress through each step of the autonomous routine

        //ensure gripper is open
        
        servoMove(0.30); 
        
        // segment 1: strafe right to centre of second row of tiles (28) or to align with poles (40), 2 second timer
        
        encoderDrive(DRIVE_SPEED,  0,  45, 5.0); 
        sleep(1000);
        encoderDrive(DRIVE_SPEED,  0,  -4, 2.0);
        sleep(1000);
        
        runtime.reset(); // reset elapsed time timer
        
        // segment 2: raise cone to scoring height for mid junction
        
        liftMotor.setTargetPosition(-2130);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.60);
        while (opModeIsActive() && liftMotor.isBusy())
        {
            telemetry.addData("encoder-lift", liftMotor.getCurrentPosition() + "  busy=" + liftMotor.isBusy());
            telemetry.update();
            idle();
        }
        
        runtime.reset(); // reset elapsed time timer
        
        // segment 3: drive forward to position cone over junction
        
        encoderDrive(DRIVE_SPEED, 6,  0,  2.0);
        runtime.reset(); // reset elapsed time timer
        sleep(1000);
        
        // step 4: close gripper to release cone
        
        servoMove(0.08); 
        
        while (opModeIsActive() &&
            (runtime.seconds() < 2.0)) {
            telemetry.addData("encoder-lift", liftMotor.getCurrentPosition() + "  busy=" + liftMotor.isBusy());
            telemetry.update();
            idle();
        }
        
        runtime.reset(); // reset elapsed time timer
        // step 5: drive backwards to clear junction
        
        encoderDrive(DRIVE_SPEED, -6,  0,  2.0); 
        runtime.reset(); // reset elapsed time timer
        sleep(1000);
        
        // step 6: lower gripper to travel height
        
        liftMotor.setTargetPosition(-180); 
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.40);
        while (opModeIsActive() && liftMotor.isBusy())
        {
            telemetry.addData("encoder-lift", liftMotor.getCurrentPosition() + "  busy=" + liftMotor.isBusy());
            telemetry.update();
            idle();
        }
        runtime.reset(); // reset elapsed time timer
        
        // step 7: strafe left to get to second row of tiles
        
        encoderDrive(DRIVE_SPEED,  0,  -12, 5.0);  
        runtime.reset(); // reset elapsed time timer
        sleep(1000);
        
        // step 8: move to the correct tile column, based on randomized target
        
            if (randomizationTarget == 18) {
                encoderDrive(DRIVE_SPEED,  20,  0,  5.0); // S2: Forward 23.5 Inches with 5 Sec timeout
            } else if (randomizationTarget == 33) {
                encoderDrive(DRIVE_SPEED,  -20,  0,  5.0);
            } else if (randomizationTarget == 4){
                encoderDrive(DRIVE_SPEED, 0,  0,  5.0);
            }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message
        // end of autonomous steps
    }

    public void encoderDrive(double speed,
                             double forwardInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;

        // ensure that the opmode is still active
        if (opModeIsActive()) {

            // determine new drive base target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition()
                + (int)((forwardInches - rightInches)* COUNTS_PER_INCH);
            newRearLeftTarget = rearLeft.getCurrentPosition()
                + (int)((forwardInches + rightInches)* COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition()
                + (int)((forwardInches - rightInches)* COUNTS_PER_INCH);
            newRearRightTarget = rearRight.getCurrentPosition()
                + (int)((forwardInches + rightInches)* COUNTS_PER_INCH);
                
            frontLeft.setTargetPosition(newFrontLeftTarget);
            rearLeft.setTargetPosition(newRearLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            rearRight.setTargetPosition(newRearRightTarget);
            
            // turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeft.isBusy() && rearLeft.isBusy() && frontRight.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                //telemetry.addData("Currently at",  " at %7d :%7d",
                                            //leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            rearLeft.setPower(0);
            frontRight.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            sleep(250);   // optional pause after each move.
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
