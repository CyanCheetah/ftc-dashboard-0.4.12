/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @author CyanCheeah
 * This is the TeleOp but for the blue side
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp
public class CyanCheetahOpBlue extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 2; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontl   = null;  //  Used to control the left front drive wheel
    private DcMotor frontr  = null;  //  Used to control the right front drive wheel
    private DcMotor bottoml    = null;  //  Used to control the left back drive wheel
    private DcMotor bottomr   = null;
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private DcMotor rightHang = null;
    private Servo servoOne = null;
    private Servo servoTwo = null;
    //private Servo Bucket = null;
    // private Servo Swing = null;
    // private Servo Turn = null;
    private DcMotor leftHang = null;//  Used to control the right back drive wheel
    private static Servo DroneLauncher = null;
    private static Servo DroneLinkage = null;

    private static CRServo IntakeUno = null;
    private static CRServo IntakeDos = null;
    private static CRServo IntakeRoller = null;
    private static Servo IntakePos = null;
    private static Servo OuttakeClaw = null;
    private static Servo OuttakeFlip = null;

    private static Servo OuttakeSpin = null;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public void moveServos (Servo right, Servo left, double position){
        left.setPosition(.5 + position);
        right.setPosition(.51 - position);
    }
    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        frontl = hardwareMap.get(DcMotor.class, "leftUpper");
        frontr = hardwareMap.get(DcMotor.class, "rightUpper");
        bottoml = hardwareMap.get(DcMotor.class, "leftLower");
        bottomr = hardwareMap.get(DcMotor.class, "rightLower");
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        rightHang = hardwareMap.get(DcMotor.class,"rightHang");
        leftHang = hardwareMap.get(DcMotor.class,"leftHang");
        //Turn = hardwareMap.get(Servo.class, "Turn");
        //Servo servoOne = hardwareMap.servo.get("servoOne");
        //Servo servoTwo = hardwareMap.servo.get("servoTwo");
        //Bucket = hardwareMap.get(Servo.class, "Bucket");
        //Swing = hardwareMap.get(Servo.class, "Swing");
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        DroneLinkage = hardwareMap.get(Servo.class, "DroneLinkage");
        IntakeUno = hardwareMap.get(CRServo.class, "IntakeUno");
        IntakeDos = hardwareMap.get(CRServo.class, "IntakeDos");
        IntakeRoller = hardwareMap.get(CRServo.class, "IntakeRoller");
        IntakePos = hardwareMap.get(Servo.class, "IntakePos");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeFlip = hardwareMap.get(Servo.class, "OuttakeFlip");
        OuttakeSpin = hardwareMap.get(Servo.class, "OuttakeSpin");



        //Setting Directions of motors.
        frontl.setDirection(DcMotor.Direction.FORWARD);
        bottoml.setDirection(DcMotor.Direction.FORWARD);
        frontr.setDirection(DcMotor.Direction.REVERSE);
        bottomr.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        //Brake immedietly after joystick hits 0 instead of coasting down
        frontl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottoml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        //MotorConstantValues constants = new MotorConstantValues();
        //double SwingOutPosition = constants.getSwingOutPosition();
        //double SwingScorePosition = constants.getSwingScorePosition();
        //double SwingInPosition = constants.getSwingInPosition();
        //double BucketOutPosition = constants.getBucketOutPosition();
        //double BucketInPosition = constants.getBucketInPosition();
        //double BucketSuperUp = BucketInPosition + .025;
        //double mainLiftPower = 0;
        // ------ the values above may change often
        //double clawClose = constants.getClawClose();
        //double clawSemiOpen = constants.getClawSemiOpen();
        //double clawFullOpen = constants.getClawFullOpen();
        MotorConstantValues constants = new MotorConstantValues();
        double triggerPowerAdjust = 1;
        double intakeUp = constants.getIntakeUp();
        double intakeDown = constants.getIntakeDown();
        double outClose = constants.getOutClose();
        double outOpen = constants.getOutOpen();
        double flipOut = constants.getFlipOut();
        double flipIn = constants.getFlipIn();
        double Top = constants.getSpinTop();
        double topLeft = constants.getSpinTopLeft();
        double left = constants.getSpinLeft();
        double bottomRight = constants.getSpinBottomRight();
        double right = constants.getSpinRight();
        double topRight = constants.getSpinTopRight();
        int stuff = 0;
        //double intakeUp = 0.92; GOOD STACK 5
        //double intakeDown = .93; GOOD STACK 4
        //double bucketPos = 0.37 ;
        //boolean swingOut = false;
        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            if (gamepad2.left_stick_y > 0.3) {
                rightLift.setPower((.2));
                leftLift.setPower((-.2));
            }
            else if (gamepad2.left_stick_y < -0.3) {
                rightLift.setPower((-.2));
                leftLift.setPower((.2));
            }
            else if (gamepad2.dpad_down) {
                rightLift.setPower((.65));
                leftLift.setPower((-.65));
            }
            else if (gamepad2.dpad_up) {
                rightLift.setPower((-.65));
                leftLift.setPower((.65));
            }
            else {
                rightLift.setPower((0));
                leftLift.setPower((0));
            }



            if (gamepad2.right_stick_y < -0.5 && (gamepad2.right_stick_x >= -0.1 && gamepad2.right_stick_x <= 0.1)){
                telemetry.addData("Top", stuff);
                OuttakeSpin.setPosition(Top);
            }
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y < -.55){
                telemetry.addData("TopLeft", stuff);
                OuttakeSpin.setPosition(topLeft);

            }
            if (gamepad2.right_stick_x < -0.5 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                telemetry.addData("Left", stuff);
                OuttakeSpin.setPosition(left);

            }
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomLeft", stuff);

            }
            if (gamepad2.right_stick_x > 0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomRight", stuff);
                OuttakeSpin.setPosition(bottomRight);

            }
            if (gamepad2.right_stick_x > 0.55 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                telemetry.addData("Right", stuff);
                OuttakeSpin.setPosition(right);

            }
            if (gamepad2.right_stick_x >  0.55 && gamepad2.right_stick_y < -.55){
                telemetry.addData("TopRight", stuff);
                OuttakeSpin.setPosition(topRight);

            }
            if (true) {
                telemetry.addData("rightX", gamepad2.right_stick_x);
                telemetry.addData("rightY", gamepad2.right_stick_y);
            }
            /*if(gamepad1.dpad_up) {
                index1 += 1;
                IntakePos.setPosition(hello[index1]);
            }*/

            //movement to the bucket stuff
            /* if (gamepad2.x) {
                moveServos(servoOne, servoTwo, -.375);
            }
            if (gamepad2.b) {
                moveServos(servoOne, servoTwo, -.3);
            }
            if (gamepad2.y) {
                moveServos(servoOne, servoTwo, -.32);
            }
            if (gamepad2.a) {
                moveServos(servoOne, servoTwo, -.395);
            }
             */
            if (gamepad2.x) {
                IntakeUno.setPower((.8));
                IntakeDos.setPower((-.8));
                IntakeRoller.setPower((-1));
            }
            else if (gamepad2.y) {
                IntakeUno.setPower((-.8));
                IntakeDos.setPower((.8));
                IntakeRoller.setPower((1));
            }
            else {
                IntakeUno.setPower((0));
                IntakeDos.setPower((0));
                IntakeRoller.setPower((0));
            }
            if (gamepad2.dpad_left) {
                OuttakeSpin.setPosition(left);
            }
            if (gamepad2.dpad_right) {
                OuttakeSpin.setPosition(Top);
            }
            if (gamepad2.left_bumper) {
                OuttakeFlip.setPosition(flipIn);
                OuttakeSpin.setPosition(Top);
                OuttakeClaw.setPosition(outOpen);
            }
            if (gamepad2.right_bumper) {
                OuttakeFlip.setPosition(flipOut);
            }
            if (gamepad2.a){
                IntakePos.setPosition(intakeUp);
            }

            if (gamepad2.b){
                IntakePos.setPosition(intakeDown);
            }
            if (gamepad2.left_trigger > 0.5) {
                OuttakeClaw.setPosition(outClose);
            }
            if (gamepad2.right_trigger > 0.5) {
                OuttakeClaw.setPosition(outOpen);
            }
            //          ************************************************ GAMEPAD 1 CONTROLS ************************************************
            /**
             * Gamepad 1 Controls:
             * Dpad Up: Nothing
             * Dpad Down: April Tags
             * Dpad Left: April Tags
             * Dpad Right: April Tags
             * Left Joystick: NOTHING
             * - Up: Forward
             * - Down: Backwards
             * - Left: Strafe Left
             * - Right: Strafe Right
             * Right Joystick:
             * - Left: Turn in position left
             * - Right: Turn in position right
             * X: NOTHING
             * Y: Hang goes Down
             * A: Hang goes Up
             * B: Drone Launcher
             * Left Bumper: Drone Linkage
             * Right Trigger: Trigger Power Adjust: Slows the robot down by given amount
             * Right Bumper: Drone Linkage
             * Left Trigger: AprilTags
             */
            if (gamepad1.right_trigger > 0) {
                triggerPowerAdjust = .4;
            } else {
                triggerPowerAdjust = 1;
            }
            //left and right hang code
            if (gamepad1.y) {
                leftHang.setPower(-.4);
                rightHang.setPower(.4);
            }
            else if (gamepad1.a) {
                leftHang.setPower(.6);
                rightHang.setPower(-.6);
            }
            else {
                leftHang.setPower(0);
                rightHang.setPower(0);
            }
            //minute adjustions for the bucket angle. The bucketPos double variable
            //despite the game controller 2's bucket movement. The values are updated.
            //drone launch code.
            if(gamepad1.b) {
                DroneLinkage.setPosition(1);
                //DroneLinkage.setPosition(0.85);
            }
            if(gamepad1.left_bumper) {
                double pos = 1.0;
                //DroneLinkage.setPosition(.87);
                for (int i = 0; i < 6; i++) {
                    DroneLinkage.setPosition(pos);
                    pos = pos - .01;
                    sleep(200);
                }

            }

            telemetry.addData("Drone", DroneLinkage.getPosition());
            if(gamepad1.right_bumper) {
                DroneLauncher.setPosition(0.3);
            }
            if(gamepad1.dpad_left) {
                DESIRED_TAG_ID = 1;
            }
            if (gamepad1.dpad_down) {
                DESIRED_TAG_ID = 2;
            }
            if (gamepad1.dpad_right) {
                DESIRED_TAG_ID = 3;
            }
            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Trigger to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("X",  "%5.1f inches", desiredTag.ftcPose.x);
                telemetry.addData("Y","%5.1f inches", desiredTag.ftcPose.y);
                telemetry.addData("Z","%5.1f inches", desiredTag.ftcPose.z);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_trigger > 0.3 && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y  / 1.4 * triggerPowerAdjust;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 1.4 * triggerPowerAdjust;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 2 * triggerPowerAdjust;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontl.setPower(leftFrontPower);
        frontr.setPower(rightFrontPower);
        bottoml.setPower(leftBackPower);
        bottomr.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "cameraMonitorViewId"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}