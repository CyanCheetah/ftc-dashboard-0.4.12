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
import org.firstinspires.ftc.teamcode.util.MotorConstantValues;
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
public class
CyanCheetahOpRed extends LinearOpMode
{
    private DcMotor frontl   = null;  //  Used to control the left front drive wheel
    private DcMotor frontr  = null;  //  Used to control the right front drive wheel
    private DcMotor bottoml    = null;  //  Used to control the left back drive wheel
    private DcMotor bottomr   = null;
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private DcMotor rightHang = null;
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
    private static double MOTOR_ADJUST = 0.75;
    @Override public void runOpMode()
    {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        frontl = hardwareMap.get(DcMotor.class, "leftUpper");
        frontr = hardwareMap.get(DcMotor.class, "rightUpper");
        bottoml = hardwareMap.get(DcMotor.class, "leftLower");
        bottomr = hardwareMap.get(DcMotor.class, "rightLower");
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        rightHang = hardwareMap.get(DcMotor.class,"rightHang");
        leftHang = hardwareMap.get(DcMotor.class,"leftHang");
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
        waitForStart();
        MotorConstantValues constants = new MotorConstantValues();
        double triggerPowerAdjust = 1;
        double speedAdjust = 1.4;
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
        double bottomLeft = constants.getSpinBottomLeft();
        double intakeSemiUp = constants.getIntakeSemiUp();
        double intakeSemiDown = constants.getIntakeSemiDown();
        int stuff = 0;
        while (opModeIsActive())
        {

            //elevator height code
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


            //pixels position code

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
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomLeft", stuff);
                OuttakeSpin.setPosition(bottomLeft);

            }
            if (true) {
                telemetry.addData("rightX", gamepad2.right_stick_x);
                telemetry.addData("rightY", gamepad2.right_stick_y);
            }


            //intake code
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
            //hang code
            if (gamepad2.a) {
                leftHang.setPower(-.4);
                rightHang.setPower(.4);
            }
            else if (gamepad2.b) {
                leftHang.setPower(.6);
                rightHang.setPower(-.6);
            }
            else {
                leftHang.setPower(0);
                rightHang.setPower(0);
            }
            //outtake position
            if (gamepad2.dpad_left) {
                OuttakeSpin.setPosition(left);
            }
            if (gamepad2.dpad_right) {
                OuttakeSpin.setPosition(Top);
            }
            //sequence for the outtake flipping
            if (gamepad2.left_bumper) {
                OuttakeFlip.setPosition(flipIn);
                OuttakeSpin.setPosition(Top);
                OuttakeClaw.setPosition(outOpen);
            }
            if (gamepad2.right_bumper) {
                OuttakeFlip.setPosition(flipOut);
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
             * Dpad Up: DrownUp
             * Dpad Down: DroneDown
             * Dpad Left: NOTHING
             * Dpad Right: NOTHING
             * Left Joystick: NOTHING
             * - Up: Forward
             * - Down: Backwards
             * - Left: Strafe Left
             * - Right: Strafe Right
             * Right Joystick:
             * - Left: Turn in position left
             * - Right: Turn in position right
             * X: intakeUp
             * Y: intakeSemiUp
             * A: IntakePosDown
             * B: intakeSemiDown
             * Left Bumper & Right Bumper: Drone Shoot
             * Right Trigger: Trigger Power Adjust: Slows the robot down by given amount
             * Left Trigger: Makes Robot Faster
             */

            //slows down
            if (gamepad1.right_trigger > 0) {
                triggerPowerAdjust = .4;
            } else {
                triggerPowerAdjust = 1;
            }
            //speeds up
            if (gamepad1.left_trigger > 0) {
                speedAdjust = 1.5;
            } else {
                speedAdjust = 1;
            }

            //drone position
            if(gamepad1.dpad_up) {
                double pos = 1.0;
                //DroneLinkage.setPosition(.87);
                for (int i = 0; i < 6; i++) {
                    DroneLinkage.setPosition(pos);
                    pos = pos - .01;
                    sleep(100);
                }
                //DroneLinkage.setPosition(0.85);
            }
            if(gamepad1.dpad_down) {
                DroneLinkage.setPosition(1);
            }

            telemetry.addData("Drone", DroneLinkage.getPosition());
            if(gamepad1.left_bumper && gamepad1.right_bumper) {
                DroneLauncher.setPosition(0.3);
            }

            if(gamepad1.a){
                IntakePos.setPosition(intakeDown);
            }
            if(gamepad1.b){
                IntakePos.setPosition(intakeSemiDown);
            }
            if(gamepad1.y){
                IntakePos.setPosition(intakeSemiUp);
            }
            if(gamepad1.x){
                IntakePos.setPosition(intakeUp);
            }

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1 = (v1 * triggerPowerAdjust * -1) * speedAdjust;
            v2 = (v2 * triggerPowerAdjust * -1) * speedAdjust;
            v3 = (v3 * triggerPowerAdjust * -1) * speedAdjust;
            v4 = (v4 * triggerPowerAdjust * -1) * speedAdjust;
            frontl.setPower(v1 * 1);
            frontr.setPower(v2 * 1);
            bottoml.setPower(v3 * 1);
            bottomr.setPower(v4 * 1);
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

}