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
import org.firstinspires.ftc.teamcode.util.MotorConstantValues;

@TeleOp
public class CyanCheetahOpBlue extends LinearOpMode
{
    private final DcMotor frontl = hardwareMap.get(DcMotor.class, "leftUpper");
    private final DcMotor frontr = hardwareMap.get(DcMotor.class, "rightUpper");
    private final DcMotor bottoml = hardwareMap.get(DcMotor.class, "leftLower");
    private final DcMotor bottomr = hardwareMap.get(DcMotor.class, "rightLower");
    private final DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftLift");
    private final DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");
    private final DcMotor rightHang = hardwareMap.get(DcMotor.class, "rightHang");
    private final DcMotor leftHang = hardwareMap.get(DcMotor.class, "leftHang");
    private final Servo droneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
    private final Servo droneLinkage = hardwareMap.get(Servo.class, "DroneLinkage");
    private final Servo intakePos = hardwareMap.get(Servo.class, "IntakePos");
    private final Servo outtakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
    private final Servo outtakeFlip = hardwareMap.get(Servo.class, "OuttakeFlip");
    private final Servo outtakeSpin = hardwareMap.get(Servo.class, "OuttakeSpin");
    private final CRServo intakeUno = hardwareMap.get(CRServo.class, "IntakeUno");
    private final CRServo intakeDos = hardwareMap.get(CRServo.class, "IntakeDos");
    private final CRServo intakeRoller = hardwareMap.get(CRServo.class, "IntakeRoller");
    @Override public void runOpMode()
    {
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
        waitForStart();
        while (opModeIsActive())
        {
//          ************************************************ GAMEPAD 2 CONTROLS ************************************************
            /**
             * Gamepad 2 Controls:
             * Dpad Up: Elevator Up (Faster Speed)
             * Dpad Down: Elevator Down (Faster Speed)
             * Dpad Left: NOTHING
             * Dpad Right: NOTHING
             * Left Joystick: Elevator
             * - Up: Elevator Up
             * - Down: Elevator Down
             * Right Joystick:
             * - Turn the position of Pixels.
             * - 8 different configurations.
             * X: IntakeIn
             * Y: IntakeOut
             * A: HangUp
             * B: HangDown
             * Left Bumper: Outtake In
             * Right Bumper: Outtake Out
             * Right Trigger: Outtake Close
             * Left Trigger: Outtake Open
             */

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
                outtakeSpin.setPosition(Top);
            }
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y < -.55){
                telemetry.addData("TopLeft", stuff);
                outtakeSpin.setPosition(topLeft);

            }
            if (gamepad2.right_stick_x < -0.5 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                telemetry.addData("Left", stuff);
                outtakeSpin.setPosition(left);

            }
            if (gamepad2.right_stick_x > 0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomRight", stuff);
                outtakeSpin.setPosition(bottomRight);

            }
            if (gamepad2.right_stick_x > 0.55 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                telemetry.addData("Right", stuff);
                outtakeSpin.setPosition(right);

            }
            if (gamepad2.right_stick_x >  0.55 && gamepad2.right_stick_y < -.55){
                telemetry.addData("TopRight", stuff);
                outtakeSpin.setPosition(topRight);

            }
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomLeft", stuff);
                outtakeSpin.setPosition(bottomLeft);

            }

            //intake code
            if (gamepad2.x) {
                intakeUno.setPower((.8));
                intakeDos.setPower((-.8));
                intakeRoller.setPower((-1));
            }
            else if (gamepad2.y) {
                intakeUno.setPower((-.8));
                intakeDos.setPower((.8));
                intakeRoller.setPower((1));
            }
            else {
                intakeUno.setPower((0));
                intakeDos.setPower((0));
                intakeRoller.setPower((0));
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
                outtakeSpin.setPosition(left);
            }
            if (gamepad2.dpad_right) {
                outtakeSpin.setPosition(Top);
            }

            //sequence for the outtake flipping
            if (gamepad2.left_bumper) {
                outtakeFlip.setPosition(flipIn);
                outtakeSpin.setPosition(Top);
                outtakeClaw.setPosition(outOpen);
            }
            if (gamepad2.right_bumper) {
                outtakeFlip.setPosition(flipOut);
            }

            if (gamepad2.left_trigger > 0.5) {
                outtakeClaw.setPosition(outClose);
            }
            if (gamepad2.right_trigger > 0.5) {
                outtakeClaw.setPosition(outOpen);
            }

            //          ************************************************ GAMEPAD 1 CONTROLS ************************************************
            /**
             * Gamepad 1 Controls:
             * Dpad Up: DrownUp
             * Dpad Down: DroneDown
             * Dpad Left: NOTHING
             * Dpad Right: NOTHING
             * Left Joystick:
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

            //drone position up
            if(gamepad1.dpad_up) {
                double pos = 1.0;
                //DroneLinkage.setPosition(.87);
                for (int i = 0; i < 6; i++) {
                    droneLinkage.setPosition(pos);
                    pos = pos - .01;
                    sleep(100);
                }
                //DroneLinkage.setPosition(0.85);
            }

            //DroneDown
            if(gamepad1.dpad_down) {
                droneLinkage.setPosition(1);
            }

            //drone Launch
            if(gamepad1.left_bumper && gamepad1.right_bumper) {
                droneLauncher.setPosition(0.3);
            }

            //intake positions
            if(gamepad1.a){
                intakePos.setPosition(intakeDown);
            }
            if(gamepad1.b){
                intakePos.setPosition(intakeSemiDown);
            }
            if(gamepad1.y){
                intakePos.setPosition(intakeSemiUp);
            }
            if(gamepad1.x){
                intakePos.setPosition(intakeUp);
            }

            //drive code
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

            //telemetry on screen
            telemetry.addData("Drone", droneLinkage.getPosition());//dronePosition
            telemetry.addData("rightX", gamepad2.right_stick_x);//right Stick Position in X
            telemetry.addData("rightY", gamepad2.right_stick_y);//right Stick Position in Y
            telemetry.update();
        }
    }
}