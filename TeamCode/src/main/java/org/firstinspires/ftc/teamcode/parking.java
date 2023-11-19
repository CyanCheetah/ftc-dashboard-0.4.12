/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

// TODO: remove Actions from the core module?
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@Autonomous
public class parking extends LinearOpMode {
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    private static DcMotor frontl = null;
    private static DcMotor frontr = null;
    private static DcMotor bottoml = null;
    private static DcMotor bottomr = null;
    private static DcMotor rightLift = null;
    private static DcMotor leftLift = null;
    private static Servo servoOne = null;
    private static Servo servoTwo = null;
    private static Servo Bucket = null;
    private static Servo Swing = null;
    private static Servo Turn = null;
    //-------------------------------------------------------------------------------//
    enum PowerLevel {MAX, HALF, QUARTER, STOP}     // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();
    private static double MOTOR_ADJUST = 1;
    public void moveServos (Servo right, Servo left, double position){
        left.setPosition(.5 + position);
        right.setPosition(.51 - position);
    }

    public static void moveBack (double power, long duration, boolean stop) throws InterruptedException {
        frontl.setPower(power);
        frontr.setPower(power);
        bottoml.setPower(power);
        bottomr.setPower(power);

        Thread.sleep(duration);
        if(stop == true){
            frontl.setPower(0);
            frontr.setPower(0);
            bottoml.setPower(0);
            bottomr.setPower(0);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {


        {
            PowerLevel powerLevel = PowerLevel.HALF.QUARTER;
            frontl = hardwareMap.get(DcMotor.class, "leftUpper");
            frontr = hardwareMap.get(DcMotor.class, "rightUpper");
            bottoml = hardwareMap.get(DcMotor.class, "leftLower");
            bottomr = hardwareMap.get(DcMotor.class, "rightLower");
            leftLift = hardwareMap.get(DcMotor.class,"leftLift");
            rightLift = hardwareMap.get(DcMotor.class,"rightLift");
            Turn = hardwareMap.get(Servo.class, "Turn");
            Servo servoOne = hardwareMap.servo.get("servoOne");
            Servo servoTwo = hardwareMap.servo.get("servoTwo");
            Bucket = hardwareMap.get(Servo.class, "Bucket");
            Swing = hardwareMap.get(Servo.class, "Swing");
            //Setting Directions of motors.
            frontl.setDirection(DcMotor.Direction.FORWARD);
            frontr.setDirection(DcMotor.Direction.REVERSE);
            bottoml.setDirection(DcMotor.Direction.FORWARD);
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

            Turn.setPosition(1);

            waitForStart();
            runtime.reset();
            frontl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottoml.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bottomr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bottoml.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            bottomr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // Setup a variable for each drive wheel
            double frontlPower = 0;
            double frontrPower = 0;
            double bottomlPower = 0;
            double bottomrPower = 0;
            // -------
            double SwingOutPosition = 0;
            double SwingInPosition = 0.3;
            double BucketOutPosition = .24;
            double BucketInPosition = .41;
            // ------ the values above may change often
            double clawClose = 1;
            double clawSemiOpen = .95;
            double clawFullOpen = .775;
            double triggerPowerAdjust = 1;

            while (opModeIsActive()) {
                moveBack(-.2, 3500, true);

                Turn.setPosition(clawFullOpen);
                moveBack(0,30000,true);



            }
        }


    }
}