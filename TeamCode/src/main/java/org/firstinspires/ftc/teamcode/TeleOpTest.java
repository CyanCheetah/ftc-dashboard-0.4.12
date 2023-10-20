package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;


@TeleOp(name="TeleOpTest")
public class TeleOpTest extends LinearOpMode {
    //private static Servo Turn = null;
    private static DcMotor leftUpper = null;
    private static DcMotor leftLower = null;
    private static DcMotor rightUpper = null;
    private static DcMotor rightLower = null;
    private static DcMotor rightLift = null;
    private static DcMotor leftLift = null;
    //private static DcMotor leftUpper = null;
   // private static DcMotor leftUpper = null;

    private static Servo Flip = null;
   // private static Servo Flip = null;
 //   private static Servo Flip = null;

    private static double MOTOR_ADJUST = 0.60;

    enum PowerLevel {MAX, HALF, QUARTER, STOP}

    ;

    @Override
    public void runOpMode() throws InterruptedException {
        //Servo Turn = hardwareMap.servo.get("Turn");
        DcMotor leftUpper = hardwareMap.dcMotor.get("leftUpper");
        DcMotor leftLower = hardwareMap.dcMotor.get("leftLower");
        DcMotor rightUpper = hardwareMap.dcMotor.get("rightUpper");
        DcMotor rightLower = hardwareMap.dcMotor.get("rightLower");
        DcMotor leftLift = hardwareMap.dcMotor.get("leftLift");
        DcMotor rightLift = hardwareMap.dcMotor.get("rightLift");

        PowerLevel powerLevel = PowerLevel.MAX;

        leftUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLower.setDirection(DcMotorSimple.Direction.FORWARD);
        rightUpper.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLower.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        telemetry.update();
        //double TurnPosition = 1;
        //double TurnPo = .75;
        //double turnP = Turn.getPosition();
        double denominator;
        double leftUpperPower = 0;
        double leftLowerPower = 0;
        double rightUpperPower = 0;
        double rightLowerPower = 0;
        double rightLiftPower = 0;
        double leftLiftPower = 0;

        double r = Math.hypot(-gamepad1.left_stick_y, -gamepad1.right_stick_x);
        double robotAngle = Math.atan2(gamepad1.right_stick_x, gamepad1.left_stick_y) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftUpper.setPower(v1 * MOTOR_ADJUST);
        leftLower.setPower(v2 * MOTOR_ADJUST);
        rightUpper.setPower(v3 * MOTOR_ADJUST);
        rightLower.setPower(v4 * MOTOR_ADJUST);
        //when the left trigger is pressed then the intake spins
        if (gamepad1.y) {
            powerLevel = PowerLevel.MAX;
        }
        if (gamepad1.b) {
            powerLevel = PowerLevel.HALF;
        }
        if (gamepad1.x) {
            powerLevel = PowerLevel.QUARTER;
        }
        if (gamepad1.x) {
            powerLevel = PowerLevel.STOP;
        }
        if (powerLevel == PowerLevel.MAX) {
            MOTOR_ADJUST = .75;
        }
        if (powerLevel == PowerLevel.HALF) {
            MOTOR_ADJUST = .50;
        }
        if (powerLevel == PowerLevel.QUARTER) {
            MOTOR_ADJUST = .25;
        }
        if (powerLevel == PowerLevel.STOP) {
            MOTOR_ADJUST = .0;
        }
       /* Turn.setPosition(TurnPosition);

        while (gamepad1.x) {
            Turn.setPosition(TurnPo);
        }
        while (gamepad1.b) {
            Turn.setPosition(TurnPosition);
        }*/
        if (gamepad2.left_trigger > 0.5) {
            rightLift.setPower(-1);
            leftLift.setPower(-1);
        }
        if (gamepad2.right_trigger > 0.5) {
            rightLift.setPower(1);
            leftLift.setPower(1);
        }


    }

}
