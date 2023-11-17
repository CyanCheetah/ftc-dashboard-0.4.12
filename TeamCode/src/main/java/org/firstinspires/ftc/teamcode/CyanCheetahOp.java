

package org.firstinspires.ftc.teamcode;//Hollow World
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
//------------------------------------------------------------------------------//
@TeleOp
public class CyanCheetahOp extends LinearOpMode {
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
    private static DcMotor rightHang = null;
    private static Servo servoOne = null;
    private static Servo servoTwo = null;
    private static Servo Bucket = null;
    private static Servo Swing = null;
    private static Servo Turn = null;
    private static DcMotor leftHang = null;
    //-------------------------------------------------------------------------------//
    enum PowerLevel {MAX, HALF, QUARTER, STOP}     // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();
    private static double MOTOR_ADJUST = 0.9;
    private static double SMALL_GEAR_TEETH = 40; //Smaller gear is directly on the motor
    private static double LARGE_GEAR_TEETH = 120; //Larger gear is connected to the arm
    private static double GEAR_RATIO = (LARGE_GEAR_TEETH / SMALL_GEAR_TEETH); //States the gear ratio based on the teeth of gears used
    private static double ORIBITAL_60_PULSE_PER_ROTATION = 1680; //Motor is an AndyMark Neverest 60 GearMotor
    private static double ARM_PULSE_PER_ROTATION = ORIBITAL_60_PULSE_PER_ROTATION * GEAR_RATIO; //Amount of rotation per motor to gears
    private static double DEGREES_PER_TICK = 360 / ARM_PULSE_PER_ROTATION; //Converts Radians into Degrees
    //Power for Motor Arm
    private static double MOTOR_ARM_MAX_POWER = .7;
    private static double MOTOR_ARM_MIN_POWER = .3;
    private static double ARM_HOLDING_POWER = .15;
    private static int OPT_PICKUP_HEIGHT = 186;
    private static int OPT_TRAV_HEIGHT = 459;
    private final long BILLION = 1000000000;
    private static double SIDEWAYS_DRIFT_CORRECTION = 1.125;
    private int x = 1;
    private int I = 0;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean ypressed = false;
    //double initialServoPosition = ServoArm.getPosition();
    public void moveServos (Servo right, Servo left, double position){
        left.setPosition(.5 + position);
        right.setPosition(.51 - position);
    }
    @Override
    public void runOpMode() {
        PowerLevel powerLevel = PowerLevel.HALF.QUARTER;
        frontl = hardwareMap.get(DcMotor.class, "leftUpper");
        frontr = hardwareMap.get(DcMotor.class, "rightUpper");
        bottoml = hardwareMap.get(DcMotor.class, "leftLower");
        bottomr = hardwareMap.get(DcMotor.class, "rightLower");
        leftLift = hardwareMap.get(DcMotor.class,"leftLift");
        rightLift = hardwareMap.get(DcMotor.class,"rightLift");
        rightHang = hardwareMap.get(DcMotor.class,"rightHang");
        leftHang = hardwareMap.get(DcMotor.class,"leftHang");
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
        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
//------------------------------------------------------------------ Start of Match ---------------------------------------------------
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
        double BucketOutPosition = .145;
        double BucketInPosition = .325;
        double mainLiftPower = 0;
        // ------ the values above may change often
        double clawClose = 1;
        double clawSemiOpen = .95;
        double clawFullOpen = .775;
        double triggerPowerAdjust = 1;
        while (opModeIsActive()) {  //While Teleop is in session
            Servo Turn = hardwareMap.servo.get("Turn");
//          ************************************************ GAMEPAD 2 CONTROLS ************************************************
            /**
            if (gamepad1.dpad_up) {
                rightHang.setPower(.4);
                leftHang.setPower(-.4);
            }
            else if (gamepad1.dpad_down) {
                rightHang.setPower(-.4);
                leftHang.setPower(.4);
            } else {
                rightHang.setPower(0);
                leftHang.setPower(0);
            }
             **/
            //Pushes the left hang up
            if (gamepad1.a) {
                leftHang.setPower(-.3);
            } //pushes the left hang down
            else if (gamepad1.x){
                leftHang.setPower(.6);
            }
            else {
                leftHang.setPower(0);
            }
            //Pushes the right hang up
            if (gamepad1.b) {
                rightHang.setPower(.3);
            } //pushes the right hang down
            else if (gamepad1.y){
                rightHang.setPower(-.6);
            } else {
                rightHang.setPower(0);
            }
            if (gamepad2.x) {
                moveServos(servoOne, servoTwo, -.375);
            }
            if (gamepad2.b) {
                moveServos(servoOne, servoTwo, -.3);
            }
            if (gamepad2.y) {
                moveServos(servoOne, servoTwo, -.32);
            }
            if (gamepad2.a) {
                moveServos(servoOne, servoTwo, -.44);
            }
            if (gamepad2.dpad_left){
                if(Turn.getPosition() > .94){
                    moveServos(servoOne, servoTwo, .2);
                    sleep(275);
                    Turn.setPosition(clawSemiOpen);
                    sleep(200);
                    moveServos(servoOne, servoTwo, -.3);
                    sleep(300);
                    Turn.setPosition(clawFullOpen);
                }
            }
            if (gamepad2.dpad_right){
                Turn.setPosition(clawClose);
            }
            //if (gamepad2.right_trigger > .5){
            //    Swing.setPosition(SwingOutPosition);
            //    sleep(350);
            //    Bucket.setPosition(BucketOutPosition);
            //}
            if (gamepad2.left_trigger > .5){
                Bucket.setPosition(BucketInPosition);
                sleep(100);
                Swing.setPosition(SwingInPosition);
            }
            if (gamepad2.left_bumper){
                Swing.setPosition(SwingOutPosition);
            }
            if (gamepad2.right_bumper) {
                Bucket.setPosition(BucketOutPosition);
            }
            //          ************************************************ GAMEPAD 1 CONTROLS ************************************************
            if (gamepad1.right_trigger > 0) {
                triggerPowerAdjust = .3
                ;
            } else {
                triggerPowerAdjust = 1;
            }
            //See the Desmos for an explanation, this is code that's basically modified from what they (FTC) gave us
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;
            v1 = v1 * MOTOR_ADJUST * triggerPowerAdjust;
            v2 = v2 * MOTOR_ADJUST * triggerPowerAdjust;
            v3 = v3 * MOTOR_ADJUST * triggerPowerAdjust;
            v4 = v4 * MOTOR_ADJUST * triggerPowerAdjust;
            frontl.setPower(v1 * 1);
            frontr.setPower(v2 * 1);
            bottoml.setPower(v3 * 1);
            bottomr.setPower(v4 * 1);
//          ************************************************ Telemetry ************************************************
            telemetry.addData("FrontL", -1 * frontl.getCurrentPosition());
            telemetry.addData("FrontR", -1 * frontr.getCurrentPosition());
            telemetry.addData("BackL", -1 * bottoml.getCurrentPosition());
            telemetry.addData("BackR", -1 * bottomr.getCurrentPosition());
            telemetry.addData("Battery Voltage", getBatteryVoltage());
            telemetry.update();
        }//While opMode
    }//void runOpMode
}
    /*
    Final Notes from Andy: Apressed sets the slide puller to optimum pickup, Bpressed sets slide puller to optimum travel and same concept with elevator motors just with Xpressed and Ypressed. Just need to do cleanup and redo elevator motors since we added another one
    */