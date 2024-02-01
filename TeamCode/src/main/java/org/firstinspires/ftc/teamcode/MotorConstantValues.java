package org.firstinspires.ftc.teamcode.util;
/**
 * @author CyanCheeah
 * This is motor constants for the elevator and bucket and servo.
 */
public class MotorConstantValues {

    private double triggerPowerAdjust = 1;

    private double intakeUp = 0.925;

    private double intakeSemiUp = 0.938;
    private double intakeSemiDown = 0.95;

    private double intakeDown = 0.99;

    private double outClose = 0.03;

    private double outOpen = 0.275;

    private double flipOut = 0.78;

    private double flipIn = 0.245;

    private double Top = .489;

    private double Left = 1-.185;

    private double topLeft = 1-.37;

    private double bottomRight = 0;

    private double topRight = .349;

    private double bottomLeft = 1;

    private double Right = .155;
    public double getIntakeUp(){
        return intakeUp;
    }
    public double getIntakeDown(){
        return intakeDown;
    }
    public double getIntakeSemiUp(){
        return intakeSemiUp;
    }
    public double getIntakeSemiDown(){
        return intakeSemiDown;
    }
    public double getOutClose(){
        return outClose;
    }
    public double getOutOpen(){
        return outOpen;
    }
    public double getFlipOut(){
        return flipOut;
    }
    public double getFlipIn(){
        return flipIn;
    }
    public double getSpinTop(){
        return Top;
    }
    public double getSpinTopLeft(){
        return topLeft;
    }
    public double getSpinBottomRight(){
        return bottomRight;
    }
    public double getSpinTopRight(){
        return topRight;
    }
    public double getSpinRight(){
        return Right;
    }
    public double getSpinLeft(){
        return Left;
    }
    public double getSpinBottomLeft(){
        return bottomLeft;
    }

}