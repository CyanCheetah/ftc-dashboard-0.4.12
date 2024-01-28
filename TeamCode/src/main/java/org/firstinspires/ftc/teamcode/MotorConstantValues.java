package org.firstinspires.ftc.teamcode;
/**
 * @author CyanCheeah
 * This is motor constants for the elevator and bucket and servo.
 */
public class MotorConstantValues {

    private double triggerPowerAdjust = 1;

    private double intakeUp = 0.85;

    private double intakeDown = 0.99;

    private double outClose = 0.03;

    private double outOpen = 0.275;

    private double flipOut = 0.78;

    private double flipIn = 0.245;

    private double Top = .655;

    private double Left = 0.98;

    private double topLeft = .785;

    private double bottomRight = .14125;

    private double topRight = .515;

    private double Right = 0.3125;
    public double getIntakeUp(){
        return intakeUp;
    }
    public double getIntakeDown(){
        return intakeDown;
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


}
