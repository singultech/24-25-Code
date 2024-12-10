package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SlidePair {
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    private boolean isActive;
    private final int maxHeight;
    private double power;

    public SlidePair(DcMotorEx left, DcMotorEx right, int maxH, double startingPower){
        rightSlide = right;
        leftSlide = left;
        maxHeight = maxH;
        power = startingPower;
        init();
    }
    public SlidePair(DcMotorEx left, DcMotorEx right, int maxH){
        rightSlide = right;
        leftSlide = left;
        maxHeight = maxH;
        power = 1.0;
        init();
    }

    private void init(){
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);
        setTargetPosition(0);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTargetPosition(int pos){
        if (pos > maxHeight) {
            leftSlide.setTargetPosition(maxHeight);
            double rightMaxHeight = 0.713679*maxHeight-12.7546;
            rightSlide.setTargetPosition((int) rightMaxHeight);
            return;
        }
        if (pos <= 0){
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            return;
        }
        leftSlide.setTargetPosition(pos);
        double rightSidePosition = 0.713679*pos-12.7546;
        rightSlide.setTargetPosition((int) rightSidePosition);

    }

    public void changeTargetPosition(int amt){
        if (leftSlide.getTargetPosition() + amt >= 0 && leftSlide.getTargetPosition() + amt <= maxHeight){
            setTargetPosition(leftSlide.getTargetPosition() + amt);
        }
    }

    public void setPower(double p){
        power = p;
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        isActive = (power > 0);
    }

    public double getPower() { return power; }

    public boolean isActive(){
        return isActive;
    }

    public int getTargetPosition(){
        return leftSlide.getTargetPosition();
    }

    public int getLeftPosition(){
        return leftSlide.getCurrentPosition();
    }

    public int getRightPosition(){
        return rightSlide.getCurrentPosition();
    }

    public void resetPosition(){
        setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
