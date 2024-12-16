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
        setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTargetPosition(int pos){
        if (pos > maxHeight) {
            leftSlide.setTargetPosition(maxHeight);
            rightSlide.setTargetPosition(maxHeight);
            return;
        }
        if (pos <= 0){
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            return;
        }
        leftSlide.setTargetPosition(pos);
        rightSlide.setTargetPosition(pos);

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

    public void performTimedMove(int changeOfPosition, long waitTimeMillis) {
        int startPosition = getLeftPosition();
        int positionToGo = startPosition + changeOfPosition;
        new Thread(() -> {
            try {
                setTargetPosition(positionToGo);

                while (Math.abs(getLeftPosition() - positionToGo) > 20) {
                    Thread.sleep(20);
                }

                Thread.sleep(waitTimeMillis);

                setTargetPosition(startPosition);

                while (Math.abs(getLeftPosition()) > 20) {
                    Thread.sleep(20);
                }

            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    public void performTimedMove(int changeOfPosition, int finalPosition, long waitTimeMillis) {
        int startPosition = getLeftPosition();
        int positionToGo = startPosition + changeOfPosition;
        new Thread(() -> {
            try {
                setTargetPosition(positionToGo);

                while (Math.abs(getLeftPosition() - positionToGo) > 20) {
                    Thread.sleep(20);
                }

                Thread.sleep(waitTimeMillis);

                setTargetPosition(finalPosition);

                while (Math.abs(getLeftPosition()) > 20) {
                    Thread.sleep(20);
                }

            } catch (InterruptedException e) {
                // Handle interruption
                Thread.currentThread().interrupt();
            }
        }).start();
    }
}
