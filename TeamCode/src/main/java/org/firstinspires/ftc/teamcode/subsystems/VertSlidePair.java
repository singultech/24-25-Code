package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class VertSlidePair {
    DcMotorEx rightSlide;
    DcMotorEx leftSlide;
    Servo leftHook;
    Servo rightHook;
    private boolean isLeftHookOut;
    private boolean isRightHookOut;
    private boolean isActive;
    private final int maxHeight;
    private double power;

    public VertSlidePair(int maxH, double startingPower, HardwareMap hmap){
        rightSlide = hmap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hmap.get(DcMotorEx.class, "leftSlide");
        rightHook = hmap.get(Servo.class, "rightHook");
        leftHook = hmap.get(Servo.class, "leftHook");
        maxHeight = maxH;
        power = startingPower;
        init();
    }
    public VertSlidePair(int maxH, HardwareMap hmap){
        rightSlide = hmap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hmap.get(DcMotorEx.class, "leftSlide");
        rightHook = hmap.get(Servo.class, "rightHook");
        leftHook = hmap.get(Servo.class, "leftHook");
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
        rightHook.setDirection(Servo.Direction.REVERSE);
        //leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        isLeftHookOut = false;
        isRightHookOut = false;
        raiseHook("l");
        raiseHook("r");
    }

    public void raiseHook(String side){
        if (side.equals("l")) leftHook.setPosition(0.5);
        else rightHook.setPosition(0.5);
    }
    public void lowerHook(String side){
        if (side.equals("l")) leftHook.setPosition(0);
        else rightHook.setPosition(0);
    }
    public boolean isLeftHookOut(){return isLeftHookOut;}
    public boolean isRightHookOut(){return isRightHookOut;}

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

    public void setTargetPosition(String side, int pos){
        if (side.equals("l")){
            if (pos > maxHeight) {
                leftSlide.setTargetPosition(maxHeight);
                return;
            }
            if (pos <= 0){
                leftSlide.setTargetPosition(0);
                return;
            }
            leftSlide.setTargetPosition(pos);
        }
        else{
            if (pos > maxHeight) {
                rightSlide.setTargetPosition(maxHeight);
                return;
            }
            if (pos <= 0){
                rightSlide.setTargetPosition(0);
                return;
            }
            rightSlide.setTargetPosition(pos);
        }
    }

    public void changeTargetPosition(int amt){
        if (leftSlide.getTargetPosition() + amt >= 0 && leftSlide.getTargetPosition() + amt <= maxHeight){
            setTargetPosition(leftSlide.getTargetPosition() + amt);
        }
    }

    public void changeTargetPosition(String side, int amt){
        if (side.equals("l")){
            if (leftSlide.getTargetPosition() + amt >= 0 && leftSlide.getTargetPosition() + amt <= maxHeight){
                setTargetPosition("l",leftSlide.getTargetPosition() + amt);
            }
        }
        else{
            if (rightSlide.getTargetPosition() + amt >= 0 && rightSlide.getTargetPosition() + amt <= maxHeight){
                setTargetPosition("r",rightSlide.getTargetPosition() + amt);
            }
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

    public int getLeftTargetPosition(){
        return leftSlide.getTargetPosition();
    }
    public int getRightTargetPosition(){
        return rightSlide.getTargetPosition();
    }

    public int getAvgCurrentPosition() { return (leftSlide.getCurrentPosition()+rightSlide.getCurrentPosition())/2;}


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
                    continue;
                }

                Thread.sleep(waitTimeMillis);

                setTargetPosition(startPosition);

                while (Math.abs(getLeftPosition()) > 20) {
                    continue;
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