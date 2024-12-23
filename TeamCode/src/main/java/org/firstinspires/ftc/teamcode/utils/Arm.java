package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo armServo;
    private final double upPosition;
    private final double downPosition;
    private boolean isUp;

    public Arm(double upPos, double downPos, HardwareMap hmap){
        armServo = hmap.servo.get("rightFlip");;
        upPosition = upPos;
        downPosition = downPos;
        armServo.setDirection(Servo.Direction.REVERSE);
        isUp = true;
    }

    public void up(){
        armServo.setPosition(upPosition);
        isUp = true;
    }

    public void down(){
        armServo.setPosition(downPosition);
        isUp = false;
    }

    public boolean isUp(){ return  isUp; }




}