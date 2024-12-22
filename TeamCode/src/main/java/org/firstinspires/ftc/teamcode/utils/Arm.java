package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo armServo;
    private final double upPosition;
    private final double downPosition;
    private boolean isUp;

    public Arm(double upPos, double downPos, Servo servo){
        armServo = servo;
        upPosition = upPos;
        downPosition = downPos;
        servo.setDirection(Servo.Direction.REVERSE);
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