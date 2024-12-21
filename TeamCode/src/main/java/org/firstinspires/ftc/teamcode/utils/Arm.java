package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo leftServo;
    private final Servo rightServo;
    private final double upPosition;
    private final double downPosition;
    private boolean isUp;

    public Arm(double upPos, double downPos, Servo lServo, Servo rServo){
        leftServo = hardwareMap.servo.get("leftFlip");
        rightServo = hardwareMap.servo.get("rightFlip");
        rightServo.setDirection(Servo.Direction.REVERSE);
        upPosition = upPos;
        downPosition = downPos;
        isUp = true;
    }

    public void up(){
        leftServo.setPosition(upPosition);
        rightServo.setPosition(upPosition);
        isUp = true;
    }

    public void down(){
        leftServo.setPosition(downPosition);
        rightServo.setPosition(downPosition);
        isUp = false;
    }

    public boolean isUp(){ return  isUp; }




}
