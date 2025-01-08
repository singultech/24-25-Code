package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final double upPosition;
    private final double downPosition;
    private boolean isUp;
    AnalogInput leftEncoder;
    AnalogInput rightEncoder;

    private double currentAngle = 0.0;
    private double previousAngle = 0.0;
    private double totalRotation = 0.0;

    public Arm(double upPos, double downPos, HardwareMap hmap){
        leftServo = hmap.crservo.get("leftFlip");
        rightServo = hmap.crservo.get("rightFlip");
        leftEncoder = hmap.get(AnalogInput.class, "leftArmEncoder");
        rightEncoder = hmap.get(AnalogInput.class, "rightArmEncoder");
        upPosition = upPos;
        downPosition = downPos;
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        isUp = true;
    }

    public void up(){
        leftServo.setPower(1);
        isUp = true;
    }

    public void down(){
        leftServo.setPower(0);
        isUp = false;
    }

    public boolean isUp(){ return  isUp; }




}