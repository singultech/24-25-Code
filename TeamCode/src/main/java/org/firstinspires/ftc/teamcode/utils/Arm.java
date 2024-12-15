package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private final Servo leftServo;
    private final Servo rightServo;

    public Arm(){
        leftServo = hardwareMap.servo.get("leftFlip");
        rightServo = hardwareMap.servo.get("rightFlip");
        rightServo.setDirection(Servo.Direction.REVERSE);
    }


}
