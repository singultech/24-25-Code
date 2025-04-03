package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private final Servo armServo;
    private final Servo wristServo;


    private final double ARM_FORWARD_POSITION = 1;
    private final double ARM_BACK_POSITION = 0;
    private final double WRIST_FORWARD_POSITION = 1;
    private final double WRIST_BACK_POSITION = 0;


    private double currentArmPos;
    private double currentWristPos;

    public FrontArm(HardwareMap hmap){
        armServo = hmap.servo.get("frontFlip");
        wristServo = hmap.servo.get("frontWrist");
        armServo.setDirection(Servo.Direction.REVERSE);
    }

    public void armForward(){
        setArmPosition(ARM_FORWARD_POSITION);
    }

    public void armBack(){
        setArmPosition(ARM_BACK_POSITION);
    }

    public boolean isArmForward(){ return (currentArmPos == ARM_FORWARD_POSITION); }

    public boolean isArmBack(){ return (currentArmPos == ARM_BACK_POSITION);}

    public void setArmPosition(double position){
        if(position >= 0 && position <= 1) {
            armServo.setPosition(position);
            currentArmPos = position;
        }
    }
    public double getArmPosition(){return currentArmPos;}

    public void wristForward(){
        setWristPosition(WRIST_FORWARD_POSITION);
    }

    public void wristBack(){
        setWristPosition(WRIST_BACK_POSITION);
    }

    public boolean isWristForward(){ return (currentWristPos == WRIST_FORWARD_POSITION); }

    public boolean isWristBack(){ return (currentWristPos == WRIST_BACK_POSITION);}

    public void setWristPosition(double position){
        if(position >= 0 && position <= 1) {
            wristServo.setPosition(position);
            currentWristPos = position;
        }
    }
    public double getWristPosition(){return currentWristPos;}

    @TeleOp(name = "FrontArmTest")
    public static class FrontArmTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FrontArm frontArm = new FrontArm(hardwareMap);
            GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);

            waitForStart();
            while (!isStopRequested()) {
                gamepads.copyStates();

                if (gamepads.isPressed("up_dpad")) frontArm.armForward();
                if (gamepads.isPressed("down_dpad")) frontArm.armBack();
                if (gamepads.isPressed("right_dpad")) frontArm.wristForward();
                if (gamepads.isPressed("left_dpad")) frontArm.wristBack();

                telemetry.addData("Frontarm position", frontArm.isArmForward() ? "forward" : "back");
                telemetry.addData("Wrist position", frontArm.isWristForward() ? "forward" : "back");
                telemetry.update();
            }
        }
    }


}
