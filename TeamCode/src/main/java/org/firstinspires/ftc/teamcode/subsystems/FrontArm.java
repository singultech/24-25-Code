package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrontArm {
    private final Servo armServo;
    private final Servo wristServo;
    private Position currentPosition;

    public enum Position {
        GRAB_FROM_WALL(0,0),
        HANG_SPECIMEN(0,0),
        HANG_BASKET(0,0);
        private final double armPosition;
        private final double wristPosition;
        Position(double armPosition, double wristPosition){
            this.armPosition = armPosition;
            this.wristPosition = wristPosition;
        }
        public double getArmPosition(){
            return armPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }

    public FrontArm(HardwareMap hmap){
        armServo = hmap.servo.get("frontArm");
        wristServo = hmap.servo.get("frontWrist");
        armServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(Position position){
        setArmPosition(position.getArmPosition());
        setWristPosition(position.getWristPosition());
        currentPosition = position;
    }

    public Position getPosition(){return currentPosition;}

    private void setArmPosition(double position){
        if(position >= 0 && position <= 1) {
            armServo.setPosition(position);
        }
    }
    private void setWristPosition(double position){
        if(position >= 0 && position <= 1) {
            wristServo.setPosition(position);
        }
    }

    @NonNull
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Front Arm/Wrist Position: %s", getPosition());
    }

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

                if (gamepads.isPressed("up_dpad")) frontArm.setPosition(FrontArm.Position.HANG_SPECIMEN);
                if (gamepads.isPressed("down_dpad")) frontArm.setPosition(Position.GRAB_FROM_WALL);
                if (gamepads.isPressed("right_dpad")) frontArm.setPosition(Position.HANG_BASKET);

                telemetry.addLine(frontArm.toString());
                telemetry.update();
            }
        }
    }


}
