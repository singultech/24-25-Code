package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "RTP Diffy&Arm Test", group = "Dev")
public class RTPDiffyTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap, true);
        BackArm arm = new BackArm(hardwareMap, true);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();
            diffy.update();
            arm.update();

            if (gamepads.isPressed(-1, "dpad_up")){
                diffy.setLeftTargetRotation(diffy.getLeftTargetRotation()+5);
            }
            if (gamepads.isPressed(-1, "dpad_down")){
                diffy.setLeftTargetRotation(diffy.getLeftTargetRotation()-5);
            }
            if (gamepads.isPressed(-1, "triangle")){
                diffy.setRightTargetRotation(diffy.getRightTargetRotation()+5);
            }
            if (gamepads.isPressed(-1, "cross")){
                diffy.setRightTargetRotation(diffy.getRightTargetRotation()-5);
            }
            if (gamepads.isPressed(-1, "dpad_left")){
                arm.setTargetRotation(arm.getTargetRotation()-15);
            }
            if (gamepads.isPressed(-1, "dpad_right")){
                arm.setTargetRotation(arm.getTargetRotation()+15);
            }
            if (gamepads.isPressed(-1, "left_bumper")){
                diffy.rotateGrabber(-10);
            }
            if (gamepads.isPressed(-1, "right_bumper")){
                diffy.rotateGrabber(10);
            }


            telemetry.addData("Left Target", diffy.getLeftTargetRotation());
            telemetry.addData("Left Current", diffy.getLeftTotalRotation());
            telemetry.addData("Right Target", diffy.getRightTargetRotation());
            telemetry.addData("Right Current", diffy.getRightTotalRotation());
            telemetry.addData("Arm Target", arm.getTargetRotation());
            telemetry.addData("Arm Current", arm.getPosition());
            telemetry.update();
        }
    }
}
