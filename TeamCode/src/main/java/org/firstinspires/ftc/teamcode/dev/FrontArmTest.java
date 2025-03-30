package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.FrontArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "FrontArmTest")
public class FrontArmTest extends LinearOpMode {
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