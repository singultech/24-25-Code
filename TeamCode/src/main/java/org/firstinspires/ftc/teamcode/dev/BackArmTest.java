package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BackArm;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Back Arm Test", group = "test")
public class BackArmTest extends LinearOpMode {

    public static int DEGREE_INCREMENT = 15;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BackArm backArm = new BackArm(hardwareMap);
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        boolean manualMode = false;
        double maxPower = 0.5;
        
        waitForStart();
        while (!isStopRequested()) {
            gamepads.copyStates();
            backArm.update();

            if(gamepads.isPressed("triangle")){
                manualMode = !manualMode;
                backArm.setManualMode(manualMode);
            }

            if(manualMode) {
                if (gamepads.isHeld(-1,"left_dpad")) backArm.setManualPower(-maxPower);
                else if (gamepads.isHeld(-1, "right_dpad")) backArm.setManualPower(maxPower);
                else backArm.setManualPower(0);
            } else {
                if (gamepads.isPressed("up_dpad")) backArm.changeTargetRotation(DEGREE_INCREMENT);
                if (gamepads.isPressed("down_dpad")) backArm.changeTargetRotation(-DEGREE_INCREMENT);
            }

            telemetry.addLine(backArm.log());
            telemetry.update();
        }
    }
}