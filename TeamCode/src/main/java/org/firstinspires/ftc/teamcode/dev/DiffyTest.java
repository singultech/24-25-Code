package org.firstinspires.ftc.teamcode.dev;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Diffy;
import org.firstinspires.ftc.teamcode.subsystems.GamepadPair;

@TeleOp(name = "Diffy Test", group = "test")
public class DiffyTest extends LinearOpMode {

    public static double MANUAL_POWER_MULTIPLIER = 0.3;
    public static int DEGREE_INCREMENT = 15;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadPair gamepads = new GamepadPair(gamepad1, gamepad2);
        Diffy diffy = new Diffy(hardwareMap);
        diffy.setManualMode(true);
        waitForStart();

        while (opModeIsActive()) {
            gamepads.copyStates();


            if(gamepads.isPressed("triangle")) diffy.setManualMode(!diffy.isManualMode());

            if(diffy.isManualMode()) {
                double verticalPower = gamepads.joystickValue(2, "left", "y");
                double rotationPower = -gamepads.joystickValue(2, "right", "x");

                double leftPower = (verticalPower + rotationPower) * MANUAL_POWER_MULTIPLIER;
                double rightPower = (verticalPower - rotationPower) * MANUAL_POWER_MULTIPLIER;

                diffy.setLeftPower(leftPower);
                diffy.setRightPower(rightPower);
            }
            else {
                if(gamepads.isPressed("dpad_right")) diffy.changeTargetRotation(Diffy.Side.LEFT, DEGREE_INCREMENT);
                if(gamepads.isPressed("dpad_left")) diffy.changeTargetRotation(Diffy.Side.LEFT, -DEGREE_INCREMENT);
                if(gamepads.isPressed("circle")) diffy.changeTargetRotation(Diffy.Side.RIGHT, DEGREE_INCREMENT);
                if(gamepads.isPressed("square")) diffy.changeTargetRotation(Diffy.Side.RIGHT, -DEGREE_INCREMENT);
            }

            telemetry.addLine(diffy.log());
            telemetry.addLine(diffy.isManualMode() ? "Manual Mode" : "Auto Mode(Run to position)");
            telemetry.update();
            diffy.update();
        }
    }
}
