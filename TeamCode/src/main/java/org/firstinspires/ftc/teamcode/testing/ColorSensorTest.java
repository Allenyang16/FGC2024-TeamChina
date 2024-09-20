package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "ColorSensor")
public class ColorSensorTest extends LinearOpMode {
    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private TelemetryPacket packet = new TelemetryPacket();
    private ColorSensor colorSensor ; //Either of the color is greater than 1000, then it is on the intake position



    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");

        waitForStart();
        while (opModeIsActive()) {
            mTelemetry.addData("RGB-red",colorSensor.red());
            mTelemetry.addData("RGB-blue",colorSensor.blue());
            mTelemetry.addData("RGB-green",colorSensor.green());
            mTelemetry.update();
        }
    }
}
