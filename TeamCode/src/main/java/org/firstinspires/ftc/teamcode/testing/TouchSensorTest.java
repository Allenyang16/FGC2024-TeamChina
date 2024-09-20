package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "TouchSensor")
public class TouchSensorTest extends LinearOpMode {
    //private TouchSensor magnetic;
    //private DistanceSensor distanceSensor;
    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private TelemetryPacket packet = new TelemetryPacket();
    private TouchSensor upperMag;
    private TouchSensor lowerMag;



    @Override
    public void runOpMode() throws InterruptedException {
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        upperMag = hardwareMap.get(TouchSensor.class, "upperMagnetic");
        lowerMag = hardwareMap.get(TouchSensor.class, "lowerMagnetic");

        waitForStart();
        while (opModeIsActive()) {
            mTelemetry.addData("Upper Trigger", upperMag.isPressed());
            mTelemetry.addData("Lower Trigger", lowerMag.isPressed());
//            mTelemetry.addData("Is Magnetic Triggerd", touch.isPressed());
//            mTelemetry.addData("Value of Magnetic", touch.getValue());
//            mTelemetry.addData("Distance of Sensor", distanceSensor.getDistance(DistanceUnit.CM));
            mTelemetry.update();
        }
    }
}
