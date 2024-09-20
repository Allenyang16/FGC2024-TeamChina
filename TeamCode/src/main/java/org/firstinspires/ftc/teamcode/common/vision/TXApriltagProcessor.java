package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

public class TXApriltagProcessor extends AprilTagProcessorImpl {


    public TXApriltagProcessor(double fx, double fy, double cx, double cy, DistanceUnit outputUnitsLength,
                               AngleUnit outputUnitsAngle, AprilTagLibrary tagLibrary, boolean drawAxes,
                               boolean drawCube, boolean drawOutline, boolean drawTagID, TagFamily tagFamily,
                               int threads, boolean suppressCalibrationWarnings) {
        super(fx, fy, cx, cy, outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube, drawOutline,
                drawTagID, tagFamily, threads, suppressCalibrationWarnings);

    }






}