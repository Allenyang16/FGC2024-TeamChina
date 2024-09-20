package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//@Config
public class RobotHardware {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor strafeDrive = null;
    private DcMotor frontSlide = null;
    private DcMotor backSlide = null;

    private Servo strafe = null;
    private Servo backDoor = null;
    private Servo frontDoor_left = null;
    private Servo frontDoor_right = null;

    private IMU imu = null;

    private HardwareMap hardwareMap;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final String webCamName = "WebCamFGC";

    private static RobotHardware instance = null;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        return instance;
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotor.class,"strafe_drive");

        frontSlide = hardwareMap.get(DcMotor.class, "frontSlide");
        backSlide = hardwareMap.get(DcMotor.class,"backSlide");
        strafe = hardwareMap.get(Servo.class,"strafe");

        backDoor = hardwareMap.get(Servo.class,"doorBack");
        frontDoor_left = hardwareMap.get(Servo.class,"doorLeft");
        frontDoor_right = hardwareMap.get(Servo.class,"doorRight");
    }

//    public void startCamera() {
//        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(FeedingTheFutureGameDatabase.getFeedingTheFutureTagLibrary())
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, webCamName))
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .addProcessors(aprilTag)
//                .enableLiveView(true)
//                .build();
//
//        visionPortal.setProcessorEnabled(aprilTag, true);
//    }
//
//    public Pose2d getApriltagDetection() {
//        if(aprilTag != null) {
//            List<AprilTagDetection> detections = aprilTag.getDetections();
//            for (AprilTagDetection detection : detections) {
//                if(detection.metadata != null) {
//
//                }
//            }
//            return new Pose2d();
//        }
//        else {
//            return null;
//        }
//    }


}
