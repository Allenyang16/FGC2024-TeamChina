package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "PIDSecondSlideTest")
@Config
public class PIDSecondSlideTest extends LinearOpMode {
  private final MultipleTelemetry mTelemetry =
      new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
  private static double slideHeightTicks = 0;
  private DcMotorEx mIntakeLeft;
  private DcMotorEx mInakeRight;

  @Override
  public void runOpMode() throws InterruptedException {
    mIntakeLeft = hardwareMap.get(DcMotorEx.class, "intakeLeft");
    mInakeRight = hardwareMap.get(DcMotorEx.class, "intakeRight");

    mIntakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
    mInakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

    mIntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    mInakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    //        mFrontSlide.resetEncoder();
    //        mBackSlide.resetEncoder();
    //
    //        mFrontSlide.setPositionCoefficient(0.05);
    //        mBackSlide.setPositionCoefficient(0.05);
    //
    //        mFrontSlide.setPositionTolerance(10);
    //        mBackSlide.setPositionTolerance(10);
    //
    //        mFrontSlide.setRunMode(Motor.RunMode.PositionControl);
    //        mBackSlide.setRunMode(Motor.RunMode.PositionControl);

    waitForStart();
    while (opModeIsActive()) {

      if(gamepad1.a) {
        mIntakeLeft.setPower(1);
        mInakeRight.setPower(1);
      } else if (gamepad1.b) {
        mIntakeLeft.setPower(-1);
        mInakeRight.setPower(-1);
      }
      else {
        mIntakeLeft.setPower(0);
        mInakeRight.setPower(0);
      }
        //            mFrontSlide.setTargetDistance(slideHeightTicks);
        //            mBackSlide.setTargetDistance(slideHeightTicks);
        //
        //            mFrontSlide.set(0);
        //            mBackSlide.set(0);
        //
        //            if(!mFrontSlide.atTargetPosition()) {
        //                mFrontSlide.set(0.75);
        //            }
        //
        //            if(!mBackSlide.atTargetPosition()) {
        //                mBackSlide.set(0.75);
        //            }

        mTelemetry.update();
    }
  }
}
