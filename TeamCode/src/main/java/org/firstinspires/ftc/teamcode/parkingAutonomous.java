package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous
public class parkingAutonomous extends LinearOpMode {
    RobotConfig robot = new RobotConfig();
    final double blockWidth = 8;
    SampleMecanumDriveREVOptimized drive;

    int color = 1;
    int side = 1;

    public void runOpMode() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        while (!isStarted()) { //use this for switching between red and blue sides
            if (isStopRequested())
                return;

            if (gamepad1.a)
                color = 1;
            if (gamepad1.b)
                color = -1;
            if (gamepad1.x)
                side = 1;
            if (gamepad1.y)
                side = -1;

            telemetry.addData("color", color == 1 ? "red" : "blue");
            telemetry.addData("side", side == 1 ? "left" : "right");
            telemetry.update();
        }


        waitForStart();
        if (isStopRequested())
            return;



//        if (color == 1) {
//            drive.setPoseEstimate(new Pose2d(-63.5, -39, 0));
//        } else {
//           drive.setPoseEstimate(new Pose2d(63.5, -39, 0));
//        }

        drive.setPoseEstimate(new Pose2d(-63.5 * color, -39, .5 * Math.PI - Math.PI * color));
        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(color * -48 - 12 * side, 0, Math.PI/2)).build());



    }
}
