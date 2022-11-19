package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoTerminalBlueLeft extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    RobotNW Robot = new RobotNW();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    DriveTrainMecanum WB = new DriveTrainMecanum();
    static final double FEET_PER_METER = 3.28084;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double y = 6;
    double x = -40;
    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 12;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajectory
            telemetry.addLine("FIRST");
            telemetry.update();
            y = 6;
        }else if(tagOfInterest.id == MIDDLE){
            //trajectory
            telemetry.addLine("SECOND");
            telemetry.update();
            y = 36;
        }else {
            //trajectory
            telemetry.addLine("THIRD");
            telemetry.update();
            y = 56;
        }


        Pose2d position = new Pose2d(x, y);
        Pose2d StartPose = new Pose2d(-60, 36);
        waitForStart();
        drive.setPoseEstimate(StartPose);
        Trajectory myTrajectorySpline = drive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(-60, -68))
                .build();
        drive.followTrajectory(myTrajectorySpline);

        Trajectory myTrajectorySpline2 = drive.trajectoryBuilder(myTrajectorySpline.end())
                .lineTo(new Vector2d(-60, position.getY()))
                .build();
        drive.followTrajectory(myTrajectorySpline2);

        Trajectory myTrajectorySpline3 = drive.trajectoryBuilder(myTrajectorySpline2.end())
                .lineTo(new Vector2d(position.getX(), position.getY()))
                .build();
        drive.followTrajectory(myTrajectorySpline3);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

/*Pose2d position = new Pose2d(x, y);
        Pose2d StartPose = new Pose2d(-60, 36);
        waitForStart();
        drive.setPoseEstimate(StartPose);
        Trajectory myTrajectorySpline = drive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(-60, -68))
                .build();
        drive.followTrajectory(myTrajectorySpline);

        Trajectory myTrajectorySpline2 = drive.trajectoryBuilder(myTrajectorySpline.end())
                .lineTo(new Vector2d(-60, position.getY()))
                .build();
        drive.followTrajectory(myTrajectorySpline2);

        Trajectory myTrajectorySpline3 = drive.trajectoryBuilder(myTrajectorySpline2.end())
                .lineTo(new Vector2d(position.getX(), position.getY()))
                .build();
        drive.followTrajectory(myTrajectorySpline3);*/