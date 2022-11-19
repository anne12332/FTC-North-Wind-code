package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.TensorFlow;

@Autonomous
public class camera_test extends LinearOpMode {
    //TensorFlow camera = new camera1();
    private TensorFlow tf = null;
    @Override
    public void runOpMode() throws InterruptedException {
        tf = new TensorFlow(hardwareMap,  this,  telemetry, "model_unquant.tflite", "labels.txt");
        telemetry.addData("", "ready");
        telemetry.update();
        waitForStart();
        while(!isStopRequested()) {
            tf.detectRingThread();
        }
        /*while(!isStarted()){
            Bitmap bm = camera.getImage();
            //camera.colorAnalyser_pixel(bm);
            camera.colorAnalyser(bm);
        }
        waitForStart();
        while(!isStopRequested()){
            Bitmap bm = camera.getImage();
            //camera.colorAnalyser_pixel(bm);
            camera.colorAnalyser(bm);
        }*/
    }
}
