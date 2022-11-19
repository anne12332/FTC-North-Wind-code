package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Math.min;
import static java.lang.Math.max;

import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.Color;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.VuforiaWebcam;

import java.util.ArrayList;
import java.util.List;

public class camera1 {
    private LinearOpMode lop;
    private VuforiaLocalizerImpl vuforia;
    private VuforiaTrackables targets   = null;
    private static final float mmPerInch = 25.4f;
    private List<VuforiaTrackable> allTrackables;
    private OpenGLMatrix lastLocation   = null;

    public void init(LinearOpMode LOP) {
        lop = LOP;
        int cameraMonitorViewId = lop.hardwareMap.appContext.getResources().getIdentifier(/*"cameraMonitorViewId"*/"Webcam 1", "id", lop.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AaF9H1n/////AAABmabQscXArEAiiWNlpiWUdjJvARlxhvHwuotHcq1BmBZUii5ot+91xY1cYV9EcTvQGa5mYOzOewWVwrpVK4fMrQo+TK6qNGDKcUXjzHpNhiPE3rVkmZChsNIJd7Ed+ABuYIacAYh8DBkr8idnsPh0V2AukxW+u1Leqgvos9FEFT6x8STzJjeIuZW/b9Bp93TPvrd1eG0YNTwMcwU0qjHzafyzRnrRpoexT9o5AVznqkdAo+t36/E6VUrfTU0MVH47zI84euLah7GKazGCSUme9p6tCq3fPgLbQh0F8WZH6KUYQzoRhOWXSVbbv1YetAIx0Yb/4EJqtwCJzjnIjBrGEq5FRVXRiYEsyxyGJgUHmOD7";
        parameters.useExtendedTracking = false;

        vuforia = new VuforiaLocalizerImpl(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        CameraDevice.getInstance().setFlashTorchMode(true);
        // CameraDevice.getInstance().setFocusMode(CameraDevice.FOCUS_MODE.FOCUS_MODE_NORMAL);

        targets = this.vuforia.loadTrackablesFromAsset("team-sleeve");
        // targets = this.vuforia.loadTrackablesFromFile()

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        identifyTarget(0, "yellow_triangle");
        identifyTarget(1, "green_sq");
        identifyTarget(2, "cian_circle");

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT);

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        targets.activate();
    }

    void    identifyTarget(int targetIndex, String targetName) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
    }

    public Bitmap getImage() throws InterruptedException {
        Image img;
        // get current frame and transform it to Bitmap
        img = getImagefromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        Bitmap bm_img = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm_img.copyPixelsFromBuffer(img.getPixels());

        return bm_img;
    }

    @Nullable
    private Image getImagefromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {
        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }
        }

        return null;
    }

    @SuppressLint("DefaultLocale")
    public int colorAnalyser(Bitmap bm_img) /*throws InterruptedException*/ {
        //Bitmap bm_img = getImage();
        vec3 cen = new vec3(0);
        int n = 0;
        final int cnt = 20;

        for (VuforiaTrackable trackable : allTrackables) {
            for (int i = 0; i < cnt; i++) {
                OpenGLMatrix robotLocationTransform =
                        ((VuforiaTrackableDefaultListener) trackable.getListener()).getFtcCameraFromTarget();

                if (robotLocationTransform == null)
                    continue;

                // MC.debug("Visible Target", trackable.getName());

                // OpenGLMatrix robotLocationTransform =
                // ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                lastLocation = robotLocationTransform;

                VectorF translation = lastLocation.getTranslation();

                cen.add(translation.get(0),
                        translation.get(1),
                        translation.get(2));

                n++;
                // express the rotation of the robot in degrees.
                // Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                // MC.debug("Rot (deg)", String.format("{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle));
            }
        }

        if (n != 0)
            cen.set(cen.X / n,
                    cen.Y / n,
                    cen.Z / n);

        lop.telemetry.addData("Pos (inches)", String.format("{X, Y, Z} = %.1f, %.1f, %.1f | Expected: %d",
                cen.X / mmPerInch,
                cen.Y / mmPerInch,
                cen.Z / mmPerInch,
                cen.X / mmPerInch < -5 ? 1 : (cen.X / mmPerInch < 6 ? 2 : 3)));
        lop.telemetry.update();

        if (cen.X == 0 && cen.Y == 0 && cen.Z == 0) return 3;
        else return cen.X / mmPerInch > 6 ? 3 : (cen.X / mmPerInch < -5 ? 1 : 2);
//        int y_pixels_counter[] = new int[3]; // color counters
//        int cur_color_int;
//        vec3 rgb = new vec3(0);
//        vec3 hsv = new vec3(0);
//        vec3 yellow = new vec3(200, 200, 0);
//
//        int width = bm_img.getWidth(); // width in landscape mode
//        int height = bm_img.getHeight(); // height in landscape mode
//
//        CameraDevice.getInstance().setFlashTorchMode(false);
//
//        int pos = 3, max_yellow_counter = -1;
//
//        for (int part = 0; part < 3; part++)
//            for (int i = 0, di = 8; i < height; i += di)
//                for (int j = width * part / 3, dj; j < width * (part + 1) / 3; j += dj) {
//                    cur_color_int = bm_img.getPixel(j, i);
//                    rgb.set(Color.red(cur_color_int), Color.green(cur_color_int), Color.blue(cur_color_int)); // 203 103 3
//                    rgb_to_hsv(rgb, hsv);
//                    /*
//                    if (Cone(yellow, 0.25, rgb)) {
//                        y_pixels_counter[part]++;//252 90 3
//                        di = dj = 2;
//                    }*/
//                    if (Math.abs(hsv.X - 60) < 15 && hsv.Y > 0.5 && hsv.Z > 0.5) {
//                        y_pixels_counter[part]++;//252 90 3
//                        di = dj = 2;
//                    }
//                    else
//                        di = dj = 8;
//
//                    if (max_yellow_counter < y_pixels_counter[part]) {
//                        pos = 3 - part;
//                        max_yellow_counter = y_pixels_counter[part];
//                    }
//                }
//
//        MC.log("POSITION", pos);
//        MC.log("left", y_pixels_counter[2]);
//        MC.log("center", y_pixels_counter[1]);
//        MC.lognl("right", y_pixels_counter[0]);
//
//        return pos;
    }

    @SuppressLint("DefaultLocale")
    public int colorAnalyser_pixel(Bitmap bm_img) /*throws InterruptedException*/ {
        //Bitmap bm_img = getImage();
        int yellow_cyan_magenta[] = new int[5]; // color counters
        int cur_color_int;
        vec3 rgb = new vec3(0);
        //vec3 hsv = new vec3(0);

        int width = bm_img.getWidth(); // width in landscape mode
        int height = bm_img.getHeight(); // height in landscape mode

        CameraDevice.getInstance().setFlashTorchMode(false);

        int pos = 1, max_pixel_counter = -1;
        cur_color_int = bm_img.getPixel(width / 2, height / 2);
        rgb.set(Color.red(cur_color_int), Color.green(cur_color_int), Color.blue(cur_color_int)); // 203 103 3
        lop.telemetry.addData("Pixel", String.format("{X, Y, Z} = %.1f, %.1f, %.1f",
                rgb.X,
                rgb.Y,
                rgb.Z));
        for (int i = 0, di = 8; i < height; i += di) {
            for (int j = 0, dj = 1; j < width; j += dj) {
                cur_color_int = bm_img.getPixel(j, i);
                rgb.set(Color.red(cur_color_int), Color.green(cur_color_int), Color.blue(cur_color_int)); // 203 103 3
                //rgb_to_hsv(rgb, hsv);
                /*
                if (Cone(yellow, 0.25, rgb)) {
                    y_pixels_counter[part]++;//252 90 3
                    di = dj = 2;
                }*/
                /*if (Math.abs(hsv.X - 60) < 15 && hsv.Y > 0.5 && hsv.Z > 0.5) {
                    y_pixels_counter[part]++;//252 90 3
                    di = dj = 2;
                }
                else
                    di = dj = 8;

                if (max_yellow_counter < y_pixels_counter[part]) {
                    pos = 3 - part;
                    max_yellow_counter = y_pixels_counter[part];
                }*/
                if ((rgb.X / rgb.Y > 0.75 && rgb.Y / rgb.X > 0.75) && max(rgb.Y, rgb.X) > 130 && rgb.Z < 100) {
                    yellow_cyan_magenta[0]++;
                }
                if ((rgb.Z / rgb.Y > 0.75 && rgb.Y / rgb.Z > 0.75) && max(rgb.Y, rgb.Z) > 80 && rgb.X < 30) {
                    yellow_cyan_magenta[1]++;
                }
                if ((rgb.Z / rgb.X > 0.75 && rgb.X / rgb.Z > 0.75) && (rgb.X / rgb.Y < 0.75 || rgb.Y / rgb.X < 0.75) && max(rgb.X, rgb.Z) > 100) {
                    yellow_cyan_magenta[2]++;
                }
                if ((rgb.Y / rgb.X < 0.75 || rgb.X / rgb.Y < 0.75) && (rgb.Y / rgb.Z < 0.75 && rgb.Z / rgb.Y < 0.75) && rgb.Y > 70) {
                    yellow_cyan_magenta[3]++;
                }
                if ((rgb.Z / rgb.X > 0.75 && rgb.X / rgb.Z > 0.75) && (rgb.Y / rgb.X > 0.75 && rgb.X / rgb.Y > 0.75) && rgb.max() > 150) {
                    yellow_cyan_magenta[4]++;
                }
            }
        }

        max_pixel_counter = max(yellow_cyan_magenta[0], max(yellow_cyan_magenta[1], yellow_cyan_magenta[2]));
        if (yellow_cyan_magenta[1] == max_pixel_counter) pos = 2;
        if (yellow_cyan_magenta[2] == max_pixel_counter) pos = 3;

        lop.telemetry.addData("POSITION", pos);
        lop.telemetry.addData("yellow", yellow_cyan_magenta[0]);
        lop.telemetry.addData("cyan", yellow_cyan_magenta[1]);
        lop.telemetry.addData("magenta", yellow_cyan_magenta[2]);
        lop.telemetry.addData("green", yellow_cyan_magenta[3]);
        lop.telemetry.addData("white", yellow_cyan_magenta[4]);
        lop.telemetry.update();

        return pos;
    }

    public boolean Cone(vec3 or, double alpha, vec3 er) {
        return (alpha > Math.acos((or.X * er.X + or.Y * er.Y + or.Z * er.Z) / or.len() / er.len()));
    }

    public void rgb_to_hsv(vec3 rgb, vec3 hsv){
        /*if ((hsv.Z = rgb.max()) == 0) {
            hsv.set(0);
            return;
        }
        hsv.Y = hsv.Z - rgb.min();
        if (hsv.Z == rgb.X)
            hsv.X = (((rgb.Y - rgb.Z) / hsv.Z) % 6) * 60;
        else if (hsv.Z == rgb.Y)
            hsv.X = (((rgb.Z - rgb.X) / hsv.Z) + 2) * 60;
        else
            hsv.X = (((rgb.X - rgb.Y) / hsv.Z) + 4) * 60;
        hsv.Y /= hsv.Z;
        hsv.Z /= 255;*/
        rgb.set(rgb.mul(1 / 255.));
        double min_val = rgb.min(), max_val = rgb.max(), r = rgb.X, g = rgb.Y, b = rgb.Z;
        if (min_val == max_val)
            hsv.X = 0;
        if (max_val == r && g >= b)
            hsv.X = ((g - b) / (max_val - min_val)) * 60;
        else if (max_val == r && g < b)
            hsv.X = ((g - b) / (max_val - min_val)) * 60 + 360;
        else if (max_val == g)
            hsv.X = ((b - r) / (max_val - min_val)) * 60 + 120;
        else
            hsv.X = ((r - g) / (max_val - min_val)) * 60 + 240;

        if (max_val == 0) hsv.Y = 0;
        else hsv.Y = 1 - min_val / max_val;
        hsv.Z = max_val;
    }
}
