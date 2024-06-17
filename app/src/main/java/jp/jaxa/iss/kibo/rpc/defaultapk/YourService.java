package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.math.QuaternionUtil;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Kinematics;

import org.opencv.aruco.Aruco;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import jp.jaxa.iss.kibo.rpc.defaultapk.BoundingBox;
import jp.jaxa.iss.kibo.rpc.defaultapk.Detector;
import java.io.IOException;
import java.io.InputStream;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();
    private Detector detector;

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Start mission!!!");

        // The mission starts.
        api.startMission();

        // Detectorのセットアップ
        try {
            detector = new Detector(getApplicationContext(), "model.tflite", "labels.txt");
            detector.setup();
        } catch (IOException e) {
            Log.e(TAG, "Detector setup failed", e);
            return;
        }

        // 例としてアセットから画像を読み込む
        Bitmap asset_image = loadImageFromAssets("image.png");
        if (asset_image != null) {
            List<BoundingBox> boundingBoxes = detector.detect(asset_image);
            if (boundingBoxes != null) {
                //画像検出の結果を表示
                processDetectionResult(boundingBoxes);
            } else {
                Log.i(TAG, "No objects detected");
            }
        } else {
            Log.e(TAG, "Failed to load image from assets");
        }

        // Take a snapshot of the target item.→ミッション終了
        api.takeTargetItemSnapshot();

    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here
    }

    // You can add your method.
    private String yourMethod() {
        return "your method";
    }

    //アセットから画像を読み込む
    private Bitmap loadImageFromAssets(String fileName) {
        try (InputStream inputStream = getApplicationContext().getAssets().open(fileName)) {
            return BitmapFactory.decodeStream(inputStream);
        } catch (IOException e) {
            Log.e(TAG, "Error loading image from assets: " + fileName, e);
            return null;
        }
    }

    //バウンディングボックスから情報を表示する
    private void processDetectionResult(List<BoundingBox> boundingBoxes) {
        for (BoundingBox box : boundingBoxes) {
            Log.i(TAG, "Detected object: " + box.getClsName() + " with confidence " + box.getCnf());
        }
    }
}
