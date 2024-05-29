package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.ArrayList;
import java.util.List;
import java.io.IOException;
import java.io.InputStream;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.android.Utils;
import org.opencv.imgproc.Imgproc;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        // Log.i(TAG,"Start mission!!!");

        // The mission starts.
        api.startMission();

        // Area1への移動(とりあえず(x,y,z_min + x,y,z_max) / 2 , 向きも適当)
        // KIZの中でKOZを避けたい
        // 特徴量の多いルートを通りたい

        // Area1の中心座標
        Point area1CenterPoint = new Point(10.95, -10.58, 5.195);
        Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Result result1 = api.moveTo(area1CenterPoint, quaternion1, true);

        final int LOOP_MAX = 5;

        // 結果をチェックし、moveToapiが成功しない間はループする。(外乱に強いプログラム)
        int loopCounter = 0;
        while (!result1.hasSucceeded() && loopCounter < LOOP_MAX) {
            // retry
            result1 = api.moveTo(area1CenterPoint, quaternion1, true);
            ++loopCounter;
        }

        // Get a camera image. NavCam → 画像処理用のカメラ
        Mat image = api.getMatNavCam();

        // imageがnullの場合の対処を書く

        api.saveMatImage(image, "file_name.png");

        /* *********************************************************************** */
        /* 各エリアにあるアイテムの種類と数を認識するコード */
        /* *********************************************************************** */

        // ARタグを検知する
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        // カメラ行列の取得
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        // 歪み係数の取得
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        // 歪みのないimage
        Mat undisortImg = new Mat();
        Calib3d.undistort(image, undisortImg, cameraMatrix, cameraCoefficients);

        // ARタグからカメラまでの距離と傾きを求めて、
        // 撮影した画像での座標に変換して画像用紙の部分だけを切り抜く

        // 画像認識

        // AreaとItemの紐付け
        // setAreaInfo(areaId,item_name,item_number)
        api.setAreaInfo(1, "item_name", 1);

        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

        // point2に移動して画像認識するコード

        // point3に移動して画像認識するコード

        // point4に移動して画像認識するコード

        // 宇宙飛行士の前に移動するコード

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which item the astronaut has. */
        /* ********************************************************** */

        // 宇宙飛行士の持つTargetItemを認識したことを通知
        api.notifyRecognitionItem();

        /*
         * *****************************************************************************
         * **************************
         */
        /*
         * Write your code to move Astrobee to the location of the target item (what the
         * astronaut is looking for)
         */
        /*
         * *****************************************************************************
         * **************************
         */

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
}
