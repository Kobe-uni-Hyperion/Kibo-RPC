package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.math.QuaternionUtil;

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1() {
        Log.i(TAG, "Start mission!!!");

        // The mission starts.
        api.startMission();

        // 最初の座標は(9.815, -9.806, 4.293)
        // Area1への移動(とりあえず(x,y,z_min + x,y,z_max) / 2 , 向きも適当)
        // KIZの中でKOZを避けたい
        // 特徴量の多いルートを通りたい

        Log.i(TAG, "StartLocation!!!!");
        Kinematics startLocation = api.getRobotKinematics();
        Point startLocationPoint = startLocation.getPosition();
        // ↓(10.267100213116851, -9.796862709327995, 4.270871767291918)の結果が出た
        // つまり、Astrobeeはまず最初に勝手にこの座標に移動する。よって、この座標をスタート地点として考慮する必要がある.
        Log.i(TAG, "StartLocationIs: " + startLocationPoint.getX() + ", " + startLocationPoint.getY() + ", " + startLocationPoint.getZ());

        // KIZ2からKIZ1へ移動するために、まずZ軸正方向に移動し、その後、X軸正方向に移動する
        Point kiz2ToKiz1FirstMove = new Point(10.26, -9.806, 4.56);
        // ここでは回転の必要なし
        Quaternion quaternionKiz2ToKiz1FirstMove = QuaternionUtil.rotate(0, 0, 1, 0);
        Result resultMoveToKiz1FirstMove = api.moveTo(kiz2ToKiz1FirstMove, quaternionKiz2ToKiz1FirstMove, true);

        int loopCounterKiz2ToKiz1FirstMove = 0;
        while (!resultMoveToKiz1FirstMove.hasSucceeded() && loopCounterKiz2ToKiz1FirstMove < 5) {
            // retry
            resultMoveToKiz1FirstMove = api.moveTo(kiz2ToKiz1FirstMove, quaternionKiz2ToKiz1FirstMove, true);
            ++loopCounterKiz2ToKiz1FirstMove;
        }

        // 続いて、KIZ1とKIZ2の重なった部分のほぼ中心に移動する。y座標はそのまま、x座標とz座標はKIZ1とKIZ2の重なった部分の中心.
        Point kiz2ToKiz1 = new Point(10.4, -9.806, 4.56);
        // ここでは回転の必要なし
        Quaternion quaternionKiz2ToKiz1 = QuaternionUtil.rotate(0, 0, 1, 0);
        Result resultMoveToEntranceOfKiz1 = api.moveTo(kiz2ToKiz1, quaternionKiz2ToKiz1, true);

        int loopCounterKiz2ToKiz1 = 0;
        while (!resultMoveToEntranceOfKiz1.hasSucceeded() && loopCounterKiz2ToKiz1 < 5) {
            // retry
            resultMoveToEntranceOfKiz1 = api.moveTo(kiz2ToKiz1, quaternionKiz2ToKiz1, true);
            ++loopCounterKiz2ToKiz1;
        }

        Log.i(TAG, "GetIntoKIZ1!!!!");

        // Area1の中心座標
        // Area1の中心は(10.95,−10.58,5.195)
        // とりあえず、Area1の中心から法線ベクトル上にある点に移動する
        // y座標はそのまま、x座標とz座標はArea1の中心から法線ベクトル上にある点
        Point area1FirstViewPoint = new Point(10.95, -9.806, 5.195);
        // z軸負方向を軸として、90度回転
        // 視野: ｘ軸正方向 => y軸負方向
        Quaternion quaternion1 = QuaternionUtil.rotate(0, 0, -1, (float) (Math.PI * 0.5));
        Result resultMoveToArea1 = api.moveTo(area1FirstViewPoint, quaternion1, true);

        final int LOOP_MAX = 5;

        // 結果をチェックし、moveToApiが成功しない間はループする。(外乱に強いプログラム)
        int loopCounter = 0;
        while (!resultMoveToArea1.hasSucceeded() && loopCounter < LOOP_MAX) {
            // retry
            resultMoveToArea1 = api.moveTo(area1FirstViewPoint, quaternion1, true);
            ++loopCounter;
        }

        Log.i(TAG, "InFrontOfArea1!!!!");

        // Get a camera image. NavCam → 画像処理用のカメラ
        Mat image = api.getMatNavCam();

        // imageがnullの場合の処理
        if (image.empty()) {
            Log.e(TAG, "Failed to capture image form NavCam.");
            return;
        }
        api.saveMatImage(image, "file_name.png");

        // ARタグを検知する
        //if (!markerIds.empty()) {
        //Log.i(TAG, "ARtag!!!");
        // カメラ行列の取得
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        // Log.i(TAG,"cameraMatrix is" + cameraMatrix);

        // 歪み係数の取得
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        // Log.i(TAG,"cameraCoefficients is" + cameraCoefficients);

        // 歪みのないimageに変換
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);
        Mat unDistortedImg = new Mat();
        Calib3d.undistort(image, unDistortedImg, cameraMatrix, cameraCoefficients);
        api.saveMatImage(unDistortedImg, "unDistortedImgOfArea1.png");


        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        ArrayList<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        Mat corner = corners.get(0); // 最初のARマーカーのコーナー4点(複数ARマーカーがある場合、最初に検知したもの)
        final org.opencv.core.Point[] points = new org.opencv.core.Point[4];

        for (int i = 0; i < 4; i++) {
            points[i] = new org.opencv.core.Point(corner.get(0, i));
            Log.i(TAG, "points[" + i + "]" + points[i]);
        }


        // y座標が昇順になるように座標4点をソート
        Arrays.sort(points, (p1, p2) -> Double.compare(p1.y, p2.y));
        org.opencv.core.Point leftTop, rightTop, leftBottom, rightBottom;

        // 四角形の上二つの点の左右を決定
        if (points[0].x < points[1].x) {
            leftTop = points[0];
            rightTop = points[1];
        } else {
            leftTop = points[1];
            rightTop = points[0];
        }
        // 四角形の下二つの点の左右を決定
        if (points[2].x < points[3].x) {
            leftBottom = points[2];
            rightBottom = points[3];
        } else {
            leftBottom = points[3];
            rightBottom = points[2];
        }
        Log.i(TAG, "leftTop, rightTop, leftBottom, rightBottom are " + leftTop + rightTop + leftBottom + rightBottom);

        double width = Math.sqrt(Math.pow(leftTop.x - rightTop.x, 2) + Math.pow(leftTop.y - rightTop.y, 2));
        double height = Math.sqrt(Math.pow(leftTop.x - leftBottom.x, 2) + Math.pow(leftTop.y - leftBottom.y, 2));

        Log.i(TAG, "width is " + width + "height is " + height);
        // ここでコーナーの順序を調整
        points[0] = leftTop;
        points[1] = rightTop;
        points[2] = rightBottom;
        points[3] = leftBottom;

        Mat transformMatrix;
        {
            MatOfPoint2f srcPoints = new MatOfPoint2f(points);
            srcPoints.convertTo(srcPoints, CvType.CV_32F);

            MatOfPoint2f dstPoints = new MatOfPoint2f(
                    new org.opencv.core.Point(0, 0),
                    new org.opencv.core.Point(width - 1, 0),
                    new org.opencv.core.Point(width - 1, height - 1),
                    new org.opencv.core.Point(0, height - 1)
            );
            dstPoints.convertTo(dstPoints, CvType.CV_32F);

            // 変換前の座標と変換後の座標から透視変換行列(切り抜きたい領域を長方形に変換するための行列)を作る
            transformMatrix = Imgproc.getPerspectiveTransform(srcPoints, dstPoints);
        }

        Log.i(TAG, "transformMatrix is" + transformMatrix);
        // transformMatrix の値をログに出力
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                double value = transformMatrix.get(row, col)[0];
                Log.i(TAG, "transformMatrix[" + row + "][" + col + "] = " + value);
            }
        }

        Mat clippedImage = Mat.zeros((int) width, (int) height, image.type());
        //Mat clippedImage = new Mat((int) width, (int) height, unDistortedImg.type());
        //Mat clippedImage = new Mat();

        Imgproc.warpPerspective(unDistortedImg, clippedImage, transformMatrix, clippedImage.size());
        api.saveMatImage(clippedImage, "clippedImage.png");


        // unDistortedImg切り抜き
        //Mat clippedImage = ImageUtil.clipAR(unDistortedImg);
        //if (clippedImage != null && !clippedImage.empty()) {
        //  api.saveMatImage(clippedImage, "clippedImage.png");
        //} else {
        //  Log.i(TAG, "clippedImage = null or clippedImage is empty");
        //}

        // } else{
        //     Log.i(TAG, "No AR markers detected");
        // }

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
