package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.defaultapk.image.ImageUtil;
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

        // unDistortedImg切り抜き
        Mat clippedImage = ImageUtil.clipAR(unDistortedImg);
        api.saveMatImage(clippedImage, "clippedImage.png");

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
