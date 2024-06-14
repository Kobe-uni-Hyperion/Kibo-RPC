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

/**
 * Class meant to handle commands from the Ground Data System and execute them
 * in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    public static boolean isInKOZ(double x, double z, double[][] koz){
        double largeX = Math.max(koz[0][0], koz[1][0]);
        double smallX = Math.min(koz[0][0], koz[1][0]);
        double largeZ = Math.max(koz[0][2], koz[1][2]);
        double smallZ = Math.min(koz[0][2], koz[1][2]);

        return smallX <= x && x <= largeX && smallZ <= z && z <= largeZ;
    }

    public static double calculateDistance(double[] point1, double[] point2) {
        return Math.sqrt(Math.pow(point1[0] - point2[0], 2)
                       + Math.pow(point1[1] - point2[1], 2)
                       + Math.pow(point1[2] - point2[2], 2));
    }

    @Override
    protected void runPlan1() {
        double kiz1XMin = 10.3;
        double kiz1XMax = 11.55;
        double kiz1ZMin = 4.32;
        double kiz1ZMax = 5.57;

        // 20cm余分に取っている
        double[][] koz1Position1 = {{10.67, -9.5, 4.07}, {11.8, -9.45, 5.17}};
        double[][] koz1Position2 = {{10.05, -9.5, 4.77}, {11.07, -9.45, 5.82}};

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

        // imageがnullの場合の対処を書く

        api.saveMatImage(image, "area1.png");

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
        Mat unDistortedImg = new Mat();
        Calib3d.undistort(image, unDistortedImg, cameraMatrix, cameraCoefficients);

        api.saveMatImage(unDistortedImg, "unDistortedImgOfArea1.png");

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

        // 現在の座標を取得
        Kinematics kinematicsAfterArea1 = api.getRobotKinematics();
        Point pointAfterArea1 = kinematicsAfterArea1.getPosition();

        double minDistanceToArea2 = Double.MAX_VALUE;
        double[] bestPointToGoThroughKOZ1 = new double[3];

        // 0.05刻みでpointInFrontOfKOZ1を探索する
        for (double x = kiz1XMin + 0.2; x <= kiz1XMax - 0.2 ; x += 0.01) {
            for(double z = kiz1ZMin + 0.2; z <= kiz1ZMax - 0.2; z += 0.01){
                // KOZを避けているか確認
                if (isInKOZ(x, z, koz1Position1) || isInKOZ(x, z, koz1Position2)) {
                    continue;
                }

                double[] point1InFrontOfKOZ1 = {x, -9.7, z};
                double[] point2InFrontOfKOZ1 = {x, -9.25, z};
                double[] pointInFrontOfArea2 = {10.925, -8.875, 4.26203};

                // 距離の計算
                double distance = calculateDistance(new double[]{pointAfterArea1.getX(), pointAfterArea1.getY(), pointAfterArea1.getZ()}, point1InFrontOfKOZ1) + calculateDistance(point2InFrontOfKOZ1, pointInFrontOfArea2);

                // 最短距離を更新
                if (distance < minDistanceToArea2) {
                    minDistanceToArea2 = distance;
                    bestPointToGoThroughKOZ1 = point1InFrontOfKOZ1;
                }
            }
        }

        /**
         * KOZ1の前まで行く
         */
        // 得られた最短距離の座標に移動
        Point point1ToGoThroughKOZ1 = new Point(bestPointToGoThroughKOZ1[0], -9.7, bestPointToGoThroughKOZ1[2]);
        // z軸負方向を軸として、-90度回転
        Quaternion quaternion1WhenKOZ1 = QuaternionUtil.rotate(0, 0, -1, (float) ((-0.5) * Math.PI));
        Result result1MoveToKOZ1 = api.moveTo(point1ToGoThroughKOZ1, quaternion1WhenKOZ1, true);

        int loopCounter1KOZ1 = 0;
        while (!result1MoveToKOZ1.hasSucceeded() && loopCounter1KOZ1 < 5) {
            // retry
            result1MoveToKOZ1 = api.moveTo(point1ToGoThroughKOZ1, quaternion1WhenKOZ1, true);
            ++loopCounter1KOZ1;
        }

        Log.i(TAG, "InFrontOfKOZ1!!!!");

        /**
         * KOZ1を通過する
         */
        Point point2ToGoThroughKOZ1 = new Point(bestPointToGoThroughKOZ1[0], -9.25, bestPointToGoThroughKOZ1[2]);
        // z軸負方向を軸として、-90度回転
        Quaternion quaternion2WhenKOZ1 = QuaternionUtil.rotate(0, 0, -1, (float) ((-0.5) * Math.PI));
        Result result2MoveToKOZ1 = api.moveTo(point2ToGoThroughKOZ1, quaternion2WhenKOZ1, true);

        int loopCounter2KOZ1 = 0;
        while (!result2MoveToKOZ1.hasSucceeded() && loopCounter2KOZ1 < 5) {
            // retry
            result2MoveToKOZ1 = api.moveTo(point2ToGoThroughKOZ1, quaternion2WhenKOZ1, true);
            ++loopCounter2KOZ1;
        }

        Log.i(TAG, "GoneThroughKOZ1!!!!");

        /**
         * Area2に移動する
         */
        Point pointInFrontOfArea2 = new Point(10.925, -8.875, 4.26203);
        // y軸正方向を軸として、90度回転
        // 視野: z軸負方向へ変わる
        Quaternion quaternionInFrontOfArea2 = QuaternionUtil.rotate(0, 1, 0, (float) (0.5 * Math.PI));
        Result resultMoveToArea2 = api.moveTo(pointInFrontOfArea2, quaternionInFrontOfArea2, true);

        int loopCounterArea2 = 0;
        while (!resultMoveToArea2.hasSucceeded() && loopCounterArea2 < 5) {
            // retry
            resultMoveToArea2 = api.moveTo(pointInFrontOfArea2, quaternionInFrontOfArea2, true);
            ++loopCounterArea2;
        }

        Log.i(TAG, "InFrontOfArea2!!!!");

        Mat image2 = api.getMatNavCam();

        // image2がnullの場合の対処を書く

        api.saveMatImage(image2, "area2.png");

        /* *********************************************************************** */
        /* 各エリアにあるアイテムの種類と数を認識するコード */
        /* *********************************************************************** */

        // ARタグを検知する
        Dictionary dictionary2 = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners2 = new ArrayList<>();
        Mat markerIds2 = new Mat();
        Aruco.detectMarkers(image2, dictionary2, corners2, markerIds2);

        // カメラ行列の取得
        Mat cameraMatrix2 = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix2.put(0, 0, api.getNavCamIntrinsics()[0]);
        // 歪み係数の取得
        Mat cameraCoefficients2 = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients2.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients2.convertTo(cameraCoefficients2, CvType.CV_64F);

        // 歪みのないimage
        Mat unDistortedImg2 = new Mat();
        Calib3d.undistort(image2, unDistortedImg2, cameraMatrix2, cameraCoefficients2);

        api.saveMatImage(unDistortedImg2, "unDistortedImgOfArea2.png");

        // ARタグからカメラまでの距離と傾きを求めて、
        // 撮影した画像での座標に変換して画像用紙の部分だけを切り抜く

        // 画像認識

        // AreaとItemの紐付け
        // setAreaInfo(areaId,item_name,item_number)
        api.setAreaInfo(1, "item_name", 1);


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
