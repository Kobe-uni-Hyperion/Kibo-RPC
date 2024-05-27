package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.aruco.dictionary;
import org.opencv.calib3d.Calib3d;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){
        Log.i(TAG. msg:"Start mission!!!");

        // The mission starts.
        api.startMission();

        //Area1への移動(とりあえず(x,y,z_min + x,y,z_max) / 2 , 向きも適当)
        //KIZの中でKOZを避けたい
        //特徴量の多いルートを通りたい

        Point point1 = new Point(10.95, -10.58, 5.195);
        Quaternion quaternion1 = new Quaternion(0f, 0f, -0.707f, 0.707f);
        Result result1 = api.moveTo(point1, quaternion1, false);

        final int LOOP_MAX = 5;

        //結果をチェックし、moveToapiが成功しない間はループする。(外乱に強いプログラム)
        int loopCounter = 0;
        while(!result1.hasSucceeded() && LoopCounter < LOOP_MAX){
            //retry
            result1 = api.moveTo(point1,quaternion1,true);
            ++loopCounter;
        }

        // Get a camera image. NavCam → 画像処理用のカメラ 
        Mat image = api.getMatNavCam();
        
        if(image == null){

            //error hundring

        }else{

            String imageStr = yourMethod(image);

        }

        api.saveMatImage(image, imageName:"file_name.png")

        /* *********************************************************************** */
        /* 各エリアにあるアイテムの種類と数を認識するコード*/
        /* *********************************************************************** */

        // ARタグを検知する
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat  markerIds = new Mat()
        Aruco.detectMarkers(image, dictionary, corners, markerIds);

        //カメラ行列の取得
        Mat cameraMatrix = new mat(rows: 3, cols: 3, CvType.CV_64F);
        cameraMatrix.put(row: 0, col: 0, api.getNavCamIntrinsics()[0]);
        //歪み係数の取得
        Mat cameraCoefficients = new Mat(rows: 1, cols: 5, CvType.CV_64F);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        //歪みのないimage
        Mat undisortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);





        // When you recognize items, let’s set the type and number.
        api.setAreaInfo(1, "item_name", 1);

        /* **************************************************** */
        /* Let's move to the each area and recognize the items. */
        /* **************************************************** */

        //point2に移動して画像認識するコード

        //point3に移動する画像認識するコード

        //point4に移動する画像認識するコード

        //宇宙飛行士の前に移動するコード

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which item the astronaut has. */
        /* ********************************************************** */

        // 宇宙飛行士の持つTargetItemを認識したことを通知
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.→ミッション終了
        api.takeTargetItemSnapshot();

    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method.
    private String yourMethod(){
        return "your method";
    }
}

