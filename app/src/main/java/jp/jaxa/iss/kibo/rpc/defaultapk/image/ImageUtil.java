package jp.jaxa.iss.kibo.rpc.defaultapk.image;

import org.jetbrains.annotations.Nullable;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;

public class ImageUtil {
    public static Mat clip(Mat image, Mat corner) {

        final org.opencv.core.Point[] points = new org.opencv.core.Point[4];

        for (int i = 0; i < 4; i++) {
            points[i] = new org.opencv.core.Point(corner.get(i, 0));
        }

        // y座標が降順になるように座標4点をソート
        Arrays.sort(points, (p1, p2) -> Double.compare(p2.y, p1.y));
        Point leftTop, rightTop, leftBottom, rightBottom;

        // 四角形の上二つの左右を決定
        if (points[0].x < points[1].x) {
            leftTop = points[0];
            rightTop = points[1];
        } else {
            leftTop = points[1];
            rightTop = points[0];
        }
        // 四角形の下二つの左右を決定
        if (points[2].x < points[3].x) {
            leftBottom = points[2];
            rightBottom = points[3];
        } else {
            leftBottom = points[3];
            rightBottom = points[2];
        }

        double width = Math.sqrt(Math.pow(leftTop.x - rightTop.x, 2) + Math.pow(leftTop.y - rightTop.y, 2));
        double height = Math.sqrt(Math.pow(leftTop.x - leftBottom.x, 2) + Math.pow(leftTop.y - leftBottom.y, 2));

        Mat transformMatrix;
        {
            MatOfPoint2f srcPoints = new MatOfPoint2f(points);
            srcPoints.convertTo(srcPoints, CvType.CV_32F); // タイプをCV_32Fに変換

            MatOfPoint2f dstPoint = new MatOfPoint2f(
                    new Point(0, 0),
                    new Point(width - 1, 0),
                    new Point(width - 1, height - 1),
                    new Point(0, height - 1)
            );
            dstPoint.convertTo(dstPoint, CvType.CV_32F); // タイプをCV_32Fに変換

            // 変換前の座標と変換後の座標から透視変換行列(切り抜きたい領域を長方形に変換するための行列)を作る
            transformMatrix = Imgproc.getPerspectiveTransform(srcPoints, dstPoint);
        }

        Mat clippedImage = Mat.zeros((int) width, (int) height, image.type());
        Imgproc.warpPerspective(image, clippedImage, transformMatrix, clippedImage.size());
        return clippedImage;

    }

    @Nullable
    public static Mat clipAR(Mat image) {
        // ARタグを検知する
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        ArrayList<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);
        if (corners.isEmpty()) return null;
        return clip(image, corners.get(0));
    }

}
