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

public class ImageUtil {
    public static Mat clip(Mat image, Mat corner) {

        final org.opencv.core.Point[] points = new org.opencv.core.Point[4];
        // point[0]:ARコードの左上の座標, point[1]:右上の座標, point[2]:右下の座標, point[3]:左下の座標
        for (int i = 0; i < 4; i++) {
            points[i] = new org.opencv.core.Point(corner.get(0, i));
        }
        double[] corner0 = corner.get(0, 0);
        double[] corner1 = corner.get(0, 1);
        double[] corner2 = corner.get(0, 2);
        double[] corner3 = corner.get(0, 3);

        points[0] = new org.opencv.core.Point(-83 / 20 * (corner1[0] - corner0[0]) - 1 / 4 * (corner3[0] - corner0[0]) + corner0[0], -83 / 20 * (corner1[1] - corner0[1]) - 1 / 4 * (corner3[1] - corner0[1]) + corner0[1]);
        points[1] = new org.opencv.core.Point(17 / 20 * (corner1[0] - corner0[0]) - 1 / 4 * (corner3[0] - corner0[0]) + corner0[0], 17 / 20 * (corner1[1] - corner0[1]) - 1 / 4 * (corner3[1] - corner0[1]) + corner0[1]);
        points[2] = new org.opencv.core.Point(17 / 20 * (corner1[0] - corner0[0]) + 15 / 4 * (corner3[0] - corner0[0]) + corner0[0], 17 / 20 * (corner1[1] - corner0[1]) + 15 / 4 * (corner3[1] - corner0[1]) + corner0[1]);
        points[3] = new org.opencv.core.Point(-83 / 20 * (corner1[0] - corner0[0]) + 15 / 4 * (corner3[0] - corner0[0]) + corner0[0], -83 / 20 * (corner1[1] - corner0[1]) + 15 / 4 * (corner3[1] - corner0[1]) + corner0[1]);

        double width = Math.sqrt(Math.pow(points[0].x - points[1].x, 2) + Math.pow(points[0].y - points[1].y, 2));
        double height = Math.sqrt(Math.pow(points[0].x - points[3].x, 2) + Math.pow(points[0].y - points[3].y, 2));

        Mat transformMatrix;
        {
            MatOfPoint2f srcPoints = new MatOfPoint2f(points);
            srcPoints.convertTo(srcPoints, CvType.CV_32F); // タイプをCV_32Fに変換

            MatOfPoint2f dstPoints = new MatOfPoint2f(
                    new Point(0, 0),
                    new Point(width - 1, 0),
                    new Point(width - 1, height - 1),
                    new Point(0, height - 1)
            );
            dstPoints.convertTo(dstPoints, CvType.CV_32F); // タイプをCV_32Fに変換

            // 変換前の座標と変換後の座標から透視変換行列(切り抜きたい領域を長方形に変換するための行列)を作る
            transformMatrix = Imgproc.getPerspectiveTransform(srcPoints, dstPoints);
        }

        Mat clippedImage = Mat.zeros((int) height, (int) width, image.type());
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