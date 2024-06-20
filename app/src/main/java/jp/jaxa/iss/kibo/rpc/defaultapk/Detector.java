package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;

import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;
import org.tensorflow.lite.support.common.TensorOperator;
import org.tensorflow.lite.support.common.ops.CastOp;
import org.tensorflow.lite.support.common.ops.NormalizeOp;
import org.tensorflow.lite.support.image.ImageProcessor;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

public final class Detector {

    static {
        System.loadLibrary("tensorflowlite_jni");
    }

    private Interpreter interpreter;  // TensorFlow Liteのインタプリタ
    private int tensorWidth;          // 入力テンソルの幅
    private int tensorHeight;         // 入力テンソルの高さ
    private int numChannel;           // 出力テンソルのチャネル数
    private int numElements;          // 出力テンソルの要素数
    private List<String> labels;      // クラスラベルのリスト
    private final ImageProcessor imageProcessor; // 画像前処理のためのImageProcessor
    private final Context context;    // コンテキスト
    private final String modelPath;   // モデルファイルのパス
    private final String labelPath;   // ラベルファイルのパス

    // 正規化のための定数
    private static final float INPUT_MEAN = 0.0F;
    private static final float INPUT_STANDARD_DEVIATION = 255.0F;
    private static final DataType INPUT_IMAGE_TYPE = DataType.FLOAT32;
    private static final DataType OUTPUT_IMAGE_TYPE = DataType.FLOAT32;
    private static final float CONFIDENCE_THRESHOLD = 0.50F;
    private static final float IOU_THRESHOLD = 0.4F;

    // コンストラクタ
    public Detector(Context context, String modelPath, String labelPath) {
        this.context = context;
        this.modelPath = modelPath;
        this.labelPath = labelPath;
        this.labels = new ArrayList<>();
        this.imageProcessor = new ImageProcessor.Builder()
                .add(new NormalizeOp(INPUT_MEAN, INPUT_STANDARD_DEVIATION))
                .add(new CastOp(INPUT_IMAGE_TYPE))
                .build();
    }

    // モデルとラベルの読み込みおよびセットアップ
    public void setup() throws IOException {
        // モデルファイルの読み込み
        InputStream inputStream = context.getAssets().open(modelPath);
        ByteBuffer model = ByteBuffer.allocateDirect(inputStream.available());
        byte[] buffer = new byte[inputStream.available()];
        inputStream.read(buffer);
        model.put(buffer);
        model.rewind();

        Interpreter.Options options = new Interpreter.Options();
        options.setNumThreads(4); // スレッド数の設定
        interpreter = new Interpreter(model, options);

        // 入力と出力のテンソルの形状を取得
        int[] inputShape = interpreter.getInputTensor(0).shape();
        int[] outputShape = interpreter.getOutputTensor(0).shape();

        tensorWidth = inputShape[1];
        tensorHeight = inputShape[2];
        numChannel = outputShape[1];
        numElements = outputShape[2];

        // ラベルファイルの読み込み
        try (InputStream labelInputStream = context.getAssets().open(labelPath);
             BufferedReader reader = new BufferedReader(new InputStreamReader(labelInputStream))) {
            String line;
            while ((line = reader.readLine()) != null) {
                labels.add(line);
            }
        }
    }

    // 画像の検出処理
    public List<BoundingBox> detect(Bitmap image) {
        if (interpreter != null && tensorWidth != 0 && tensorHeight != 0 && numChannel != 0 && numElements != 0) {
            // 入力画像のリサイズ
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(image, tensorWidth, tensorHeight, false);
            TensorImage tensorImage = new TensorImage(DataType.FLOAT32);
            tensorImage.load(resizedBitmap);
            // 画像の前処理
            TensorImage processedImage = imageProcessor.process(tensorImage);
            ByteBuffer imageBuffer = processedImage.getBuffer();
            // 出力テンソルの作成
            TensorBuffer output = TensorBuffer.createFixedSize(new int[]{1, numChannel, numElements}, OUTPUT_IMAGE_TYPE);
            // 推論の実行
            interpreter.run(imageBuffer, output.getBuffer());

            float[] outputArray = output.getFloatArray();
            // バウンディングボックスの抽出
            return bestBox(outputArray);
        }
        return null;
    }

    // バウンディングボックスの抽出
    private List<BoundingBox> bestBox(float[] array) {
        List<BoundingBox> boundingBoxes = new ArrayList<>();
        for (int c = 0; c < numElements; c++) {
            float maxConf = -1.0F;
            int maxIdx = -1;
            for (int j = 4; j < numChannel; j++) {
                int arrayIdx = c + numElements * j;
                if (array[arrayIdx] > maxConf) {
                    maxConf = array[arrayIdx];
                    maxIdx = j - 4;
                }
            }

            if (maxConf > CONFIDENCE_THRESHOLD) {
                String clsName = labels.get(maxIdx);
                float cx = array[c];
                float cy = array[c + numElements];
                float w = array[c + numElements * 2];
                float h = array[c + numElements * 3];
                float x1 = cx - w / 2.0F;
                float y1 = cy - h / 2.0F;
                float x2 = cx + w / 2.0F;
                float y2 = cy + h / 2.0F;
                if (x1 >= 0.0F && x1 <= 1.0F && y1 >= 0.0F && y1 <= 1.0F && x2 >= 0.0F && x2 <= 1.0F && y2 >= 0.0F && y2 <= 1.0F) {
                    boundingBoxes.add(new BoundingBox(x1, y1, x2, y2, cx, cy, w, h, maxConf, maxIdx, clsName));
                }
            }
        }

        if (boundingBoxes.isEmpty()) {
            return null;
        } else {
            return applyNMS(boundingBoxes);
        }
    }

    // 非最大抑制 (NMS) の適用
    private List<BoundingBox> applyNMS(List<BoundingBox> boxes) {
        boxes.sort(new Comparator<BoundingBox>() {
            @Override
            public int compare(BoundingBox o1, BoundingBox o2) {
                return Float.compare(o2.getCnf(), o1.getCnf());
            }
        });
        List<BoundingBox> selectedBoxes = new ArrayList<>();
        while (!boxes.isEmpty()) {
            BoundingBox first = boxes.remove(0);
            selectedBoxes.add(first);
            Iterator<BoundingBox> iterator = boxes.iterator();
            while (iterator.hasNext()) {
                BoundingBox nextBox = iterator.next();
                if (calculateIoU(first, nextBox) >= IOU_THRESHOLD) {
                    iterator.remove();
                }
            }
        }
        return selectedBoxes;
    }

    // IoU (Intersection over Union) の計算
    private float calculateIoU(BoundingBox box1, BoundingBox box2) {
        float x1 = Math.max(box1.getX1(), box2.getX1());
        float y1 = Math.max(box1.getY1(), box2.getY1());
        float x2 = Math.min(box1.getX2(), box2.getX2());
        float y2 = Math.min(box1.getY2(), box2.getY2());
        float intersectionArea = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
        float box1Area = box1.getW() * box1.getH();
        float box2Area = box2.getW() * box2.getH();
        return intersectionArea / (box1Area + box2Area - intersectionArea);
    }

    public Bitmap drawBoundingBoxesOnBitmap(Bitmap bitmap, List<BoundingBox> boundingBoxes) {
        if (bitmap == null || boundingBoxes == null) {
            return null;
        }

        Bitmap resultBitmap = bitmap.copy(bitmap.getConfig(), true);
        Canvas canvas = new Canvas(resultBitmap);
        Paint boxPaint = new Paint();
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setColor(Color.RED);
        boxPaint.setStrokeWidth(5);

        Paint textPaint = new Paint();
        textPaint.setColor(Color.RED);
        textPaint.setTextSize(24);

        for (BoundingBox box : boundingBoxes) {
            float x1 = box.getX1() * bitmap.getWidth();
            float y1 = box.getY1() * bitmap.getHeight();
            float x2 = box.getX2() * bitmap.getWidth();
            float y2 = box.getY2() * bitmap.getHeight();

            canvas.drawRect(x1, y1, x2, y2, boxPaint);

            String text = box.getClsName() + ": " + String.format("%.2f", box.getCnf());
            canvas.drawText(text, x1, y1 - 10, textPaint);
        }

        return resultBitmap;
    }


}
