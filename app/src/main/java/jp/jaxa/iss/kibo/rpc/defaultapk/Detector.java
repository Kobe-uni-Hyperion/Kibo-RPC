package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.Context;
import android.graphics.Bitmap;

import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;
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
import java.nio.MappedByteBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

public final class Detector {
    private Interpreter interpreter;
    private int tensorWidth;
    private int tensorHeight;
    private int numChannel;
    private int numElements;
    private List<String> labels;
    private final ImageProcessor imageProcessor;
    private final Context context;
    private final String modelPath;
    private final String labelPath;
    private final DetectorListener detectorListener;

    private static final float INPUT_MEAN = 0.0F;
    private static final float INPUT_STANDARD_DEVIATION = 255.0F;
    private static final DataType INPUT_IMAGE_TYPE = DataType.FLOAT32;
    private static final DataType OUTPUT_IMAGE_TYPE = DataType.FLOAT32;
    private static final float CONFIDENCE_THRESHOLD = 0.01F;
    private static final float IOU_THRESHOLD = 0.4F;

    public Detector(Context context, String modelPath, String labelPath, DetectorListener detectorListener) {
        this.context = context;
        this.modelPath = modelPath;
        this.labelPath = labelPath;
        this.detectorListener = detectorListener;
        this.labels = new ArrayList<>();
        this.imageProcessor = new ImageProcessor.Builder()
                .add(new NormalizeOp(INPUT_MEAN, INPUT_STANDARD_DEVIATION))
                .add(new CastOp(INPUT_IMAGE_TYPE))
                .build();
    }

    public void setup() throws IOException {
        MappedByteBuffer model = FileUtil.loadMappedFile(context, modelPath);
        Interpreter.Options options = new Interpreter.Options();
        options.setNumThreads(4);
        interpreter = new Interpreter(model, options);

        int[] inputShape = interpreter.getInputTensor(0).shape();
        int[] outputShape = interpreter.getOutputTensor(0).shape();

        tensorWidth = inputShape[1];
        tensorHeight = inputShape[2];
        numChannel = outputShape[1];
        numElements = outputShape[2];

        try (InputStream inputStream = context.getAssets().open(labelPath);
             BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream))) {
            String line;
            while ((line = reader.readLine()) != null) {
                labels.add(line);
            }
        }
    }

    public void detect(Bitmap image) {
        if (interpreter != null && tensorWidth != 0 && tensorHeight != 0 && numChannel != 0 && numElements != 0) {
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(image, tensorWidth, tensorHeight, false);
            TensorImage tensorImage = new TensorImage(DataType.FLOAT32);
            tensorImage.load(resizedBitmap);
            TensorImage processedImage = imageProcessor.process(tensorImage);
            ByteBuffer imageBuffer = processedImage.getBuffer();
            TensorBuffer output = TensorBuffer.createFixedSize(new int[]{1, numChannel, numElements}, OUTPUT_IMAGE_TYPE);
            interpreter.run(imageBuffer, output.getBuffer());

            float[] outputArray = output.getFloatArray();
            List<BoundingBox> bestBoxes = bestBox(outputArray);
            if (bestBoxes == null) {
                detectorListener.onEmptyDetect();
            } else {
                detectorListener.onDetect(bestBoxes);
            }
        }
    }

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

    public interface DetectorListener {
        void onDetect(List<BoundingBox> boundingBoxes);

        void onEmptyDetect();
    }
}
