/*
 * Copyright 2016 Mario Contreras - marioc@nazul.net.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package mx.iteso.msc.ardronedemo2;

import de.yadrone.base.ARDrone;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import javax.imageio.ImageIO;
import javax.swing.UIManager;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.objdetect.Objdetect;

/**
 *
 * @author Mario Contreras - marioc@nazul.net
 */
public class ARDroneDemo2 extends javax.swing.JFrame {

    // A timer for processing the video stream
    private ScheduledExecutorService videoTimer;
    // A timer for processing the drone movement
    private ScheduledExecutorService droneTimer;
    // Flag to change the buttons behavior
    private boolean droneActive;
    private boolean droneTracking;
    // A flag to determinate if mouse was clicked in an area so we can select the color around that section
    private boolean mouseClicked;
    // Mouse coordinates
    private int mx, my;
    // Drone
    private ARDrone drone;
    // Current frame
    BufferedImage currentFrame;
    // Face size
    private int absoluteFaceSize;
    // OpenCV classifier for face detection
    private final CascadeClassifier faceCascade;
    // A flag to determinate if a tracked object exists on screen
    private boolean objectDetected = false;
    // The center of the currently tracked object
    private Point trackedObject = new Point(0.0d, 0.0d);

    /**
     * Creates new form ARDroneDemo2
     */
    public ARDroneDemo2() {
        initComponents();
        // Center
        this.setLocationRelativeTo(null);
        // Load the native OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        // Init components
        droneActive = droneTracking = false;
        faceCascade = new CascadeClassifier(getClass().getResource("/mx/iteso/msc/ardronedemo2/resources/lbpcascade_frontalface.xml").getFile().substring(1).replace("/", "\\"));
        absoluteFaceSize = 0;
        // Update status bar
        slidersStateChanged(null);
        // Connect to drone
        try {
            drone = new ARDrone();
            System.out.println("Connect drone controller");
            drone.start();
            drone.getVideoManager().addImageListener((BufferedImage newImage) -> {
                currentFrame = newImage;
            });
            // Grab a frame every 33 ms (30 frames/sec)
            Runnable frameProcess = () -> {
                BufferedImage imageToShow = processFrame();
                cameraPanel.getGraphics().drawImage(imageToShow, 0, 0, 640, 360, null);
            };
            videoTimer = Executors.newSingleThreadScheduledExecutor();
            videoTimer.scheduleAtFixedRate(frameProcess, 0, 33, TimeUnit.MILLISECONDS);
            // If drone is active and tracking objects, send move commands every 500 ms
            Runnable droneProcess = () -> {
                if (droneActive && droneTracking && objectDetected) {
                    if (trackedObject.x < 540) {
                        drone.spinRight();
                        System.out.println("Spin right");
                    }
                    if (trackedObject.x > 740) {
                        drone.spinLeft();
                        System.out.println("Spin left");
                    }
                }
            };
            droneTimer = Executors.newSingleThreadScheduledExecutor();
            droneTimer.scheduleAtFixedRate(droneProcess, 0, 500, TimeUnit.MILLISECONDS);
        } catch (Exception exc) {
            exc.printStackTrace();

            if (drone != null) {
                drone.stop();
            }
            System.exit(-1);
        }
    }

    private Mat findAndDrawObjects(Mat maskedImage, Mat frame) {
        // Init
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Find contours
        Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        // If any contour exist...
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            // for each contour, display it in blue
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                Imgproc.drawContours(frame, contours, idx, new Scalar(250, 0, 0));
            }
        }
        return frame;
    }

    private BufferedImage mat2Image(Mat frame) {
        // Create a temporary buffer
        MatOfByte buffer = new MatOfByte();
        // Encode the frame in the buffer, according to the PNG format
        Imgcodecs.imencode(".png", frame, buffer);
        // Build and return an Image created from the image encoded in the buffer
        // Return new BufferedImage(new ByteArrayInputStream(buffer.toArray()));
        BufferedImage img = null;
        try {
            img = ImageIO.read(new ByteArrayInputStream(buffer.toArray()));
        } catch (Exception e) {
            // log the error
            System.err.println("Exception while converting frame: " + e);
        }
        return img;
    }

    private Mat image2Mat(BufferedImage frame) {
        Mat result = new Mat(frame.getHeight(), frame.getWidth(), CvType.CV_8UC3);

        DataBufferByte data = (DataBufferByte) frame.getRaster().getDataBuffer();
        result.put(0, 0, data.getData());

        return result;
    }
    
    private void drawCrosshairs(Mat frame, int x, int y) {
        // Show crosshair
        Imgproc.circle(frame, new Point(x, y), 20, new Scalar(0, 255, 0), 2);
        Imgproc.line(frame, new Point(x, y), new Point(x, y - 25), new Scalar(0, 255, 0), 2);
        Imgproc.line(frame, new Point(x, y), new Point(x, y + 25), new Scalar(0, 255, 0), 2);
        Imgproc.line(frame, new Point(x, y), new Point(x - 25, y), new Scalar(0, 255, 0), 2);
        Imgproc.line(frame, new Point(x, y), new Point(x + 25, y), new Scalar(0, 255, 0), 2);
        Imgproc.putText(frame, "Tracking object at (" + x + "," + y + ")", new Point(x, y), 1, 1, new Scalar(255, 0, 0), 2);
    }

    private void processFaces(Mat frame) {
        MatOfRect faces = new MatOfRect();
        Mat grayFrame = new Mat();

        // convert the frame in gray scale
        Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_BGR2GRAY);
        // equalize the frame histogram to improve the result
        Imgproc.equalizeHist(grayFrame, grayFrame);

        // compute minimum face size (20% of the frame height, in our case)
        if (this.absoluteFaceSize == 0) {
            int height = grayFrame.rows();
            if (Math.round(height * 0.2f) > 0) {
                this.absoluteFaceSize = Math.round(height * 0.2f);
            }
        }

        // detect faces
        this.faceCascade.detectMultiScale(grayFrame, faces, 1.1, 2, Objdetect.CASCADE_SCALE_IMAGE, new Size(
                this.absoluteFaceSize, this.absoluteFaceSize), new Size());

        // each rectangle in faces is a face: draw them!
        Rect[] facesArray = faces.toArray();
        for (Rect face : facesArray) {
            Imgproc.rectangle(frame, face.tl(), face.br(), new Scalar(0, 255, 0, 255), 3);
        }
        // Get the first face and use it as a tracking object
        objectDetected = false;
        if (facesArray.length > 0) {
            objectDetected = true;
            trackedObject.x = facesArray[0].x + facesArray[0].width / 2;
            trackedObject.y = facesArray[0].y + facesArray[0].height / 2;
            drawCrosshairs(frame, (int)trackedObject.x, (int)trackedObject.y);
        }
    }
    
    private void processHsv(Mat frame) {
        // Init
        Mat blurredImage = new Mat();
        Mat hsvImage = new Mat();
        Mat mask = new Mat();
        Mat morphOutput = new Mat();

        // Remove some noise
        Imgproc.blur(frame, blurredImage, new Size(7, 7));

        // Convert the frame to HSV
        Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
        hsvPanel.getGraphics().drawImage(this.mat2Image(hsvImage), 0, 0, 213, 120, null);

        // Get color of current coordinates
        if (mouseClicked) {
            try {
                int hmin = 180, smin = 255, vmin = 255;
                int hmax = 0, smax = 0, vmax = 0;

                for (int i = mx - 50; i < mx + 50; i++) {
                    for (int j = my - 50; j < my + 50; j++) {
                        double[] hsv = hsvImage.get(j, i);
                        hmin = (int) (hsv[0] < hmin ? hsv[0] : hmin);
                        hmax = (int) (hsv[0] > hmax ? hsv[0] : hmax);
                        smin = (int) (hsv[1] < smin ? hsv[1] : smin);
                        smax = (int) (hsv[1] > smax ? hsv[1] : smax);
                        vmin = (int) (hsv[2] < vmin ? hsv[2] : vmin);
                        vmax = (int) (hsv[2] > vmax ? hsv[2] : vmax);
                    }
                }
                hueMinSlider.setValue(hmin);
                hueMaxSlider.setValue(hmax);
                saturationMinSlider.setValue(smin);
                saturationMaxSlider.setValue(smax);
                valueMinSlider.setValue(vmin);
                valueMaxSlider.setValue(vmax);
                Imgproc.rectangle(frame, new Point(mx - 50, my - 50), new Point(mx + 50, my + 50), new Scalar(255, 0, 255), 4);
            } catch (Exception ex) {

            } finally {
                mouseClicked = false;
            }
        }

        // Get thresholding values from the UI
        // Remember: H ranges 0-180, S and V range 0-255
        Scalar minValues = new Scalar(this.hueMinSlider.getValue(), this.saturationMinSlider.getValue(), this.valueMinSlider.getValue());
        Scalar maxValues = new Scalar(this.hueMaxSlider.getValue(), this.saturationMaxSlider.getValue(), this.valueMaxSlider.getValue());

        // Threshold HSV image to select object
        Core.inRange(hsvImage, minValues, maxValues, mask);

        // Morphological operators
        // Dilate with large element, erode with small ones
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

        Imgproc.erode(mask, morphOutput, erodeElement);
        Imgproc.erode(mask, morphOutput, erodeElement);
        // Show the partial output
        erodePanel.getGraphics().drawImage(this.mat2Image(morphOutput), 0, 0, 213, 120, null);

        Imgproc.dilate(mask, morphOutput, dilateElement);
        Imgproc.dilate(mask, morphOutput, dilateElement);
        // Show the partial output
        dilatePanel.getGraphics().drawImage(this.mat2Image(morphOutput), 0, 0, 213, 120, null);

        // Find the object(s) contours and show them
        frame = this.findAndDrawObjects(morphOutput, frame);

        // Calculate centers
        Mat temp = new Mat();
        morphOutput.copyTo(temp);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        objectDetected = false;
        for (int i = 0; i < contours.size(); i++) {
            Rect objectBoundingRectangle = Imgproc.boundingRect(contours.get(i));
            int x = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
            int y = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
            if (i == 0) {
                objectDetected = true;
                trackedObject.x = x;
                trackedObject.y = y;
                drawCrosshairs(frame, x, y);
            }
        }
    }

    private void processRgb(Mat frame) {
        // Init
        Mat blurredImage = new Mat();
        Mat mask = new Mat();
        Mat morphOutput = new Mat();

        // Remove some noise
        Imgproc.blur(frame, blurredImage, new Size(7, 7));

        // Get thresholding values from the UI
        Scalar minValue = new Scalar(this.rValueSlider.getValue(), this.gValueSlider.getValue(), this.bValueSlider.getValue());
        Scalar maxValue = new Scalar(this.rValueSlider.getValue(), this.gValueSlider.getValue(), this.bValueSlider.getValue());

        // Threshold image to select object
        Core.inRange(blurredImage, minValue, maxValue, mask);

        // Morphological operators
        // Dilate with large element, erode with small ones
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

        Imgproc.erode(mask, morphOutput, erodeElement);
        Imgproc.erode(mask, morphOutput, erodeElement);
        // Show the partial output
        erodePanel.getGraphics().drawImage(this.mat2Image(morphOutput), 0, 0, 213, 120, null);

        Imgproc.dilate(mask, morphOutput, dilateElement);
        Imgproc.dilate(mask, morphOutput, dilateElement);
        // Show the partial output
        dilatePanel.getGraphics().drawImage(this.mat2Image(morphOutput), 0, 0, 213, 120, null);

        // Find the object(s) contours and show them
        frame = this.findAndDrawObjects(morphOutput, frame);

        // Calculate centers
        Mat temp = new Mat();
        morphOutput.copyTo(temp);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        objectDetected = false;
        for (int i = 0; i < contours.size(); i++) {
            Rect objectBoundingRectangle = Imgproc.boundingRect(contours.get(i));
            int x = objectBoundingRectangle.x + objectBoundingRectangle.width / 2;
            int y = objectBoundingRectangle.y + objectBoundingRectangle.height / 2;
            if (i == 0) {
                objectDetected = true;
                trackedObject.x = x;
                trackedObject.y = y;
                drawCrosshairs(frame, x, y);
            }
        }
    }

    private BufferedImage processFrame() {
        // Init everything
        BufferedImage imageToShow = null;
        Mat frame;

        // Check if the capture is open
        if (currentFrame != null) {
            try {
                // Read the current frame
                frame = image2Mat(currentFrame);
                // Flip image for easy object manipulation
                Core.flip(frame, frame, 1);
                if (hsvColorDetectionRadioButton.isSelected()) {
                    processHsv(frame);
                }
                if (rgbColorDetectionRadioButton.isSelected()) {
                    processRgb(frame);
                }
                if (faceDetectionRadioButton.isSelected()) {
                    processFaces(frame);
                }
                // If the drone is in tracking mode, then draw boundaries
                if (droneTracking && frame != null) {
                    Imgproc.rectangle(frame, new Point(0, 0), new Point(540, 720), new Scalar(255, 0, 255), 4);
                    Imgproc.rectangle(frame, new Point(740, 0), new Point(1280, 720), new Scalar(255, 0, 255), 4);
                }
                // convert the Mat object (OpenCV) to Image (Java AWT)
                imageToShow = mat2Image(frame);
            } catch (Exception e) {
                // log the error
                System.err.println("Exception during the frame elaboration: " + e);
            }
        }
        return imageToShow;
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        buttonGroup1 = new javax.swing.ButtonGroup();
        cameraPanel = new javax.swing.JPanel();
        processPanel = new javax.swing.JPanel();
        hsvPanel = new javax.swing.JPanel();
        erodePanel = new javax.swing.JPanel();
        dilatePanel = new javax.swing.JPanel();
        huePanel = new javax.swing.JPanel();
        hueMinSlider = new javax.swing.JSlider();
        hueMaxSlider = new javax.swing.JSlider();
        saturationPanel = new javax.swing.JPanel();
        saturationMinSlider = new javax.swing.JSlider();
        saturationMaxSlider = new javax.swing.JSlider();
        valuePanel = new javax.swing.JPanel();
        valueMinSlider = new javax.swing.JSlider();
        valueMaxSlider = new javax.swing.JSlider();
        rgbPanel = new javax.swing.JPanel();
        rValueSlider = new javax.swing.JSlider();
        gValueSlider = new javax.swing.JSlider();
        bValueSlider = new javax.swing.JSlider();
        rgbColorDetectionRadioButton = new javax.swing.JRadioButton();
        hsvColorDetectionRadioButton = new javax.swing.JRadioButton();
        faceDetectionRadioButton = new javax.swing.JRadioButton();
        startDroneButton = new javax.swing.JButton();
        startTrackingButton = new javax.swing.JButton();
        statusLabel = new javax.swing.JLabel();

        setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
        setPreferredSize(new java.awt.Dimension(820, 600));
        setResizable(false);
        getContentPane().setLayout(new org.netbeans.lib.awtextra.AbsoluteLayout());

        cameraPanel.setBorder(javax.swing.BorderFactory.createEtchedBorder(javax.swing.border.EtchedBorder.RAISED));
        cameraPanel.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                cameraPanelMouseClicked(evt);
            }
        });

        javax.swing.GroupLayout cameraPanelLayout = new javax.swing.GroupLayout(cameraPanel);
        cameraPanel.setLayout(cameraPanelLayout);
        cameraPanelLayout.setHorizontalGroup(
            cameraPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 636, Short.MAX_VALUE)
        );
        cameraPanelLayout.setVerticalGroup(
            cameraPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 356, Short.MAX_VALUE)
        );

        getContentPane().add(cameraPanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(0, 0, 640, 360));

        processPanel.setBorder(javax.swing.BorderFactory.createEtchedBorder(javax.swing.border.EtchedBorder.RAISED));
        processPanel.setLayout(new org.netbeans.lib.awtextra.AbsoluteLayout());

        hsvPanel.setBorder(javax.swing.BorderFactory.createEtchedBorder());

        javax.swing.GroupLayout hsvPanelLayout = new javax.swing.GroupLayout(hsvPanel);
        hsvPanel.setLayout(hsvPanelLayout);
        hsvPanelLayout.setHorizontalGroup(
            hsvPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 209, Short.MAX_VALUE)
        );
        hsvPanelLayout.setVerticalGroup(
            hsvPanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 116, Short.MAX_VALUE)
        );

        processPanel.add(hsvPanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(0, 0, 213, 120));

        erodePanel.setBorder(javax.swing.BorderFactory.createEtchedBorder());

        javax.swing.GroupLayout erodePanelLayout = new javax.swing.GroupLayout(erodePanel);
        erodePanel.setLayout(erodePanelLayout);
        erodePanelLayout.setHorizontalGroup(
            erodePanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 209, Short.MAX_VALUE)
        );
        erodePanelLayout.setVerticalGroup(
            erodePanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 116, Short.MAX_VALUE)
        );

        processPanel.add(erodePanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(214, 0, 213, 120));

        dilatePanel.setBorder(javax.swing.BorderFactory.createEtchedBorder());

        javax.swing.GroupLayout dilatePanelLayout = new javax.swing.GroupLayout(dilatePanel);
        dilatePanel.setLayout(dilatePanelLayout);
        dilatePanelLayout.setHorizontalGroup(
            dilatePanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 209, Short.MAX_VALUE)
        );
        dilatePanelLayout.setVerticalGroup(
            dilatePanelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 116, Short.MAX_VALUE)
        );

        processPanel.add(dilatePanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(427, 0, 213, 120));

        getContentPane().add(processPanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(0, 360, 640, 120));

        huePanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Hue"));
        huePanel.setPreferredSize(new java.awt.Dimension(300, 60));

        hueMinSlider.setMaximum(180);
        hueMinSlider.setValue(0);
        hueMinSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        hueMinSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        huePanel.add(hueMinSlider);

        hueMaxSlider.setMaximum(180);
        hueMaxSlider.setValue(180);
        hueMaxSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        hueMaxSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        huePanel.add(hueMaxSlider);

        getContentPane().add(huePanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(640, 90, 150, 90));

        saturationPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Saturation"));
        saturationPanel.setPreferredSize(new java.awt.Dimension(300, 60));

        saturationMinSlider.setMaximum(255);
        saturationMinSlider.setValue(0);
        saturationMinSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        saturationMinSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        saturationPanel.add(saturationMinSlider);

        saturationMaxSlider.setMaximum(255);
        saturationMaxSlider.setValue(255);
        saturationMaxSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        saturationMaxSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        saturationPanel.add(saturationMaxSlider);

        getContentPane().add(saturationPanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(640, 180, 150, 90));

        valuePanel.setBorder(javax.swing.BorderFactory.createTitledBorder("Value"));
        valuePanel.setPreferredSize(new java.awt.Dimension(300, 60));

        valueMinSlider.setMaximum(255);
        valueMinSlider.setValue(0);
        valueMinSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        valueMinSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        valuePanel.add(valueMinSlider);

        valueMaxSlider.setMaximum(255);
        valueMaxSlider.setValue(255);
        valueMaxSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        valueMaxSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        valuePanel.add(valueMaxSlider);

        getContentPane().add(valuePanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(640, 270, 150, 90));

        rgbPanel.setBorder(javax.swing.BorderFactory.createTitledBorder("RGB"));
        rgbPanel.setPreferredSize(new java.awt.Dimension(300, 60));

        rValueSlider.setMaximum(255);
        rValueSlider.setValue(0);
        rValueSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        rValueSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        rgbPanel.add(rValueSlider);

        gValueSlider.setMaximum(255);
        gValueSlider.setValue(0);
        gValueSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        gValueSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        rgbPanel.add(gValueSlider);

        bValueSlider.setMaximum(255);
        bValueSlider.setValue(255);
        bValueSlider.setPreferredSize(new java.awt.Dimension(140, 26));
        bValueSlider.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                slidersStateChanged(evt);
            }
        });
        rgbPanel.add(bValueSlider);

        getContentPane().add(rgbPanel, new org.netbeans.lib.awtextra.AbsoluteConstraints(640, 360, 150, 120));

        buttonGroup1.add(rgbColorDetectionRadioButton);
        rgbColorDetectionRadioButton.setText("Color Detection (RGB)");
        rgbColorDetectionRadioButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                detectionButtonClicked(evt);
            }
        });
        getContentPane().add(rgbColorDetectionRadioButton, new org.netbeans.lib.awtextra.AbsoluteConstraints(650, 500, -1, -1));

        buttonGroup1.add(hsvColorDetectionRadioButton);
        hsvColorDetectionRadioButton.setText("Color Detection (HSV)");
        hsvColorDetectionRadioButton.setCursor(new java.awt.Cursor(java.awt.Cursor.DEFAULT_CURSOR));
        hsvColorDetectionRadioButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                detectionButtonClicked(evt);
            }
        });
        getContentPane().add(hsvColorDetectionRadioButton, new org.netbeans.lib.awtextra.AbsoluteConstraints(650, 480, -1, -1));

        buttonGroup1.add(faceDetectionRadioButton);
        faceDetectionRadioButton.setSelected(true);
        faceDetectionRadioButton.setText("Face Detection");
        faceDetectionRadioButton.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                detectionButtonClicked(evt);
            }
        });
        getContentPane().add(faceDetectionRadioButton, new org.netbeans.lib.awtextra.AbsoluteConstraints(650, 520, -1, -1));

        startDroneButton.setText("Start Drone");
        startDroneButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                startDroneButtonActionPerformed(evt);
            }
        });
        getContentPane().add(startDroneButton, new org.netbeans.lib.awtextra.AbsoluteConstraints(650, 20, 140, -1));

        startTrackingButton.setText("Start Tracking");
        startTrackingButton.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                startTrackingButtonActionPerformed(evt);
            }
        });
        getContentPane().add(startTrackingButton, new org.netbeans.lib.awtextra.AbsoluteConstraints(650, 50, 140, -1));

        statusLabel.setText("[statusLabel]");
        statusLabel.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.LOWERED));
        getContentPane().add(statusLabel, new org.netbeans.lib.awtextra.AbsoluteConstraints(0, 490, 640, 20));

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void cameraPanelMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_cameraPanelMouseClicked
        mouseClicked = true;
        mx = evt.getX() * 2;
        my = evt.getY() * 2;
    }//GEN-LAST:event_cameraPanelMouseClicked

    private void slidersStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_slidersStateChanged
        String valuesToPrint = "";
        
        if (hsvColorDetectionRadioButton.isSelected()) {
            valuesToPrint = "Hue range: " + this.hueMinSlider.getValue() + "-" + this.hueMaxSlider.getValue()
                    + ". Sat. range: " + this.saturationMinSlider.getValue() + "-" + this.saturationMaxSlider.getValue()
                    + ". Value range: " + this.valueMinSlider.getValue() + "-" + this.valueMaxSlider.getValue();
        }
        if (rgbColorDetectionRadioButton.isSelected()) {
            valuesToPrint = "Red: " + this.rValueSlider.getValue()
                    + ". Green: " + this.gValueSlider.getValue()
                    + ". Blue: " + this.bValueSlider.getValue();
        }
        statusLabel.setText(valuesToPrint);
    }//GEN-LAST:event_slidersStateChanged

    private void startDroneButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_startDroneButtonActionPerformed
        if (!droneActive) {
            drone.takeOff();
            startDroneButton.setText("Stop Drone");
        } else {
            drone.landing();
            startDroneButton.setText("Start Drone");
        }
        droneActive = !droneActive;
    }//GEN-LAST:event_startDroneButtonActionPerformed

    private void startTrackingButtonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_startTrackingButtonActionPerformed
        if (!droneTracking) {
            startTrackingButton.setText("Stop Tracking");
        } else {
            startTrackingButton.setText("Start Tracking");
        }
        droneTracking = !droneTracking;
    }//GEN-LAST:event_startTrackingButtonActionPerformed

    private void detectionButtonClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_detectionButtonClicked
        slidersStateChanged(null);
    }//GEN-LAST:event_detectionButtonClicked

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        /* Instead of Nimbus look and feel, let's use System look and feel */
        //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
        /* For details see http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html */
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (ClassNotFoundException | InstantiationException | IllegalAccessException | javax.swing.UnsupportedLookAndFeelException ex) {
            java.util.logging.Logger.getLogger(ARDroneDemo2.class.getName()).log(java.util.logging.Level.SEVERE, null, ex);
        }
        //</editor-fold>

        /* Create and display the form */
        java.awt.EventQueue.invokeLater(() -> {
            new ARDroneDemo2().setVisible(true);
        });
    }

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JSlider bValueSlider;
    private javax.swing.ButtonGroup buttonGroup1;
    private javax.swing.JPanel cameraPanel;
    private javax.swing.JPanel dilatePanel;
    private javax.swing.JPanel erodePanel;
    private javax.swing.JRadioButton faceDetectionRadioButton;
    private javax.swing.JSlider gValueSlider;
    private javax.swing.JRadioButton hsvColorDetectionRadioButton;
    private javax.swing.JPanel hsvPanel;
    private javax.swing.JSlider hueMaxSlider;
    private javax.swing.JSlider hueMinSlider;
    private javax.swing.JPanel huePanel;
    private javax.swing.JPanel processPanel;
    private javax.swing.JSlider rValueSlider;
    private javax.swing.JRadioButton rgbColorDetectionRadioButton;
    private javax.swing.JPanel rgbPanel;
    private javax.swing.JSlider saturationMaxSlider;
    private javax.swing.JSlider saturationMinSlider;
    private javax.swing.JPanel saturationPanel;
    private javax.swing.JButton startDroneButton;
    private javax.swing.JButton startTrackingButton;
    private javax.swing.JLabel statusLabel;
    private javax.swing.JSlider valueMaxSlider;
    private javax.swing.JSlider valueMinSlider;
    private javax.swing.JPanel valuePanel;
    // End of variables declaration//GEN-END:variables
}
