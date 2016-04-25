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

import org.opencv.core.Scalar;

/**
 *
 * @author Mario Contreras - marioc@nazul.net
 */
public class TrackedObject {
    private final TrackedObjectColor type;
    private Scalar hsvMin, hsvMax;
    private Scalar color;
    
    public TrackedObject(TrackedObjectColor type) {
        this.type = type;
        switch(type) {
            case BLUE:
                hsvMin = new Scalar(92, 0, 0);
                hsvMax = new Scalar(124, 256, 256);
                color = new Scalar(255, 0, 0);
                break;
            case GREEN:
                hsvMin = new Scalar(34, 50, 50);
                hsvMax = new Scalar(80, 220, 220);
                color = new Scalar(0, 255, 0);
                break;
            case RED:
                hsvMin = new Scalar(0, 200, 0);
                hsvMax = new Scalar(19, 255, 255);
                color = new Scalar(0, 0, 255);
                break;
            case YELLOW:
                hsvMin = new Scalar(20, 124, 123);
                hsvMax = new Scalar(30, 256, 256);
                color = new Scalar(0, 255, 255);
                break;
            case CUSTOM:
                hsvMin = new Scalar(0, 0, 0);
                hsvMax = new Scalar(179, 255, 255);
                color = new Scalar(255, 255, 255);
                break;
        }
    }

    /**
     * @return the type
     */
    public TrackedObjectColor getType() {
        return type;
    }

    /**
     * @return the hsvMin
     */
    public Scalar getHsvMin() {
        return hsvMin;
    }

    /**
     * @param hsvMin the hsvMin to set
     */
    public void setHsvMin(Scalar hsvMin) {
        this.hsvMin = hsvMin;
    }

    /**
     * @return the hsvMax
     */
    public Scalar getHsvMax() {
        return hsvMax;
    }

    /**
     * @param hsvMax the hsvMax to set
     */
    public void setHsvMax(Scalar hsvMax) {
        this.hsvMax = hsvMax;
    }

    /**
     * @return the color
     */
    public Scalar getColor() {
        return color;
    }

    /**
     * @param color the color to set
     */
    public void setColor(Scalar color) {
        this.color = color;
    }
}

// EOF
