# 👁️ ROS Hybrid Computer Vision Pipeline

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![Python](https://img.shields.io/badge/Python-3.8-yellow) ![C++](https://img.shields.io/badge/C++-11-green) ![OpenCV](https://img.shields.io/badge/OpenCV-4.5-red)

A dual-node ROS package that implements a real-time computer vision pipeline. The system leverages **Python** for rapid prototyping of image processing algorithms (Shape & Color Detection) and **C++** for high-performance data serialization and visualization.

## 📋 Project Overview

This project demonstrates a decoupled architecture common in robotics:
1.  **The Processor (Python):** Captures raw camera frames, applies OpenCV algorithms to detect geometric shapes (Triangles, Rectangles, Circles) and specific colors (Orange, White), and packages this metadata into JSON.
2.  **The Visualizer (C++):** Subscribes to the data stream, parses the JSON metadata using `jsoncpp`, and overlays bounding boxes and confidence scores onto the video feed.

### Key Features
* **Hybrid Architecture:** Seamless communication between Python and C++ nodes.
* **Geometric Shape Detection:** Uses Contour Approximation (`cv2.approxPolyDP`) to classify shapes.
* **Color Tracking:** HSV masking to track specific colored objects.
* **JSON Serialization:** Passes complex detection data (Labels, Bounding Boxes, Confidence) via standard `std_msgs/String` topics.
* **Performance Metrics:** Real-time FPS calculation and display.

## 🛠️ System Architecture

### Node 1: `cv_publisher_node` (Python)
* **Role:** Sensor Interface & Processing.
* **Input:** USB Webcam (Video ID 0).
* **Logic:** Canny Edge Detection, Contour finding, HSV Color Filtering.
* **Output:** Publishes annotated images and a JSON string containing detection stats.

### Node 2: `detection_subscriber` (C++)
* **Role:** Data Consumption & UI.
* **Input:** Subscribes to image streams and JSON detection strings.
* **Logic:** deserializes JSON data to draw dynamic overlays on the frame.

## 📦 Dependencies

Ensure you have the following installed in your ROS environment:

* ROS Noetic (or Melodic)
* OpenCV (`sudo apt install libopencv-dev python3-opencv`)
* **JsonCpp** (Required for C++ parsing):
    ```bash
    sudo apt-get install libjsoncpp-dev
    ```

## 🚀 Installation & Build

1.  **Clone the repository** into your catkin workspace:
    ```bash
    cd ~/catkin_ws/src
    git clone [https://github.com/YourUsername/cv_hybrid_pipeline.git](https://github.com/YourUsername/cv_hybrid_pipeline.git)
    ```

2.  **Update `CMakeLists.txt`:**
    Ensure you link the `jsoncpp` library in your C++ node configuration:
    ```cmake
    add_executable(detection_subscriber src/detection_subscriber.cpp)
    target_link_libraries(detection_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} jsoncpp)
    ```

3.  **Build the package:**
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## 💻 Usage

**1. Start the ROS Master:**
```bash
roscore
````

**2. Launch the Processor (Python):**
This will open the camera and start publishing data.

```bash
rosrun your_package_name cv_publisher_node.py
```

  * *Controls:* Press `s` to save a frame, `q` to quit.

**3. Launch the Visualizer (C++):**
*Note: Ensure topics match via remapping if necessary.*

```bash
rosrun your_package_name detection_subscriber /detections:=/cv/detections
```

## 📡 ROS Topics

| Topic | Type | Node | Description |
| :--- | :--- | :--- | :--- |
| `/cv/image_annotated` | `sensor_msgs/Image` | Publisher (Py) | The processed video frame with OpenCV drawings. |
| `/cv/detections` | `std_msgs/String` | Publisher (Py) | JSON string containing shape counts and FPS. |
| `/camera/image_raw` | `sensor_msgs/Image` | Subscriber (Cpp) | Raw input stream for the C++ node. |
| `/detections` | `std_msgs/String` | Subscriber (Cpp) | Input for metadata parsing. |

## 📊 Example JSON Payload

The Python node broadcasts data in this format:

```json
{
  "shapes_detected": 3,
  "colors_tracked": 2,
  "fps": 28.5
}
```

## 🔮 Future Improvements

  * Implement Deep Learning (YOLOv8) inference in the Python node.
  * Synchronize image and detection messages using `message_filters` to prevent lag.
  * Add a 3D pose estimation utilizing PnP (Perspective-n-Point).

-----

*Project created for Computer Vision Systems Course.*
