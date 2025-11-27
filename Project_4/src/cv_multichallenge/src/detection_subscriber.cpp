#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <jsoncpp/json/json.h>

class DetectionSubscriber {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber detection_sub_;
    cv::Mat current_frame_;

public:
    DetectionSubscriber() : it_(nh_) {
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &DetectionSubscriber::imageCallback, this);
        detection_sub_ = nh_.subscribe("/detections", 1, &DetectionSubscriber::detectionCallback, this);

        ROS_INFO("DetectionSubscriber initialized and waiting for data...");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            current_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("Camera View", current_frame_);
        cv::waitKey(1);
    }

    void detectionCallback(const std_msgs::String::ConstPtr& msg) {
        Json::Reader reader;
        Json::Value detections;
        if (!reader.parse(msg->data, detections)) {
            ROS_WARN("Failed to parse detection JSON: %s", msg->data.c_str());
            return;
        }

        ROS_INFO("Received detection data:");
        for (const auto& det : detections["objects"]) {
            std::string label = det["label"].asString();
            double confidence = det["confidence"].asDouble();
            int x = det["bbox"]["x"].asInt();
            int y = det["bbox"]["y"].asInt();
            int w = det["bbox"]["w"].asInt();
            int h = det["bbox"]["h"].asInt();

            ROS_INFO("Label: %s | Conf: %.2f | BBox: [%d, %d, %d, %d]", 
                    label.c_str(), confidence, x, y, w, h);

            if (!current_frame_.empty()) {
                cv::rectangle(current_frame_, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
                cv::putText(current_frame_, label, cv::Point(x, y - 5), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                cv::imshow("Detections", current_frame_);
                cv::waitKey(1);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "detection_subscriber");
    DetectionSubscriber ds;
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}
