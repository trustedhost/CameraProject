#include "FaceLandmark.hpp"
#include <iostream>
#include <cmath>

#define FACE_LANDMARKS 468

// Helper function
bool __isIndexValid(int idx) {
    if (idx < 0 || idx >= FACE_LANDMARKS) {
        std::cerr << "Index " << idx << " is out of range (" \
                  << FACE_LANDMARKS << ")." << std::endl;
        return false;
    }
    return true;
}

my::FaceLandmark::FaceLandmark(std::string modelPath):
    FaceDetection(modelPath),
    m_landmarkModel(modelPath + std::string("/face_landmark.tflite")) {}

void my::FaceLandmark::runInference() {
    FaceDetection::runInference();
    auto roi = FaceDetection::getFaceRoi();
    if (roi.empty()) return;

    auto face = FaceDetection::cropFrame(roi);
    m_landmarkModel.loadImageToInput(face);
    m_landmarkModel.runInference();
}

cv::Point my::FaceLandmark::getFaceLandmarkAt(int index) const {
    if (__isIndexValid(index)) {
        auto roi = FaceDetection::getFaceRoi();

        float _x = m_landmarkModel.getOutputData()[index * 3];
        float _y = m_landmarkModel.getOutputData()[index * 3 + 1];

        int x = (int)(_x / m_landmarkModel.getInputShape()[2] * roi.width) + roi.x;
        int y = (int)(_y / m_landmarkModel.getInputShape()[1] * roi.height) + roi.y;

        return cv::Point(x,y);
    }
    return cv::Point();
}

std::vector<cv::Point> my::FaceLandmark::getAllFaceLandmarks() const {
    if (FaceDetection::getFaceRoi().empty())
        return std::vector<cv::Point>();

    std::vector<cv::Point> landmarks(FACE_LANDMARKS);
    for (int i = 0; i < FACE_LANDMARKS; ++i) {
        landmarks[i] = getFaceLandmarkAt(i);
    }
    return landmarks;
}

std::vector<float> my::FaceLandmark::loadOutput(int index) const {
    return m_landmarkModel.loadOutput();
}

// New method to detect head direction
cv::Vec2f my::FaceLandmark::detectHeadDirection() const {
    auto landmarks = getAllFaceLandmarks();
    
    if (!landmarks.size()) {
        return cv::Vec2f(0, 0);
    }
    // Using specific landmark indices for eyes and nose
    cv::Point leftEye = landmarks[33]; // Left eye
    cv::Point rightEye = landmarks[263]; // Right eye
    cv::Point nose = landmarks[4];      // Nose

    // Calculate eye midpoint
    cv::Point2f eyeMidpoint(
        (leftEye.x + rightEye.x) / 2.0f,
        (leftEye.y + rightEye.y) / 2.0f
    );
    
    // Calculate horizontal angle (yaw)
    float eyeDistance = cv::norm(leftEye - rightEye);
    float normalizedX = (nose.x - eyeMidpoint.x) / eyeDistance;
    float yaw = atan2(normalizedX, 1.0) * 180.0 / CV_PI;
    
    // Calculate vertical angle (pitch)
    float normalizedY = (nose.y - eyeMidpoint.y) / eyeDistance;
    float pitch = atan2(normalizedY, 1.0) * 180.0 / CV_PI;
    
    return cv::Vec2f(yaw, pitch);
}
