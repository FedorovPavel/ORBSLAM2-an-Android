#include <jni.h>
#include <string>
#include <iostream>
#include "System.h"
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <time.h>
#include "Plane.h"
#include "Process.h"


#include <android/log.h>

#define LOG_TAG    "native-dev"
#define CURRENT_LOG_LEVEL ANDROID_LOG_INFO
#define LOGGER(...)  __android_log_print(CURRENT_LOG_LEVEL, LOG_TAG, __VA_ARGS__)

#define CENTER_W 320
#define CENTER_H 240
#define BOWISBIN

//ORB_SLAM2::System SLAM("/storage/emulated/0/ORBvoc.txt","/storage/emulated/0/TUM1.yaml",ORB_SLAM2::System::MONOCULAR,false);
#ifdef BOWISBIN
#endif
ORB_SLAM2::System SLAM("/storage/emulated/0/SLAM/VOC/ORBvoc.bin",
                       "/storage/emulated/0/SLAM/Calibration/mi6.yaml",
                       ORB_SLAM2::System::MONOCULAR, false);

std::chrono::steady_clock::time_point t0;
double ttrack = 0;

cv::Mat Plane2World = cv::Mat::eye(4, 4, CV_32F);
cv::Mat Marker2World = cv::Mat::eye(4, 4, CV_32F);
cv::Mat lastTranslate = cv::Mat(3,1,CV_32F);
cv::Mat prevPos = cv::Mat(3,1,CV_32F);
cv::Mat centroid;

bool load_as_text(ORB_SLAM2::ORBVocabulary *voc, const std::string infile) {
    clock_t tStart = clock();
    bool res = voc->loadFromTextFile(infile);
    LOGGER("Loading fom text: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);
    return res;
}

void save_as_binary(ORB_SLAM2::ORBVocabulary *voc, const std::string outfile) {
    clock_t tStart = clock();
    voc->saveToBinaryFile(outfile);
    LOGGER("Saving as binary: %.2fs\n", (double) (clock() - tStart) / CLOCKS_PER_SEC);
}

void txt_2_bin() {
    ORB_SLAM2::ORBVocabulary *voc = new ORB_SLAM2::ORBVocabulary();
    load_as_text(voc, "/storage/emulated/0/SLAM/VOC/ORBvoc.txt");
    save_as_binary(voc, "/storage/emulated/0/SLAM/VOC/ORBvoc.bin");
}

cv::Point2f Camera2Pixel(cv::Mat poseCamera, cv::Mat mk) {
    return Point2f(
            poseCamera.at<float>(0, 0) / poseCamera.at<float>(2, 0) * mk.at<float>(0, 0) +
            mk.at<float>(0, 2),
            poseCamera.at<float>(1, 0) / poseCamera.at<float>(2, 0) * mk.at<float>(1, 1) +
            mk.at<float>(1, 2)
    );
}


extern "C"
JNIEXPORT jfloatArray JNICALL
Java_com_example_ys_orbtest_OrbTest_CVTest(JNIEnv *env, jobject instance, jlong matAddr, jlong rtAddr) {

#ifndef BOWISBIN
    if(ttrack == 0)
    txt_2_bin();
    ttrack++;
#else
    LOGGER("Tracking current frame");
    cv::Mat *pMat = (cv::Mat *) matAddr;
    cv::Mat *rtMat = (cv::Mat *) rtAddr;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ttrack = std::chrono::duration_cast<std::chrono::duration<double >>(t1 - t0).count();

    clock_t start, end;
    start = clock();

    //LOGI("new frame come here===============");
    //ttrack Indicates the frame number
    cv::Mat pose = SLAM.TrackMonocular(*pMat, ttrack, *rtMat);
    end = clock();
    LOGGER("Get Pose Use Time=%f\n", ((double) end - start) / CLOCKS_PER_SEC);

    static bool instialized = false;
    static bool markerDetected = false;
    if (SLAM.MapChanged()) {
        instialized = false;
        markerDetected = false;
    }

    float dist = -1.0f;
    bool calc = false;
    if (!pose.empty()) {

        cv::Mat rotationVec;
        cv::Rodrigues(pose.colRange(0, 3).rowRange(0, 3), rotationVec);
        cv::Mat translateVec = pose.col(3).rowRange(0, 3);

        const vector<ORB_SLAM2::MapPoint *> vpMPs = SLAM.mpTracker->mpMap->GetAllMapPoints();
        const vector<ORB_SLAM2::MapPoint *> vpTMPs = SLAM.GetTrackedMapPoints();
        vector<cv::KeyPoint> vKPs = SLAM.GetTrackedKeyPointsUn();


        //  #MapPointToImage
        if (vpMPs.size() > 0) {
            std::vector<cv::Point3f> allmappoints;
            for (size_t i = 0; i < vpMPs.size(); i++) {
                if (vpMPs[i]) {

                    cv::Point3f pos = cv::Point3f(vpMPs[i]->GetWorldPos());
                    allmappoints.push_back(pos);
//                  LOGE("Point's world pose is %f %f %f",pos.x,pos.y,pos.z );
                }
            }
            LOGGER("all map points size %d", allmappoints.size());
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(allmappoints, rotationVec, translateVec, SLAM.mpTracker->mK,
                              SLAM.mpTracker->mDistCoef, projectedPoints);

            float bestCentered = INT_MAX;
            float tempPos;
            int index;
            for (size_t j = 0; j < projectedPoints.size(); ++j) {
                cv::Point2f r1 = projectedPoints[j];
                if (r1.x < 640 && r1.x > 0 && r1.y > 0 && r1.y < 480) {
                    tempPos = sqrt((r1.x - CENTER_W) * (r1.x - CENTER_W) + (r1.y - CENTER_H) * (r1.y - CENTER_H));
                    if (tempPos < bestCentered) {
                        bestCentered = tempPos;
                        index = j;
                    }
                }
            }
            //



            for (size_t j = 0; j < projectedPoints.size(); ++j) {
                cv::Point2f r1 = projectedPoints[j];
                if (r1.x < 640 && r1.x > 0 && r1.y > 0 && r1.y < 480)
                    if (index != j) {
                        cv::circle(*pMat, cv::Point(r1.x, r1.y), 2, cv::Scalar(0, 255, 0), 1, LINE_8);
                        continue;
                    }
                    else {
                        cv::circle(*pMat, cv::Point(r1.x, r1.y), 4, cv::Scalar(255, 0, 0), 1,
                                   LINE_8);
                        cv::circle(*pMat, cv::Point(r1.x, r1.y), 3, cv::Scalar(255, 0, 0), 1,
                                   LINE_8);
                        cv::circle(*pMat, cv::Point(r1.x, r1.y), 2, cv::Scalar(255, 0, 0), 1,
                                   LINE_8);
                    }
            }

            //  Product logic
            /*
            cv::Point3f dot = allmappoints[index];
            float tX = 0;
            float tY = 0;
            float tZ = 0;
            cv::Mat curPos = (*rtMat).col(0).rowRange(0,3);
            float scaleX = (curPos.at<float>(0,0) - prevPos.at<float>(0,0));
            if (abs(translateVec.at<float>(0,0) - lastTranslate.at<float>(0,0)) > 0.01) {
                scaleX = abs(scaleX / (translateVec.at<float>(0,0) - lastTranslate.at<float>(0,0)));
            } else {
                scaleX = 1;
            }
            float scaleY = (curPos.at<float>(1,0) - prevPos.at<float>(1,0));
            if (abs((translateVec.at<float>(1,0) - lastTranslate.at<float>(1,0))) > 0.01) {
                scaleY = abs (scaleY / (translateVec.at<float>(1,0) - lastTranslate.at<float>(1,0)));
            } else {
                scaleY = 1;
            }
            float scaleZ = (curPos.at<float>(2,0) - prevPos.at<float>(2,0));
            if (abs((translateVec.at<float>(1,0) - lastTranslate.at<float>(1,0))) > 0.01) {
                scaleZ = abs(scaleZ / (translateVec.at<float>(1,0) - lastTranslate.at<float>(1,0)));
            } else {
                scaleZ = 1;
            }
            dist = sqrt((dot.x / scaleX - tX) * (dot.x * scaleX - tX)  + (dot.y * scaleY - tY) * (dot.y * scaleY - tY) + (dot.z * scaleZ - tZ)*(dot.z * scaleZ - tZ));
            calc = true;
            lastTranslate = translateVec.clone();
            if (sqrt(pow(prevPos.at<float>(0,0) - curPos.at<float>(0,0), 2) + pow(prevPos.at<float>(1,0) - curPos.at<float>(1,0),2) + pow(prevPos.at<float>(2,0) - curPos.at<float>(2,0),2) > 0.8))
                prevPos = (*rtMat).col(0).rowRange(0, 3);


            if (instialized == false){
                Plane mplane;
                cv::Mat tempTpw, rpw, rwp, tpw, twp;
                LOGGER("Detect  Plane");
                tempTpw = mplane.DetectPlane(pose, vpTMPs, 50);
                if (!tempTpw.empty()) {
                    LOGGER("Find  Plane");
                    rpw = tempTpw.rowRange(0, 3).colRange(0, 3);
                    for (int row = 0; row < 4; row++) {
                        LOGGER(" tempTpw %f %f %f %f", tempTpw.at<float>(row, 0),
                             tempTpw.at<float>(row, 1), tempTpw.at<float>(row, 2),
                             tempTpw.at<float>(row, 3));
                    }
                    tpw = tempTpw.col(3).rowRange(0, 3);
                    rwp = rpw.t();
                    twp = -rwp * tpw;
                    rwp.copyTo(Plane2World.rowRange(0, 3).colRange(0, 3));
                    for (int row = 0; row < 3; row++) {
                        LOGGER(" rwp %f %f %f", rwp.at<float>(row, 0), rwp.at<float>(row, 1),
                             rwp.at<float>(row, 2));
                    }
                    twp.copyTo(Plane2World.col(3).rowRange(0, 3));
                    for (int row = 0; row < 4; row++) {
                        LOGGER(" Plane2World %f %f %f %f", Plane2World.at<float>(row, 0),
                             Plane2World.at<float>(row, 1), Plane2World.at<float>(row, 2),
                             Plane2World.at<float>(row, 3));
                    }
                    centroid = mplane.o;
                    LOGGER("Centroid is %f %f %f", mplane.o.at<float>(0, 0), mplane.o.at<float>(1, 0),
                         mplane.o.at<float>(2, 0));
                    instialized = true;
                    LOGGER("Find  Plane");
                    Plane2World = tempTpw;
                }

            } else {

                cv::Mat Plane2Camera = pose * Plane2World;
//                vector<cv::Mat> axisPoints(4);
//                axisPoints[0] = (cv::Mat_ <float>(4,1)<<0,0,0,1);
//                axisPoints[1] = (cv::Mat_ <float>(4,1)<<0.3,0,0,1);
//                axisPoints[2] = (cv::Mat_ <float>(4,1)<<0,0.3,0,1);
//                axisPoints[3] = (cv::Mat_ <float>(4,1)<<0,0,0.3,1);
//                vector<cv::Point2f> drawPoints(4);
//                for(int i = 0 ; i < 4; i++){
//                    axisPoints[i] = Plane2Camera*axisPoints[i];
//
//                    drawPoints[i] = Camera2Pixel(axisPoints[i],SLAM.mpTracker->mK);
//                    LOGE("drawPoints x y %f %f",drawPoints[i].x,drawPoints[i].y);
//                }
//                LOGE("axisPoints x y %f %f %f",axisPoints[0].at<float>(0,0),axisPoints[0].at<float>(1,0),axisPoints[0].at<float>(2,0));
//                cv::line(*pMat, drawPoints[0],drawPoints[1], cv::Scalar(250, 0, 0), 5);
//                cv::line(*pMat, drawPoints[0],drawPoints[2], cv::Scalar(0, 250, 0), 5);
//                cv::line(*pMat, drawPoints[0],drawPoints[3], cv::Scalar(0, 0, 250), 5);

                vector<cv::Point3f> drawPoints(8);
                drawPoints[0] = cv::Point3f(0, 0, 0);
                drawPoints[1] = cv::Point3f(0.3, 0.0, 0.0);
                drawPoints[2] = cv::Point3f(0.0, 0, 0.3);
                drawPoints[3] = cv::Point3f(0.0, 0.3, 0);
                drawPoints[4] = cv::Point3f(0, 0.3, 0.3);
                drawPoints[5] = cv::Point3f(0.3, 0.3, 0.3);
                drawPoints[6] = cv::Point3f(0.3, 0, 0.3);
                drawPoints[7] = cv::Point3f(0.3, 0.3, 0);



//                for(int i = 0 ; i < 4 ;i ++){
//                    drawPoints[i].x += centroid.at<float>(0,0);
//                    drawPoints[i].y += centroid.at<float>(1,0);
//                    drawPoints[i].z += centroid.at<float>(2,0);
//                }

                for (int row = 0; row < 4; row++) {
                    LOGGER(" Plane2Camera %f %f %f %f", Plane2Camera.at<float>(row, 0),
                         Plane2Camera.at<float>(row, 1), Plane2Camera.at<float>(row, 2),
                         Plane2Camera.at<float>(row, 3));
                }
                cv::Mat Rcp, Tcp;
                cv::Rodrigues(Plane2Camera.rowRange(0, 3).colRange(0, 3), Rcp);
                LOGGER(" rwp %f %f %f", Rcp.at<float>(0, 0), Rcp.at<float>(1, 0),
                     Rcp.at<float>(2, 2));

                Tcp = Plane2Camera.col(3).rowRange(0, 3);
                LOGGER("Tcp %f %f %f", Tcp.at<float>(0, 0), Tcp.at<float>(1, 0), Tcp.at<float>(2, 0));
//                cv::Rodrigues(pose.rowRange(0,3).colRange(0,3),Rcp);
//                Tcp = pose.col(3).rowRange(0,3);
                cv::projectPoints(drawPoints, Rcp, Tcp, SLAM.mpTracker->mK,
                                  SLAM.mpTracker->mDistCoef, projectedPoints);

//                cv::line(*pMat, projectedPoints[0],projectedPoints[1], cv::Scalar(250, 0, 0), 5); //画X轴 红色
//                cv::line(*pMat, projectedPoints[0],projectedPoints[2], cv::Scalar(0, 250, 0), 5);//画Z轴  绿色
//                cv::line(*pMat, projectedPoints[0],projectedPoints[3], cv::Scalar(0, 0, 250), 5);//画Y轴 蓝色
//                cv::line(*pMat, projectedPoints[1],projectedPoints[7], cv::Scalar(10, 0, 50), 2);
//                cv::line(*pMat, projectedPoints[3],projectedPoints[7], cv::Scalar(20, 0, 50),2);
//                cv::line(*pMat, projectedPoints[3],projectedPoints[4], cv::Scalar(30, 0, 50),2 );
//                cv::line(*pMat, projectedPoints[2],projectedPoints[4], cv::Scalar(40, 0, 50),2);
//                cv::line(*pMat, projectedPoints[1],projectedPoints[6], cv::Scalar(50, 0, 50), 2);
//                cv::line(*pMat, projectedPoints[2],projectedPoints[6], cv::Scalar(60, 0, 50), 2);
//                cv::line(*pMat, projectedPoints[4],projectedPoints[5], cv::Scalar(70, 0, 50), 2);
//                cv::line(*pMat, projectedPoints[5],projectedPoints[6], cv::Scalar(80, 0, 50), 2);
//                cv::line(*pMat, projectedPoints[5],projectedPoints[7], cv::Scalar(90, 0, 50), 2);

//                cv::circle(*pMat, projectedPoints[0], 2, cv::Scalar(0, 0, 250), 1, 8);

            }
            */
        }
    }
    char rt_string[1000];

//    sprintf(rt_string, "[%.2f,%.2f,%.2f]\0", (*rtMat).at<float>(0,0),(*rtMat).at<float>(0,1),(*rtMat).at<float>(0,2));
//    cv::putText(*pMat, rt_string , cv::Point(0,150), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);
//    sprintf(rt_string, "[%.2f,%.2f,%.2f]\0", (*rtMat).at<float>(1,0),(*rtMat).at<float>(1,1),(*rtMat).at<float>(1,2));
//    cv::putText(*pMat, rt_string , cv::Point(0,170), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);
//    sprintf(rt_string, "[%.2f,%.2f,%.2f]\0", (*rtMat).at<float>(2,0), (*rtMat).at<float>(2,1), (*rtMat).at<float>(2,2));
//    cv::putText(*pMat, rt_string , cv::Point(0,190), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);
//
//    sprintf(rt_string, "[%.2f,%.2f,%.2f]\0", (*rtMat).at<float>(0,3),(*rtMat).at<float>(1,3),(*rtMat).at<float>(2,3));
//    cv::putText(*pMat, rt_string , cv::Point(0,210), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(170,170,255), 2);

    switch (SLAM.GetTrackingState()) {
        case -1: {
            cv::putText(*pMat, "SYSTEM NOT READY", cv::Point(0, 400), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 0, 0), 2);
        }
            break;
        case 0: {
            cv::putText(*pMat, "NO IMAGES YET", cv::Point(0, 400), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 0, 0), 2);
        }
            break;
        case 1: {
            cv::putText(*pMat, "SLAM NOT INITIALIZED", cv::Point(0, 400), cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(255, 0, 0), 2);
        }
            break;
        case 2: {
            cv::putText(*pMat, "SLAM ON", cv::Point(0, 400), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
            break;
        }
        case 3: {
            cv::putText(*pMat, "SLAM LOST", cv::Point(0, 400), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 0, 0), 2);
            break;
        }
        default:
            break;
    }



    /**Return the resulting camera pose matrix to the java code to update the opengl camera pose later.**/
    cv::Mat ima = pose;
    jfloatArray resultArray = env->NewFloatArray(ima.rows * ima.cols + 1);
    jfloat *resultPtr;

    resultPtr = env->GetFloatArrayElements(resultArray, 0);
    for (int i = 0; i < ima.rows; i++)
        for (int j = 0; j < ima.cols; j++) {
            float tempdata = ima.at<float>(i, j);
            resultPtr[i * ima.rows + j] = tempdata;
        }

    resultPtr[ima.rows * ima.cols] = dist;

    env->ReleaseFloatArrayElements(resultArray, resultPtr, 0);
    return resultArray;
#endif
}

