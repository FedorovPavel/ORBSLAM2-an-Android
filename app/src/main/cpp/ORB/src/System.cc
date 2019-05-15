/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
//#include <pangolin/pangolin.h>
#include <iomanip>
#include <unistd.h>
#include<ctime>
#include <android/log.h>

#define LOG_TAG    "system.cc@@@native-dev"
#define LOG_LEVEL ANDROID_LOG_INFO
#define LOGGER(...)  __android_log_print(LOG_LEVEL, LOG_TAG, __VA_ARGS__)

namespace ORB_SLAM2 {

    System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)),
                                            mbReset(false), mbActivateLocalizationMode(false),
                                            mbDeactivateLocalizationMode(false) {
        // Output welcome message
        cout << endl <<
             "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
             "This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
             "This is free software, and you are welcome to redistribute it" << endl <<
             "under certain conditions. See LICENSE.txt." << endl << endl;

        cout << "Input sensor was set to: ";
        LOGGER("Initiate ORB-SLAM2  System================\nInput sensor was set to: ");

        if (mSensor == MONOCULAR) {
            LOGGER("Monocular");
            cout << "Monocular" << endl;
        } else if (mSensor == STEREO)
            cout << "Stereo" << endl;
        else if (mSensor == RGBD)
            cout << "RGB-D" << endl;

        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            LOGGER("Failed to open settings file at: %s", strSettingsFile.c_str());
            return;
        }

        //Load ORB Vocabulary
        LOGGER("Loading ORB Vocabulary. This could take a while...");

        mpVocabulary = new ORBVocabulary();
        // bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        bool bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

        if (!bVocLoad) {
            LOGGER("Wrong path to vocabulary. ");
            LOGGER("Falied to open at: ");
            return;
        }

        LOGGER("Vocabulary loaded using binary file!");

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        LOGGER("Create KeyFrame Database");

        //Create the Map
        mpMap = new Map();
        LOGGER("Create the Map");

        //    Create Drawers. These are used by the Viewer
        //    mpFrameDrawer = new FrameDrawer(mpMap);
        //    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

        //Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap,
                    mpKeyFrameDatabase, strSettingsFile, mSensor);

        //Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR);
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

        //Initialize the Viewer thread and launch
        //    if(bUseViewer)
        //    {
        //        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        //        mptViewer = new thread(&Viewer::Run, mpViewer);
        //        mpTracker->SetViewer(mpViewer);
        //    }

        //Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
        LOGGER("ORB_SLAM system initialization finished\n========================================");
    }


    cv::Mat System::TrackMonocular(cv::Mat &im, const double &timestamp, cv::Mat &rt) {
        if (mSensor != MONOCULAR) {
            LOGGER("ERROR: you called TrackMonocular but input sensor was not set to Monocular.");
            cv::Mat nothing;
            return nothing;
        }
        LOGGER("[R]:\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",rt.at<float>(0,0), rt.at<float>(0,1), rt.at<float>(0,2), rt.at<float>(1,0), rt.at<float>(1,1), rt.at<float>(1,2), rt.at<float>(2,0), rt.at<float>(2,1), rt.at<float>(2,2));
        LOGGER("[T]:\n%f\n%f\n%f\n", rt.at<float>(3,0), rt.at<float>(3,1), rt.at<float>(3,2));

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                LOGGER("Request Stop !!!");
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }
                LOGGER("Only Tracking !!!!");
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                LOGGER("Deactiveate Localize !!!!");
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                LOGGER("Reset !!");
                mpTracker->Reset();
                mbReset = false;
            }
        }

        clock_t frame_start, frame_end;
        LOGGER("Start Grab Image Monocular!!");
        frame_start = clock();
        cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp, rt);
        frame_end = clock();
        LOGGER("oneFrame total Use Time=%f\n", ((double) frame_end - frame_start) / CLOCKS_PER_SEC);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;


        LOGGER("Track  Monocular finished!!!!");
        return Tcw;
    }


    void System::ActivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged() {
        static int n = 0;
        int curn = mpMap->GetLastBigChangeIdx();
        if (n < curn) {
            n = curn;
            return true;
        } else
            return false;
    }

    void System::Reset() {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown() {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
//    if(mpViewer)
//    {
//        mpViewer->RequestFinish();
//        while(!mpViewer->isFinished())
//            usleep(5000);
//    }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
               mpLoopCloser->isRunningGBA()) {
            usleep(5000);
        }

//    if(mpViewer)
//        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

    int System::GetTrackingState() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

} //namespace ORB_SLAM
