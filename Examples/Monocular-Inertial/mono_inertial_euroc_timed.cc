/**
 * Timed ORB-SLAM3 Monocular-Inertial EuRoC-style runner.
 *
 * Measures per-frame computation time for:
 *  - image disk read
 *  - IMU packet batching
 *  - ORB-SLAM3 TrackMonocular()
 *  - total compute time
 *  - real-time budget and deadline miss
 *
 * Output:
 *  - timing_orbslam3_mono_inertial.csv
 *  - CameraTrajectory.txt
 *  - KeyFrameTrajectory.txt
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "System.h"

using namespace std;

static double now_ms()
{
    using clock = std::chrono::steady_clock;
    static const auto t0 = clock::now();
    auto t = clock::now();
    return std::chrono::duration<double, std::milli>(t - t0).count();
}

static void LoadImages(
    const string &strPathToSequence,
    const string &strPathTimes,
    vector<string> &vstrImageFilenames,
    vector<double> &vTimestamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());

    if (!fTimes.is_open())
    {
        cerr << "ERROR: Cannot open times file: " << strPathTimes << endl;
        exit(EXIT_FAILURE);
    }

    vTimestamps.clear();
    vstrImageFilenames.clear();

    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;

            // ORB-SLAM3 EuRoC convention: times file may be in ns.
            // If it is large, convert to seconds.
            if (t > 1e12)
                t = t / 1e9;

            vTimestamps.push_back(t);

            string imageName = s;
            // Keep original string for filename. If it was ns, the file name is ns.png.
            vstrImageFilenames.push_back(strPathToSequence + "/mav0/cam0/data/" + imageName + ".png");
        }
    }
}

static void LoadIMU(
    const string &strPathToSequence,
    vector<double> &vTimestampsImu,
    vector<cv::Point3f> &vAcc,
    vector<cv::Point3f> &vGyro)
{
    string imuPath = strPathToSequence + "/mav0/imu0/data.csv";
    ifstream fImu;
    fImu.open(imuPath.c_str());

    if (!fImu.is_open())
    {
        cerr << "ERROR: Cannot open IMU file: " << imuPath << endl;
        exit(EXIT_FAILURE);
    }

    vTimestampsImu.clear();
    vAcc.clear();
    vGyro.clear();

    while (!fImu.eof())
    {
        string s;
        getline(fImu, s);

        if (s.empty() || s[0] == '#')
            continue;

        string item;
        vector<string> tokens;
        stringstream ss(s);

        while (getline(ss, item, ','))
            tokens.push_back(item);

        if (tokens.size() < 7)
            continue;

        double t = stod(tokens[0]);

        // NTU/EuRoC IMU CSV timestamp is normally ns.
        if (t > 1e12)
            t = t / 1e9;

        cv::Point3f gyro(
            stof(tokens[1]),
            stof(tokens[2]),
            stof(tokens[3])
        );

        cv::Point3f acc(
            stof(tokens[4]),
            stof(tokens[5]),
            stof(tokens[6])
        );

        vTimestampsImu.push_back(t);
        vGyro.push_back(gyro);
        vAcc.push_back(acc);
    }
}

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        cerr << endl
             << "Usage: ./mono_inertial_euroc_timed "
             << "path_to_vocabulary path_to_settings path_to_sequence path_to_times "
             << "[output_csv]" << endl;
        return 1;
    }

    const string vocabPath = argv[1];
    const string settingsPath = argv[2];
    const string sequencePath = argv[3];
    const string timesPath = argv[4];

    string outputCsv = "timing_orbslam3_mono_inertial.csv";
    if (argc >= 6)
        outputCsv = argv[5];

    vector<string> imageFiles;
    vector<double> imageTimes;
    vector<double> imuTimes;
    vector<cv::Point3f> acc;
    vector<cv::Point3f> gyro;

    cout << "Loading images..." << endl;
    LoadImages(sequencePath, timesPath, imageFiles, imageTimes);
    cout << "Loaded images: " << imageFiles.size() << endl;

    cout << "Loading IMU..." << endl;
    LoadIMU(sequencePath, imuTimes, acc, gyro);
    cout << "Loaded IMU samples: " << imuTimes.size() << endl;

    if (imageFiles.empty())
    {
        cerr << "ERROR: No images loaded." << endl;
        return 1;
    }

    if (imuTimes.empty())
    {
        cerr << "ERROR: No IMU samples loaded." << endl;
        return 1;
    }

    ofstream csv(outputCsv);
    if (!csv.is_open())
    {
        cerr << "ERROR: Cannot write CSV: " << outputCsv << endl;
        return 1;
    }

    csv << "frame_id,timestamp_sec,dataset_dt_ms,"
        << "image_read_ms,imu_batch_ms,track_ms,total_compute_ms,"
        << "realtime_budget_ms,realtime_factor,deadline_miss,"
        << "sleep_ms,num_imu_samples,image_width,image_height\n";

    cout << endl;
    cout << "Starting ORB-SLAM3 timed run..." << endl;
    cout << "Output timing CSV: " << outputCsv << endl;

    ORB_SLAM3::System SLAM(
        vocabPath,
        settingsPath,
        ORB_SLAM3::System::IMU_MONOCULAR,
        true
    );

    size_t imuIndex = 0;

    double wallStartMs = now_ms();
    double datasetStartTime = imageTimes[0];

    for (size_t i = 0; i < imageFiles.size(); ++i)
    {
        const double frameStartMs = now_ms();

        const double tFrame = imageTimes[i];

        double datasetDtMs = 0.0;
        if (i > 0)
            datasetDtMs = (imageTimes[i] - imageTimes[i - 1]) * 1000.0;
        else if (imageTimes.size() > 1)
            datasetDtMs = (imageTimes[1] - imageTimes[0]) * 1000.0;
        else
            datasetDtMs = 100.0;

        const double tRead0 = now_ms();
        cv::Mat im = cv::imread(imageFiles[i], cv::IMREAD_UNCHANGED);
        const double imageReadMs = now_ms() - tRead0;

        if (im.empty())
        {
            cerr << "ERROR: Failed to read image: " << imageFiles[i] << endl;
            continue;
        }

        const double tImu0 = now_ms();
        vector<ORB_SLAM3::IMU::Point> vImuMeas;

        // Include IMU samples up to the current image timestamp.
        while (imuIndex < imuTimes.size() && imuTimes[imuIndex] <= tFrame)
        {
            vImuMeas.push_back(
                ORB_SLAM3::IMU::Point(
                    acc[imuIndex],
                    gyro[imuIndex],
                    imuTimes[imuIndex]
                )
            );
            imuIndex++;
        }

        const double imuBatchMs = now_ms() - tImu0;

        const double tTrack0 = now_ms();
        SLAM.TrackMonocular(im, tFrame, vImuMeas);
        const double trackMs = now_ms() - tTrack0;

        const double totalComputeMs = now_ms() - frameStartMs;

        const double realtimeBudgetMs = datasetDtMs;
        const double realtimeFactor =
            realtimeBudgetMs > 0.0 ? totalComputeMs / realtimeBudgetMs : 0.0;

        const int deadlineMiss = totalComputeMs > realtimeBudgetMs ? 1 : 0;

        double sleepMs = 0.0;

        // Real-time playback pacing:
        // If processing is faster than the dataset clock, sleep until real-time.
        const double targetElapsedMs = (tFrame - datasetStartTime) * 1000.0;
        const double wallElapsedMs = now_ms() - wallStartMs;

        if (targetElapsedMs > wallElapsedMs)
        {
            sleepMs = targetElapsedMs - wallElapsedMs;
            std::this_thread::sleep_for(std::chrono::milliseconds((int)sleepMs));
        }

        csv << i << ","
            << fixed << setprecision(9) << tFrame << ","
            << setprecision(3) << datasetDtMs << ","
            << imageReadMs << ","
            << imuBatchMs << ","
            << trackMs << ","
            << totalComputeMs << ","
            << realtimeBudgetMs << ","
            << realtimeFactor << ","
            << deadlineMiss << ","
            << sleepMs << ","
            << vImuMeas.size() << ","
            << im.cols << ","
            << im.rows << "\n";

        if (i % 20 == 0)
        {
            cout << "Frame " << i << "/" << imageFiles.size()
                 << " | total=" << totalComputeMs << " ms"
                 << " | budget=" << realtimeBudgetMs << " ms"
                 << " | factor=" << realtimeFactor
                 << " | miss=" << deadlineMiss
                 << endl;
        }
    }

    csv.close();

    cout << endl;
    cout << "Shutting down SLAM..." << endl;
    SLAM.Shutdown();

    cout << "Saving trajectory to CameraTrajectory.txt ..." << endl;
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");

    cout << "Saving keyframe trajectory to KeyFrameTrajectory.txt ..." << endl;
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    cout << endl;
    cout << "Timed run complete." << endl;
    cout << "Timing CSV: " << outputCsv << endl;

    return 0;
}
