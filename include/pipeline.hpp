#ifndef PIPELINE_H
#define PIPELINE_H

#include "data/navframe.hpp"
#include "PGParameters.hpp"
#include "poseGraph.hpp"
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>

double deg2rad = M_PI / 180;

class Pipeline {
  public:
    Pipeline(std::shared_ptr<isae::SLAMCore> slam,
            std::shared_ptr<PGParameters> param)
        : _slam(slam), _param(param) {

        // Ellipsoid parameters of the WGS84 convention
        _a  = 6378137.0f;                           //< semi-major axis (cf. Earth radius) [m]
        _f  = (1.0f / 298.257223563);               //< flattening constant
        _e2 = 1 - (1 - _f) * (1 - _f);              //< ellipsoidal shape parameter

        _T_n_f   = Eigen::Affine3d::Identity();     //< transformation estimate local -> navigation frame (ENU)
        _T_n_w   = Eigen::Affine3d::Identity();     //< transformation estimate world -> navigation frame (ECEF)
        _is_init = false;
        _pg      = std::make_shared<PoseGraph>();   //
        _llh_ref = Eigen::Vector3d::Zero();         //< origin of local frame as llh
        _ecef_ref = Eigen::Vector3d::Zero();        //< origin of local frame as ECEF
    };

    void setRef(const Eigen::Vector3d &llh_ref);
    const Eigen::Vector3d llhToEcef(const Eigen::Vector3d &llh);
    const Eigen::Vector3d ecefToENU(const Eigen::Vector3d &ecef);
    const Eigen::Vector3d enuToECEF(const Eigen::Vector3d &enu);

    std::shared_ptr<NavFrame> next();
    void calibrateRotation();
    void calibrateRotation4DoF();
    void updateRelativeFactors();

    /*!
     * @brief TODO 
     */
    void initProfiling(const std::filesystem::path& p);
    void initProfiling() {
        // Create an empty path object to use default initialization
        initProfiling(std::filesystem::path("log_pg"));
    }

    /*!
     * @brief A function to monitor the GVIO behaviour
     */
    void profiling();

    void run();
    void init();
    void step();

    // bool rfConsistencyCheck();

    void publishLatestNF() {
      if (!_nav_frames.empty()) {
        std::lock_guard<std::mutex> lock(mutex_pub);
        _nf_to_pub.push(_nav_frames.back());
      }
    }

    std::shared_ptr<NavFrame> getPublishableNF() {
      if (!_nf_to_pub.empty()) {
        std::lock_guard<std::mutex> lock(mutex_pub);
        std::shared_ptr<NavFrame> nf = _nf_to_pub.front();
        _nf_to_pub.pop();
        return nf;
      } else {
        return nullptr;
      }
    }

    std::shared_ptr<isae::SLAMCore> _slam; // VSLAM
    std::shared_ptr<PoseGraph> _pg;        // Pose graph
    bool _is_init, _is_aligned;
    double _a, _f, _e2;      // Ellipsoid parameters for Earth coordinates
    Eigen::Affine3d _T_n_f;  // Current pose in local ENU frame
    Eigen::Matrix3d _R_n_e;  // Rotation between ENU and ECEF
    Eigen::Affine3d _T_n_w;  // Rotation between ENU frame and SLAM (world) frame
    Eigen::Affine3d _T_a_f;  // Transformation between antena and frame
    double _thresh_cov;      // Threshold on the covariance of GNSS estimates
    uint _window_size;       // Size of the sliding window
    bool _remove_z_estimate; // Remove the z estimate from the GNSS
    Eigen::Vector3d _llh_ref, _ecef_ref;
    std::queue<std::shared_ptr<NavFrame>> _nf_queue;      // Queue of frames waiting to be processed
    std::deque<std::shared_ptr<NavFrame>> _nav_frames;    // All frames in the current sliding window
    std::queue<std::shared_ptr<NavFrame>> _nf_to_pub;     // Queue of processed frames waiting to be published / visualized
    std::vector<std::pair<unsigned long long, Eigen::Affine3d>> _removed_frame_poses, _removed_vo_poses;
    std::shared_ptr<NavFrame> _nf, _nf_init;
    std::shared_ptr<PGParameters> _param;

  protected:  
    std::filesystem::path profiling_path;
    std::mutex mutex_pub;

  private:
    double __t_offset_gnss_img; // clock offset GNSS to camera [s]
};

#endif // PIPELINE_H