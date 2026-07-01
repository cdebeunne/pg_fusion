#ifndef CLOCKSYNCHRONIZER_HPP
#define CLOCKSYNCHRONIZER_HPP

#include <filesystem>
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

#include <Eigen/Dense>

typedef std::pair<unsigned long long int, unsigned long long int> clock_pair_t;

/**
 * t1 is ref
 * Model:
 *  offset(k+1) = offset(k) + drift(k)*dt   + jitter noise
 *  drift(k+1)  = drift(k)                  + drift noise
 * Simple case:
 *  t2 = t1 + offset
//  * Advanced case:
//  *  t2 = drift * t1 + offset
 */
class ClockSynchronizer {
  public:
    void init(clock_pair_t meas) {
        double jitter_noise_s       = 1e-8;
        double drift_noise_s_per_s  = 1e-8;
        Eigen::Vector2d q(jitter_noise_s*jitter_noise_s, drift_noise_s_per_s*drift_noise_s_per_s);
        _Q = q.asDiagonal();

        double clock_dt_noise_s = 2e-1;
        _R = clock_dt_noise_s*clock_dt_noise_s;

        _F = Eigen::Matrix2d::Identity();
        // set off-diagonal elements dynamically    
        
        _H(0,0) = 1; 
        _H(0,1) = 0;

        double offset_std_s         = 1;
        double drift_std_s_per_s    = 1;
        Eigen::Vector2d p(offset_std_s*offset_std_s, drift_std_s_per_s*drift_std_s_per_s);
        _P = p.asDiagonal();

        _state(0) = (1e-9) * (double) (meas.second - meas.first);
        _state(1) = 0;

        _t1_nanos = meas.first;

        profiling();
        _is_init = true;
    }

    void propagate(unsigned long long int t1) {
        double dt_s = (1e-9) * (double) (t1 - _t1_nanos);
        _F(0,1) = dt_s;
        _state = _F * _state;
        _P = _F * _P * _F.transpose() + _Q;
        _t1_nanos = t1;
    }

    bool update(clock_pair_t meas) {
        // _H(0,1) = t1;
        bool isOutlier = testTimePair(meas);
        std::cout << "Clock KF: " << _state.transpose() << std::endl;
        std::cout << "Clock KF: " << _P << std::endl;
        profiling();
        if (!isOutlier && _S > 1e-12) {
            Eigen::Vector2d K = _P * _H.transpose() / _S;
            _state = _state + K * _innovation_s;
            Eigen::Matrix2d tmp = (Eigen::Matrix2d::Identity() - K*_H);
            _P = tmp * _P * tmp.transpose() + K*_R*K.transpose();
            profiling();
        }
        return isOutlier;
    }

    /**
     * Also updates the innovation / innovation covariance
     */
    bool testTimePair(clock_pair_t &meas)
    {
            unsigned long long int t1_nanos = meas.first;
            std::cout << "Clock KF: t1 " << t1_nanos << std::endl; 
            unsigned long long int t2_nanos = meas.second;
            std::cout << "Clock KF: t2 " << t2_nanos << std::endl; 
            unsigned long long int pred_t2_nanos = asT2(t1_nanos);
            std::cout << "Clock KF: t2*" << pred_t2_nanos << std::endl;  
            _S = _H * _P * _H.transpose() + _R;
            if (t2_nanos > pred_t2_nanos)
                _innovation_s = (1e-9) * (double) (t2_nanos - pred_t2_nanos);
            else   
                _innovation_s = (1e-9) *(-1)* (double) (pred_t2_nanos - t2_nanos);
            std::cout << "Clock KF: innovation (s) " << _innovation_s << std::endl;
            std::cout << "Clock KF: innov. std (s) " << sqrt(_S) << std::endl;

            // test
            if (abs(_innovation_s) > (4 * sqrt(_S)))
                return true;
            else
                return false;
    }

    unsigned long long int asT2(unsigned long long int t1_nanos) {
        // double offset = _state(0);
        // double drift = _state(1);
        // unsigned long long int t2 = drift * (offset + t1);
        // _H(0,1) = t1;
        std::cout << "Clock KF:_t1 " << _t1_nanos << std::endl;
        std::cout << "Clock KF: t1 " << t1_nanos << std::endl; 
        double dt_s = (1e-9) * (double) (t1_nanos - _t1_nanos);
        std::cout << "Clock KF: dt " << dt_s << std::endl; 
        _F(0,1) = dt_s;
        unsigned long long int offset_nanos = (unsigned long long int) ((1e9) *_H * _F * _state);
        std::cout << "Clock KF: offset " << offset_nanos << std::endl;
        unsigned long long int t2_nanos = offset_nanos + t1_nanos;
        std::cout << "Clock KF: t2*" << t2_nanos << std::endl;
        return t2_nanos;
    }

    unsigned long long int asT1(unsigned long long int t2) {
        
        unsigned long long int t1 = 0;
        return t1;
    }

    void profiling() {
        std::ofstream out(_outpath / "res.csv", std::ofstream::out | std::ofstream::app);
        if (!_is_init) {
            if (std::filesystem::is_regular_file(_outpath))
                _outpath = _outpath.parent_path();

            if (!std::filesystem::is_directory(_outpath))
                std::filesystem::create_directory(_outpath);

            out << "t (ns), ";
            out << "state(0), " << "state(1), ";
            out << "P(00)," << "P(01), " << "P(11), ";
            out << "innovation, "; 
            out << "sqrt(S), ";
            out << "\n";
        }
        out << _t1_nanos << ",";
        out << _state(0) << "," << _state(1) << ",";
        out << _P(0,0) << "," << _P(0,1) << "," << _P(1,1) << ",";
        out << _innovation_s << ",";
        out << sqrt(_S) << ",";
        out << "\n";
        out.close();
    }

    bool _is_init = false;
  protected:
    std::filesystem::path _outpath = std::filesystem::path("log_clockKF");

    unsigned long long int _t1_nanos;
    Eigen::Vector2d _state;
    Eigen::Matrix2d _P;
    Eigen::Matrix2d _Q;
    Eigen::Matrix2d _F;
    Eigen::Matrix<double, 1, 2> _H;
    double _R, _S, _innovation_s;
    
};
#endif