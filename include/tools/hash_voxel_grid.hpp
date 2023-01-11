#ifndef HASH_VOXEL_GRID_H
#define HASH_VOXEL_GRID_H

#include <iostream>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HASH_P 116101
#define MAX_N 10000000000

using namespace std;

template<typename PointT>
class VOXEL_LOC {
public:
    int64_t vloc[3];

    VOXEL_LOC(const int64_t vx = 0, const int64_t vy = 0, const int64_t vz = 0) {
        vloc[0] = vx;
        vloc[1] = vy;
        vloc[2] = vz;
    }

    VOXEL_LOC(const PointT p, const double lx, const double ly, const double lz) {
        vloc[0] = (int64_t) (p.x / lx);
        vloc[1] = (int64_t) (p.y / ly);
        vloc[2] = (int64_t) (p.z / lz);
    }

    bool operator==(const VOXEL_LOC &other) const {
        return (vloc[0] == other.vloc[0] && vloc[1] == other.vloc[1] && vloc[2] == other.vloc[2]);
    }
};

// Hash value
namespace std {
    template<typename PointT>
    struct hash<VOXEL_LOC < PointT>> {
    int64_t operator()(const VOXEL_LOC <PointT> &s) const {
        using std::hash;
        using std::size_t;
        return ((((s.vloc[2]) * HASH_P) % MAX_N + (s.vloc[1])) * HASH_P) % MAX_N + (s.vloc[0]);
    }
};
}

template<typename PointT>
class Voxel {

public:
    pcl::PointCloud<PointT> points;

    Eigen::Vector3d mean_;

    VOXEL_LOC<PointT>         pos;
    vector<VOXEL_LOC<PointT>> nearby_pos_;
    vector<VOXEL_LOC<PointT>> vertical_pos_;

    int size() {
        return points.size();
    }

    Voxel(VOXEL_LOC<PointT> _pos, int _adj = 2, int _ver = 5) {

        pos = _pos;

        int      adj = _adj;
        for (int i   = -adj; i <= adj; i++) {
            for (int j = -adj; j <= adj; j++) {
                for (int k = -adj; k <= adj; k++) {
                    if (i == 0 && j == 0 && k == 0) continue;
                    VOXEL_LOC<PointT> near_pos;
                    near_pos.vloc[0] = pos.vloc[0] + i;
                    near_pos.vloc[1] = pos.vloc[1] + j;
                    near_pos.vloc[2] = pos.vloc[2] + k;
                    nearby_pos_.push_back(near_pos);
                }
            }
        }
        int      ver = _ver;
        for (int i   = -ver; i < 0; i++) {
            VOXEL_LOC ver_pos = pos;
            ver_pos.vloc[2] += i;
            vertical_pos_.push_back(ver_pos);
        }
    }
};

template<typename PointT>
class HashVoxelGrid {

private:
    std::unordered_map<VOXEL_LOC<PointT>, Voxel<PointT>> voxels_;
public:
    // We follow original VoxelGrid:
    // https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/voxel_grid.h
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;
    using VoxelLoc = VOXEL_LOC<PointT>;

    PointCloud input_;

    vector<float> leaf_size_ = {0.0, 0.0, 0.0};

    int min_points_per_voxel_ = 0;

    PCL_MAKE_ALIGNED_OPERATOR_NEW;

    /** \brief Destructor. */
    HashVoxelGrid () {};
    ~HashVoxelGrid () {};

    inline void
    setLeafSize (float lx, float ly, float lz)
    {
        leaf_size_[0] = lx;
        leaf_size_[1] = ly;
        leaf_size_[2] = lz;
    }

    /** \brief Set the minimum number of points required for a voxel to be used.
    * \param[in] min_points_per_voxel the minimum number of points for required for a voxel to be used
    */
    inline void
    setMinimumPointsNumberPerVoxel (unsigned int min_points_per_voxel) { min_points_per_voxel_ = min_points_per_voxel; }

    /** \brief Return the minimum number of points required for a voxel to be used.
     */
    inline unsigned int
    getMinimumPointsNumberPerVoxel () const { return min_points_per_voxel_; }

    inline void setInputCloud(const PointCloudPtr &cloud) {
        input_ = *cloud;
    }

    inline void filter(PointCloud& output) {
        voxels_.clear();

        for (const auto& point: input_.points) {
            double loc_xyz[3] = {point.x / leaf_size_[0], point.y / leaf_size_[1], point.z / leaf_size_[2]};

            for (int j = 0; j < 3; j++) {
                if (loc_xyz[j] < 0) {
                    loc_xyz[j] -= 1.0;
                }
            }

            VoxelLoc pos((int64_t) loc_xyz[0], (int64_t) loc_xyz[1], (int64_t) loc_xyz[2]);

            auto iter = voxels_.find(pos);
            if (iter != voxels_.end()) {
                iter->second.points.emplace_back(point);
            } else {
                Voxel<PointT> new_vox(pos);
                new_vox.points.emplace_back(point);
                voxels_.insert({pos, new_vox});
            }
        }

        output.reserve(input_.size());
        PointT centroid_pt;
        for (const auto& voxel: voxels_) {
            if (voxel.second.points.size() < min_points_per_voxel_) {
                continue;
            }

            Eigen::Vector3d centroid(0,0,0);

            for (auto p: voxel.second.points) {
                Eigen::Vector3d pt(p.x, p.y, p.z);

                        centroid += pt;
            }
            centroid = centroid / voxel.second.points.size();
            centroid_pt.x = centroid(0);
            centroid_pt.y = centroid(1);
            centroid_pt.z = centroid(2);
            output.emplace_back(centroid_pt);
        }
    }

//    void rejectNeighboringPoints() {
//        // Compute each normal in the voxel
//        vector<VoxelLoc> voxels_to_be_deleted;
//        for (auto        &voxel: voxels_) {
//
//            if (voxel.second.points.size() < min_points_per_voxel_) {
//                voxels_to_be_deleted.push_back(voxel.first);
//                continue;
//            }
//
//            Eigen::Vector3d center(0, 0, 0);
//            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
//
//            for (auto p: voxel.second.points) {
//                Eigen::Vector3d eig_p(p.x, p.y, p.z);
//
//                center += eig_p;
//                covariance += eig_p * eig_p.transpose();
//            }
//            center     = center / voxel.second.points.size();
//            covariance = covariance / voxel.second.points.size() - center * center.transpose();
//
//
//
//        }
//
//        for (auto idx: voxels_to_be_deleted) {
//            voxels_.erase(idx);
//        }
//
//        cout << "remaining num of voxels_: " << voxels_.size() << " (min # of points: " << min_points_per_voxel_ << ")"
//             << endl;
//    }

};

#endif
