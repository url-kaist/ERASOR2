//
// Created by Hyungtae Lim on 22.12.22.
//
#include "dataloader/dataloader.h"

int main() {
    string mode = "SemanticKITTI";

    cout << "Load complete" << endl;
    std::unique_ptr<DataLoader> loader;
    cout << "Load complete2" << endl;
    if (mode == "SemanticKITTI") {
        string cloud_dir = "/media/shapelim/Elements/semantic_kitti_raw/00/velodyne";
        string cloud_format = "bin";
        string pose_path = "whassup";
        loader = std::move(std::make_unique<SemanticKITTILoader>(cloud_dir, cloud_format, pose_path));
//        loader = std::move(std::make_unique<SemanticKITTILoader>());
    }
    cout << "Load complete3" << endl;
    loader->printTest();
    cout << loader->cloud_dir_ << endl;
    cout << loader->cloud_format_ << endl;
    cout << loader->pose_path_ << endl;
//     => "I'm Kitti Loader / /home/shapelim"

    return 0;
}