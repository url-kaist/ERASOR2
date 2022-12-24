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
        string abs_data_dir = "/media/shapelim/Elements/semantic_kitti_raw";
        string sequence = "05";
        loader = std::move(std::make_unique<SemanticKITTILoader>(abs_data_dir, sequence));
//        loader = std::move(std::make_unique<SemanticKITTILoader>());
    }
    cout << "Load complete3" << endl;
    loader->testInheritance();
//     => "I'm Kitti Loader / /home/shapelim"

    return 0;
}