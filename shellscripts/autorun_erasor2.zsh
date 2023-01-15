s_fs=(4390 150 860 2350 630 0)
e_fs=(4530 250 950 2670 820 800)
seqs=("00" "01" "02" "05" "07" "19")

for jj in 1 2 3 4 5 6
do
    start_frame=${s_fs[$jj]}
    end_frame=${e_fs[$jj]}
    sequence=${seqs[$jj]}
    rosparam load "/home/shapelim/catkin_ws/src/ERASOR2/config/seq_"$sequence".yaml"
    for inst_seg in "cais" "hdbscan"
    do
        rosparam set "/dataloader/instance_seg_method" $inst_seg
        for rp_thr in 0.8 0.85 0.90
        do
            for gain in 1.0 1.5 2.0 2.5 3.0
            do
                for increment in 0.10 0.15 0.20 0.25 0.3
                do
                    rosparam set "erasor2/log_odds/increment_gain" $gain
                    rosparam set "erasor2/log_odds/increment" $increment
                    rosparam set "erasor2/region_proposal_thr" $rp_thr

                    rosrun erasor2 run_erasor2

                    python2.7 /home/shapelim/catkin_ws/src/ERASOR2/scripts/analysis.py --gt "/media/shapelim/UX980/erasor_outputs/SemanticKITTI/"$sequence"_"$start_frame"_to_"$end_frame"_w_interval_2_voxel_0_2.pcd" --est "/media/shapelim/UX980/erasor_outputs/SemanticKITTI/"$sequence"_"$start_frame"_to_"$end_frame"_estimated.pcd" --seq $sequence"_"$inst_seg
                done
            done
        done
    done
done
