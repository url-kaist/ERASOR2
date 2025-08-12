s_fs=(4390 150 860 2350 630 0)
e_fs=(4530 250 950 2670 820 800)
seqs=("00" "01" "02" "05" "07" "19")

for jj in 2
do
    start_frame=${s_fs[$jj]}
    end_frame=${e_fs[$jj]}
    sequence=${seqs[$jj]}
    rosparam load "/home/shapelim/catkin_ws/src/ERASOR2/config/seq_"$sequence".yaml"
    echo "/home/shapelim/catkin_ws/src/ERASOR2/config/seq_"$sequence".yaml"

    for rp_thr in 0.6 0.7
    do
        for gain in 2.0
        do
            for increment in 0.2 0.3
            do
                rosparam set "erasor2/log_odds/increment_gain" $gain
                rosparam set "erasor2/log_odds/increment" $increment
                rosparam set "erasor2/region_proposal_thr" $rp_thr
                echo $gain
                echo $increment
                echo $rp_thr
                echo $gain
                echo $increment
                echo $rp_thr

                rosrun erasor2 run_erasor2

                python2.7 /home/shapelim/catkin_ws/src/ERASOR2/scripts/analysis.py --gt "/media/shapelim/UX980/erasor_outputs/SemanticKITTI/"$sequence"_"$start_frame"_to_"$end_frame"_w_interval_1_voxel_0_2.pcd" --est "/media/shapelim/UX980/erasor_outputs/SemanticKITTI/"$sequence"_"$start_frame"_to_"$end_frame"_estimated.pcd" --seq $sequence"_"$inst_seg
            done
        done
    done

done
