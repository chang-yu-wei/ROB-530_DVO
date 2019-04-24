# for rpe
# python3 evaluate_ate.py groundtruth.txt trajectory.txt --fixed_delta --verbose --plot plot_path --title for plotting --save path_for saving results
python3 evaluate_rpe.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/dvo_fr2_desk_t.txt --fixed_delta --verbose --plot ../data/traj_log/fr2_desk/dvo_rpe.png --title 'DVO' --save '../data/traj_log/fr2_desk/dvo_err.txt'
python3 evaluate_rpe.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/BA_full_t.txt --fixed_delta --verbose --plot ../data/traj_log/fr2_desk/BA_full_rpe.png --title 'Local BA' --save '../data/traj_log/fr2_desk/ba_err.txt'
python3 evaluate_rpe.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/Pose_graph_full_t.txt --fixed_delta --verbose --plot ../data/traj_log/fr2_desk/Pose_graph_full_rpe.png --title 'Global BA' --save '../data/traj_log/fr2_desk/pose_graph_err.txt'
python3 evaluate_rpe.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/orb_slam.txt --fixed_delta --verbose --plot ../data/traj_log/fr2_desk/orb_slam_rpe.png --title 'ORBSLAM' --save '../data/traj_log/fr2_desk/orbslam_err.txt'

