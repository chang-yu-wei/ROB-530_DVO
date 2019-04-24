# for rpe
# python3 evaluate_ate.py groundtruth.txt trajectory.txt --fixed_delta --verbose --plot plot_path --title for plotting --save path_for saving results
python3 evaluate_rpe.py ../data/traj_log/groundtruth.txt ../data/traj_log/dvo_fr2_desk_t.txt --fixed_delta --verbose --plot ../data/traj_log/dvo_rpe.png --title 'DVO' --save '../data/traj_log/dvo_err.txt'
python3 evaluate_rpe.py ../data/traj_log/groundtruth.txt ../data/traj_log/BA_full_t.txt --fixed_delta --verbose --plot ../data/traj_log/BA_full_rpe.png --title 'Local BA' --save '../data/traj_log/ba_err.txt'
python3 evaluate_rpe.py ../data/traj_log//groundtruth.txt ../data/traj_log/Pose_graph_full_t.txt --fixed_delta --verbose --plot ../data/traj_log/Pose_graph_full_rpe.png --title 'Global BA' --save '../data/traj_log/pose_graph_err.txt'
python3 evaluate_rpe.py ../data/traj_log/groundtruth.txt ../data/traj_log/orb_slam.txt --fixed_delta --verbose --plot ../data/traj_log/orb_slam_rpe.png --title 'ORBSLAM' --save '../data/traj_log/orbslam_err.txt'

