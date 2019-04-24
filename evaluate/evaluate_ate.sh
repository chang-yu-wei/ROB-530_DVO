# for ate
# python3 evaluate_ate.py groundtruth.txt trajectory.txt --verbose --plot plot_path --title for plotting
python3 evaluate_ate.py ../data/traj_log/groundtruth.txt ../data/traj_log/dvo_fr2_desk_t.txt --verbose --plot ../data/traj_log/dvo_plot.png --title 'DVO' 
python3 evaluate_ate.py ../data/traj_log/groundtruth.txt ../data/traj_log/BA_full_t.txt --verbose --plot ../data/traj_log/ba_plot.png --title 'Local BA' 
python3 evaluate_ate.py ../data/traj_log/groundtruth.txt ../data/traj_log/Pose_graph_full_t.txt --verbose --plot ../data/traj_log/pose_graph_plot.png --title 'Global BA' 
python3 evaluate_ate.py ../data/traj_log/groundtruth.txt ../data/traj_log/orb_slam.txt --verbose --plot ../data/traj_log/orbslam_plot.png --title 'orbslam' 
