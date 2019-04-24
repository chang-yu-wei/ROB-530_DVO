# for ate
# python3 evaluate_ate.py groundtruth.txt trajectory.txt --verbose --plot plot_path --title for plotting
python3 evaluate_ate.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/dvo_fr2_desk_t.txt --verbose --plot ../data/traj_log/fr2_desk/dvo_plot.png --title 'DVO' 
python3 evaluate_ate.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/BA_full_t.txt --verbose --plot ../data/traj_log/fr2_desk/ba_plot.png --title 'Local BA' 
python3 evaluate_ate.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/Pose_graph_full_t.txt --verbose --plot ../data/traj_log/fr2_desk/pose_graph_plot.png --title 'Global BA' 
python3 evaluate_ate.py ../data/traj_log/fr2_desk/groundtruth.txt ../data/traj_log/fr2_desk/orb_slam.txt --verbose --plot ../data/traj_log/fr2_desk/orbslam_plot.png --title 'orbslam' 
