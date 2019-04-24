import numpy as np

def main():
    # target file name
    file_name = "Pose_graph_full"

    # Trajectory
    f = open('../data/traj_log/fr2_desk/'+file_name+'.txt', 'r')
    x = f.readlines()
    f.close()
    x = [line.split(" ") for line in x]
    # remove \n
    x = [element.rstrip() for line in x for element in line]
    x = np.asarray(x).reshape(-1, 9)
    # delete first two column
    x = np.delete(x, [1], 1)
    x = x.astype(float)
    n = len(x)

    # read association file to recover timestamps
    f = open('../data/traj_log/fr2_desk/fr2_desk.txt', 'r')
    t = f.readlines()
    f.close()
    t = [line.split(" ") for line in t]
    # remove \n
    t = [element.rstrip() for line in t for element in line]
    t = np.asarray(t).reshape(-1, 4)
    time = t[:, 0].reshape(-1, 1).astype(float)
    t_out = time[x[:,0].astype(int)]
    x = np.delete(x, [0],1)
    # stack time
    n_time = len(time)
    x = np.hstack((x, t_out))
    # x = np.hstack((x,time[0:n]))
    # rearrange columns: [qw qx qy qz tx ty tz timestamp] to [timestamp tx ty tz qx qy qz qw]
    perm = [7, 4, 5, 6, 1, 2, 3, 0]
    x = x[:, perm]
    np.savetxt('../data/traj_log/fr2_desk/'+file_name+'_t.txt', x, delimiter = ' ', fmt = '%.6f')

if __name__ == '__main__':
    main()
