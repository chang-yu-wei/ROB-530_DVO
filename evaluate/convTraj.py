import numpy as np

def main():
    # Trajectory
    f = open('fr2_desk/TrajectoryTUM_fr2_desk.txt', 'r')
    x = f.readlines()
    f.close()
    x = [line.split(" ") for line in x]
    # remove \n
    x = [element.rstrip() for line in x for element in line]
    x = np.asarray(x).reshape(-1, 9)
    # delete first two column
    x = np.delete(x, [0, 1], 1)
    x = x.astype(float)

    # read association file to recover timestamps
    f = open('fr2_desk/fr2_desk.txt', 'r')
    t = f.readlines()
    f.close()
    t = [line.split(" ") for line in t]
    # remove \n
    t = [element.rstrip() for line in t for element in line]
    t = np.asarray(t).reshape(-1, 4)
    time = t[:, 0].reshape(-1, 1).astype(float)
    # stack time
    x = np.hstack((x, time))
    # rearrange columns: [qw qx qy qz tx ty tz timestamp] to [timestamp tx ty tz qx qy qz qw]
    perm = [7, 4, 5, 6, 1, 2, 3, 0]
    x = x[:, perm]
    np.savetxt('fr2_desk/TrajectoryTUM_fr2_desk_t.txt', x, delimiter = ' ', fmt = '%.4f')

if __name__ == '__main__':
    main()
