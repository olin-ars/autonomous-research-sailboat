import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class MDP:
    def __init__(self, setup = (10, 10)):
        self.statespace = setup
        # Init utility matrix
        self.U = np.zeros((setup[0]*setup[1], 1), dtype=float)

        # Init base rewards matrix
        self.R = np.zeros(self.U.shape, dtype=float)
        # Set the static rewards at points (8,9), (3,8), (5,4), and (8,4)
        ## Shifted to match zero-indexing
        self.R[[78, 27, 43, 73]] = np.array([10, 3, -5, -10]).reshape((4,1))

        # Init rewards matrix for determining bouncing
        self.bR = np.identity(self.U.shape[0], dtype=float)*-1

        # Init vectors of actions
        # Index corresponds to following actions: [up, down, left, right]
        self.acts = np.linspace(0,3,4, dtype=int)
        self.policy = np.zeros(self.U.shape)

        # List of transition matrices for each action
        self.T = self._T()

    def value_iteration(self, gamma = 0.9, n = 10):
        # Init utility matrix
        self.U = np.zeros((self.statespace[0]*self.statespace[1], 1), dtype=float)

        k = 0 # iteration count
        temp_U = np.zeros((self.U.shape[0],4))
        curr_U = self.U[:]

        while True:
            if k >= n:
                return self.U, self.policy
                break
            k += 1

            temp_U = np.column_stack(list(map(lambda x: self._R(x) + gamma*self.T[x].dot(self.U), self.acts)))

            curr_U = np.amax(temp_U, axis=1).reshape(self.U.shape)
            self.policy = list(map(lambda x: np.where(temp_U[x]==curr_U[x])[0],
                                   list(range(len(curr_U)))))

            if (np.linalg.norm(curr_U - self.U, ord=np.inf) < 1):
                self.U = curr_U[:]
                return self.U, self.policy
            self.U = curr_U[:]

    def plot_results(self, steps = 100, start=(0,0)):
        curr_val = start
        pos_track = list()
        pos_track.append(self._true_pos(curr_val))
        for i in list(range(steps)):
            idx = curr_val[0]*self.statespace[1]+curr_val[1]
            a = self.policy[idx][0]
            if a == 0:
                curr_val = (curr_val[0]-1, curr_val[1])
                pos_track.append(self._true_pos(curr_val))
            elif a == 1:
                curr_val = (curr_val[0]+1, curr_val[1])
                pos_track.append(self._true_pos(curr_val))
            elif a == 2:
                curr_val = (curr_val[0], curr_val[1]-1)
                pos_track.append(self._true_pos(curr_val))
            elif a == 3:
                curr_val = (curr_val[0], curr_val[1]+1)
                pos_track.append(self._true_pos(curr_val))
            else:
                pos_track.append(self._true_pos(curr_val))

        self.pt = np.array(pos_track)
        fg = plt.figure()
        plt.plot(self.pt[:,0], self.pt[:,1], "*", self.pt[:,0], self.pt[:,1], "r")
        plt.title("Plot of optimal track dictated by policy, starting from %s" % (pos_track[0],))
        plt.xlabel("X position")
        plt.ylabel("Y position")
        plt.legend(["Actual pos", "Track"])

    def _R(self, a):
        return np.diag(self.T[a].dot(self.bR)).reshape(self.U.shape) + self.R

    def _T(self):
        t_list = list()
        for a in self.acts:
            t_vec_list = list()
            for i in list(range(self.statespace[0])):
                for j in list(range(self.statespace[1])):
                    s = (i, j)
                    idx = s[0] * self.statespace[1] + s[1]
                    t_mat = np.zeros(self.U.shape, dtype=float)
                    if idx != 78 and idx != 27:
                        o_idx = np.array([s[0], s[0], s[1], s[1]])
                        c_idx = np.array([s[0]-1, s[0]+1, s[1]-1, s[1]+1])

                        # define transition probs
                        probs = np.ones(4) * 0.1
                        probs[a] *= 7

                        mps = list(map(lambda x: [c_idx[x], probs[x]] if (0 <= c_idx[x] <= 9) else [o_idx[x], probs[x]], self.acts))

                        real_indices = np.array([mps[0][0]*self.statespace[1] + s[1],
                                                 mps[1][0]*self.statespace[1] + s[1],
                                                 s[0]*self.statespace[1] + mps[2][0],
                                                 s[0]*self.statespace[1] + mps[3][0]])

                        for k in self.acts:
                            t_mat[real_indices[k]] += mps[k][1]

                    t_vec_list.append(t_mat)

            t_list.append(np.column_stack(t_vec_list).T)

        return t_list

    def _true_pos(self, pos):
        return (pos[1], -1*pos[0]+9)


if __name__ == "__main__":
    mdp = MDP()
    U, Pi = mdp.value_iteration()
