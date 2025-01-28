# !/usr/bin/python3
import numpy as np
from scipy.stats import mvn
from scipy.linalg import block_diag
import copy

class ProbStar(object):
    """
        Truncated Version of Probabilistic Star Class from https://github.com/V2A2/StarV
        ==========================================================================
        Star set defined by
        x = c + a[1]*v[1] + a[2]*v[2] + ... + a[n]*v[n]
            = V * b,
        where V = [c v[1] v[2] ... v[n]],
                b = [1 a[1] a[2] ... a[n]]^T,
                C*a <= d, constraints on a[i],
                a~N(mu,sigma) a normal distribution
        ==========================================================================
    """

    def __init__(self, *args):
        """
           Key Attributes:
           V = []; % basis matrix
           C = []; % constraint matrix
           d = []; % constraint vector
           dim = 0; % dimension of the probabilistic star set
           mu = []; % mean of the multivariate normal distribution
           Sig = []; % covariance (positive semidefinite matrix)
           nVars = []; number of predicate variables
           prob = []; % probability of the probabilistic star
           predicate_lb = []; % lower bound of predicate variables
           predicate_ub = []; % upper bound of predicate variables
        """
        if len(args) == 7:
            [V, C, d, mu, Sig, pred_lb, pred_ub] = copy.deepcopy(args)
            self.V = np.asarray(V)
            self.C = np.asarray(C)
            self.d = np.asarray(d)
            self.dim = V.shape[0]
            self.nVars = V.shape[1] - 1
            self.mu = np.asarray(mu)
            self.Sig = np.asarray(Sig)
            self.pred_lb = np.asarray(pred_lb)
            self.pred_ub = np.asarray(pred_ub)
        if len(args) == 5:
            [V, C, d, mu, Sig] = copy.deepcopy(args)
            self.V = np.asarray(V)
            self.C = np.asarray(C)
            self.d = np.asarray(d)
            self.dim = V.shape[0]
            self.nVars = V.shape[1] - 1
            self.mu = np.asarray(mu)
            self.Sig = np.asarray(Sig)
            self.pred_lb = None
            self.pred_ub = None
        assert isinstance(V, np.ndarray), 'error: \
        basis matrix should be a 2D numpy array'
        assert isinstance(mu, np.ndarray), 'error: \
        median vector should be a 1D numpy array'
        assert isinstance(Sig, np.ndarray), 'error: \
        covariance matrix should be a 2D numpy array'
        assert len(V.shape) == 2, 'error: \
        basis matrix should be a 2D numpy array'
        if len(C) != 0:
            assert len(C.shape) == 2, 'error: \
            constraint matrix should be a 2D numpy array'
            assert len(d.shape) == 1, 'error: \
            constraint vector should be a 1D numpy array'
            assert V.shape[1] == C.shape[1] + 1, 'error: \
            Inconsistency between basic matrix and constraint matrix'
            assert C.shape[0] == d.shape[0], 'error: \
            Inconsistency between constraint matrix and constraint vector'
            assert C.shape[1] == mu.shape[0], 'error: Inconsistency \
            between the number of predicate variables and median vector'
            assert C.shape[1] == Sig.shape[1] and \
                C.shape[1] == Sig.shape[0], 'error: Inconsistency between \
                the number of predicate variables and covariance matrix'
        assert len(mu.shape) == 1, 'error: \
        median vector should be a 1D numpy array'
        assert len(Sig.shape) == 2, 'error: \
        covariance matrix should be a 2D numpy array'
        assert np.all(np.linalg.eigvals(Sig) > 0), 'error: \
        covariance matrix should be positive definite'

    def estimateProbability(self):
        """estimate probability of a probstar
           using Genz method, Botev method 
           may be a better option
        """
        if len(self.C) == 0:
            prob, _ = mvn.mvnun(self.pred_lb, self.pred_ub, self.mu, self.Sig)
          
        else:
            C = self.C
            d = self.d
            if self.pred_lb is not None:   
                C = np.vstack((C, -np.eye(self.nVars)))
                d = np.concatenate([d, -self.pred_lb])
            if self.pred_lb is not None:   
                C = np.vstack((C, np.eye(self.nVars)))
                d = np.concatenate([d, self.pred_ub])
            A = np.matmul(np.matmul(C, self.Sig), np.transpose(C)) # A = C*Sig*C'
            if np.all(np.linalg.eigvals(A) >= 0):  # No need to introduce auxilary normal variables
                # Check Truncated Normal Matlab Toolbox of Botev to understand this conversion
                new_lb = np.NINF*np.ones(len(d),)  # lb = l - A*mu
                new_ub = d - np.matmul(C, self.mu)  # ub = u - A*mu 
                new_mu = np.zeros(len(d),)          # new_mu = 0
                new_Sig = np.matmul(np.matmul(C, self.Sig), np.transpose(C)) # new_Sig = A*Sig*A'
                prob, _ = mvn.mvnun(new_lb, new_ub, new_mu, new_Sig)

            else:  # Need to introduce auxilary normal variables
                # step 1: SVD decomposition
                # [U, Q, L] = SVD(C), C = U*Q*L'
                # decompose Q = [Q_(r x r); 0_(m-r x r)]
                # U'*U = L'*L = I_r
                U, Q, L = np.linalg.svd(C)
                Q1 = np.diag(Q)

                L1 = L[0:Q1.shape[1],:]
                Q1 = np.matmul(Q1, L1)

                # linear transformation a_r' = Q1*a_r of original normal variables
                mu1 = np.matmul(Q1, self.mu)
                Sig1 = np.matmul(np.matmul(Q1, self.Sig), np.transpose(Q1))
                m = U.shape[0] - len(Q)  # number of auxilary variables
                mu2 = np.zeros(m,)  # auxilary normal variables mean
                Sig2 = (1e-10)*np.eye(m)  # auxilary normal variables variance

                new_mu = np.concatenate([mu1, mu2])
                new_Sig = block_diag(Sig1, Sig2)

                new_lb = np.NINF*np.ones(len(d),)
                new_ub = d - np.matmul(U, new_mu)
                new_Sig = np.matmul(np.matmul(U, new_Sig), np.transpose(U))

                prob, _ = mvn.mvnun(new_lb, new_ub, np.zeros(len(d),), new_Sig)

        return prob