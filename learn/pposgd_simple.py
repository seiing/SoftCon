# ================================================================================ #
import os
import sys
import numpy as np
from numpy import linalg as LA
import time
from collections import deque
import matplotlib.pyplot as plt
import tensorflow as tf
from mpi4py import MPI
# ================================================================================ #
from baselines.common import Dataset, explained_variance, fmt_row, zipsame
from baselines.common.mpi_adam import MpiAdam
from baselines.common.mpi_moments import mpi_moments
from baselines import logger
import baselines.common.tf_util as U
# ================================================================================ #
import timechecker
# ================================================================================ #

class Plot(object):
    def __init__(self,reward_keys):
        self.iteration = []
        self.reward_total = []
        self.reward = {}
        for key in reward_keys:
            self.reward[key] = []
        self.name = timechecker.str_datetime()

    def add(self,iteration,reward,reward_total):
        self.iteration.append(iteration)
        for key in reward.keys():
            self.reward[key].append(np.mean(reward[key]))
        self.reward_total.append(reward_total)

    def draw(self):
        plt.ion()
        plt.figure(1,figsize=(12,12))
        plt.clf()
        plt.title('reward_graph')
        ax1 = plt.subplot(2,1,1)
        ax1.set_title("total_reward")
        ax1.plot(self.iteration,self.reward["total"],c="r")
        ax2 = plt.subplot(2,1,2)
        ax2.set_title("detail_reward")
        for key in self.reward.keys():
            if key != 'total':
                ax2.plot(self.iteration,self.reward[key],label=str(key))
        ax2.legend(loc=2)
        plt.show()
        plt.pause(0.001)
        file_path = os.path.dirname(os.getcwd())+"/result/"
        if not os.path.isdir(file_path):
            os.mkdir(file_path)
        plt.savefig(file_path+"reward_"+self.name+".png")

def traj_segment_generator(pi, env, horizon, stochastic):
    t = 0
    ac = env.action_space.sample() # not used, just so we have the datatype
    new = True # marks if we're on first timestep of an episode
    ob = env.reset(True)

    cur_ep_ret = 0 # return in current episode
    cur_ep_len = 0 # len of current episode
    ep_rets = [] # returns of completed episodes in this segment
    ep_lens = [] # lengths of ...

    # Initialize history arrays
    obs = np.array([ob for _ in range(horizon)])
    rews = np.zeros(horizon, 'float32')
    vpreds = np.zeros(horizon, 'float32')
    news = np.zeros(horizon, 'int32')
    acs = np.array([ac for _ in range(horizon)])
    prevacs = acs.copy()

    reward_keys = env.reward_keys
    cur_ep_ret_each = {}
    ep_rets_each = {}
    for key in reward_keys:
        cur_ep_ret_each[key] = 0
        ep_rets_each[key] = []

    tc = timechecker.TimeChecker()
    tc.begin()
    while True:
        prevac = ac
        ac, vpred = pi.act(stochastic, ob)
        # Slight weirdness here because we need value function at time T
        # before returning segment [0, T-1] so we get the correct
        # terminal value
        if t > 0 and t % horizon == 0:
            yield {"ob" : obs, "rew" : rews, "vpred" : vpreds, "new" : news,
                    "ac" : acs, "prevac" : prevacs, "nextvpred": vpred * (1 - new),
                    "ep_rets" : ep_rets, "ep_lens" : ep_lens,
                    "ep_rets_each" : ep_rets_each}
            cur_ep_ret = 0
            cur_ep_len = 0
            env.reset(True) 
            # Be careful!!! if you change the downstream algorithm to aggregate
            # several of these batches, then be sure to do a deepcopy
            ep_rets = []
            ep_lens = []
            for key in reward_keys:
                ep_rets_each[key] = []
                cur_ep_ret_each[key] = 0

        i = t % horizon
        obs[i] = ob
        vpreds[i] = vpred
        news[i] = new
        acs[i] = ac
        prevacs[i] = prevac
        np.set_printoptions(threshold=np.inf)
        ob, rew, new, _ = env.step(ac)

        rews[i] = rew['total']

        cur_ep_ret += rew['total']
        cur_ep_len += 1

        for key in reward_keys:
            val = rew[key]
            cur_ep_ret_each[key] += val

        if new:
            print('\t\ttarget reward: ',cur_ep_ret_each['target'])
            ep_rets.append(cur_ep_ret)
            ep_lens.append(cur_ep_len)
            cur_ep_ret = 0
            cur_ep_len = 0
            for key in reward_keys:
                ep_rets_each[key].append(cur_ep_ret_each[key])
                cur_ep_ret_each[key] = 0
            ob = env.reset(True)

        t += 1

def add_vtarg_and_adv(seg, gamma, lam):
    """
    Compute target value using TD(lambda) estimator, and advantage with GAE(lambda)
    """
    new = np.append(seg["new"], 0) # last element is only used for last vtarg, but we already zeroed it if last new = 1
    vpred = np.append(seg["vpred"], seg["nextvpred"])
    T = len(seg["rew"])
    seg["adv"] = gaelam = np.empty(T, 'float32')
    rew = seg["rew"]
    lastgaelam = 0
    for t in reversed(range(T)):
        nonterminal = 1-new[t+1]
        delta = rew[t] + gamma * vpred[t+1] * nonterminal - vpred[t]
        gaelam[t] = lastgaelam = delta + gamma * lam * nonterminal * lastgaelam
    seg["tdlamret"] = seg["adv"] + seg["vpred"]

def learn(env, policy_fn, *,
        timesteps_per_actorbatch, # timesteps per actor per update
        clip_param, entcoeff, # clipping parameter epsilon, entropy coeff
        optim_epochs, optim_stepsize, optim_batchsize,# optimization hypers
        gamma, lam, # advantage estimation
        max_timesteps=0, max_episodes=0, max_iters=0, max_seconds=0,  # time constraint
        callback=None, # you can do anything in the callback, since it takes locals(), globals()
        adam_epsilon=1e-3,
        schedule='constant', # annealing for stepsize parameters (epsilon and adam)
        model_path=None,
        model_iter=None,
        mode='train'
        ):
    # Setup losses and stuff
    # ----------------------------------------
    plot = Plot(env.reward_keys)
    ob_space = env.observation_space
    ac_space = env.action_space
    pi = policy_fn("pi", ob_space, ac_space) # Construct network for new policy
    oldpi = policy_fn("oldpi", ob_space, ac_space) # Network for old policy
    atarg = tf.placeholder(dtype=tf.float32, shape=[None]) # Target advantage function (if applicable)
    ret = tf.placeholder(dtype=tf.float32, shape=[None]) # Empirical return

    lrmult = tf.placeholder(name='lrmult', dtype=tf.float32, shape=[]) # learning rate multiplier, updated with schedule
    clip_param = clip_param * lrmult # Annealed cliping parameter epislon

    ob = U.get_placeholder_cached(name="ob")
    ac = pi.pdtype.sample_placeholder([None])

    kloldnew = oldpi.pd.kl(pi.pd)
    ent = pi.pd.entropy()
    meankl = tf.reduce_mean(kloldnew)
    meanent = tf.reduce_mean(ent)
    pol_entpen = (-entcoeff) * meanent

    ratio = tf.exp(pi.pd.logp(ac) - oldpi.pd.logp(ac)) # pnew / pold
    surr1 = ratio * atarg # surrogate from conservative policy iteration
    surr2 = tf.clip_by_value(ratio, 1.0 - clip_param, 1.0 + clip_param) * atarg #
    pol_surr = - tf.reduce_mean(tf.minimum(surr1, surr2)) # PPO's pessimistic surrogate (L^CLIP)
    vf_loss = tf.reduce_mean(tf.square(pi.vpred - ret))
    total_loss = pol_surr + pol_entpen + vf_loss
    losses = [pol_surr, pol_entpen, vf_loss, meankl, meanent]
    loss_names = ["pol_surr", "pol_entpen", "vf_loss", "kl", "ent"]

    var_list = pi.get_trainable_variables()
    lossandgrad = U.function([ob, ac, atarg, ret, lrmult], losses + [U.flatgrad(total_loss, var_list, 1.0)])
    adam = MpiAdam(var_list, epsilon=adam_epsilon)

    assign_old_eq_new = U.function([],[], updates=[tf.assign(oldv, newv)
        for (oldv, newv) in zipsame(oldpi.get_variables(), pi.get_variables())])
    compute_losses = U.function([ob, ac, atarg, ret, lrmult], losses)

    #loading
    if mode == 'play':
        U.load_state(model_path)
        return pi
    if mode == 'train':
        U.initialize()
    if mode == 'retrain':
        U.load_state(model_path)

    adam.sync()
    
    # ----------------------------------------
    seg_gen = traj_segment_generator(pi, env, timesteps_per_actorbatch, stochastic=True)

    episodes_so_far = 0
    timesteps_so_far = 0
    #just for file name
    if(model_path == None and model_iter == None):
        iters_so_far = 0
    else:
        iters_so_far = model_iter
    tstart = time.time()
    lenbuffer = deque(maxlen=100) # rolling buffer for episode lengths
    rewbuffer = deque(maxlen=100) # rolling buffer for episode rewards

    assert sum([max_iters>0, max_timesteps>0, max_episodes>0, max_seconds>0])==1, "Only one time constraint permitted"

    reward_keys = env.reward_keys
    reweachbuffer = {}
    for key in reward_keys:
        reweachbuffer[key] = deque(maxlen=100)

    while True:
        if callback: callback(locals(), globals())
        if max_timesteps and timesteps_so_far >= max_timesteps:
            break
        elif max_episodes and episodes_so_far >= max_episodes:
            break
        elif max_iters and iters_so_far >= max_iters:
            break
        elif max_seconds and time.time() - tstart >= max_seconds:
            break

        if schedule == 'constant':
            cur_lrmult = 1.0
        elif schedule == 'linear':
            cur_lrmult =  max(1.0 - float(timesteps_so_far) / max_timesteps, 0)
        else:
            raise NotImplementedError

        logger.log("********** Iteration %i ************"%iters_so_far)

        seg = seg_gen.__next__()
        add_vtarg_and_adv(seg, gamma, lam)

        ob, ac, atarg, tdlamret = seg["ob"], seg["ac"], seg["adv"], seg["tdlamret"]
        vpredbefore = seg["vpred"] # predicted value function before udpate
        atarg = (atarg - atarg.mean()) / atarg.std() # standardized advantage function estimate
        d = Dataset(dict(ob=ob, ac=ac, atarg=atarg, vtarg=tdlamret), shuffle=not pi.recurrent)
        optim_batchsize = optim_batchsize or ob.shape[0]

        if hasattr(pi, "ob_rms"): pi.ob_rms.update(ob) # update running mean/std for policy

        assign_old_eq_new() # set old parameter values to new parameter values
        logger.log("Optimizing...")
        logger.log(fmt_row(13, loss_names))
        # Here we do a bunch of optimization epochs over the data
        for _ in range(optim_epochs):
            losses = [] # list of tuples, each of which gives the loss for a minibatch
            for batch in d.iterate_once(optim_batchsize):
                *newlosses, g = lossandgrad(batch["ob"], batch["ac"], batch["atarg"], batch["vtarg"], cur_lrmult)
                adam.update(g, optim_stepsize * cur_lrmult)
                losses.append(newlosses)
            logger.log(fmt_row(13, np.mean(losses, axis=0)))

        logger.log("Evaluating losses...")
        losses = []
        for batch in d.iterate_once(optim_batchsize):
            newlosses = compute_losses(batch["ob"], batch["ac"], batch["atarg"], batch["vtarg"], cur_lrmult)
            losses.append(newlosses)
        meanlosses,_,_ = mpi_moments(losses, axis=0)
        logger.log(fmt_row(13, meanlosses))
        for (lossval, name) in zipsame(meanlosses, loss_names):
            logger.record_tabular("loss_"+name, lossval)
        logger.record_tabular("ev_tdlam_before", explained_variance(vpredbefore, tdlamret))
        lrlocal = (seg["ep_lens"], seg["ep_rets"]) # local values
        listoflrpairs = MPI.COMM_WORLD.allgather(lrlocal) # list of tuples
        lens, rews = map(flatten_lists, zip(*listoflrpairs))
        lenbuffer.extend(lens)
        rewbuffer.extend(rews)
        for key in reward_keys:
            reweachbuffer[key].extend(seg["ep_rets_each"][key])
        logger.record_tabular("EpLenMean", np.mean(lenbuffer))
        logger.record_tabular("EpRewMean", np.mean(rewbuffer))
        logger.record_tabular("EpThisIter", len(lens))
        episodes_so_far += len(lens)
        timesteps_so_far += sum(lens)
        logger.record_tabular("EpisodesSoFar", episodes_so_far)
        logger.record_tabular("TimestepsSoFar", timesteps_so_far)
        logger.record_tabular("TimeElapsed", time.time() - tstart)
        if MPI.COMM_WORLD.Get_rank()==0:
            logger.dump_tabular()
            if(iters_so_far % 10 == 0):
                U.save_state(os.path.dirname(os.getcwd())+'/result/train_%s_%s'%(timechecker.str_datetime(),iters_so_far))
            plot.add(iters_so_far,reweachbuffer,np.mean(rewbuffer))
            plot.draw()

        env.eval = []
        iters_so_far += 1

    return pi

def flatten_lists(listoflists):
    return [el for list_ in listoflists for el in list_]