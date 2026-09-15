"""
train.py — PPO training entrypoint for the quadrotor hover/waypoint task.

Usage:
    python train.py --task hover --timesteps 2_000_000 --n_envs 8

Logs to ./logs (tensorboard) and checkpoints to ./checkpoints.
"""

import argparse
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.monitor import Monitor

from quad_env import QuadrotorEnv


def make_env(task, max_steps, domain_randomize):
    def _init():
        env = QuadrotorEnv(task=task, max_steps=max_steps, domain_randomize=domain_randomize)
        return Monitor(env)
    return _init


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", choices=["hover", "waypoint"], default="hover")
    parser.add_argument("--timesteps", type=int, default=2_000_000)
    parser.add_argument("--n_envs", type=int, default=8)
    parser.add_argument("--max_steps", type=int, default=500)
    parser.add_argument("--domain_randomize", action="store_true")
    parser.add_argument("--logdir", default="./logs")
    parser.add_argument("--ckptdir", default="./checkpoints")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    os.makedirs(args.logdir, exist_ok=True)
    os.makedirs(args.ckptdir, exist_ok=True)

    env_fns = [make_env(args.task, args.max_steps, args.domain_randomize) for _ in range(args.n_envs)]
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    vec_env_cls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    vec_env = vec_env_cls(env_fns) if args.n_envs > 1 else DummyVecEnv(env_fns)

    # Normalizing observations/rewards matters a lot for PPO on continuous-control tasks.
    vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    eval_env = DummyVecEnv([make_env(args.task, args.max_steps, False)])
    eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)

    # PPO hyperparameters tuned for continuous, dense-reward, low-dim control tasks
    # (this is the same neighborhood used in most quadrotor-PPO papers/repos).
    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=3e-4,
        n_steps=2048 // max(1, args.n_envs) * args.n_envs // args.n_envs,  # per-env rollout length
        n_epochs=10,
        batch_size=256,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128])),
        tensorboard_log=args.logdir,
        seed=args.seed,
        verbose=1,
    )

    checkpoint_cb = CheckpointCallback(
        save_freq=max(50_000 // args.n_envs, 1),
        save_path=args.ckptdir,
        name_prefix="ppo_quad",
        save_vecnormalize=True,
    )
    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path=args.ckptdir,
        log_path=args.logdir,
        eval_freq=max(20_000 // args.n_envs, 1),
        n_eval_episodes=10,
        deterministic=True,
    )

    model.learn(total_timesteps=args.timesteps, callback=[checkpoint_cb, eval_cb])

    model.save(os.path.join(args.ckptdir, "ppo_quad_final"))
    vec_env.save(os.path.join(args.ckptdir, "vecnormalize_final.pkl"))
    print("Training complete. Final model + VecNormalize stats saved to", args.ckptdir)


if __name__ == "__main__":
    main()
