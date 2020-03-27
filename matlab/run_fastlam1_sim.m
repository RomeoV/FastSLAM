clear; clc; close all; rng(0); addpath('fastslam1');

% Load data
S = load('example_webmap.mat');

% Run FastSLAM 1.0 simulation
data = fastslam1_sim(S.lm, S.wp);