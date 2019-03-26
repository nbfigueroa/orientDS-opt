% GMR with time as input and unit quaternion as output by relying on Riemannian manifold. 
%
% Writing code takes time. Polishing it and making it available to others takes longer! 
% If some parts of the code were useful for your research of for a better understanding 
% of the algorithms, please cite the related publications.
%
% @article{Zeestraten17RAL,
%   author="Zeestraten, M. J. A. and Havoutis, I. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
%   title="An Approach for Imitation Learning on Riemannian Manifolds",
%   journal="{IEEE} Robotics and Automation Letters ({RA-L})",
%   doi="",
%   year="2017",
%   month="",
%   volume="",
%   number="",
%   pages=""
% }
% 
% Copyright (c) 2017 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon, http://calinon.ch/
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.
clear all; close all; clc
choose_dim = 2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Set Estimation Parameters   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbIter    = 10; %Number of iteration for the Gauss Newton algorithm
nbIterEM  = 10; %Number of iteration for the EM algorithm

model.nbStates = 6; %Number of states in the GMM
switch choose_dim
    case 1
        model.nbVar    = 4; %Dimension of the tangent space (incl. time)
        model.nbVarMan = 5; %Dimension of the manifold (incl. time)
    case 2
        model.nbVar    = 5; %Dimension of the tangent space (incl. time)
        model.nbVarMan = 6; %Dimension of the manifold (incl. time)
end
model.dt       = 0.01; %Time step duration
model.params_diagRegFact = 1E-4; %Regularization of covariance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Generate artificial unit quaternion as output data from handwriting data %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData    = 50; %Number of datapoints
nbSamples = 4; %Number of demonstrations
demos=[];

% Data for quaternion tangent space
demos_S = load('demo-data/2Dletters/N.mat');
demos = demos_S.demos;

% Data for quaternion tangent space
demos_C = load('demo-data/2Dletters/C.mat');
demos_input = demos_C.demos;

uIn=[]; uOut=[];
for n=1:nbSamples
	s(n).Data = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	uOut = [uOut, s(n).Data([1:end,end],:)*2E-2];        
    switch choose_dim
        case 1
            % Input dimension is time
            uIn_ = [[1:nbData]*model.dt];
        case 2                        
            % Input dimension in 2d Euclidean space
            c(n).Data = spline(1:size(demos_input{n}.pos,2), demos_input{n}.pos, linspace(1,size(demos_input{n}.pos,2),nbData));
            uIn_ = c(n).Data([1:2],:);
    end    
	uIn = [uIn, uIn_];
end
xIn = uIn;
xOut = expmap(uOut, [0; 1; 0; 0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize GMM Parameters with standard GMM %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Euclidean Input, Riemannian Output  (Quaternion)
x = [xIn; xOut];

% Further parameters and init GMM in tangent space
N = nbData*nbSamples;
M_in = size(xIn,1);


% Tangent Space of Riemmanian Manifold (quat looses a dim, euclidean same)
uIn = xIn;
uOut = logmap(xOut,[0;1;0;0]);
u = [uIn; uOut];

% Estimate initial GMM on tangent space
est_options = [];
est_options.type             = 1;   % GMM Estimation Alorithm Type
est_options.maxK             = 20;  % Maximum Gaussians for Type 1
est_options.fixed_K          = model.nbStates;  % Fix K and estimate with EM for Type 1
est_options.do_plots         = 0;   % Plot Estimation Statistics
est_options.sub_sample       = 1;   % Size of sub-sampling of trajectories
[Priors_tang, Mu_tang, Sigma_tang] = fit_gmm(u, [], est_options);

% Populate model parameters
model.Priors = Priors_tang;
model.Mu = Mu_tang;
model.Sigma = Sigma_tang;

%%%%%%%%%%%%%%%%%%%%%%%%
%% Fit Riemannian GMM %%
%%%%%%%%%%%%%%%%%%%%%%%%
[model, U0, uTmp] = fit_rgmm(model, xIn, xOut, M_in, N, nbIter, nbIterEM);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Regress a Riemannian GMM %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch choose_dim
    case 1
        % 1d- Euclidean Manifold
        in=1; out=2:4; outMan=2:5;
    case 2
        % 2d- Euclidean Manifold
        in=1:2; out=3:5; outMan=3:6;
end
% Regress quaternions from Euclidean input
[uhat,xhat,expSigma] = rgmr_regressor(model, xIn, in, out, outMan, U0, nbIter);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%        Plot Regression Results on Riemmanian Manifold 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clrmap = lines(model.nbStates);
%Timeline plot
figure('PaperPosition',[0 0 6 8],'position',[10,10,650,650],'name','Timeline data','Color',[1 1 1]); 
for k=1:4
	subplot(2,2,k); hold on; 
	for n=1:nbSamples
		plot(x(1,(n-1)*nbData+1:n*nbData), x(M_in+k,(n-1)*nbData+1:n*nbData), '-','color',[.6 .6 .6]);
	end
	plot(x(1,1:nbData), xhat(k,1:nbData), '-','linewidth',2,'color',[.8 0 0]);
    grid on;
	xlabel('t'); ylabel(['q_' num2str(k)]);
end

% Tangent space plot
figure('PaperPosition',[0 0 6 8],'position',[670,10,650,650],'name','Tangent space data','Color',[1 1 1]); 
for i=1:model.nbStates
	subplot(ceil(model.nbStates/2),2,i); hold on; axis off; title(['k=' num2str(i) ', output space']);
	plot(0,0,'+','markersize',40,'linewidth',1,'color',[.7 .7 .7]);
	plot(uTmp(M_in+1,:,i), uTmp(M_in+2,:,i), '.','markersize',4,'color',[0 0 0]);
	plotGMM_pbdLib(model.Mu(M_in+1:M_in+2,i), model.Sigma(M_in+1:M_in+2,M_in+1:M_in+2,i)*3, clrmap(i,:), .3);
	axis equal;
    grid on;
end		 

