function [ aic, bic ] = gmm_ic_metrics(Data, K, LogLik, cov_type)
%moVMF_BIC Information criterion Metrics
%
%  input ------------------------------------------------------------------
%
%   o Data:     D x N array representing N datapoints of D dimensions.
%
%   o K:        1 x K array representing the number of components.

%  output -----------------------------------------------------------------
%
%   o bic   : (1 x 1)
%
%   o aic   : (1 x 1)




[D,N]       = size(Data);
num_param   = K-1 + K * D;

if strcmp(cov_type,'full') == true
    num_param = num_param + K * ( D * ( D - 1)/2 );
elseif strcmp(cov_type,'diag') == true
    num_param = num_param + K * D;
elseif strcmp(cov_type,'isot') == true
    num_param = num_param + K * 1;
else
   error(['no such covariance type: ' cov_type '  only full | diag | isot ']); 
end

aic    = - 2 * LogLik + 2 * num_param;
bic    = - 2 * LogLik + num_param * log(N);

end

