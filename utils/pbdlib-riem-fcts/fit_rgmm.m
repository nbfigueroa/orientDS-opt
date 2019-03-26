function [model, U0, uTmp] = fit_rgmm(model, xIn, xOut, M_in, N, nbIter, nbIterEM)
model.MuMan = [model.Mu(1:M_in,:); expmap(model.Mu(M_in+1:end,:), [0; 1; 0; 0])]; %Center on the manifold %Data(1,nbData/2)
model.Mu = zeros(model.nbVar,model.nbStates); %Center in the tangent plane at point MuMan of the manifold

uTmp = zeros(model.nbVar,N,model.nbStates);
for nb=1:nbIterEM
	%E-step
	L = zeros(model.nbStates,size(xIn,2));
	for i=1:model.nbStates
		L(i,:) = model.Priors(i) * gaussPDF([xIn-repmat(model.MuMan(1:M_in,i),[1 length(xIn)]); logmap(xOut, model.MuMan(M_in+1:end,i))], model.Mu(:,i), model.Sigma(:,:,i));
	end
	GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
	GAMMA2 = GAMMA ./ repmat(sum(GAMMA,2),1,N);
	
    
    %M-step
	for i=1:model.nbStates
		%Update Priors
		model.Priors(i) = sum(GAMMA(i,:)) / (N);
		%Update MuMan
		for n=1:nbIter
			uTmp(:,:,i) = [xIn-repmat(model.MuMan(1:M_in,i),[1 length(xIn)]); logmap(xOut, model.MuMan(M_in+1:end,i))];
            
            
			model.MuMan(:,i) = [(repmat(model.MuMan(1:M_in,i), [1 length(uTmp(1:M_in,:,i))]) + uTmp(1:M_in,:,i))*GAMMA2(i,:)'; expmap(uTmp(M_in+1:end,:,i)*GAMMA2(i,:)', model.MuMan(M_in+1:end,i))];
		end
		%Update Sigma
		model.Sigma(:,:,i) = uTmp(:,:,i) * diag(GAMMA2(i,:)) * uTmp(:,:,i)' + eye(model.nbVar) * model.params_diagRegFact;
	end
end

%Eigendecomposition of Sigma
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(:,:,i));
	U0(:,:,i) = V * D.^.5;
end

end