function x = expmap(u, mu)
	x = QuatMatrix(mu) * expfct(u);
end