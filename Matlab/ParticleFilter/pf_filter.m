
function filtered = pf_filter(x, nParticles, noise, lambda)

particles = ones(nParticles,1)*x(1) ;
weights = ones(nParticles,1);
filtered=zeros(1,length(x));
for i = 1:length(x);
    for j = 1:nParticles 
        rn = sqrt(-2.0*log(rand))*cos(2*pi*rand); % randn c++
        noisyP = particles(j) + noise*rn ;
        dist = abs(noisyP - x(i));
        tmp = exp(-lambda*dist);
        if isfinite(tmp) && tmp > 0
            particles(j) = noisyP;
            weights(j) = tmp;
        end
    end
    if sum(weights(:)) > 0
         weights = weights ./sum(weights(:));
    end

    filtered(i) = weights'*particles;
    particles = pf_resample(particles, weights);
end