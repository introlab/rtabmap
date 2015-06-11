
function filtered = pf_filter(x, nParticles, noise, lambda)

particles = zeros(nParticles,1) ;
weights = zeros(nParticles,1);
filtered=zeros(1,length(x));
for i = 1:length(x);
    for j = 1:nParticles 
        rn = sqrt(-2.0*log(rand))*cos(2*pi*rand); % randn c++
        particles(j) = particles(j) + noise*rn ;
        dist = abs(particles(j) - x(i));
        weights(j) = exp(-lambda*dist);
    end
    weights = weights ./(sum(weights(:)));
    filtered(i) = weights'*particles;
    particles = pf_resample(particles, weights);
end