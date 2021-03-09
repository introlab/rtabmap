
function newParticles=pf_resample(particles,weights)
pcum = zeros(length(weights),1);
sum = 0;
for i=1:length(weights)
    pcum(i) = weights(i) + sum;
    sum = sum + weights(i);
end
pcum = pcum./pcum(end);
newParticles = 0.*particles;

%
for i = 1:length(newParticles)
    indexx = 1;
    randnum = rand;
    for j = 1:length(pcum)
        if(randnum < pcum(j))
            indexx = j;
            break;
        end
    end
    newParticles(i) = particles(indexx);
end
