function likelihood = computeLikelihood(sign, memory, dictionary)
%GETLIKELIHOOD Compute the likelihood of the signature sign.
%   likelihood = getLikelihood(sign, memory, dictionary)
%
%   Mem : the memory (Mem = [signId1 WordRefIds...; signId2 WordRefIds...; ...])
%   Dict : the visual words dictionary (Dict = [wordId1 SignRefIds...;
%          wordId2 SignRefIds...; ...])
%   sign : The signature reference which we want the likelihood with all
%          others in the memory (sign = [id wordsRef...])
likelihood = zeros(size(memory,1),1);

nwi = 0;    % nwi is the number of a specific word referenced by a place
ni = 0;     % ni is the total of words referenced by a place
nw = 0;     % nw is the number of places referenced by a specific word
N = 0;      % N is the total number of places

N = size(memory,1);

words = unique(sign(2:end));

for i=1:length(words)
    if words(i) ~= 0
        % "Inverted index" - For each places referenced by the word
        refs = unique(dictionary(find(dictionary(:,1) == words(i),1), 2:end));
        refs = refs(refs~=0);
        nw = 0;
        for j=1:length(refs)
            nw = nw + 1;
        end       
        logNnw = log10(N/nw); % TODO : division by 0 (not supposed to occur)

        if logNnw ~= 0
            for j=1:length(refs)
                pos = find(memory(:,1) == refs(j),1);
                sign = memory(pos, :);

                nwi = sum(sign(2:end) == words(i));
                ni = sum(sign(2:end)>0);

                if ni ~= 0
                    likelihood(pos) = likelihood(pos) + ( nwi  * logNnw ) / ni;
                end
            end
        end
    end
end