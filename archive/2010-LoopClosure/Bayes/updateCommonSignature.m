function [CS DU] = updateCommonSignature(Mem, Dict)
%UPDATECOMMONSIGNATURE Update the common signature (virtual place)
%   [CS DU] = updateCommonSignature(Mem, Dict)
%
%   CS = common signature
%   DU dictionary updated
%   Mem : the memory (Mem = [signId1 WordRefIds...; signId2 WordRefIds...; ...])
%   Dict : the visual words dictionary (Dict = [wordId1 SignRefIds...; wordId2 SignRefIds...; ...])

CS = Mem(1,:);
CSid = Mem(1,1);

% Clear references to the virtual place
for i=2:length(CS)
    index = find(Dict(:,1) == CS(i));
    if isempty(index) ~= 1
        indexes = find(Dict(index, :) == CSid);
        for j=1:length(indexes)
            Dict(index,indexes(j)) = 0;
        end
    end
end
%clear words in the common signature
CS = CS(1);

%How many words we want... take the average of words by signature
nbCommonWords = 0;
memSize = size(Mem,1)-1; %Don't count the virtual place
if(memSize > 0)
    totalActiveRef = sum(sum(Dict(:,2:end) ~= 0));
    nbCommonWords = floor(totalActiveRef / memSize);
end
if nbCommonWords>0
    %get common words
    list = [(sum(Dict(:,2:end)' ~= 0)') Dict(:,1)];
    list = sortrows(list)
    wordsAdded=0;
    for i=size(list,1):-1:1
        if i ~= size(list,1) && length(CS)>1
            ratio = floor(list(i+1,1)/list(i,1));
            len = length(CS);
            for j=2:ratio
                for k=2:len
                    CS = [CS CS(k)];
                    wordsAdded = wordsAdded + 1;
                    if wordsAdded >= nbCommonWords
                        break;
                    end
                end
                if wordsAdded >= nbCommonWords
                    break;
                end
            end
        end
        
        if wordsAdded < nbCommonWords
            CS = [CS list(i,2)];
            wordsAdded = wordsAdded + 1;
        end
        if wordsAdded >= nbCommonWords
            break;
        end
    end
    CS = [CS zeros(1,size(Mem,2) - length(CS))];
    % add references
    DU = updateDictionary(Dict, CS);
else
    DU = Dict;
end