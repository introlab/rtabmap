function DU = updateDictionary(Dict, Sign)
%UPDATEDICTIONARY Update the dictionary with the new signature (place)
%   DU = updateDictionary(Dict, Sign)
%
%   DU = dictionary updated
%   dictionary = [wordId1 SignRefIds...; wordId2 SignRefIds...; ...]
%   sign = [signId WordRefIds...]

signId = Sign(1);
Sign = Sign(Sign~=0);
for i=2:length(Sign)
   indexWord = [];
   if isempty(Dict) ~= 1
       indexWord = find(Dict(:,1) == Sign(i),1);
   end
   if isempty(indexWord) == 1
       Dict(size(Dict,1)+1,1) = Sign(i); %Word id
       Dict(size(Dict,1),2) = signId; %Signature id ref
   else
       ii = 1:length(Dict(indexWord,1:end));
       indexesZero = ii(Dict(indexWord,1:end) == 0);
       if isempty(indexesZero)
           Dict(indexWord, size(Dict,2)+1) = signId;
       else
           Dict(indexWord, indexesZero(1)) = signId;
       end
   end
end
DU = Dict;