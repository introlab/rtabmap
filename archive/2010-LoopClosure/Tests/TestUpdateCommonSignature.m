close all
clear all

% Test AvpdCore::KeypointMemory::updateCommonSignature()

% load data
memory = dlmread('090306-3_db-Signatures.txt', ' ', 1, 0);
dictionary = dlmread('090306-3_db-Dictionary.txt', ' ', 1, 0);

[virtualPlace dictionary] = updateCommonSignature(memory, dictionary);

%just sorting words, putting the zeros at the end
indexZero = find(virtualPlace == 0,1);
if isempty(indexZero)
    indexZero = length(virtualPlace);
elseif indexZero>1
    indexZero = indexZero-1;
end
virtualPlace(2:indexZero) = sort(virtualPlace(2:indexZero));
commonWords = virtualPlace(2:indexZero)
disp('Total common words:')
disp(length(commonWords))

r = sum(sum((commonWords - [5,8,9,21,22,23,27,30,32,34,36,40,45,53,61,62,64,67,68,77,85,86,97,98,99,100,103,106,122,124,125,127,129,131,143,157,161,164,168,169,172,175,181,182,187,196,197,208,210,217,218,219,220,234,235,237,244,252,286,307,308,310,315,327,328,346,347,350,355,362,372,373,387,393,394,398,404,412,423,428,429,441,442,456,465,470,476,495,496,498,506,520,558,572,584,585,587,596,620,634,637,643,646,647,658,667,668,681,709,721,726,741,766,777,785,790,791,793,808,809,880,896,906,908,919,933,940,954,958,974,981,1002,1023,1026,1058,1060,1071,1074,1097,1146,1152,1166,1215,1217,1229,1334,1341,1387,1478,1510,1539,1549,1566,1601,1624,1649,1693,1814,2046,2141,2424,2442,2674;]) ~= 0)); 
if r ~= 0
    error('commonWords is not valid!')
end
