clc;clear
digitDatasetPath = fullfile('C:\Users\HP\Desktop\PFA');
imds = imageDatastore(digitDatasetPath, ...
    'IncludeSubfolders',true,'LabelSource','foldernames');

for i=1:length(imds.Labels)
img=readimage(imds,i);
img1 = imresize(img,[227 227]);

 imwrite(img1,cell2mat(imds.Files(i)))
end