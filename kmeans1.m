clear all;
close all;

[baseName, folder]= uigetfile({'*.*'},'File Selector');
fullFileName = fullfile(folder, baseName);
image = imread(fullFileName);

image_decor=decorrstretch(image);
subplot(1,2,1);
imshow(image);
subplot(1,2,2);
imshow(image_decor);

temp_img = image_decor;
image = rgb2hsv(image_decor);
depth = size(image,3);
resh_img = reshape(image,[],depth);

k =4;
positions=[];
points = [];
iter_lim = 10;
meanpoint_plot=zeros(iter_lim,k,depth);

for i = (1:k)
    positions = [positions; uint16(ginput(1))];
    temp_img = insertMarker(temp_img,[positions(i,1) positions(i,2)],'size',8);
    points =[points; reshape(image(positions(i,2),positions(i,1),:),1,[])];
    imshow(temp_img);
end

%k-means clustering
mean_points = points;
clusters =zeros(k,size(image,1),size(image,2));
%basic clustering

for iter = (1:iter_lim)
    %distance calculation
    for i = (1:k)
       distance2 = abs(double(image)-double(mean_points(i)));
       distance = zeros(size(image,1),size(image,2));
       for j = (1:size(distance2,depth)-2)       %drop intentsity details at the end for better results
        distance = distance + double(distance2(:,:,j).^2);
       end
       clusters(i,:,:)=distance;
    end

    %select minimum distance
    min_dist = clusters(1,:,:);
    for i = (2:k)
        min_dist = min(min_dist,clusters(i,:,:));
    end

    segmentations = zeros(k,size(image,1)*size(image,2),depth);
    ind={};
    for i = (1:k)
        ind{i} = find(min_dist==clusters(i,:,:));
        index = double(ind{i});
        segmentations(i,index,:) = resh_img(index,:);
        mean_points(i,:) = mean(resh_img(index,:));
    end
    
    meanpoint_plot(iter,:,:) = mean_points;
end


figure;

subplot(4,3,1);
plot(meanpoint_plot(:,1,1),'r');
subplot(4,3,2);
plot(meanpoint_plot(:,1,2),'g');
subplot(4,3,3);
plot(meanpoint_plot(:,1,3),'b');
subplot(4,3,4);
plot(meanpoint_plot(:,2,1),'r');
subplot(4,3,5);
plot(meanpoint_plot(:,2,2),'g');
subplot(4,3,6);
plot(meanpoint_plot(:,2,3),'b');
subplot(4,3,7);
plot(meanpoint_plot(:,3,1),'r');
subplot(4,3,8);
plot(meanpoint_plot(:,3,2),'g');
subplot(4,3,9);
plot(meanpoint_plot(:,3,3),'b');
subplot(4,3,10);
plot(meanpoint_plot(:,4,1),'r');
subplot(4,3,11);
plot(meanpoint_plot(:,4,2),'g');
subplot(4,3,12);
plot(meanpoint_plot(:,4,3),'b');

segment = zeros(size(image,1),size(image,2),depth);

mosaic = zeros(size(image,1)*size(image,2),depth);
for i = (1:k) 
    figure;
    segment = reshape(segmentations(i,:,:),size(image,1),size(image,2),depth);  
    picture=hsv2rgb(segment);
    imshow(picture);
    
    index = double(ind{i});    
    for j = (1:depth)
        mosaic(index,j)=mean_points(i,j);
    end  
    
    picture2 = rgb2gray(segment);
    [~, threshold] = edge(picture2, 'sobel');
    fudgeFactor = .5;
    BWs = edge(picture2,'sobel', threshold * fudgeFactor);
    
    mosaic(BWs,3)=0.0;
end


figure
mosaic = reshape(mosaic,size(image,1),size(image,2),depth);
imshow(hsv2rgb(mosaic));