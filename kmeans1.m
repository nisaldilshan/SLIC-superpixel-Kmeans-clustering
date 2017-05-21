clear all;
close all;

image = imread('bee.jpg');

original_image= rgb2hsv(image);
adjusted_image=decorrstretch(image);
subplot(1,2,1);
imshow(image);
subplot(1,2,2);
imshow(adjusted_image);
saveas(gcf,'output\Original_Kmeans/original.tif','tiff');

temp_img = adjusted_image;
image = rgb2hsv(adjusted_image);
depth = size(image,3);
resh_img = reshape(image,[],depth);
original_resh_img = reshape(original_image,[],depth);

k =4;
points = [];
N = 10;
meanpoint_plot=zeros(N,k,depth);

cluster_points = uint16(ginput(k));
for i = (1:k)    
    points =[points; reshape(image(cluster_points(i,2),cluster_points(i,1),:),1,[])];
end

%k-means clustering
mean_points = points;
original_points = points;
ori_mean_points = points;
clusters =zeros(k,size(image,1),size(image,2));


for iteration = (1:N)
    %distance calculation
    for i = (1:k)
       dist = abs(double(image)-double(mean_points(i)));
       distance = zeros(size(image,1),size(image,2));
       for j = (1:size(dist,depth)-2)       
        distance = distance + double(dist(:,:,j).^2);
       end
       clusters(i,:,:)=distance;
    end

    %calculate minimum distance matrix
    minimum_distance = clusters(1,:,:);
    for i = (2:k)
        minimum_distance = min(minimum_distance,clusters(i,:,:));
    end

    segmentations = zeros(k,size(image,1)*size(image,2),depth);
    ori_segmentations = zeros(k,size(image,1)*size(image,2),depth);
    ind={};
    for i = (1:k)
        ind{i} = find(minimum_distance==clusters(i,:,:));
        index = double(ind{i});
        segmentations(i,index,:) = resh_img(index,:);
        ori_segmentations(i,index,:) = original_resh_img(index,:);
        mean_points(i,:) = mean(resh_img(index,:));
        ori_mean_points(i,:) = mean(original_resh_img(index,:));
    end
    
    %update mean points table to be plot
    meanpoint_plot(iteration,:,:) = mean_points;
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
saveas(gcf,'output\Original_Kmeans/plot.tif','tiffn');

segment = zeros(size(image,1),size(image,2),depth);

segmented_image = zeros(size(image,1)*size(image,2),depth);

for i = (1:k) 
    figure;
    segment = reshape(segmentations(i,:,:),size(image,1),size(image,2),depth);  
    picture=hsv2rgb(segment);
    imshow(picture);
    saveas(gcf,['output\Original_Kmeans/cluster' num2str(i) '.tif'],'tiff');
    
    index = double(ind{i});    
    for j = (1:depth)
        segmented_image(index,j)=mean_points(i,j);
    end  
    
end


segmented_image = reshape(segmented_image,size(image,1),size(image,2),depth);


for j=1:size(image,2)-1
    for i=1:size(image,1)-1
        if segmented_image(i,j) ~= segmented_image(i+1,j)
            segmented_image(i,j,3) = 0;
        elseif segmented_image(i,j) ~= segmented_image(i,j+1)
            segmented_image(i,j,3) = 0;
        end
        
    end
end


figure;
imshow(hsv2rgb(segmented_image));
saveas(gcf,'output\Original_Kmeans/Segmented.tif','tiff');
