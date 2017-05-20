close all;
clear all;

run('C:\VLfeat\vlfeat-0.9.20\toolbox/vl_setup')

im = imread('white.jpg');
imborder_draw = im;
im1 = im2single(im);
imd = im2double(im);

imlab = vl_xyz2lab(vl_rgb2xyz(im1)) ;

imlab1 = im2single(imlab);

segments = vl_slic(im1, 30, 0.1) ;

N = max(max(segments));
sumR=0;
sumG=0;
sumB=0;
count=0;

[x,y,color] = size(im);

A = ones(x,y);
B = ones(x,y,3);

for parts = 0:N
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for j=1:y
    for i=1:x
        if segments(i,j) == parts
            sumR = sumR + imd(i,j,1);
            sumG = sumG + imd(i,j,2);
            sumB = sumB + imd(i,j,3);
            count = count +1;
        end
    end
  end
  avgR=sumR/count;
  avgG=sumG/count;
  avgB=sumB/count;

  sumR = 0;
  sumG = 0;
  sumB = 0;
  count = 0;
  
  for j=1:y
    for i=1:x
        if segments(i,j) == parts
            B(i,j,1)=avgR;
            B(i,j,2)=avgG;
            B(i,j,3)=avgB;
        end
    end
  end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

SLICborder_draw = B;

for j=1:y-1
    for i=1:x-1
        if segments(i,j) ~= segments(i+1,j)
            imborder_draw(i,j,1) = 0; 
            imborder_draw(i,j,2) = 0;
            imborder_draw(i,j,3) = 0;
            A(i,j)=255;
            SLICborder_draw(i,j,1)=0;
            SLICborder_draw(i,j,2)=0;
            SLICborder_draw(i,j,3)=0;
        elseif segments(i,j) ~= segments(i,j+1)
            imborder_draw(i,j,1) = 0;
            imborder_draw(i,j,2) = 0;
            imborder_draw(i,j,3) = 0;
            A(i,j)=255;
            SLICborder_draw(i,j,1)=0;
            SLICborder_draw(i,j,2)=0;
            SLICborder_draw(i,j,3)=0;
        end
        
    end
end

A = uint8(A);
B = im2uint8(B);
SLICborder_draw = im2uint8(SLICborder_draw);

% %%%%%%%%%%%%%%%  K-means  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cform = makecform('srgb2lab');
% lab_he = applycform(B,cform);
% 
% 
% ab = double(lab_he(:,:,2:3));
% nrows = size(ab,1);
% ncols = size(ab,2);
% ab = reshape(ab,nrows*ncols,2);
% 
% nColors = 5;
% % repeat the clustering 3 times to avoid local minima
% [cluster_idx, cluster_center] = kmeans(ab,nColors,'distance','sqEuclidean','Replicates',3);
% 
% pixel_labels = reshape(cluster_idx,nrows,ncols);
% %figure;
% %imshow(pixel_labels,[]), title('image labeled by cluster index');
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


subplot(2,2,1);
imshow (im);
subplot(2,2,2);
imshow (A);
subplot(2,2,3);
imshow (B);
%subplot(2,2,4);
%imshow(pixel_labels,[]));

figure;
imshow (imborder_draw);
figure;
imshow (SLICborder_draw);