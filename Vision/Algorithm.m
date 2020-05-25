clear;
clc;
%% Origin
imag = imread('Road.jpg');
subplot(2,2,1);
imshow(imag);
title('Origin');
[height,width,depth] = size(imag);  % Get hight and width
%% Grey
imag_Grey = rgb2gray(imag);  % Turn to gray
subplot(2,2,2);
imshow(imag_Grey);
title('Grey');
%% Gaussian
imag_Gaus = imag;   % Gaussian Filter
core = [0.0751 0.1238 0.0751;0.1238 0.2042 0.1238;0.0751 0.1238 0.0751];
for i = 2:width-1
    for j = 2:height-1
        for k = 1:3
            add = 0;
            for a = 1:3
                for b = 1:3
                    add = add + imag_Gaus(i+a-2,j+b-2,k) * core(a,b);
                end
            end
            imag_Gaus(i,j,k) = add;
        end
    end
end
subplot(2,2,3);
imshow(imag_Gaus);
title('After Gaussian');
%% Sobel
imag_Sobel = imag_Grey;
U = double(imag_Grey);
for i = 2 : height-1
    for j = 2 : width-1
        Gx = (U(i+1,j-1) + 2*U(i+1,j) + U(i+1,j+1)) - (U(i-1,j-1) + 2*U(i-1,j) + U(i-1,j+1));
        Gy = (U(i-1,j+1) + 2*U(i,j+1) + U(i+1,j+1)) - (U(i-1,j-1) + 2*U(i,j-1) + U(i+1,j-1));
        imag_Sobel(i,j) = abs(Gx) + abs(Gy);
    end
end
%% Binarization
Thres = 250;
imag_Bin = imag_Sobel;
for i = 2:height - 1
    for j = 2:width - 1
        if imag_Bin(i,j) < Thres
            imag_Bin(i,j)=0;
        elseif  imag_Bin(i,j) >= Thres
            imag_Bin(i,j)=255;
        end
    end
end
subplot(2,2,4);
imshow(imag_Bin);
title('After Sobel & Binarization');