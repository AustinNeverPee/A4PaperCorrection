clear all;
close all;
clc;

% % 读入图片
% I = imread('/home/austin/Documents/计算机视觉与模式识别/Project1/Dataset1/sm_*-1.jpg');
% % I = imread('/home/austin/Documents/计算机视觉与模式识别/Project1/Dataset2/sm_*-0.jpg');
% % I = imread('/home/austin/Documents/计算机视觉与模式识别/Project1/Owndataset/example2.jpg');

% 循环所有测试图片
for i = 29:29
    % 读入图片
    inputimgname=strcat('/home/austin/Documents/计算机视觉与模式识别/Project1/Input/', num2str(i), '.jpg');
    I=imread(inputimgname);
    result_img = uint8(A4Warper(I));
    outputimgname=strcat('/home/austin/Documents/计算机视觉与模式识别/Project1/Output/', num2str(i), '.jpg');
    imwrite(result_img, outputimgname);
end