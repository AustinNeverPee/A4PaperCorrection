function result_img = A4Warper( I )
%A4WARPER Summary of this function goes here
%   Detailed explanation goes here
    GRAY = rgb2gray(I);

    % 图像大小
    img_size = size(GRAY);

    % 显示原图
    figure(1)
    % imshow(GRAY)
    imshow(I)
    title('The Original Image')

    % 显示霍夫空间
    % 设置阈值
    threshold_edge = 0.6;
    threshold_edge_change = 0.02;
    while true
        if threshold_edge <= 0
            break
        end
        
        BW = edge(GRAY, 'canny', threshold_edge);
        imshow(BW)
        [H, theta, rho] = hough(BW, 'ThetaResolution', 0.2);
        figure(2)
        imshow(H, [], 'XData', theta, 'YData', rho, 'InitialMagnification', 'fit')
        axis on, axis normal
        hold on
        xlabel('\theta'), ylabel('\rho')
        threshold_peak = 8;
        peaks = houghpeaks(H, threshold_peak, 'Threshold', 0.4*max(max(H)));
        plot(theta(peaks(:, 2)), rho(peaks(:, 1)), 's', 'color', 'r')
        title('The peak point location')

        % 在原图中标出直线
        lines = houghlines(BW, theta, rho, peaks);
        figure(3)
        imshow(I)
        hold on
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        title('霍夫变换检测出来的所有直线')

        % 排除多余的直线
        % 0、排除近似的直线（霍夫空间中距离较近的点）
        threshold_theta = 5;
        threshold_rho = 50;
        line_flag_similar = zeros(length(lines), 1);
        count = length(lines);
        for j = length(lines):-1:2
            for k = j-1:-1:1
                if lines(j).rho == lines(k).rho && lines(j).theta == lines(k).theta
                    continue
                end
                if abs(lines(k).theta-lines(j).theta)<=threshold_theta && abs(lines(k).rho-lines(j).rho)<=threshold_rho
                    line_flag_similar(j) = 1;
                    count = count-1;
                    break
                end
            end
        end

        % 区分不同的直线
        line_point = zeros(count*2, 2);
        line_point(1, :) = lines(1).point1(1, :);
        line_point(2, :) = lines(1).point2(1, :);
        count = 2;
        sep = [];
        sep(1) = 1;
        i = 2;
        for j = 1 : length(lines)-1
            if line_flag_similar(j+1) == 1
                continue
            end
            line_point(count*2-1, :) = lines(j+1).point1(1, :);
            line_point(count*2, :) = lines(j+1).point2(1, :);

            if lines(j).rho ~= lines(j+1).rho || lines(j).theta ~= lines(j+1).theta
                sep(i) = (count-1)*2+1;
                i = i+1;
            end
            count = count+1;
        end
        sep(i) = (count-1)*2+1;

        % 在原图中标出直线
        figure(4)
        imshow(I)
        hold on
        for k = 1:length(line_point)/2
            xy = [line_point(k*2-1, :); line_point(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        title('经方法零过滤后的直线')
        
        if length(sep)-1 >= 4
            break;
        else
            % 根据检测出的直线数目来调整边缘检测的阈值
            threshold_edge = threshold_edge-threshold_edge_change;
        end
    end

    % 拟合直线方程
    all_lines = zeros(length(sep)-1, 2);
    j = 1;
    for i = 1 : length(sep)-1
       all_lines(j, :) = polyfit(line_point(sep(i):sep(i+1)-1, 1), line_point(sep(i):sep(i+1)-1, 2), 1);
       j = j + 1;
    end

    if length(sep)-1 < 4
        disp('检测出少于4条直线')
        return
    elseif length(sep)-1 > 4
        % 1、统计直线斜率，排除独特斜率的直线
        threshold_slope_sub = 0.19;
        threshold_slope_minus = 0.1;
        threshold_slope_max = 27;
        line_flag_unique = zeros(length(sep)-1, 1);
        for i = 1 : length(sep)-2
            if line_flag_unique(i) == 1
                continue
            else
                for j = i+1 : length(sep)-1
                    if all_lines(i, 1) > threshold_slope_max
                        k1 = threshold_slope_max;
                    elseif all_lines(i, 1) < -threshold_slope_max
                        k1 = -threshold_slope_max;
                    else
                        k1 = all_lines(i, 1);
                    end
                    if all_lines(j, 1) > threshold_slope_max
                        k2 = threshold_slope_max;
                    elseif all_lines(j ,1) < -threshold_slope_max
                        k2 = -threshold_slope_max;
                    else
                        k2 = all_lines(j, 1);
                    end
                    if abs(1-abs(k1/k2))<=threshold_slope_sub || abs(k1-k2)<=threshold_slope_minus
                        line_flag_unique(i)=1;
                        line_flag_unique(j)=1;
                    end
                end
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_unique(i) == 0
                line_point(sep(i)-remove_sum:sep(i+1)-1-remove_sum, :) = [];
                remove_sum = remove_sum+sep(i+1)-sep(i);
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_unique(i) == 0
                all_lines(i-remove_sum, :) = [];
                remove_sum = remove_sum+1;
            end
        end
        for i = 1 : length(sep)-1
            if line_flag_unique(i) == 0
                remove_sum = sep(i+1)-sep(i);
                for j = i+1 : length(sep)
                    sep(j) = sep(j)-remove_sum;
                end
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_unique(i) == 0
                sep(i-remove_sum) = [];
                remove_sum = remove_sum+1;
            end
        end
        % 在原图中标出直线
        figure(5)
        imshow(I)
        hold on
        for k = 1:length(line_point)/2
            xy = [line_point(k*2-1, :); line_point(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        title('经方法一过滤后的直线')

        % 2、统计直线斜率，排除不与其它直线垂直的直线
        slope_multi = zeros((length(sep)-1)*(length(sep)-2)/2, 1);
        count = 1;
        for i = 1 : length(sep)-2
            for j = i+1 : length(sep)-1
                slope_multi(count) = all_lines(i, 1)*all_lines(j,1);
                count = count + 1;
            end
        end
        slope_multi = sortrows(slope_multi, 1);
        threshold_slope_multi = 2;
        line_flag_vertical = zeros(length(sep)-1, 1);
        for i = 1 : length(sep)-1
            count = 0;
            for j = 1 : length(sep)-1
                if i == j
                    continue
                end
                tmp_slope_multi = all_lines(i, 1)*all_lines(j,1);
                if abs(tmp_slope_multi-(-1)) <= threshold_slope_multi
                    count = count+1;
                end

                if count > 1
                    line_flag_vertical(i) = 1;
                    break;
                end
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_vertical(i) == 0
                line_point(sep(i)-remove_sum:sep(i+1)-1-remove_sum, :) = [];
                remove_sum = remove_sum+sep(i+1)-sep(i);
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_vertical(i) == 0
                all_lines(i-remove_sum, :) = [];
                remove_sum = remove_sum+1;
            end
        end
        for i = 1 : length(sep)-1
            if line_flag_vertical(i) == 0
                remove_sum = sep(i+1)-sep(i);
                for j = i+1 : length(sep)
                    sep(j) = sep(j)-remove_sum;
                end
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_vertical(i) == 0
                sep(i-remove_sum) = [];
                remove_sum = remove_sum+1;
            end
        end
        % 在原图中标出直线
        figure(6)
        imshow(I)
        hold on
        for k = 1:length(line_point)/2
            xy = [line_point(k*2-1, :); line_point(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        title('经方法二过滤后的直线')

        % 3、排除A4纸内的直线
        % 计算所有直线两侧直线
        line_point_plus = zeros(length(line_point), 2);
        line_point_minus = zeros(length(line_point), 2);
        distance = 1;
        for i = 1 : length(sep)-1
            for j = sep(i) : sep(i+1)-1
                b_change = distance*sqrt(all_lines(i, 1)^2+1);
                % 分两种情况：斜率绝对值小于等于1；斜率绝对值大于1
                if abs(all_lines(i, 1)) <= 1
                    line_point_plus(j, 1) = line_point(j, 1);
                    line_point_plus(j, 2) = int32(line_point(j, 2)+b_change);
                    if line_point_plus(j, 2) > img_size(1)
                        line_point_plus(j, 2) = img_size(1);
                    end
                    line_point_minus(j, 1) = line_point(j, 1);
                    line_point_minus(j, 2) = int32(line_point(j, 2)-b_change);
                    if line_point_minus(j, 2) < 1
                        line_point_minus(j, 2) = 1;
                    end
                else
                    line_point_plus(j, 1) = int32(line_point(j, 1)+b_change/all_lines(i,1));
                    line_point_plus(j, 2) = line_point(j, 2);
                    if line_point_plus(j, 1) > img_size(2)
                        line_point_plus(j, 1) = img_size(2);
                    end
                    if line_point_plus(j, 1) < 1
                        line_point_plus(j, 1) = 1;
                    end
                    line_point_minus(j, 1) = int32(line_point(j, 1)-b_change/all_lines(i,1));
                    line_point_minus(j, 2) = line_point(j, 2);
                    if line_point_minus(j, 1) > img_size(2)
                        line_point_minus(j, 1) = img_size(2);
                    end
                    if line_point_minus(j, 1) < 1
                        line_point_minus(j, 1) = 1;
                    end
                end

            end
        end

        % 在原图中标出直线
        figure(7)
        imshow(I)
        hold on
        for k = 1:length(line_point)/2
            xy = [line_point(k*2-1, :); line_point(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        % 直线两侧标记直线
        for k = 1:length(line_point_plus)/2
            xy = [line_point_plus(k*2-1, :); line_point_plus(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'red');
        end
        for k = 1:length(line_point_minus)/2
            xy = [line_point_minus(k*2-1, :); line_point_minus(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'blue');
        end
        title('检测出的直线和直线两侧直线')

        % 计算两侧直线之间差别
        % 均值滤波
        averfilt_size = 9;
        F = fspecial('average', averfilt_size);
        aver_I = zeros(img_size(1), img_size(2), 3);
        aver_I(:, :, 1) = filter2(F, I(:, :, 1));
        aver_I(:, :, 2) = filter2(F, I(:, :, 2));
        aver_I(:, :, 2) = filter2(F, I(:, :, 2));
        % % 中值滤波
        % medfilt_size = 121;
        % % med_GRAY = medfilt2(GRAY, [threshold_medfilt_size threshold_medfilt_size]);
        % med_I = zeros(img_size(1), img_size(2), 3);
        % med_I(:, :, 1) = medfilt2(I(:, :, 1), [medfilt_size medfilt_size]);
        % med_I(:, :, 2) = medfilt2(I(:, :, 2), [medfilt_size medfilt_size]);
        % med_I(:, :, 3) = medfilt2(I(:, :, 3), [medfilt_size medfilt_size]);
        dev_lines = zeros(length(sep)-1, 2);
        for i = 1 : length(sep)-1
            sum = 0;
            for j = sep(i) : sep(i+1)-1
        %         sum = sum + (int16(med_GRAY(line_point_plus(j, 2), line_point_plus(j, 1)))-int16(med_GRAY(line_point_minus(j, 2), line_point_minus(j, 1))))^2;
        %         sum = sum + (int16(med_I(line_point_plus(j, 2), line_point_plus(j, 1), 1))-int16(med_I(line_point_minus(j, 2), line_point_minus(j, 1), 1)))^2;
        %         sum = sum + (int16(med_I(line_point_plus(j, 2), line_point_plus(j, 1), 2))-int16(med_I(line_point_minus(j, 2), line_point_minus(j, 1), 2)))^2;
        %         sum = sum + (int16(med_I(line_point_plus(j, 2), line_point_plus(j, 1), 3))-int16(med_I(line_point_minus(j, 2), line_point_minus(j, 1), 3)))^2;
                sum = sum + (int16(aver_I(line_point_plus(j, 2), line_point_plus(j, 1), 1))-int16(aver_I(line_point_minus(j, 2), line_point_minus(j, 1), 1)))^2;
                sum = sum + (int16(aver_I(line_point_plus(j, 2), line_point_plus(j, 1), 2))-int16(aver_I(line_point_minus(j, 2), line_point_minus(j, 1), 2)))^2;
                sum = sum + (int16(aver_I(line_point_plus(j, 2), line_point_plus(j, 1), 3))-int16(aver_I(line_point_minus(j, 2), line_point_minus(j, 1), 3)))^2;

            end
            dev_lines(i, 1) = sqrt(double(sum))/(sep(i+1)-sep(i));
            dev_lines(i, 2) = i;
        end
        dev_lines = sortrows(dev_lines, 1);
        % 通过对差别设置阀值来过滤直线
        threshold_dev_scale = 0.2;
        threshold_dev = threshold_dev_scale*(dev_lines(uint8(length(dev_lines)/2), 1));
        line_flag_differ = zeros(length(sep)-1, 1);
        count = length(sep)-1;
        for i = 1 : length(dev_lines)
            if count <=4
                break
            end
            if dev_lines(i, 1) < threshold_dev
                line_flag_differ(dev_lines(i, 2))=1;
                count = count-1;
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_differ(i) == 1
                line_point(sep(i)-remove_sum:sep(i+1)-1-remove_sum, :) = [];
                line_point_minus(sep(i)-remove_sum:sep(i+1)-1-remove_sum, :) = [];
                line_point_plus(sep(i)-remove_sum:sep(i+1)-1-remove_sum, :) = [];
                remove_sum = remove_sum+sep(i+1)-sep(i);
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_differ(i) == 1
                all_lines(i-remove_sum, :) = [];
                remove_sum = remove_sum+1;
            end
        end
        for i = 1 : length(sep)-1
            if line_flag_differ(i) == 1
                remove_sum = sep(i+1)-sep(i);
                for j = i+1 : length(sep)
                    sep(j) = sep(j)-remove_sum;
                end
            end
        end
        remove_sum = 0;
        for i = 1 : length(sep)-1
            if line_flag_differ(i) == 1
                sep(i-remove_sum) = [];
                remove_sum = remove_sum+1;
            end
        end

        % 在原图中标出直线
        figure(8)
        imshow(I)
        hold on
        for k = 1:length(line_point)/2
            xy = [line_point(k*2-1, :); line_point(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        % 直线两侧标记直线
        for k = 1:length(line_point_plus)/2
            xy = [line_point_plus(k*2-1, :); line_point_plus(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'red');
        end
        for k = 1:length(line_point_minus)/2
            xy = [line_point_minus(k*2-1, :); line_point_minus(k*2, :)];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'blue');
        end
        title('经方法三过滤后的直线')
    end

    disp(length(sep)-1)

    % 求四条边方程
    four_line = zeros(4, 3);
    for j = 1 : 4
        if abs(all_lines(j, 1))==Inf
            four_line(j, :) = [1 0 -line_point(sep(j), 1)];
        else
            four_line(j, :) = [all_lines(j, 1) -1 all_lines(j, 2)];
        end
    end

    % 求四个顶点，并对四个顶点排序(左上、右上、左下、右下)
    four_point = zeros(4, 2);
    count = 1;
    min_distance = img_size(1)*img_size(1);
    min_distance_index = 1;
    for i = 1 : 3
        for j = i + 1 : 4
            tmp_point = cross(four_line(i, :), four_line(j, :));
            tmp_point = tmp_point / tmp_point(3);
            if tmp_point(1) >= 0 && tmp_point(1) <= img_size(2) && tmp_point(2) >= 0 && tmp_point(2) <= img_size(1)
                four_point(count, :) = tmp_point(1:2);
                tmp_point(3) = sqrt(tmp_point(1) ^ 2 + tmp_point(2) ^ 2);
                if tmp_point(3) < min_distance
                    min_distance = tmp_point(3);
                    min_distance_index = count;
                end
                count = count + 1;
            end
        end
    end
    four_point_dis = zeros(3, 2);
    count = 1;
    for i = 1 : 4
        if i == min_distance_index
            continue
        end
        four_point_dis(count, 1) = i;
        four_point_dis(count, 2) = sqrt((four_point(min_distance_index, 1)-four_point(i, 1))^2+(four_point(min_distance_index, 2)-four_point(i, 2))^2);
        count = count+1;
    end
    tmp_point = zeros(4, 3);
    for i = 1 : 4
        tmp_point(i, :) = [four_point(i, :) i];
    end
    tmp_point = sortrows(tmp_point, 2);
    max_y = tmp_point(4, 2);
    tmp_point = zeros(4, 2);
    threshold_dis = 20;
    four_point_dis = sortrows(four_point_dis, 2);
    if four_point(min_distance_index, 1) < four_point(four_point_dis(1, 1), 1) && abs(four_point(four_point_dis(1, 1), 2)-max_y)>threshold_dis
        tmp_point(1, :) = four_point(min_distance_index, :);
        tmp_point(2, :) = four_point(four_point_dis(1, 1), :);
        tmp_point(3, :) = four_point(four_point_dis(2, 1), :);
        tmp_point(4, :) = four_point(four_point_dis(3, 1), :);
    else
        tmp_point(1, :) = four_point(four_point_dis(1, 1), :);
        tmp_point(2, :) = four_point(min_distance_index, :);
        tmp_point(3, :) = four_point(four_point_dis(3, 1), :);
        tmp_point(4, :) = four_point(four_point_dis(2, 1), :);
    end
    four_point(:, :) = tmp_point(:, :);
    % 画图验证顶点位置
    figure(9)
    imshow(I)
    hold on
    plot(four_point(:, 1)', four_point(:, 2)', 's', 'color', 'r')
    title('标出顶点位置')

    % 通过四个顶点调整图像
    % 原四个顶点
    y = [four_point(1, 2) four_point(2, 2) four_point(3, 2) four_point(4, 2)];
    x = [four_point(1, 1) four_point(2, 1) four_point(3, 1) four_point(4, 1)];

    % 获得新四边形的大小
    width=round(sqrt((x(1)-x(2))^2+(y(1)-y(2))^2));
    height=round(sqrt((x(1)-x(3))^2+(y(1)-y(3))^2));

    % 新的四个顶点
    Y = [1 1 1+height 1+height];
    X = [1 1+width 1 1+width];

    % 源图像中的点的坐标矩阵为：
    B = [x(1) y(1);
         x(2) y(2);
         x(3) y(3);
         x(4) y(4)];
    % 目标图像中对应的顶点坐标为：
    A = [X(1) Y(1);
         X(2) Y(2);
         X(3) Y(3);
         X(4) Y(4)];

    TForm = cp2tform(B, A, 'projective');
    % 从变换图像反向寻找原图像的点（避免出现空洞）
    result_img = zeros(height, width, 3);
    for i = 1 : height
        for j = 1 : width
            pix = round(tforminv(TForm,[j i]));

            while true
                if pix(1) >= 1 && pix(2) >= 1 && pix(1) <= img_size(2) && pix(2) <= img_size(1)
                    % 最邻近插值
                    result_img(i, j, :) = I(pix(2), pix(1), :);
                    break
                else
                    if pix(1) < 1
                        pix(1) = 1;
                    end
                    if pix(2) < 1
                        pix(2) = 1;
                    end
                    if pix(1) > img_size(2)
                        pix(1) = img_size(2);
                    end
                    if pix(2) > img_size(1)
                        pix(2) = img_size(1);
                    end
                end
            end
        end
    end

    % 画出矫正后的图像
    figure(10)
    imshow(uint8(result_img));
    title('矫正后的图像')
end