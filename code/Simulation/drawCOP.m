function drawCOP(ally_uav1, ally_uav2, ally_uav3, ally_uav4, ally_uav5, truck) % flight_centor_coobpinate

global COP_PAX uav_x uav_y
global xLim yLim
global BL_a BL_b BL_c
global boundary_distance
global GCS_1 GCS_2 GCS_3 GCS_4  

moving_UAV1 = '#1';
moving_UAV2 = '#2';
moving_UAV3 = '#3';
moving_UAV4 = '#4';
moving_UAV5 = '#5';
moving_truck = 'GFAC';

%if(xLim < vector_position(1) + 30)
%    xLim = vector_position(1) + 30;
%end
%
%if(yLim < vector_position(3) + 30)
%    yLim = vector_position(3) + 30;
%end

if(xLim < ally_uav1(1) + 30)
    xLim = ally_uav1(1) + 30;
end

if(yLim < ally_uav1(3) + 30)
    yLim = ally_uav1(3) + 30;
end

% COP_PAX = polaraxes(COPPanel); % Creates polar axes inside the panel
uav_x = [uav_x ally_uav1(1)];
uav_y = [uav_y ally_uav1(3)];
%BL_x =  0:0.5:200;
%BL_y = -BL_a/BL_b*BL_x - BL_c/BL_b;

if (length(uav_x) < 71)
    if(boundary_distance ==0)
        %p = plot(COP_PAX,flight_centor_coordinate(1), flight_centor_coordinate(3), 'g*',predicted_coordinate(1), predicted_coordinate(3), 'r*', uav_x, uav_y,'g:', vector_position(1), vector_position(3), 'gd', target_coordinate(1), target_coordinate(3), 'r+',  ally_uav1(1), ally_uav1(3), 'd', ally_uav2(1), ally_uav2(3), 'd', ally_uav3(1), ally_uav3(3), 'd', ally_uav4(1), ally_uav4(3), 'd', truck(1), truck(2), 'd');
        
        p = plot( COP_PAX, ally_uav1(1), ally_uav1(3), 'd', ally_uav2(1), ally_uav2(3), 'd', ally_uav3(1), ally_uav3(3), 'd', ally_uav4(1), ally_uav4(3), 'd', ally_uav5(1), ally_uav5(3), 'd', truck(1), truck(2), 'd');
   
        %p(6).Color = '#00FFFF';
        %p(7).Color = '#00FFFF';
    else
        %p = plot(COP_PAX,flight_centor_coordinate(1), flight_centor_coordinate(3), 'g*',predicted_coordinate(1), predicted_coordinate(3), 'r*',BL_x,BL_y,'r', uav_x, uav_y,'g:', vector_position(1), vector_position(3), 'gd', target_coordinate(1), target_coordinate(3), 'r+', ally_uav1(1), ally_uav1(3), 'bd', ally_uav2(1), ally_uav2(3), 'bd' , ally_uav3(1), ally_uav3(3), 'bd', ally_uav4(1), ally_uav4(3), 'bd', truck(1), truck(3), 'bd');

        p = plot( COP_PAX, ally_uav1(1), ally_uav1(3), 'g*', ally_uav2(1), ally_uav2(3), 'g*', ally_uav3(1), ally_uav3(3), 'g*', ally_uav4(1), ally_uav4(3), 'g*', ally_uav5(1), ally_uav5(3), 'g*', truck(1), truck(2), 'r*');
        %p(7).Color = '#00FFFF';
 
        
        %p(8).Color = '#00FFFF';
    end
%     hold on
    
    text(COP_PAX,  ally_uav1(1) - 5 , ally_uav1(3) + 8, moving_UAV1, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav2(1) - 5 , ally_uav2(3) + 8, moving_UAV2, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav3(1) - 5 , ally_uav3(3) + 8, moving_UAV3, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav4(1) - 5 , ally_uav4(3) + 8, moving_UAV4, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav5(1) - 5 , ally_uav5(3) + 8, moving_UAV5, 'Color', '#00FFFF')
    text(COP_PAX,  truck(1) - 5 , truck(3) + 8, moving_truck, 'Color', 'yellow') 

    lgd = legend(COP_PAX, [p(1) p(6)], {'UAV', 'GFAC'}, 'Location','southeast');
    lgd.Color = [0 0 0];
    lgd.TextColor = [1 1 1];
    lgd.EdgeColor = [1 1 1];
    COP_PAX.Color = [0 0 0];
    COP_PAX.XColor = [1 1 1];
    COP_PAX.YColor = [1 1 1];
    COP_PAX.XLim = [-100 200];
    COP_PAX.YLim = [-100 yLim];
    xlabel(COP_PAX, 'x-axis(km)')
    ylabel(COP_PAX, 'y-axis(km)')
    COP_PAX.YDir = 'reverse';
    
%     hold off
else
    if(boundary_distance == 0)
        %p = plot(COP_PAX, flight_centor_coordinate(1), flight_centor_coordinate(3), 'g*',predicted_coordinate(1), predicted_coordinate(3), 'r*',uav_x(length(uav_x) - 70:length(uav_x)), uav_y(length(uav_y) - 70:length(uav_y)),'g:', vector_position(1), vector_position(3), 'gd', target_coordinate(1), target_coordinate(3), 'r+', ally_uav1(1), ally_uav1(3), 'bd', ally_uav2(1), ally_uav2(3), 'bd', ally_uav3(1), ally_uav3(3), 'bd', compln(1), compln(3),'bd');
      
        
        p = plot( COP_PAX, ally_uav1(1), ally_uav1(3), 'd', ally_uav2(1), ally_uav2(3), 'd', ally_uav3(1), ally_uav3(3), 'd', ally_uav4(1), ally_uav4(3), 'd', ally_uav5(1), ally_uav5(3), 'd', truck(1), truck(2), 'd');
   
    else
        %p = plot(COP_PAX,flight_centor_coordinate(1), flight_centor_coordinate(3), 'g*',predicted_coordinate(1), predicted_coordinate(3), 'r*',BL_x,BL_y,'r',uav_x(length(uav_x) - 70:length(uav_x)), uav_y(length(uav_y) - 70:length(uav_y)),'g:', vector_position(1), vector_position(3), 'gd', target_coordinate(1), target_coordinate(3), 'r+', ally_uav1(1), ally_uav1(3), 'bd', ally_uav2(1), ally_uav2(3), 'bd', ally_uav3(1), ally_uav3(3), 'bd', compln(1), compln(3),'bd');
      
        
        p = plot( COP_PAX, ally_uav1(1), ally_uav1(3), 'g*', ally_uav2(1), ally_uav2(3), 'g*', ally_uav3(1), ally_uav3(3), 'g*', ally_uav4(1), ally_uav4(3), 'g*', ally_uav5(1), ally_uav5(3), 'g*', truck(1), truck(2), 'r*');
        %p(7).Color = '#00FFFF';
   
    end
%     hold on
   
    text(COP_PAX,  ally_uav1(1) - 5 , ally_uav1(3) + 8, moving_UAV1, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav2(1) - 5 , ally_uav2(3) + 8, moving_UAV2, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav3(1) - 5 , ally_uav3(3) + 8, moving_UAV3, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav4(1) - 5 , ally_uav4(3) + 8, moving_UAV4, 'Color', '#00FFFF')
    text(COP_PAX,  ally_uav5(1) - 5 , ally_uav5(3) + 8, moving_UAV5, 'Color', '#00FFFF')
    text(COP_PAX,  truck(1) - 5 , truck(3) + 8, moving_truck, 'Color', 'yellow')

    lgd = legend(COP_PAX, [p(1) p(6)], {'UAV', 'GFAC'}, 'Location','southeast');
    lgd.Color = [0 0 0];
    lgd.TextColor = [1 1 1];
    lgd.EdgeColor = [1 1 1];
    COP_PAX.Color = [0 0 0];
    COP_PAX.XColor = [1 1 1];
    COP_PAX.YColor = [1 1 1];
    COP_PAX.XLim = [-100 200];
    COP_PAX.YLim = [-100 yLim];
    xlabel(COP_PAX, 'x-axis(km)')
    ylabel(COP_PAX, 'y-axis(km)')
    COP_PAX.YDir = 'reverse';
    
%     hold off
end


end

