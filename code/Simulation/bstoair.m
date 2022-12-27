function [state] = bstoair(world,air_location,GCS1,GCS2,GCS3,GCS4, s1, s2, s3, s4)
%UNTITLED 이 함수의 요약 설명 위치
%   자세한 설명 위치
            
            ils1 = vrnode(s1, 'geometry', 'ILS', 'IndexedLineSet');
            ilscoord1 = vrnode(ils1, 'coord', 'ILSCoordinate', 'Coordinate');
            ilscolor1 = vrnode(ils1, 'color', 'ColorCoordinate', 'Color');
            ils1.coord.point = [air_location; GCS1];
            ils1.colorPerVertex = 'FALSE';
            ils1.coordIndex = [0; 1; -1];
            ils1.color.color = [1 0.2 0.2];
            drawnow;
            ils2 = vrnode(s2, 'geometry', 'ILS', 'IndexedLineSet');
            ilscoord2 = vrnode(ils2, 'coord', 'ILSCoordinate', 'Coordinate');
            ilscolor2 = vrnode(ils2, 'color', 'ColorCoordinate', 'Color');
            ils2.coord.point = [air_location; GCS2];
            ils2.colorPerVertex = 'FALSE';
            ils2.coordIndex = [0; 1; -1];
            ils2.color.color = [0 0 1];
            drawnow;
            
            ils3 = vrnode(s3, 'geometry', 'ILS', 'IndexedLineSet');
            ilscoord3 = vrnode(ils3, 'coord', 'ILSCoordinate', 'Coordinate');
            ilscolor3 = vrnode(ils3, 'color', 'ColorCoordinate', 'Color');
            ils3.colorPerVertex = 'FALSE';
            ils3.color.color = [0 1 0];
            ils3.coord.point = [air_location; GCS3];
            ils3.coordIndex = [0; 1; -1];
           drawnow;
            
            ils4 = vrnode(s4, 'geometry', 'ILS', 'IndexedLineSet');
            ilscoord4 = vrnode(ils4, 'coord', 'ILSCoordinate', 'Coordinate');
            ilscolor4 = vrnode(ils4, 'color', 'ColorCoordinate', 'Color');
            ils4.colorPerVertex = 'FALSE';
            ils4.color.color = [1 1 1];
            ils4.coord.point = [air_location; GCS4];
            ils4.coordIndex = [0; 1; -1];
            vrdrawnow;
end

