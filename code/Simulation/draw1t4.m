function [state] = draw1t4(airpln1,airpln2, airpln3, airpln4, ils1,ils2,ils3,ils4, GCS_1,GCS_2,GCS_3,GCS_4)
%UNTITLED 이 함수의 요약 설명 위치
%   자세한 설명 위치
%          for j=1:9
                ils1.coord.point = [airpln1.translation; GCS_1.translation; airpln2.translation; airpln3.translation;airpln4.translation];
                disp(airpln1.translation);
                disp(GCS_1.translation);
                ils1.colorPerVertex = 'FALSE';
                ils1.coordIndex = [0; 1; -1; 2; 1; -1; 3; 1; -1; 4; 1; -1;];
                ils1.color.color = [0 0 1; 0 0 1; 0 0 1; 0 0 1;];
vrdrawnow


                ils2.coord.point = [airpln1.translation; GCS_2.translation; airpln2.translation; airpln3.translation;airpln4.translation];
                ils2.colorPerVertex = 'FALSE';
                ils2.coordIndex = [0; 1; -1; 2; 1; -1; 3; 1; -1; 4; 1; -1;];
                ils2.color.color = [1 0 1; 1 0 1; 1 0 1; 1 0 1;];
vrdrawnow


                ils3.colorPerVertex = 'FALSE';
                ils3.coord.point = [airpln1.translation; GCS_3.translation; airpln2.translation; airpln3.translation;airpln4.translation];
                ils3.coordIndex = [0; 1; -1; 2; 1; -1; 3; 1; -1; 4; 1; -1;];
                ils3.color.color = [0 1 0; 0 1 0; 0 1 0; 0 1 0;];
vrdrawnow


                ils4.colorPerVertex = 'FALSE';
                ils4.coord.point =  [airpln1.translation; GCS_4.translation; airpln2.translation; airpln3.translation;airpln4.translation];
                ils4.coordIndex = [0; 1; -1; 2; 1; -1; 3; 1; -1; 4; 1; -1;];
                ils4.color.color = [0.8 0.33 0.1; 0.8 0.33 0.1; 0.8 0.33 0.1; 0.8 0.33 0.1;];
                vrdrawnow

            


                pause(0.1);
                delline(ils1,ils2,ils3,ils4);
                vrdrawnow
%             end
end

