function [state] = draw(airpln,ils1,ils2,ils3,ils4,GCS_1,GCS_2,GCS_3,GCS_4)
%UNTITLED 이 함수의 요약 설명 위치
%   자세한 설명 위치
 
                ils1.coord.point = [airpln.translation; GCS_1.translation;];
                disp(airpln.translation);
                disp(GCS_1.translation);
                ils1.coordIndex = [0; 1; -1;];
                 ils1.color.color = [0 0 0];


                ils2.coord.point = [airpln.translation; GCS_2.translation;];
                ils2.colorPerVertex = 'FALSE';
                ils2.coordIndex = [0; 1; -1];
                 ils2.color.color = [0 0 0];



                ils3.colorPerVertex = 'FALSE';
                 ils3.color.color = [0 0 0];
                ils3.coord.point = [airpln.translation; GCS_3.translation;];
                ils3.coordIndex = [0; 1; -1];



                ils4.colorPerVertex = 'FALSE';
                 ils4.color.color = [0 0 0];
                ils4.coord.point =  [airpln.translation; GCS_4.translation;];
                ils4.coordIndex = [0; 1; -1];
                vrdrawnow
                


                pause(0.1);
                delline(ils1,ils2,ils3,ils4);
                vrdrawnow
end

