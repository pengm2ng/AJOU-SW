function [state] = changeuav(airplntran,ils1,ils2,ils3,ils4)
%UNTITLED 이 함수의 요약 설명 위치
%   자세한 설명 위치
GCS_1 = ils1.coord.point(2,:);
GCS_2 = ils2.coord.point(2,:);
GCS_3 = ils3.coord.point(2,:);
GCS_4 = ils4.coord.point(2,:);

set(ils1.coord,'point',[airplntran.translation;GCS_1]);
set(ils2.coord,'point',[airplntran.translation;GCS_2]);
set(ils3.coord,'point',[airplntran.translation;GCS_3]);
set(ils4.coord,'point',[airplntran.translation;GCS_4]);

end

