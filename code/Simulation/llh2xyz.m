    function xyz = llh2xyz(usrllh)
        
%WGS2XYZ  Convert from latitude, longitude and height
%         to ECEF cartesian coordinates.  WGS-84
%
%	xyz = WGS2XYZ(llh)	
%
%	llh(1) = latitude in radians
%	llh(2) = longitude in radians
%	llh(3) = height above ellipsoid in meters
%
%	xyz(1) = ECEF x-coordinate in meters
%	xyz(2) = ECEF y-coordinate in meters
%	xyz(3) = ECEF z-coordinate in meters

%	Reference: Stanford GPS lab SGMP code

    wlat = usrllh(1);
    wlon = usrllh(2);
    walt = usrllh(3);

	A_EARTH = 6378137;
	flattening = 1/298.257223563;
	NAV_E2 = (2-flattening)*flattening; % also e^2
	deg2rad = pi/180;

	slat = sin(wlat*deg2rad);
	clat = cos(wlat*deg2rad);
	r_n = A_EARTH/sqrt(1 - NAV_E2*slat*slat);
	xyz = [ (r_n + walt)*clat*cos(wlon*deg2rad);  
	        (r_n + walt)*clat*sin(wlon*deg2rad);  
	        (r_n*(1 - NAV_E2) + walt)*slat ]';

	if ((wlat < -90.0) | (wlat > +90.0) |...
				(wlon < -180.0) | (wlon > +360.0))
		error('WGS lat or WGS lon out of range');
        end
return
