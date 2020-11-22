
function [x_p , y_p] = EqAzimuthal(lat,long,lat_0,long_0,R)
% This function accepts a spherical sketch in latitude and longitude
% coordinates and returns the result of projecting it onto a plane with 
% Cartesian coordinates via azimuthal equidistant projection. The projection
% plane is defined as tangent to the sphere or radius R at the center point
%(long_0,lat_0). The origin of the resulting 2-D sketch is equal to the
% center point of the projection.

% INPUTS:
%   lat: 1xN or Nx1 vector in radians
%   long: 1xN or Nx1 vector in radians 
%   R: 1x1 scalar equal to the radius of the sphere in Cartesian distance units
%       (R, lat,long) defines the spherical sketch of N points 
%   lat_0: 1x1 scalar in radians equal to the center point latitude
%   long_0: 1x1 scalar in radians equal to the center point logitude
%       (lat_0,long_0,) defines the center point of the projection 

% OUTPUTS:
%   x_p: Nx1 vector in Cartesian distance units
%   y_p: Nx1 vector in Cartesian distance units
%       (x,y) defines the  resulting sketch of N points in 2-D 
%       Euclidean space (the projection plane)


% APPLICATION TO SPHERICAL TILING AND PLATONIC SOLIDS:

% Because sphere-to-plane projections often result in significant global 
% distortions but much smaller local distortions (particularly about the
% projection center), polyhderal maps, often based on Platonic solids, can 
% be used to "unwrap" the sphere with less global distortion. This is done
% by dividing the sphere into spherical polyhedral tiles and then mapping
% each tile individually onto a plane. Inverse projections (e.g., 
% gnomonic, inverse orthographic) can be used to tile the sphere and a
% variety of projections can be used to project each tile onto the plane. 
% One option with radially symmetric distortion is the azimuthal
% equidistant projection. 

% Check that input is valid:
N = length(lat);
if ~isvector(lat) || ~isvector(long) || N ~= length(long)
    error('Input sketch must consist of two vectors lat and long of equal length.')
end

if abs(long_0)> pi || abs(lat_0) > pi/2
    error(['Center point longitude and latituide values must be expressed' ...
    'in radians and be valid values.'])
end

% Initialize variables
x_p = zeros(N,1);
y_p = zeros(N,1);

% Perform equidistant azimuthal projection
for i = 1:N
    if lat_0 == pi/2
        p = R*(pi/2-lat(i));
        theta = long(i);
    else 
        p = R*acos(sin(lat_0)*sin(lat(i))+ ....
        cos(lat_0)*cos(lat(i))*cos(long(i)-long_0));
        theta = atan2(cos(lat(i))*sin(long(i)-long_0), ...
        cos(lat_0)*sin(lat(i))-sin(lat_0)*cos(lat(i))*cos(long(i)-long_0) );
    end
    x_p(i) = p*sin(theta);
    y_p(i) = -p*cos(theta);
end
end