function [lat , long] = InverseOrtho(x_in,y_in,lat_0,long_0,R)
% This function accepts a 2-D base sketch in Cartesian coordinates and
% returns the result of mapping it onto a sphere with spherical coordinates
% via inverse orthographic projection. The projection plane is defined as
% tangent to the center point (long_0,lat_0) of the sphere or radius R. The
% origin of the base sketch is equal to the center point of the projection.

% INPUTS:
%   x_in: 1xN or Nx1 vector in Cartesian distance units
%   y_in: 1xN or Nx1 vector in Cartesian distance units
%       (x_in,y_in) defines the base sketch of N points in 2-D 
%       Euclidean space (the projection plane)
%   R: 1x1 scalar equal to the radius of the sphere in Cartesian distance units
%   lat_0: 1x1 scalar in radians equal to the center point latitude
%   long_0: 1x1 scalar in radians equal to the center point logitude
%       (long_0, lat_0) defines the center point of the projection 

% OUTPUTS:
%   lat: Nx1 vector in radians
%   long: Nx1 vector in radians
%       (R, lat, long) defines the  resulting spherical sketch in spherical 
%       coordinates 


% APPLICATION TO SPHERICAL TILING AND PLATONIC SOLIDS:

% Inverse orthographic projection is a convenient way to tile a sphere.
% Consider the circumscribed sphere of a Platonic solid. The line segment l_c 
% connecting the center of an edge and the center of the sphere
% intersects the circle at point c. The line segment l_v connecting a vertex
% and the center of the sphere intersects the circle at point v. Generic
% tiling is achieved by inverse gonomically projecting the edges onto the 
% sphere(forming a spherical polyhderon). This is equivalent to combining 
% multiple inverse orthographic projections of each edge relative to its 
% respective center point c. 
% More unique tiling can be achieved by combining multiple inverse
% orthographic projections with a base sketch that is bounded in its domain
% by an edge,but not equivalent to it. To preserve the required symmetry 
% of isohedral tiles, we can construct sketch plane tangent to the circle at
% point c. For a Platonic solid with edge length e, we can then define a 
% base sketch with origin c and domain x = [-e/2, e/2], along with the 
% following constraints:
%    - the sketch must be rotational symmetric about c.
%    - l_v must intersect the end point of the sketch: y(-e/2) = y(e/2) = 0
% The center point of the projection is a constant for each Platonic solid
% and is defined as (0,lat_0), where lat_0 = pi/2 - DihedralAngle/2.

% Check that input is valid:
N = length(x_in);
if ~isvector(x_in) || ~isvector(y_in) || N ~= length(y_in)
    error('Input sketch must consist of two vectors x_in and y_in of equal length.')
end

if range(x_in)> 2*R || range(y_in)>2*R
    warning('Input sketch values exceed size of sphere. Projection may be clipped.')
end

if abs(long_0)> pi || abs(lat_0) > pi/2
    error(['Center point longitude and latituide values must be expressed' ...
    'in radians and be valid values.'])
end

% Initialize variables
lat = zeros(N,1);
long = zeros(N,1);

% Perform inverse orthographic projection
for i = 1:N
        p = norm([x_in(i) y_in(i)]);
        c = asin(p/R);
        lat(i) = asin(cos(c)*sin(lat_0)+y_in(i)*sin(c)*cos(lat_0)/p);
        long(i) = long_0+ atan2(x_in(i)*sin(c),(p*cos(c)*cos(lat_0)-y_in(i)*sin(c)*sin(lat_0)));
end

end


