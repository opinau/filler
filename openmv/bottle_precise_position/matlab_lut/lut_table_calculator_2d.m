% matlab code for building LUT
% matrix stores x_piksxel y_pixel true_x_pos in mm
clear all

%start main code and read  spot position in image px:  152 py:  81  ...
%store px and py in matrix in first two columns, last column is y_pos in mm
%this code generates LUT in LUT.bmp (for simplicity)
%copy to camera root and restart camera

A= [
    160 2 0
    160 7.5 0
    160 10 0
    160 22 0
    160 30 0
    160 37 0
    160 44 0
    160 50 0
    160 55 0
    160 59.5 0
    160 63 0
    160 66 0
    160 69.5 0
    160 72 0
    160 74 0
    160 79 0
    160 81 0
    160 82 0
    160 87 0
    160 92 0
    160 95 0
    24 7 -50
    58 47 -50
    99 79 -50
    109 86 -50
    118 92 -50
    122 96 -50
    10 68 -100
    37 80 -100
    56 87 -100
    72 92 -100
    84 95 -100
    253 46 50
    232 67 50
    218 79 50
    210 86 50
    204 92 50
    198 96 50
    308 68 100
    278 79 100
    260 86 100
    246 92 100
    235 95 100
    139 46 -10
    117 47 -20
    96 46 -30
    75 46 -40
    42 46 -60
    24 47 -70
    5 48 -80
    179 46 10
    198 46 20
    215 46 30
    233 46 40
    272 46 60
    293 47 70
    310 47 80
    144 67 -10
    130 67 -20
    114 67 -30
    99 67 -40
    68 67 -60
    54 68 -70
    42 67 -80
    28 68 -90
    175 67 10
    190 67 20
    202 67 30
    218 67 40
    248 67 60
    262 67 70
    277 68 80
    291 68 90
    ]
    
    
plot3(A(:,1),A(:,2),A(:,3),'ro')

hold on
sf = fit([A(:,1), A(:,2)],A(:,3),'poly44')

%plot(sf)
%resulting_matrix
%fprintf(fileID,'lut_ypos = {');

%index=1;

lut_image=zeros(240,320);

for x=1:320   
    for y=1:100
        lut_image(y,x)=round(feval(sf,x,y)+125);
        if (lut_image(y,x)<0) || (lut_image(y,x)>255 )
            lut_image(y,x)=0;
        end
    end
end


imwrite(uint8(lut_image),'LUT.bmp','BMP')
%please note that LUT is shifted 125 mm in order to use only postive
%numbers (uint8), which is  aequate for scanners range
%lut is stored in bitmap image

