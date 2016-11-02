img=imread('bloggie.jpg'); %get input image
img2=imresize(img,[180,320]);
gray=rgb2gray(img2);    %convert to grayscale
[centers, radii] = imfindcircles(gray,[70 1000],'ObjectPolarity','bright','Sensitivity',0.96); %get the outer circle's center and radius
x_cen=floor(centers(1));
y_cen=floor(centers(2));
rad=floor(radii(1))
f=360/570;
output=zeros(530,570);
for t= 1:570
    x=[x_cen+10 x_cen+rad*cosd(f*t)]; %define the starting and ending points of the line segment whose pixel intensities are required
    y=[y_cen+10 y_cen+rad*sind(f*t)];
    row=improfile(gray,x,y,530);
    col=transpose(row); %improfile returns a row vector.. it is intuitive to work with column vectors :P
    
    scalefactor = rad/29;
    for i=1:530
    x_cm=(x(1) + (i/530)*(x(2)-x(1)))/scalefactor
    y_cm=y_final(x_cm,29,21,138);
    y_p=y_cm*scalefactor
    output(ceil(y_p)+1,t)=col(1,i); %assignment of column vector to the output matrix
    end
end
out=uint8(output); %very imp .. since original type of output is double 
imshow(out);
    
    