load('gradientFiled.mat', 'fx_final', 'fy_final')
[Xmesh,Ymesh]=meshgrid(-4:0.4:1.5,-3:0.4:2);
rx = [0];
ry = [0];
step = 0.1;
gx = 0;
gy = 0;
figure()
hold on
for i = 1:25
    
    G = findGradient(rx(i),ry(i),7);
    gx = (G(1))/(sqrt(G(1)^2 + G(2)^2))
    gy = (G(2))/(sqrt(G(1)^2 + G(2)^2))
    gx=gx*step;
    gy=gy*step;
    
    
    r_next_x = rx(i) + gx
    r_next_y = ry(i) + gy
    rx = [rx r_next_x];
    ry = [ry r_next_y];
    

   
end
plot(rx, ry,"Color","red")




drawSqDiag(-1,.25,.25*sqrt(2),3)
drawSqDiag(-.7,-1,.25*sqrt(2),3)
drawSqNorm(-2,-1.141,.25,3)
plotCircle(-2.5,-.75,.25,3)
streamslice(Xmesh,Ymesh,fx_final,fy_final)
% quiver(0,0,G(1,1),G(1,2))
% quiver(0,0,5*cosd(188),5*sind(188))
axis equal
hold off
    

save('trajPos.mat','rx','ry');

function grad = findGradient(xValue,yValue,scaleValue)
    
    x = xValue;
    y = yValue;
    v = 0;
    fx = 0;
    fy = 0;
    rad_sq = .25*sqrt(2); %radius for a circle to circumscribe our square obstacles

    scalerX = scaleValue;
    scalerY = scaleValue;

    for theta = 0:0.1:2*pi

        bobx = -2.5+.25.*cos(theta);
        boby = -.75+.25*sin(theta);
        sq1x = -1+rad_sq.*cos(theta);
        sq1y = .25+rad_sq.*cos(theta);
        sq2x = -.7+rad_sq.*cos(theta);
        sq2y = -1+rad_sq.*cos(theta);
        sq3x = -2+rad_sq.*cos(theta);
        sq3y = -1.41+rad_sq.*cos(theta);

        %v = - log(sqrt((x-bobx).^2 + (y-boby).^2)) + log(sqrt((x-sq1x).^2 + (y-sq1y).^2)) + log(sqrt((x-sq2x).^2 + (y-sq2y).^2)) + log(sqrt((x-sq3x).^2 + (y-sq3y).^2)) +v_wall2;


        obs1Scaler = 1.4;
        obs2Scaler = .3;
        fx = fx - scalerX.*((x-bobx)./((x-bobx).^2+(y-boby).^2)) + obs1Scaler.*((x-sq1x)./((x-sq1x).^2+(y-sq1y).^2)) + obs2Scaler.*((x-sq2x)./((x-sq2x).^2+(y-sq2y).^2)) + ((x-sq3x)./((x-sq3x).^2+(y-sq3y).^2));

        fy = fy - scalerY.*((y-boby)./((x-bobx).^2+(y-boby).^2)) + obs1Scaler.*((y-sq1y)./((x-sq1x).^2+(y-sq1y).^2)) + obs2Scaler.*((y-sq2y)./((x-sq2x).^2+(y-sq2y).^2)) + ((y-sq3y)./((x-sq3x).^2+(y-sq3y).^2));

    end



    v_wall = 0;
    fx_wall = 0;
    fy_wall = 0;

    %top wall vector field
    for a = -3.37:0.01:1

    %     v_wall = v_wall - log(sqrt((x-a).^2 + (y-1.5).^2));

        fx_wall = fx_wall - ((x-a)./((x-a).^2+(y-1.5).^2));

        fy_wall = fy_wall - ((y-1.5)./((x-a).^2+(y-1.5).^2));
    end
    %bottom wall vector field
    for a = -3.37:0.01:1

    %     v_wall = v_wall - log(sqrt((x-a).^2 + (y+2.5).^2));

        fx_wall = fx_wall - ((x-a)./((x-a).^2+(y+2.5).^2));

        fy_wall = fy_wall - ((y+2.5)./((x-a).^2+(y+2.5).^2));
    end
    %right wall vector field
    for a = -2.5:0.01:1.5

    %     v_wall = v_wall - log(sqrt((x-1).^2 + (y-a).^2));

        fx_wall = fx_wall - ((x-1)./((x-1).^2+(y-a).^2));

        fy_wall = fy_wall - ((y-a)./((x-1).^2+(y-a).^2));
    end
    %left wall vector field
    for a = -2.5:0.01:1.5

    %     v_wall = v_wall - log(sqrt((x+3.37).^2 + (y-a).^2));

        fx_wall = fx_wall - ((x+3.37)./((x+3.37).^2+(y-a).^2));

        fy_wall = fy_wall - ((y-a)./((x+3.37).^2+(y-a).^2));
    end
    
    
    
    fx_final = .01*fx_wall + fx
    fy_final = .01*fy_wall + fy
    
    grad = [fx_final fy_final]

  
end

function drawSqDiag(x,y,r,w)
    coords = zeros(2,5)    
    coords(1,1) = x 
    coords(1,2) = x-r
    coords(1,3) = x
    coords(1,4) = x+r
    coords(2,1) = y+r
    coords(2,2) = y
    coords(2,3) = y-r
    coords(2,4) = y
    coords(1,5) = x
    coords(2,5) = y+r
    plot(coords(1,:),coords(2,:),"Color","Red","LineWidth",w)
end

function drawSqNorm(x,y,d,w)
    coords = zeros(2,5)    
    coords(1,1) = x-d
    coords(1,2) = x-d
    coords(1,3) = x+d
    coords(1,4) = x+d
    coords(2,1) = y+d
    coords(2,2) = y-d
    coords(2,3) = y-d
    coords(2,4) = y+d
    coords(1,5) = x-d
    coords(2,5) = y+d
    plot(coords(1,:),coords(2,:),"Color","Red","LineWidth",w)
end

function plotCircle(x,y,r,w)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit,"Color","Green","LineWidth",w);
    hold off
end
