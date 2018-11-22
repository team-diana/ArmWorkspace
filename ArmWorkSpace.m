close all
clear all 
clc

%%% Rover Data %%%

% CHASSIS
h1=500; %chassis upper side height
h2=100; %shoulder joint height WRT chassis
h0=h1+h2; %shoulder joint height WRT ground

% ARM
% link lenghts: arm,fore arm, 2nd wrist joint to end effector end (115+150?)
l=[600 377 315];

% joints angles in degrees: 
%                                 elbow a2 (zero parallel to arm link pointing towards  )

a=[-5 120   %shoulder(zero: parallel to ground pointing forward)
   33 129   %elbow (zero: pointing towards shoulder joint parallel to arm link)
   -90 90]; %second wrist joint (zero pointing towards out palallel to forearm link) 


%% Plotting rover 
figure,hold on
axis equal

%% Import image of rover in plot, comment if not necessary
img = imread('Immagine23456.png');
image('CData',img,'XData',[ -1020 1100],'YData',[1380 0 ])

%% Plotting rover chassis, shoulder joint and wheels
%ground
plot([-1500,1500],[0,0],'k','LineWidth',2)
%chassis
S=[1 -1 -1 1 1 ; 1 1 -1 -1 1];     %definig sqare
R=(diag([-375 75])*S)+[-275;425];  %multipling square per half the size of chassis
                                   %and shifting it in order to have base
                                   %rotation positioned in x=0
plot(R(1,:),R(2,:),'k') %plotting chassis
plot(0,600,'rx')        %plotting red x in shoulder joint position            

%Wheels ([centerXpos centerYpos],radius)
viscircles([281 130],130,'Color','k')
viscircles([281-683 130],130,'Color','k')
viscircles([281-1142 130],130,'Color','k')

%% Computation of direct kinematics for arm facing forwards

j1=[0 h0]; %Shoulder joint position
i=0;

for a1=deg2rad(linspace(a(1,1),a(1,2),5)) 
   
  %Elbow joint 
  for a2=deg2rad(linspace(a(2,1),a(2,2),5))
      
    %Wrist termination + end effector
    for a3=deg2rad(linspace(a(3,1),a(3,2),5))
      
        j2=[j1(1)+l(1)*cos(a1),j1(2)+l(1)*sin(a1)];
        j3=[j2(1)+l(2)*cos(a1-a2),j2(2)+l(2)*sin(a1-a2)];
        j4=[j3(1)+l(3)*cos(a1-a2-a3),j3(2)+l(3)*sin(a1-a2-a3)]; %Tool center point
       
      %if cos(a1-a2-a3)>(0.999) % ENDEFFECTOR HORIZONTAL
      %if sin(a1-a2-a3)<(-0.999) %ENDEFFECTOR VERTICAL
        i=i+1; 
        plot([j1(1),j2(1)],[j1(2),j2(2)],'b')
        plot([j2(1),j3(1)],[j2(2),j3(2)],'g')
        plot([j3(1),j4(1)],[j3(2),j4(2)],'r')
        plot(j4(1),j4(2),'ok')
            x(i)=j4(1);
            y(i)=j4(2);
        pause(0.00001)
       %end of if
     end         
  end
end 
 
% plotting workspace boundaries
% scatter(x,y)
k = boundary(x(:),y(:));
plot(x(k),y(k),'LineWidth',2)
hold on

%%
%Chassis reach, arm facing backwords similar to above
x=0;
y=0;
i=0;
for a1=deg2rad(linspace(140,90,5)) %SHOULDER ANGLE RANGE
   
    
  %Elbow joint 
  for a2=deg2rad(linspace(a(2,1),a(2,2),5)) %angle range    
      
%Wrist termination + end effector
    for a3=deg2rad(linspace(a(3,1),a(3,2),5))
    
       j2=[j1(1)+l(1)*cos(a1),j1(2)+l(1)*sin(a1)];
       j3=[j2(1)+l(2)*cos(a1+a2),j2(2)+l(2)*sin(a1+a2)];
       j4=[j3(1)+l(3)*cos(a1+a2+a3),j3(2)+l(3)*sin(a1+a2+a3)]; %Tool center point
    
    %if cos(a1+a2+a3)<(-0.999) %HORIZONTAL
    %if sin(a1+a2+a3)<(-0.999) %VERTICAL
        i=i+1;
        plot([j1(1),j2(1)],[j1(2),j2(2)],'b')
        plot([j2(1),j3(1)],[j2(2),j3(2)],'g')
        plot([j3(1),j4(1)],[j3(2),j4(2)],'r')
        plot(j4(1),j4(2),'ok')
        pause(0.005)
          x(i)=j4(1);
          y(i)=j4(2);
     %end  
    end
  end 
end  

k = boundary(x(:),y(:));
plot(x(k),y(k),'--','LineWidth',2)
hold on



