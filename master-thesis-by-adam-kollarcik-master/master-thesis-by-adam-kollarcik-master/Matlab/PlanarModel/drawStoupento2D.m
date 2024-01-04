function []= drawStoupento2D(t,y,params,xrange,yrange,configOnly,record,secondMonitor,varargin)



%%
r = params(1);
r3 = params(2);
ang_3 = params(3);
x0 = params(4);
y0 = params(5);
l1c = params(6);
l11 = params(7);
l2c = params(8);
l4c = params(9);
l3 = params(10);
a_body = params(11);
b_body = params(12);
%%

figure(25)
if secondMonitor
    set(gcf,'Position',[-1919 -3 1920 1123]) %% lab monitor 
end








% th = y(1,6:end);
th = y(1,4:end);
[x0,x1,x12,x2,x23,x14,x4,x34,x3,x45,B] = get_points(th(1),th(2),th(3),th(4),th(5),r,r3,ang_3,0,0,l1c,l11,l2c,l4c,l3,a_body,b_body);
clf;
% axis equal
% xlim([-1 1])
% ylim([-1 1])
% viscircles(x0',r,'LineWidth',1.3);
hold on
cg = plot([x0(1) x1(1) x2(1) x3(1) x4(1) ], [x0(2) x1(2) x2(2) x3(2) x4(2)],'b*','LineWidth',1.3);
link1 = plot([x0(1) x1(1)], [x0(2) x1(2)],'r','LineWidth',1.3);
% 
link12 = plot([x1(1) x12(1)], [x1(2) x12(2)],'r','LineWidth',1.3);
link122 = plot([x12(1) x2(1)], [x12(2) x2(2)],'r','LineWidth',1.3);
link223=plot([x2(1) x23(1)], [x2(2) x23(2)],'r','LineWidth',1.3);
link14 = plot([x1(1) x14(1)], [x1(2) x14(2)],'r','LineWidth',1.3);
link141 = plot([x14(1) x4(1)], [x14(2) x4(2)],'r','LineWidth',1.3);
link45 = plot([x4(1) x45(1)], [x4(2) x45(2)],'r','LineWidth',1.3);
link2334= plot([x23(1) x34(1)], [x23(2) x34(2)],'r','LineWidth',1.3);
link233= plot([x23(1) x3(1)], [x23(2) x3(2)],'r','LineWidth',1.3);

a = patch('Vertices',B','Faces',[1 2 3 4],'Edgecolor',[100/255 100/255 100/255],'Facecolor',[100/255 100/255 100/255],'Linewidth',1.2,'FaceAlpha',0);


x_O = r*th(1);
y_O = r;

angle_q1 = (0:0.01:pi/2)+th(1);
x_q1 =r*th(1)+r* cos(angle_q1);
y_q1 = r- r*sin(angle_q1);

angle_q2 = (pi/2:0.01:pi)+th(1);
x_q2 =r*th(1)+ r*cos(angle_q2);
y_q2 = r-r*sin(angle_q2);
angle_q3 = (pi:0.01:3/2*pi)+th(1);
x_q3 = r*th(1)+r*cos(angle_q3);
y_q3 = r-r*sin(angle_q3);
angle_q4 = (3/2*pi:0.01:2*pi)+th(1);
x_q4 = r*th(1)+r*cos(angle_q4);
y_q4 = r-r*sin(angle_q4);

q1 = fill([x_O,x_q1],[y_O,y_q1],'black','EdgeColor','none');
q2 = fill([x_O,x_q2],[y_O,y_q2],'w');
q3 = fill([x_O,x_q3],[y_O,y_q3],'black','EdgeColor','none');
q4 = fill([x_O,x_q4],[y_O,y_q4],'w');
yline(0);
time_text = title("Time= "+num2str(t(1),'%.2f')+" s",'FontSize',18);
grid on
xlim(xrange)
ylim(yrange)
xlabel('x (m)','FontSize',15);
ylabel('y (m)','FontSize',15);
axis equal
axis manual



if ~configOnly
if numel(t)>1
    pause()
end

%% uncomment for video recording #1/3
 if record
 F(1) =  getframe(gcf) ;
 end
%%

for idx =  2 :numel(t)
    tic
% th = y(idx,6:end);
th = y(idx,4:end);
[x0,x1,x12,x2,x23,x14,x4,x34,x3,x45,B] = get_points(th(1),th(2),th(3),th(4),th(5),r,r3,ang_3,0,0,l1c,l11,l2c,l4c,l3,a_body,b_body);
% clf;
% xlim([-0.5 0.5])
% ylim([-1 1])
% viscircles(x0',r,'LineWidth',1.3);
% hold on
cg.XData = [x0(1) x1(1) x2(1) x3(1) x4(1) ];
cg.YData = [x0(2) x1(2) x2(2) x3(2) x4(2)];
link1.XData = [x0(1) x1(1)];
link1.YData = [x0(2) x1(2)];
link12.XData = [x1(1) x12(1)];
link12.YData = [x1(2) x12(2)];
link122.XData = [x12(1) x2(1)];
link122.YData = [x12(2) x2(2)];
link223.XData = [x2(1) x23(1)];
link223.YData = [x2(2) x23(2)];
link14.XData = [x1(1) x14(1)];
link14.YData = [x1(2) x14(2)];
link141.XData = [x14(1) x4(1)];
link141.YData = [x14(2) x4(2)];
link45.XData = [x4(1) x45(1)];
link45.YData = [x4(2) x45(2)];
link2334.XData = [x23(1) x34(1)];
link2334.YData = [x23(2) x34(2)];
link233.XData = [x23(1) x3(1)];
link233.YData = [x23(2) x3(2)];
a.Vertices = B';

x_O = r*th(1);
y_O = r;

angle_q1 = (0:0.01:pi/2)+th(1);
x_q1 =r*th(1)+r* cos(angle_q1);
y_q1 = r- r*sin(angle_q1);

angle_q2 = (pi/2:0.01:pi)+th(1);
x_q2 =r*th(1)+ r*cos(angle_q2);
y_q2 = r-r*sin(angle_q2);
angle_q3 = (pi:0.01:3/2*pi)+th(1);
x_q3 = r*th(1)+r*cos(angle_q3);
y_q3 = r-r*sin(angle_q3);
angle_q4 = (3/2*pi:0.01:2*pi)+th(1);
x_q4 = r*th(1)+r*cos(angle_q4);
y_q4 = r-r*sin(angle_q4);



q1.XData = [x_O,x_q1];
q2.XData = [x_O,x_q2];
q3.XData = [x_O,x_q3];
q4.XData = [x_O,x_q4];

q1.YData = [y_O,y_q1];
q2.YData = [y_O,y_q2];
q3.YData = [y_O,y_q3];
q4.YData = [y_O,y_q4];



time_text.String = "Time= "+num2str(t(idx),'%.2f')+" s";
% time_text.Position = [x0(1),0.8,0];

% time_text = text(x0(1),0.8,"time="+t(idx),'FontSize',18)
% axis equal
% grid on


%% uncomment for video recording #2/3
if record
 F(idx) =  getframe(gcf) ;
end
 
 %%
if idx < numel(t)
    timer = toc;
%     disp(timer)
      pause(max(t(idx+1)-t(idx)-timer,0))

% pause()
%      


% display(t(idx))
end



%% uncomment for video recording #3/3
if record
    if ~exist("videos",'dir')
        mkdir("videos");
    end

if size(varargin,2)> 0 
    filename =     "videos\"+varargin{1};
else
%     filename = "simulace_"+strrep(datestr(datetime('now')),' ','_')
filename  = "videos\"+"sim_"+datestr(datetime('now'),'dd-mm-yyyy_HH;MM;SS');

end
    
newVid = VideoWriter(filename, 'MPEG-4'); % New
newVid.FrameRate = 1/(t(2)-t(1));
newVid.Quality = 100;
open(newVid);
% write the frames to the video


for i=1:length(F)
     disp(i)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(newVid, frame);
end
% close the writer object
close(newVid);
disp("Video saved as "+pwd+"\"+filename);
end


%%
end





end

