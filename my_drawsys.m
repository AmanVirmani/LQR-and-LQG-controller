function my_drawsys(s,M,m1,m2,l1,l2,g)
x = s(1);
t1 = s(3);
t2 = s(5);

% dimensions
W = 1*sqrt(M/1000);  % cart width
H = .5*sqrt(M/1000); % cart height
wr = .2; % wheel radius
mr1 = .3*sqrt(m1/100);% mass 1 radius
mr2 = .3*sqrt(m2/100);% mass 2 radius

% positions
y = wr/2+H/2; % cart vertical position
w1x = x-.9*W/2;
w1y = 0;
w2x = x+.9*W/2-wr;
w2y = 0;

px1 = x + l1*sin(t1)/10;
py1 = y - l1*cos(t1)/10;
px2 = x + l2*sin(t2)/10;
py2 = y - l2*cos(t2)/10;

%%Plot commands

plot([-10 10],[0 0],'k','LineWidth',2)
hold on
rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1])
rectangle('Position',[w1x,w1y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])
rectangle('Position',[w2x,w2y,wr,wr],'Curvature',1,'FaceColor',[0 0 0])

plot([x px1],[y py1],'k','LineWidth',2)
plot([x px2],[y py2],'k','LineWidth',2)


rectangle('Position',[px1-mr1/2,py1-mr1/2,mr1,mr1],'Curvature',1,'FaceColor',[.1 0.1 1])
rectangle('Position',[px2-mr2/2,py2-mr2/2,mr2,mr2],'Curvature',1,'FaceColor',[.1 0.1 1])

% set(gca,'YTick',[])
% set(gca,'XTick',[])
xlim([-10 10]);
ylim([-5 5]);
set(gcf,'Position',[100 550 1000 400])
% box off
drawnow
hold off


