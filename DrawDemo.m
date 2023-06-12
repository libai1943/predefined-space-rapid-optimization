function DrawDemo()
global params_
case_id = params_.user.case_id;
figure(case_id)

xmin = params_.demo.xmin;
xmax = params_.demo.xmax;
ymin = params_.demo.ymin;
ymax = params_.demo.ymax;

axis([xmin xmax ymin ymax]);
hold on; box on; grid minor; axis equal;
xlabel('x / m', 'FontSize', 12, 'FontName', 'Arial Narrow', 'FontWeight', 'Bold');
ylabel('y / m', 'FontSize', 12, 'FontName', 'Arial Narrow', 'FontWeight', 'Bold');
axis([-20 20 -20 20])
Arrow([params_.task.x0,params_.task.y0], [params_.task.x0+ cos(params_.task.theta0),params_.task.y0+ sin(params_.task.theta0)], 'Length',12,'BaseAngle',90,'TipAngle',14,'Width',2);
Arrow([params_.task.xtf,params_.task.ytf], [params_.task.xtf+ cos(params_.task.thetatf),params_.task.ytf+ sin(params_.task.thetatf)], 'Length',12,'BaseAngle',90,'TipAngle',14,'Width',2);

for kk = 1 : params_.obstacle.Nobs
    fill(params_.obstacle.obs{kk}.x, params_.obstacle.obs{kk}.y, [125, 125, 125] ./ 255);
end


end