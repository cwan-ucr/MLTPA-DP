function ret = myevent(X0, T0, XF, TF, X0_con, XF_con, stage)
%MYEVENT 此处显示有关此函数的摘要
%   此处显示详细说明

ret = [X0_con(1) - X0(1); X0_con(2) - X0(2);
       XF_con(1) - XF(1); XF_con(2) - XF(2)];
if stage == 1
    ret = [X0_con(1) - X0(1); X0_con(2) - X0(2);
           XF_con(1) - XF(1);];
end

end

