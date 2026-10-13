function T=fk(q,m)
%FK Space product of exponentials: exp([S1]q1)...exp([S6]q6)*M.
q=panda.joint_vector(q);T=m.home;
for i=m.n:-1:1,T=MatrixExp6(VecTose3(m.Slist(:,i)*q(i)))*T;end
end
