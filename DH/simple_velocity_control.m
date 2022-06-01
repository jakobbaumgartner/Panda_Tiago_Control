function status = simple_velocity_control(Xee)

% velocity control, for testing jacobis in simulation  

% Ce je casovno trajanje spremembe izredno kratko, potem lahko na
% situacijo gledamo kot na spremembo hitrosti v poljubnem sklepu ( dqj),
% ki se odraza v hitrosti vrha robota v obliki translacijske hitrosti dp ali
% rotacijske hitrosti ω izrazena glede na osi baznega koordinatnega sis-
% tema. Omenjeno relacijo med hitrostmi v sklepih dq in hitrostmi vrha
% robota dp (v) opisuje Jacobijeva matrika, ki jo oznacujemo s crko J in
% zapisemo z enacbo:
% v = dp = J(q) dq


end