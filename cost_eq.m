clc

syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16

q = [q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16];

F = q * A(1:16,1:16) 
F1 = simplify(q * A(1:16,1:16) )