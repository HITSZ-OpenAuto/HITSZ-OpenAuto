%%
clear, clc

n1 = [1, 1, 2];
d1 = [1, 2, 4];
sys1 = tf(n1, d1);
[y11, t11] = impulse(sys1);
[y12, t12] = step(sys1);
n2 = [1, -1, 1];
d2 = [1, -2, 1, 4];
sys2 = tf(n2, d2);
[y21, t21] = impulse(sys1, 5);
[y22, t22] = step(sys2, 4.8);

figure;
subplot(2, 2, 1); plot(t11, y11); title("冲激响应"); ylabel("Amplitude"); xlabel("Time(s)");
subplot(2, 2, 2); plot(t12, y12); title("阶跃响应"); ylabel("Amplitude"); xlabel("Time(s)");
subplot(2, 2, 3); plot(t21, y21); title("冲激响应"); ylabel("Amplitude"); xlabel("Time(s)");
subplot(2, 2, 4); plot(t22, y22); title("阶跃响应"); ylabel("Amplitude"); xlabel("Time(s)");

%%
tfn1 = [2, 1];
tfd1 = [1, -1, 2];
sys1 = tf(tfn1, tfd1);
t = 0:0.01:10;
x1 = cos(6.*t);
x2 = exp(-t).*sin(8.*t);
x = 2.*x1 + 3.*x2;
out11 = lsim(sys1, x1, t);
out12 = lsim(sys1, x2, t);
out13 = lsim(sys1, x, t);

figure;
subplot(2, 3, 1); plot(t, out11); title("对输入信号x_1的响应"); ylabel("Amplitude"); xlabel("Times(s)");
subplot(2, 3, 2); plot(t, out12); title("对输入信号x_2的响应"); ylabel("Amplitude"); xlabel("Times(s)");
subplot(2, 3, 3); plot(t, out13); title("对输入信号x的响应"); ylabel("Amplitude"); xlabel("Times(s)");

tfn2 = [1, 2, 1];
tfd2 = [1, 0, -3, 1];
sys2 = tf(tfn2, tfd2);
t = 0:0.01:2;
x1 = exp(-2.*t);
x2 = exp(-t).*cos(5.*t);
x = x1 - 2.*x2;
out21 = lsim(sys2, x1, t);
out22 = lsim(sys2, x2, t);
out23 = lsim(sys2, x, t);

subplot(2, 3, 4); plot(t, out21); title("对输入信号x_1的响应"); ylabel("Amplitude"); xlabel("Times(s)");
subplot(2, 3, 5); plot(t, out22); title("对输入信号x_2的响应"); ylabel("Amplitude"); xlabel("Times(s)");
subplot(2, 3, 6); plot(t, out23); title("对输入信号x的响应"); ylabel("Amplitude"); xlabel("Times(s)");

%%
omega = -3*pi:0.05:3*pi;
H1 = h1(omega);
H2 = h2(omega);
H3 = h3(omega);
H4 = h4(omega);
figure;
subplot(4, 2, 1); plot(omega, abs(H1)); title("H_1幅频特性"); xlabel("\omega");
subplot(4, 2, 2); plot(omega, angle(H1)); title("H_1相频特性"); xlabel("\omega");
subplot(4, 2, 3); plot(omega, abs(H2)); title("H_2幅频特性"); xlabel("\omega");
subplot(4, 2, 4); plot(omega, angle(H2)); title("H_2相频特性"); xlabel("\omega");
subplot(4, 2, 5); plot(omega, abs(H3)); title("H_3幅频特性"); xlabel("\omega");
subplot(4, 2, 6); plot(omega, angle(H3)); title("H_3相频特性"); xlabel("\omega");
subplot(4, 2, 7); plot(omega, abs(H4)); title("H_4幅频特性"); xlabel("\omega");
subplot(4, 2, 8); plot(omega, angle(H4)); title("H_4相频特性"); xlabel("\omega");

%%
n4 = [2, 5];
d4 = [1, 0.6, 0.72];
n = 0:40;
x3 = sin(n.*pi./10);
x4 = 2.^(-n);

figure;
subplot(2, 2, 1); stem(n, impz(n4, d4, n)); title("对x_1[n]的零状态响应"); xlabel("n"); ylabel("Amplitude");
subplot(2, 2, 2); stem(n, stepz(n4, d4, n)); title("对x_2[n]的零状态响应"); xlabel("n"); ylabel("Amplitude");
subplot(2, 2, 3); stem(n, filter(n4, d4, x3)); title("对x_3[n]的零状态响应"); xlabel("n"); ylabel("Amplitude");
subplot(2, 2, 4); stem(n, filter(n4, d4, x4)); title("对x_4[n]的零状态响应"); xlabel("n"); ylabel("Amplitude");
%%

n1 = [2];
d1 = [1, 0.8];
n2 = [0.5, 0];
d2 = [1, 0.8];
n3 = [1, 2];
d3 = [1, 0.4, 0.8];
n4 = [1, 1, 1, 2];
d4 = [1, 0.3, 0.5, 0.1, 0.8];
omega = -pi:0.01:pi;

[H1, w1] = freqz(n1, d1, omega);
[H2, w2] = freqz(n2, d2, omega);
[H3, w3] = freqz(n3, d3, omega);
[H4, w4] = freqz(n4, d4, omega);

figure;
subplot(4, 2, 1); plot(w1, abs(H1)); title("H_1(z)的幅频特性"); xlabel("\Omega");
subplot(4, 2, 2); plot(w1, angle(H1));  title("H_1(z)的相频特性"); xlabel("\Omega");
subplot(4, 2, 3); plot(w1, abs(H2));  title("H_2(z)的幅频特性"); xlabel("\Omega");
subplot(4, 2, 4); plot(w1, angle(H2));  title("H_2(z)的相频特性"); xlabel("\Omega");
subplot(4, 2, 5); plot(w1, abs(H3));  title("H_3(z)的幅频特性"); xlabel("\Omega");
subplot(4, 2, 6); plot(w1, angle(H3));  title("H_3(z)的相频特性"); xlabel("\Omega");
subplot(4, 2, 7); plot(w1, abs(H4));  title("H_4(z)的幅频特性"); xlabel("\Omega");
subplot(4, 2, 8); plot(w1, angle(H4));  title("H_4(z)的相频特性"); xlabel("\Omega");
