close all;
N1=Nc-floor(200e-3/Tc);
N2=Nc;
segnale = vi_LFu_ac(N1:N2,2);
time = t(N1:N2);

figure; 
plot(time,segnale);
title('LFu AC current analysis')
ylabel('A');
xlabel('sec');
set(gca,'xlim',[time(1) time(end)]);
grid on


sig_fft=segnale;
Nfft=length(sig_fft)
u1=Nfft*Tc
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
bar(freq,abs(Xrange));
xlim([1 100]);
grid;
xlabel('Hz');
ylabel('V');
title('Current LFu AC spectrum')
subplot 212
bar(freq,abs(Xrange));
xlim([9500 10500]);
grid;
xlabel('Hz');
ylabel('V');
title('Current LFu AC spectrum')
print('spectrum_LFu_ac_1','-depsc');

figure; 
subplot 211
bar(freq,abs(Xrange));
xlim([19500 20500]);
grid;
xlabel('Hz');
ylabel('V');
title('Current LFu AC spectrum')
subplot 212
bar(freq,abs(Xrange));
xlim([29500 30500]);
grid;
xlabel('Hz');
ylabel('V');
title('Current LFu AC spectrum')
print('spectrum_LFu_ac_2','-depsc');

