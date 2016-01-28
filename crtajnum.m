hold on
if isempty(boja) 
    boja = 'b';
end
brojac_duzina=0;
for i = 1:2:length(rezultat)
    brojac_duzina=brojac_duzina+1;
tmp = [rezultat(i,:);rezultat(i+1,:)];
h = plot(tmp(:,1),tmp(:,2),boja);
% if (brojac_duzina==19+1)
%     plot(tmp(:,1),tmp(:,2),'k');
% end
text([1 1]*tmp(:,1)*0.5,[1 1]*tmp(:,2)*0.5,mat2str(brojac_duzina))
% pause
set(h,'LineWidth',1)
end
% axis equal