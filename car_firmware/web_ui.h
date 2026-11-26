// web_ui.h - pagina do "pensamento" servida pelo ESP32 em http://192.168.4.1
#pragma once
#include <Arduino.h>

static const char WEB_UI[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="pt-br"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrinho</title>
<style>
:root{--bg:#101418;--card:#1a2027;--tx:#e6edf3;--mu:#8b98a5;--ok:#3fb950;--wa:#d29922;--er:#f85149;--ac:#58a6ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:15px system-ui,sans-serif;padding:12px}
h1{font-size:18px;margin:0 0 10px;display:flex;justify-content:space-between;align-items:center}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block;background:var(--er);margin-right:6px}
.card{background:var(--card);border-radius:12px;padding:12px;margin-bottom:10px}
.lbl{color:var(--mu);font-size:12px;text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px}
.row{display:flex;gap:8px;flex-wrap:wrap}
button{flex:1;min-width:90px;padding:12px;border:0;border-radius:10px;background:#2d333b;color:var(--tx);font-size:15px}
button.on{background:var(--ac);color:#000}
#stop{background:var(--er);color:#fff;font-weight:700;font-size:18px;width:100%;margin-top:8px}
.big{font-size:22px;font-weight:600}
.chip{display:inline-block;padding:2px 8px;border-radius:999px;background:#2d333b;margin-right:6px;font-size:13px}
canvas{width:100%;height:auto;display:block}
.bar{height:10px;background:#2d333b;border-radius:5px;position:relative;overflow:hidden}
.bar i{position:absolute;top:0;bottom:0;left:50%;background:var(--ac)}
#log{font:12px ui-monospace,monospace;max-height:220px;overflow:auto;white-space:pre-wrap}
#log div{border-bottom:1px solid #2d333b;padding:3px 0}
</style></head><body>
<h1><span><span class="dot" id="dot"></span>Carrinho</span><span class="chip" id="armed">desarmado</span></h1>

<div class="card"><div class="lbl">Motivacao</div>
<div class="row">
<button data-g="explorar" data-src="sensor">Explorar: sensor</button>
<button data-g="explorar" data-src="visao">Explorar: visao</button>
</div>
<button id="stop">PARAR TUDO</button>
<div class="row" style="margin-top:8px">
<button id="rec">Gravar log</button>
<button id="dl">Baixar log (0)</button>
</div></div>

<div class="card"><div class="lbl">Diagnostico (rodas no chao, espaco livre; so armado, hard stop vale)</div>
<div class="row"><button data-d="armar">Armar parado (4 s)</button><button data-d="zerar_trim">Zerar trim</button></div>
<div class="row" style="margin-top:8px"><button data-d="dir">Girar a direita 1 s</button><button data-d="esq">Girar a esquerda 1 s</button></div>
<div class="row" style="margin-top:8px"><button data-d="roda_esq">Roda esq. (degraus)</button><button data-d="roda_dir">Roda dir. (degraus)</button></div>
<div class="row" style="margin-top:8px"><button data-d="reto">Reto 2 s (sem correcao)</button><button data-d="reto_corr">Reto 2 s com correcao</button></div>
<div class="row" style="margin-top:8px"><button data-d="servo">Servo do sensor (-60..+60, sem armar)</button></div>
<div id="dres" style="margin-top:8px;white-space:pre-wrap;font-variant-numeric:tabular-nums"></div>
<div class="row" id="dask" style="margin-top:8px;display:none"><button data-v="1">o carro virou a DIREITA</button><button data-v="0">o carro virou a ESQUERDA</button></div>
<div class="row" id="dwh" style="margin-top:8px;display:none"><button data-w="1">girou a roda ESQUERDA</button><button data-w="0">girou a roda DIREITA</button></div>
<div id="dver" style="margin-top:6px;font-weight:600"></div>
<div id="dsug" style="margin-top:8px;white-space:pre-wrap"></div>
<div class="row" style="margin-top:8px"><button id="wapl" disabled>Aplicar faixa das rodas</button><button id="wdef">Rodas: padrao</button></div></div>

<div class="card"><div class="lbl">Estado</div>
<div class="big" id="st">-</div>
<div id="why" style="color:var(--mu);min-height:1.2em"></div>
<div style="margin-top:8px"><span class="chip" id="src">fonte: -</span><span class="chip" id="ang">angulo: -</span><span class="chip" id="c">frente: -</span><span class="chip" id="vis">visao: -</span><span class="chip" id="up">-</span></div></div>

<div class="card"><div class="lbl">Visao do robo (frame | retroprojecao | mascara)</div>
<img id="viz" alt="aguardando o celular..." style="width:100%;display:block;image-rendering:pixelated;background:#000;min-height:60px;color:var(--mu)"></div>

<div class="card"><div class="lbl">Sensor (radar)</div><canvas id="rad" width="400" height="220"></canvas></div>

<div class="card"><div class="lbl">Motores</div>
<div style="margin-bottom:6px">E <div class="bar"><i id="bl"></i></div></div>
<div>D <div class="bar"><i id="br"></i></div></div></div>

<div class="card"><div class="lbl">Logs do celular</div><a id="logs" style="color:var(--ac)" href="#">aguardando o celular...</a></div>

<div class="card"><div class="lbl">Pensamento</div><div id="log"></div></div>

<script>
const $=id=>document.getElementById(id);
let ws,pts={};
function send(o){if(ws&&ws.readyState===1)ws.send(JSON.stringify(o));}
function conn(){
  ws=new WebSocket('ws://'+(location.hostname||'192.168.4.1')+':81/');
  ws.onopen=()=>{$('dot').style.background='var(--ok)';addLog('conectado');};
  ws.onclose=()=>{$('dot').style.background='var(--er)';setTimeout(conn,1000);};
  ws.onmessage=e=>{let m;try{m=JSON.parse(e.data)}catch(_){return}
    if(rec){m.ts=Date.now();LOG.push(m);if(LOG.length>LOG_MAX)LOG.shift();}
    handle(m);};
}
function addLog(s,color){
  const d=document.createElement('div');
  d.textContent=new Date().toLocaleTimeString()+'  '+s;
  if(color)d.style.color=color;
  const L=$('log');L.prepend(d);while(L.childNodes.length>80)L.lastChild.remove();
}
function bar(el,v){v=Math.max(-1,Math.min(1,v));el.style.left=(v>=0?50:50+v*50)+'%';el.style.width=Math.abs(v)*50+'%';
  el.style.background=v<0?'var(--wa)':'var(--ac)';}
let lastUp=null,lastRst=null,lastWhy='',vizUrl='',rec=false,LOG=[];
const LOG_MAX=60000;
function handle(m){
  if(m.t==='state'){
    const a=m.armed;$('armed').textContent=a?'ARMADO':'desarmado';$('armed').style.background=a?'var(--ok)':'#2d333b';
    const fonte=m.src==='visao'?'VISAO':'SENSOR';
    $('st').textContent=fonte+' | '+(m.mode==='auto'?(m.st+(m.sub?' / '+m.sub:'')):
      (m.esp?'ESP32 no comando: '+m.st+(m.sub?' / '+m.sub:''):'celular dirige'));
    $('src').textContent='fonte: '+(m.src||'-')+(m.vis?' (visao ok)':'')+(m.yaw?' | giro ok':'')+
      ' | camera: '+(m.mov==null||m.mov<0?'sem movimento medido':'mov '+m.mov.toFixed(1))+(m.stl?' | parado empurrando '+m.stl+' ms':'')+(m.bat?' | batidas: '+m.bat:'');
    $('ang').textContent='angulo: '+(m.ang==null?'-':m.ang+'°');
    if(m.up!=null){if(lastUp!=null&&m.up<lastUp)addLog('ESP32 REINICIOU (queda de energia?)','var(--er)');lastUp=m.up;
      $('up').textContent='ligado ha '+m.up+' s'+(m.rst?' | ultimo reset: '+m.rst:'')+(m.rest?' | retomadas: '+m.rest:'');}
    if(m.rst&&m.rst!==lastRst){if(lastRst!==null||m.up<15)addLog('motivo do ultimo reset: '+m.rst,m.rst==='LIGOU'?null:'var(--er)');lastRst=m.rst;}
    $('why').textContent=m.why||'';
    $('c').textContent='frente: '+(m.c<0?'sem eco ('+(m.cls||'-')+')':m.c.toFixed(0)+' cm');
    bar($('bl'),m.L);bar($('br'),m.R);
    if(m.why&&m.why!==lastWhy)addLog('[esp32] '+m.why,'var(--wa)');lastWhy=m.why||'';
  }else if(m.t==='range'){
    pts[Math.round(m.deg/5)*5]={cm:m.cm,t:Date.now()};
  }else if(m.t==='scan'){
    const r=m.replay?'  (reenviado, boot '+m.boot+', ms '+m.ms+')':'';
    if(m.k==='aim')addLog('[servo] vai para '+m.deg+'°'+r,'var(--ac)');
    else if(m.k==='olha'){
      if(m.got&&!m.replay)pts[Math.round(m.deg/5)*5]={cm:m.cm,t:Date.now()};
      addLog('[servo] '+m.deg+'°: '+(!m.got?'SEM LEITURA (estourou o tempo)':(m.cm==null?'livre (sem eco)':m.cm.toFixed(0)+' cm'))+r,m.got?'var(--ac)':'var(--er)');
    }else if(m.k==='decide'){
      const nt=(m.d||[]).map((d,i)=>d+'°='+(m.sc[i]<0?'-':m.sc[i])).join('  ');
      addLog('[servo] notas '+nt+' -> '+(m.x?'sem saida (re maior)':'curva '+m.ang+'°')+r,'var(--wa)');}
  }else if(m.t==='think'){
    if(m.viz&&m.viz!==vizUrl){vizUrl=m.viz;$('viz').src=m.viz+'?'+Date.now();
      try{const a=$('logs');a.href=new URL(m.viz).origin+'/logs';a.textContent='baixar logs gravados no celular';a.target='_blank';}catch(_){}}
    if(m.state)$('vis').textContent='visao: '+m.state+(m.conf!=null?' ('+Math.round(m.conf*100)+'%)':'');
    if(m.msg)addLog('[cerebro] '+m.msg);
  }else if(m.t==='goal'){
    document.querySelectorAll('[data-g]').forEach(b=>b.classList.toggle('on',b.dataset.g===m.g&&b.dataset.src===(m.src||'sensor')));
    addLog('motivacao: '+m.g+(m.g==='explorar'?' ('+(m.src||'sensor')+')':''),'var(--ac)');
  }else if(m.t==='diag'){diag(m);
  }else if(m.t==='turn'){addLog('[curva] alvo '+m.alvo+'° girou '+(m.girou==null?'? (sem yaw)':m.girou+'° erro '+m.erro+'°'+
    (Math.abs(m.erro)>=5?(m.erro*m.alvo>0?' (PASSOU: suba yaw_lead_ms)':' (CURTA: baixe yaw_lead_ms)'):'')),'var(--ac)');
  }else if(m.t==='log'){addLog('[esp32] '+m.msg);
  }else if(m.t==='hello'){addLog('ESP32 iniciou (reset: '+m.reset+')',m.reset==='BROWNOUT'?'var(--er)':null);}
}
// ---- diagnostico: o ESP32 mede (L, R, giro do giroscopio); quem olha o carro diz para onde ele virou ----
let DG={},dLast=null,wLast=null;
const NM={dir:'Girar a direita',esq:'Girar a esquerda',reto:'Reto sem correcao',reto_corr:'Reto com correcao',roda_esq:'Roda esquerda',roda_dir:'Roda direita',servo:'Servo do sensor'};
function f1(x){return x==null?'-':(+x).toFixed(1);}
// sentido medido: pelo giro mediano (confiavel); o psi so se nao houver giro
function gsign(m){const g=m.rate_med!=null?m.rate_med:m.psi;return g==null?null:g>0;}
// degrau assentado: o giro mudou pouco dentro da janela de medida (dr = ultima - 1a amostra)
function settled(r,d){return d==null||Math.abs(d)<=Math.max(20,0.15*Math.abs(r));}
// roda que gira quase igual em qualquer duty (do mais alto ao mais baixo): o PWM nao chega nela. Atrito ou motor fraco
// fazem a roda PARAR nos duties baixos; isto e o contrario (ela segue no maximo).
function pwmIgnored(m){const v=[];(m.duty||[]).forEach((d,i)=>{const r=m.rate[i];if(r!=null)v.push([d,Math.abs(r)]);});
  if(v.length<4)return '';v.sort((a,b)=>a[0]-b[0]);const mx=Math.max(...v.map(x=>x[1])),lo=v[0],hi=v[v.length-1];
  if(mx>=60&&lo[1]>=0.7*mx&&hi[0]-lo[0]>=100)return 'ATENCAO: a roda gira quase igual em qualquer duty (de '+hi[0]+' a '+lo[0]+
    '): o PWM NAO controla esta roda. Atrito ou motor fraco fariam ela parar nos duties baixos. Confira o fio do EN e a placa. Sem faixa sugerida.';
  return '';}
function wheel(m){
  const du=m.duty||[],dr=m.dr||[];
  let s=NM[m.teste]+(m.abortado?'  ABORTADO: '+m.abortado:'')+'  (so ela, para frente; a outra parada; duty descendo)\nduty -> giro (°/s)';
  du.forEach((d,i)=>{const r=m.rate[i];s+='\n  '+d+' -> '+(r==null?'sem yaw':f1(r))+(r!=null&&!settled(r,dr[i])?'  (nao assentou: '+f1(dr[i])+')':'');});
  if('minimo' in m)s+='\nmenor duty que mantem a roda girando (|giro| >= 15°/s): '+(m.minimo==null?'NAO GIROU nem com duty '+du[0]:
    m.minimo+(m.minimo===Math.min(...du)?' (o menor testado: talvez gire mais baixo ainda)':''));
  else if('partida' in m)s+='\npartida (|giro| >= 15°/s): '+(m.partida==null?'NAO PARTIU':'duty '+m.partida)+'  (teste antigo, subindo)';
  const ig=pwmIgnored(m);if(ig)s+='\n'+ig;
  const rs=(m.rate||[]).filter(r=>r!=null&&Math.abs(r)>=15),want=m.teste==='roda_esq';
  if(rs.length){const pos=rs.filter(r=>r>0).length>rs.length/2;
    s+='\no giroscopio diz: carro girou para a '+(pos?'DIREITA':'ESQUERDA')+' (esperado: '+(want?'DIREITA':'ESQUERDA')+', girando sobre a roda parada)';}
  s+='\nQual roda girou?';wLast=m;$('dwh').style.display='flex';
  return s;
}
// faixa de duty por roda a partir dos testes por roda: reta giro = k*(duty - d0) nos degraus que andaram e assentaram,
// sem o TETO (degraus do alto em que o giro quase nao sobe mais: roda no maximo/patinando; em 21/11 eles achataram a
// reta da esquerda para 2,8 graus/s por ponto). Mesma velocidade nas duas rodas para o mesmo comando: lo/hi de cada roda
// dao as velocidades vLo/vHi da roda que parte mais tarde com lo = d0 + WM e hi = lo + WSPAN; nenhuma roda abaixo do
// seu minimo (menor duty que a manteve girando no teste).
const WM=5,WSPAN=25;let WSUG=null;
function lsW(q){let n=q.length,sx=0,sy=0,sxx=0,sxy=0;q.forEach(([x,y])=>{sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;});
  const k=(n*sxy-sx*sy)/(n*sxx-sx*sx);return {k:k,d0:(sx-sy/k)/n};}
function fitW(m){if(pwmIgnored(m))return null;const p=[];
  (m.duty||[]).forEach((d,i)=>{const r=m.rate[i];if(r!=null&&Math.abs(r)>=15&&settled(r,(m.dr||[])[i]))p.push([d,Math.abs(r)]);});
  p.sort((a,b)=>a[0]-b[0]);
  while(p.length>3){const f=lsW(p.slice(0,-1)),a=p[p.length-2],b=p[p.length-1];if((b[1]-a[1])/(b[0]-a[0])<0.5*f.k)p.pop();else break;}
  if(p.length<2)return null;const f=lsW(p);if(!(f.k>0))return null;
  f.min=m.minimo!=null?m.minimo:p[0][0];f.vmax=p[p.length-1][1];return f;}
function suggest(){const a=DG.roda_esq,b=DG.roda_dir;if(!a||!b)return;
  const L=fitW(a),R=fitW(b);WSUG=null;$('wapl').disabled=true;
  if(!L||!R){$('dsug').textContent='Faixa das rodas: sem faixa sugerida. Precisa de pelo menos 2 degraus assentados com giro em CADA roda, e as duas obedecendo o duty'+
    (!L?' (esquerda: '+(pwmIgnored(a)?'nao obedece o PWM':'nao andou')+')':'')+(!R?' (direita: '+(pwmIgnored(b)?'nao obedece o PWM':'nao andou')+')':'')+
    '. Roda que nao gira nem com duty alto = problema mecanico.';return;}
  const ref=L.d0>=R.d0?L:R,c=x=>Math.max(0,Math.min(255,Math.round(x)));
  let vLo=ref.k*WM;[L,R].forEach(w=>{vLo=Math.max(vLo,w.k*(w.min-w.d0));});
  const vHi=vLo+ref.k*WSPAN;
  const s={pwm_min_l:c(L.d0+vLo/L.k),pwm_max_l:c(L.d0+vHi/L.k),pwm_min_r:c(R.d0+vLo/R.k),pwm_max_r:c(R.d0+vHi/R.k)};
  let t='Faixa sugerida (mesma velocidade nas duas rodas):\n  esquerda '+s.pwm_min_l+'..'+s.pwm_max_l+'  (parte em ~'+L.d0.toFixed(0)+', '+L.k.toFixed(1)+' °/s por ponto)'+
    '\n  direita  '+s.pwm_min_r+'..'+s.pwm_max_r+'  (parte em ~'+R.d0.toFixed(0)+', '+R.k.toFixed(1)+' °/s por ponto)';
  if(Math.max(s.pwm_max_l,s.pwm_max_r)>180)t+='\n  atencao: faixa alta; confira hold_pwm_max (200) para sobrar correcao';
  if(vHi>0.9*Math.min(L.vmax,R.vmax))t+='\n  atencao: o topo da faixa passa da maior velocidade medida em uma das rodas (fora da reta medida)';
  if(Math.abs(L.k-R.k)/Math.max(L.k,R.k)>0.4)t+='\n  as rodas rendem muito diferente (>40%): caixa/roda com atrito?';
  WSUG=s;$('wapl').disabled=false;$('dsug').textContent=t;}
$('wapl').onclick=()=>{if(!WSUG)return;send(Object.assign({t:'cfg'},WSUG));addLog('[diag] faixa das rodas enviada: '+JSON.stringify(WSUG),'var(--ac)');};
$('wdef').onclick=()=>{send({t:'cfg',pwm_min_l:-1,pwm_max_l:-1,pwm_min_r:-1,pwm_max_r:-1});addLog('[diag] rodas: faixa padrao (pwm_min/pwm_max)','var(--ac)');};
// teste do servo: com algo perto de UM lado, distancias iguais em todos os angulos = o sensor nao girou
function servoRes(m){
  const v=m.cm.filter(x=>x!=null&&x>=0),same=v.length>=4&&Math.max(...v)-Math.min(...v)<3;
  let s='Servo do sensor'+(m.abortado?'  ABORTADO: '+m.abortado:'')+'\n'+m.deg.map((d,i)=>d+'°: '+(m.cm[i]==null?'sem eco':(m.cm[i]<0?'-':m.cm[i].toFixed(0)+' cm'))).join('   ')+'\nPWM no pino do servo: '+m.hz+' Hz',bad=false;
  if(m.hz===0){s+='\nERRO: pino do servo SEM PWM (firmware). Grave o log e mande.';bad=true;}
  else if(same){s+='\nDistancias IGUAIS em todos os angulos: o sensor NAO girou. Confira o fio de sinal do servo (GPIO4 desde a 0.6.2), o 5 V e o GND do servo.';bad=true;}
  else if(v.length<4)s+='\nPoucas leituras com eco: ponha a mao ~30 cm a esquerda do carro e repita.';
  else s+='\nDistancias diferentes por angulo: o servo gira.';
  $('dres').textContent=s;addLog('[diag] '+s.replace(/\n/g,' | '),bad?'var(--er)':'var(--ok)');
}
function diag(m){
  if(m.erro){$('dres').textContent=(NM[m.teste]||m.teste)+': '+m.erro;return;}
  if(m.teste==='zerar_trim'){addLog('[diag] trim zerado','var(--ac)');return;}
  if(m.teste==='servo'){servoRes(m);return;}
  DG[m.teste]=m;
  $('dver').textContent='';$('dask').style.display='none';$('dwh').style.display='none';
  if(m.teste==='roda_esq'||m.teste==='roda_dir'){const s=wheel(m);$('dres').textContent=s;addLog('[diag] '+s.replace(/\n/g,' | '),'var(--ac)');suggest();return;}
  let s=(NM[m.teste]||m.teste)+(m.abortado?'  ABORTADO: '+m.abortado:'')+'\nL='+f1(m.L)+' R='+f1(m.R)+'  psi='+f1(m.psi)+'°  giro mediano='+f1(m.rate_med)+'°/s  trim='+f1(m.trim);
  if(m.psi==null)s+='\nSEM yaw do celular: o app esta aberto e conectado? (sem yaw nao da para conferir o giroscopio)';
  if(m.teste==='dir'||m.teste==='esq'){
    if(m.rate_med!=null&&Math.abs(m.rate_med)<15)s+='\nGirou menos de 15°/s: roda presa ou abaixo da zona morta (suba pwm_min)?';
    const g=gsign(m);
    s+='\nO giroscopio diz: '+(g==null?'?':(g?'DIREITA':'ESQUERDA'))+'. Para onde o carro virou de verdade?';
    dLast=m;$('dask').style.display='flex';
  }else{
    const r=m.rate_med;
    if(r!=null)s+='\n'+(Math.abs(r)<10?'anda reto (|giro| < 10°/s)':'puxa para a '+(r>0?'DIREITA (roda esquerda mais forte)':'ESQUERDA (roda direita mais forte)')+
      ' - vale se o giroscopio passou no teste de giro');
    const a=DG.reto,b=DG.reto_corr;
    if(a&&b&&a.rate_med!=null&&b.rate_med!=null)s+='\nsem correcao '+f1(a.rate_med)+'°/s -> com correcao '+f1(b.rate_med)+'°/s'+
      (Math.abs(b.rate_med)>=Math.abs(a.rate_med)?'  (a correcao NAO ajudou: confira os sinais)':'');
  }
  $('dres').textContent=s;addLog('[diag] '+s.replace(/\n/g,' | '),'var(--ac)');
}
document.querySelectorAll('[data-w]').forEach(b=>b.onclick=()=>{const m=wLast;$('dwh').style.display='none';if(!m)return;
  const want=m.teste==='roda_esq',real=b.dataset.w==='1',ok=want===real;
  const t=ok?'OK: o firmware acionou a roda '+(want?'ESQUERDA':'DIREITA')+' (fiacao certa).':
    'FIACAO TROCADA: pediu a roda '+(want?'esquerda':'direita')+' e girou a '+(real?'esquerda':'direita')+'. Troque os pinos ENA<->ENB no firmware (nao os fios de cada motor).';
  $('dver').textContent=t;$('dver').style.color=ok?'var(--ok)':'var(--er)';addLog('[diag] '+t,ok?'var(--ok)':'var(--er)');});
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>{const m=dLast,gyro=m?gsign(m):null;if(gyro==null){$('dask').style.display='none';return;}
  const want=m.teste==='dir',real=b.dataset.v==='1',cOk=real===want,gOk=gyro===real;
  const t=cOk&&gOk?'OK: curva e giroscopio com o sinal certo.':
    (!cOk&&gOk?'Problema na CURVA (firmware/fiacao): pediu '+(want?'direita':'esquerda')+' e virou '+(real?'direita':'esquerda')+'. O giroscopio esta certo.':
    (cOk&&!gOk?'Problema no GIROSCOPIO: virou '+(real?'a direita':'a esquerda')+' e o giro deu '+(gyro?'positivo':'negativo')+'. Inverter o sinal em YawSensor.kt (rightDps).':
    'CURVA e GIROSCOPIO invertidos: corrigir os dois (fiacao/firmware e YawSensor.kt).'));
  $('dver').textContent=t;$('dver').style.color=cOk&&gOk?'var(--ok)':'var(--er)';$('dask').style.display='none';addLog('[diag] '+t,cOk&&gOk?'var(--ok)':'var(--er)');});
document.querySelectorAll('[data-d]').forEach(b=>b.onclick=()=>{$('dres').textContent=(NM[b.dataset.d]||b.dataset.d)+': pedido...';send({t:'diag',teste:b.dataset.d});});
function draw(){
  const c=$('rad'),g=c.getContext('2d'),W=c.width,H=c.height,cx=W/2,cy=H-10,R=H-20,MAX=200,now=Date.now();
  g.clearRect(0,0,W,H);g.strokeStyle='#2d333b';g.fillStyle='#8b98a5';g.font='11px sans-serif';
  [50,100,150,200].forEach(d=>{g.beginPath();g.arc(cx,cy,R*d/MAX,Math.PI,2*Math.PI);g.stroke();g.fillText(d+'',cx+3,cy-R*d/MAX+12);});
  for(const k in pts){const p=pts[k],age=now-p.t;if(age>3000)continue;
    const a=(+k)*Math.PI/180,free=p.cm==null||p.cm<0,d=free?MAX:Math.min(p.cm,MAX),r=R*d/MAX;
    g.globalAlpha=1-age/3000;g.strokeStyle=free?'#3fb950':(p.cm<50?'#f85149':'#d29922');g.lineWidth=3;
    g.beginPath();g.moveTo(cx,cy);g.lineTo(cx+r*Math.sin(a),cy-r*Math.cos(a));g.stroke();
    if(!free){g.fillStyle=g.strokeStyle;g.beginPath();g.arc(cx+r*Math.sin(a),cy-r*Math.cos(a),4,0,7);g.fill();}}
  g.globalAlpha=1;g.lineWidth=1;requestAnimationFrame(draw);
}
document.querySelectorAll('[data-g]').forEach(b=>b.onclick=()=>{
  if(b.dataset.src==='visao')send({t:'calibrar'});   // reaprende a cor do chao (carro parado) antes de andar
  send({t:'goal',g:b.dataset.g,src:b.dataset.src});
});
$('stop').onclick=()=>{send({t:'arm',on:false});send({t:'goal',g:'parar'});};
$('viz').onerror=()=>{if(vizUrl)setTimeout(()=>{$('viz').src=vizUrl+'?'+Date.now();},2000);};
$('rec').onclick=()=>{rec=!rec;$('rec').classList.toggle('on',rec);$('rec').textContent=rec?'Gravando...':'Gravar log';};
$('dl').onclick=()=>{if(!LOG.length)return;
  const b=new Blob([LOG.map(o=>JSON.stringify(o)).join('\n')+'\n'],{type:'application/x-ndjson'});
  const a=document.createElement('a');a.href=URL.createObjectURL(b);
  a.download='carrinho_'+new Date().toISOString().replace(/[:.]/g,'-')+'.jsonl';a.click();};
setInterval(()=>{$('dl').textContent='Baixar log ('+LOG.length+')';},1000);
conn();draw();
</script></body></html>)rawliteral";