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
<button data-g="explorar">Explorar</button>
<button data-g="reto">Reto ~10 m</button>
<button data-g="parar">Parar</button>
</div>
<div class="row" style="margin-top:8px">
<button data-m="auto">Modo auto (so sensor)</button>
<button data-m="phone">Modo celular (visao)</button>
</div>
<button id="stop">PARAR TUDO</button>
<div class="row" style="margin-top:8px">
<button id="rec">Gravar log</button>
<button id="dl">Baixar log (0)</button>
</div></div>

<div class="card"><div class="lbl">Estado</div>
<div class="big" id="st">-</div>
<div id="why" style="color:var(--mu);min-height:1.2em"></div>
<div style="margin-top:8px"><span class="chip" id="c">frente: -</span><span class="chip" id="vis">visao: -</span><span class="chip" id="up">-</span></div></div>

<div class="card"><div class="lbl">Visao do robo (frame | retroprojecao | mascara)</div>
<img id="viz" alt="aguardando o celular..." style="width:100%;display:block;image-rendering:pixelated;background:#000;min-height:60px;color:var(--mu)"></div>

<div class="card"><div class="lbl">Sensor (radar)</div><canvas id="rad" width="400" height="220"></canvas></div>

<div class="card"><div class="lbl">Motores</div>
<div style="margin-bottom:6px">E <div class="bar"><i id="bl"></i></div></div>
<div>D <div class="bar"><i id="br"></i></div></div></div>

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
let lastUp=null,lastWhy='',vizUrl='',rec=false,LOG=[];
const LOG_MAX=60000;
function handle(m){
  if(m.t==='state'){
    const a=m.armed;$('armed').textContent=a?'ARMADO':'desarmado';$('armed').style.background=a?'var(--ok)':'#2d333b';
    $('st').textContent=m.mode==='auto'?(m.st+(m.sub?' / '+m.sub:'')):
      (m.esp?'ESP32 no comando: '+m.st+(m.sub?' / '+m.sub:''):'modo celular');
    if(m.up!=null){if(lastUp!=null&&m.up<lastUp)addLog('ESP32 REINICIOU (queda de energia?)','var(--er)');lastUp=m.up;
      $('up').textContent='ligado ha '+m.up+' s';}
    $('why').textContent=m.why||'';
    $('c').textContent='frente: '+(m.c<0?'livre (sem eco)':m.c.toFixed(0)+' cm');
    bar($('bl'),m.L);bar($('br'),m.R);
    document.querySelectorAll('[data-m]').forEach(b=>b.classList.toggle('on',b.dataset.m===m.mode));
    if(m.why&&m.why!==lastWhy)addLog('[esp32] '+m.why,'var(--wa)');lastWhy=m.why||'';
  }else if(m.t==='range'){
    pts[Math.round(m.deg/5)*5]={cm:m.cm,t:Date.now()};
  }else if(m.t==='think'){
    if(m.viz&&m.viz!==vizUrl){vizUrl=m.viz;$('viz').src=m.viz+'?'+Date.now();}
    if(m.state)$('vis').textContent='visao: '+m.state+(m.conf!=null?' ('+Math.round(m.conf*100)+'%)':'');
    if(m.msg)addLog('[cerebro] '+m.msg);
  }else if(m.t==='goal'){
    document.querySelectorAll('[data-g]').forEach(b=>b.classList.toggle('on',b.dataset.g===m.g));
    addLog('motivacao: '+m.g,'var(--ac)');
  }else if(m.t==='log'){addLog('[esp32] '+m.msg);
  }else if(m.t==='hello'){addLog('ESP32 iniciou (reset: '+m.reset+')',m.reset==='BROWNOUT'?'var(--er)':null);}
}
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
document.querySelectorAll('[data-g]').forEach(b=>b.onclick=()=>send({t:'goal',g:b.dataset.g}));
document.querySelectorAll('[data-m]').forEach(b=>b.onclick=()=>send({t:'mode',m:b.dataset.m}));
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