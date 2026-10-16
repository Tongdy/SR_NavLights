// SR Nav Lights - settings web page (kept in a .h file: Arduino does not
// pre-scan .h files, so the page's JavaScript cannot confuse the build)
#pragma once
#include <Arduino.h>

static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>SR Nav Lights</title>
<style>
body{background:#000;color:#fff;font-family:system-ui,Arial,sans-serif;margin:0 auto;max-width:720px;padding:10px 10px 90px;font-size:16px}
h1{color:#ffd000;font-size:22px;margin:4px 0 8px}
h2{color:#ffd000;font-size:18px;border-bottom:2px solid #ffd000;margin:22px 0 8px;padding-bottom:2px}
.live{border:2px solid #fff;border-radius:6px;padding:8px;font-weight:bold;line-height:1.6}
table{border-collapse:collapse;width:100%}td{padding:4px;border-bottom:1px solid #444;vertical-align:middle}
input,select,button{font-size:16px;background:#000;color:#fff;border:1px solid #aaa;border-radius:4px;padding:5px}
input[type=number]{width:74px}input[type=checkbox]{width:22px;height:22px}
button{color:#ffd000;border:2px solid #ffd000;font-weight:bold;padding:6px 12px}
button.on{background:#ffd000;color:#000}
button:focus,input:focus,select:focus{outline:3px solid #fff}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}
.grid button{padding:10px 6px;text-align:left}
.seq{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:6px}
.seq label{display:flex;gap:6px;align-items:center}
details{border:1px solid #666;border-radius:6px;margin:6px 0;padding:6px}summary{font-weight:bold;cursor:pointer}
.bar{position:fixed;left:0;right:0;bottom:0;background:#000;border-top:2px solid #ffd000;padding:10px;display:flex;gap:12px;align-items:center;justify-content:center}
.ok{color:#3f3}.bad{color:#f55}.note{color:#ccc;font-size:14px}
.warn{border:3px solid #f55;border-radius:6px;padding:10px;margin-top:10px;line-height:1.6}
.warn textarea{width:100%;font:15px ui-monospace,Menlo,Consolas,monospace;background:#000;color:#fff;border:1px solid #aaa;border-radius:4px;padding:6px;margin:6px 0}
.yel{color:#ffd000}
.idbox{font:bold 18px ui-monospace,Menlo,Consolas,monospace;width:11em;color:#ffd000}
</style></head><body>
<h1>SR Nav Lights <span id="ver"></span></h1>
<div class="live" id="live">Connecting...</div>
<div id="auth"></div>
<h2>1. Modes (6-position switch)</h2>
<table id="modes"></table>
<table id="tog"></table>
<h2>2. Steps</h2>
<p><b>Nav sequence</b> <span class="note">(each short press goes to the next slot; at "end" it goes back to slot 1)</span></p>
<div class="seq" id="seq"></div>
<p><b>Nav steps</b> <span class="note">(what each step does)</span></p>
<div id="nav"></div>
<p><b>Lights</b> <span class="note">(1 step = on/off, 5 steps = 20% to 100% then off)</span></p>
<table id="grp"></table>
<h2>3. Timing (ms)</h2>
<table id="tim"></table>
<p class="note">Alt double A / B use the double flash timing; B flashes half a repeat period after A.</p>
<p><b>Offsets (ms)</b> <span class="note">shift single/double/triple flash and beacon so lights don't flash together</span></p>
<table id="off"></table>
<h2>4. Pins</h2>
<p class="note">Choose the GPIO each light and switch is wired to. Saving a pin change restarts the board.
Dimmable = brightness / fade (max 6). Not dimmable = on/off only.</p>
<div id="pinwarn"></div>
<table id="pins"></table>
<div id="pinnote" class="note"></div>
<h2>5. Other</h2>
<table id="outs"></table><br>
<table id="oth"></table>
<p><button onclick="bk()">Backup settings</button> <button onclick="$('rf').click()">Restore settings</button>
<input type="file" id="rf" accept=".json" style="display:none" onchange="rs(this)"></p>
<p><button onclick="rb()">Restart board</button> <button onclick="fr()">Factory reset</button></p>
<h2>GPIO test</h2>
<p class="note">Press = LED ON, press again = LED OFF (Invert is followed, the pin level is shown). Normal lights stop while testing. Ends by itself after 2 minutes or when the phone disconnects.</p>
<div class="grid" id="gt"></div>
<p><button onclick="gEnd()">End test</button></p>
<h2>Firmware update</h2>
<p>Choose the <b>_WiFi.bin</b> file <input type="file" id="fw" accept=".bin"><br><br>
<button onclick="up()">Upload firmware</button> <span id="upm"></span></p>
<div class="bar"><button onclick="save()">Save settings</button><span id="msg"></span></div>
<script>
const PN=["Off","Steady","Single flash","Double flash","Triple flash","Beacon fade","Fast flash","Alternate A","Alternate B","Alt double A","Alt double B"];
const FN=["None","Nav lights","Instrument","Cockpit","Front light","Master off","WiFi (hold)"];
const TT=["Momentary, pressed = HIGH","Momentary, pressed = LOW","2-position (every flip)"];
const ON=["Left red","Right green","Top beacon","Bottom strobe","Landing","Tail strobe","Instrument","Cockpit","Front light"];
const PINS=[0,1,2,3,4,5,6,7,8,9,10,20,21];
const PWARN={2:'boot pin - check the board still starts',8:'boot pin + onboard status LED (status blink stops)',9:'BOOT button pin - must not be LOW at power-up',
 20:'may flicker at power-up (boot text)',21:'may flicker at power-up (boot text)'};
let GP=[5,6,7,10,20,21,2,4,3];
function psel(id,v,none){let s='<select id="'+id+'" onchange="pchk()">'+(none?'<option value="255"'+(v==255?' selected':'')+'>None</option>':'');
 PINS.forEach(p=>s+='<option value="'+p+'"'+(p==v?' selected':'')+'>GPIO'+p+'</option>');return s+'</select>';}
function pchk(){if(!$('ip0'))return true;const used={},err=[],warn=[];let pwm=0;
 const add=(name,p)=>{if(p==255)return;if(used[p]!==undefined)err.push('GPIO'+p+' used twice: '+used[p]+' and '+name);else used[p]=name;
  if(PWARN[p])warn.push(name+' on GPIO'+p+': '+PWARN[p]);};
 add('6-pos switch',+$('ip0').value);add('Toggle',+$('ip1').value);
 for(let o=0;o<9;o++){add(ON[o],+$('op'+o).value);if(+$('op'+o).value!=255&&$('pw'+o).checked)pwm++;}
 if(pwm>6)err.push('Too many dimmable lights: '+pwm+' ticked, max 6');
 $('pinwarn').innerHTML=err.map(e=>'<div class="bad">✖ '+e+'</div>').join('')+warn.map(w=>'<div class="yel">⚠ '+w+'</div>').join('');
 return err.length==0;}
let C=null,L={};const $=i=>document.getElementById(i);
const esc=s=>String(s).replace(/[&<>"]/g,'');
function sel(id,o,v){let s='<select id="'+id+'">';o.forEach((t,i)=>s+='<option value="'+i+'"'+(i==v?' selected':'')+'>'+t+'</option>');return s+'</select>';}
function num(id,v,a,b){return '<input type="number" id="'+id+'" value="'+v+'" min="'+a+'" max="'+b+'">';}
function chk(id,v){return '<input type="checkbox" id="'+id+'"'+(v?' checked':'')+'>';}
function row(a,b){return '<tr><td>'+a+'</td><td>'+b+'</td></tr>';}
function build(){
 $('ver').textContent='v'+C.ver;
 let h='<tr><td><b>Pos</b></td><td><b>Value µs</b></td><td></td><td><b>Function</b></td></tr>';
 for(let i=0;i<6;i++)h+='<tr><td>'+(i+1)+'</td><td>'+num('pu'+i,C.pu[i],800,2200)+'</td><td><button onclick="learn('+i+')">Learn</button></td><td>'+sel('pf'+i,FN,C.pf[i])+'</td></tr>';
 $('modes').innerHTML=h;
 $('tog').innerHTML=row('Toggle switch type',sel('tt',TT,C.tt))+row('Toggle HIGH above (µs)',num('th',C.th,1000,2000))+row('Toggle LOW below (µs)',num('tl',C.tl,1000,2000))
 +row('New 6-pos position counts after (ms)',num('ps',C.ps,20,2000))
 +row('Nav position: hold toggle = nav LEDs off (s)',num('nh',C.nh,1,10))
 +row('WiFi position: hold toggle = WiFi on/off (s)',num('wh',C.wh,2,30));
 const SN=C.nav.map((s,i)=>(i+1)+' '+esc(s.n));
 h='';for(let k=0;k<12;k++){h+='<label>'+(k+1)+'. <select id="q'+k+'"><option value="255"'+(C.seq[k]>11?' selected':'')+'>- end -</option>';
  SN.forEach((n,i)=>h+='<option value="'+i+'"'+(C.seq[k]==i?' selected':'')+'>'+n+'</option>');h+='</select></label>';}
 $('seq').innerHTML=h;
 h='';C.nav.forEach((s,i)=>{h+='<details><summary>Step '+(i+1)+': '+esc(s.n)+'</summary><p>Name <input id="n'+i+'n" maxlength="12" size="12" value="'+esc(s.n)+'"></p><table>';
  for(let o=0;o<6;o++)h+='<tr><td>'+ON[o]+'</td><td>'+sel('n'+i+'p'+o,PN,s.p[o])+'</td><td>'+num('n'+i+'b'+o,s.b[o],0,100)+' %</td></tr>';
  h+='</table></details>';});$('nav').innerHTML=h;
 h='';for(let g=0;g<3;g++)h+=row(ON[6+g],num('g'+g,C.g[g],1,10)+' steps');$('grp').innerHTML=h;
 h='<tr><td></td><td><b>On</b></td><td><b>Gap</b></td><td><b>Repeat</b></td></tr>';
 ['Single flash','Double flash','Triple flash'].forEach((n,f)=>{h+='<tr><td>'+n+'</td><td>'+num('f'+f+'o',C.fl[f][0],10,5000)+'</td><td>'+(f?num('f'+f+'g',C.fl[f][1],10,5000):'')+'</td><td>'+num('f'+f+'p',C.fl[f][2],50,10000)+'</td></tr>';});
 h+='<tr><td>Beacon fade</td><td></td><td></td><td>'+num('bp',C.bp,50,10000)+'</td></tr>';
 h+='<tr><td>Fast flash</td><td></td><td></td><td>'+num('fp',C.fp,20,10000)+'</td></tr>';
 h+='<tr><td>Wig-wag (Alternate)</td><td></td><td></td><td>'+num('ap',C.ap,20,10000)+'</td></tr>';
 $('tim').innerHTML=h;
 h='';for(let o=0;o<6;o++)h+=row(ON[o],num('of'+o,C.of[o],0,10000));$('off').innerHTML=h;
 GP=C.rp||GP;
 h='<tr><td><b>Light / switch</b></td><td><b>GPIO</b></td><td><b>Dimmable</b></td></tr>';
 h+='<tr><td>6-pos switch input</td><td>'+psel('ip0',C.ip[0],0)+'</td><td></td></tr>';
 h+='<tr><td>Toggle input</td><td>'+psel('ip1',C.ip[1],0)+'</td><td></td></tr>';
 for(let o=0;o<9;o++)h+='<tr><td>'+ON[o]+'</td><td>'+psel('op'+o,C.op[o],1)+'</td><td>'+chk('pw'+o,C.pw[o]).replace('<input','<input onchange="pchk()"')+'</td></tr>';
 $('pins').innerHTML=h;
 $('pinnote').innerHTML=C.pd?'<span class="bad">The saved pin map was not valid - the board is using the default pins now.</span>':'';
 pchk();
 h='<tr><td><b>Output</b></td><td><b>Max %</b></td><td><b>Invert</b></td></tr>';
 for(let o=0;o<9;o++)h+='<tr><td>'+ON[o]+' ('+(GP[o]==255?'not used':'GPIO'+GP[o])+')</td><td>'+num('mx'+o,C.mx[o],0,100)+'</td><td>'+chk('iv'+o,C.iv[o])+'</td></tr>';
 $('outs').innerHTML=h;
 $('oth').innerHTML=row('Remember lights after power-off',chk('rm',C.rm))
 +row('WiFi on after power-up if no switch moved (s, 0 = never)',num('wb',C.wb,0,600))
 +row('WiFi off when no phone connected (min, 0 = never)',num('wi',C.wi,0,60))
 +row('WiFi low power (SuperMini fix)',chk('lp',C.lp))
 +row('WiFi name','<input id="ss" maxlength="23" value="'+esc(C.ss)+'">')
 +row('New WiFi password (8+ characters, empty = keep)','<input id="pw" maxlength="23">')
 +row('PWM frequency Hz (used after restart)',num('hz',C.hz,100,20000));
 h='';for(let o=0;o<9;o++)h+='<button id="gb'+o+'" onclick="gT('+o+')"></button>';$('gt').innerHTML=h;gLab(0,0);
}
function msg(t,ok){$('msg').textContent=t;$('msg').className=ok?'ok':'bad';}
function post(u,b){return fetch(u,{method:'POST',body:b}).then(r=>r.text().then(t=>({ok:r.ok,t})));}
function learn(i){if(L.s0){$('pu'+i).value=L.w0;msg('Position '+(i+1)+' = '+L.w0+' µs, press Save',1);}else msg('No signal on the 6-pos input',0);}
function save(){if(!pchk()){msg('Not saved: fix the pins first',0);return;}const p=new URLSearchParams();document.querySelectorAll('input,select').forEach(e=>{if(!e.id||e.type=='file')return;p.append(e.id,e.type=='checkbox'?(e.checked?1:0):e.value);});
 post('/save',p).then(r=>{msg(r.t,r.ok);if(r.ok)setTimeout(load,/restarting/.test(r.t)?6000:0);}).catch(()=>msg('Not saved: no connection',0));}
function gT(o){post('/gpio',new URLSearchParams({o:o})).then(r=>{if(!r.ok)msg(r.t,0);}).catch(()=>msg('No connection',0));}
function gEnd(){post('/gpio',new URLSearchParams({end:1})).then(()=>msg('Test ended',1));}
function bk(){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(C,null,1)],{type:'application/json'}));a.download='SR_NavLights_settings.json';a.click();msg('Backup saved',1);}
function rs(inp){const f=inp.files[0];if(!f)return;f.text().then(t=>{const j=JSON.parse(t);delete j.ver;C=Object.assign({},C,j);build();save();}).catch(()=>msg('Not a settings file',0));inp.value='';}
function rb(){if(confirm('Restart the board?'))post('/reboot',new URLSearchParams()).then(()=>msg('Restarting',1));}
function fr(){if(confirm('Reset ALL settings to factory?'))post('/reset',new URLSearchParams()).then(()=>msg('Reset done, restarting',1));}
function up(){const f=$('fw').files[0];if(!f){$('upm').textContent='Choose a file first';return;}
 if(!/_(wifi|ota)\.bin$/i.test(f.name)&&!confirm('The file name does not end with _WiFi.bin. Upload anyway?'))return;
 const d=new FormData();d.append('fw',f,f.name);$('upm').textContent='Uploading, do not power off...';
 fetch('/update',{method:'POST',body:d}).then(r=>r.text()).then(t=>$('upm').textContent=t).catch(()=>$('upm').textContent='Upload failed');}
function lv(i){return L.lv[i]?L.lv[i]+'/'+L.g[i]:'OFF';}
function gLab(tm,tb){if(!C)return;for(let o=0;o<9;o++){const b=$('gb'+o);if(!b)continue;const on=!!(tm&&(tb>>o&1)),hi=on!==!!C.iv[o];
 if(GP[o]==255){b.className='';b.disabled=true;b.innerHTML=ON[o]+'<br>not used';continue;}b.disabled=false;
 b.className=on?'on':'';b.innerHTML='GPIO'+GP[o]+' '+ON[o]+'<br>LED '+(on?'ON':'OFF')+' · pin '+(hi?'HIGH':'LOW');}}
let AK='';
function authBox(j){const k=j.au+'|'+j.ao+'|'+j.kb+'|'+j.id;if(k===AK)return;AK=k;
 if(j.au){$('auth').innerHTML='<p class="note">'+(j.ao?'Activation not set up in this firmware':'Activated')+' · Board ID '+j.id+'</p>';return;}
 $('auth').innerHTML='<div class="warn"><b class="bad">NOT ACTIVATED — all lights are off</b>'+(j.kb?'<br><span class="bad">Public key in this firmware is not valid</span>':'')
 +'<br>Board ID: <input class="idbox" readonly value="'+j.id+'" onclick="this.select()"> <span class="note">send this to SrRcTech</span>'
 +'<br><textarea id="ac" rows="3" placeholder="Paste the activation code here"></textarea><button onclick="act()">Activate</button></div>';}
function act(){const c=$('ac').value;post('/activate',new URLSearchParams({code:c})).then(r=>{msg(r.t,r.ok);if(r.ok){AK='';}}).catch(()=>msg('No connection',0));}
function live(){fetch('/live').then(r=>r.json()).then(j=>{L=j;
 const t=j.tog<0?'-':(j.pr?'PRESSED':'released')+(j.tog?' HIGH':' LOW');
 const nav=j.sn==0?'no steps in sequence':(j.no?'<span class="bad">OFF</span> (slot '+(j.sl+1)+')':'Slot '+(j.sl+1)+'/'+j.sn+': Step '+(j.si+1)+' '+esc(j.nn));
 $('live').innerHTML='Position: <span class="'+(j.s0?'ok':'bad')+'">'+(j.s0?(j.pos||'-')+' ('+j.w0+' µs)':'NO SIGNAL')+'</span> &nbsp; Toggle: <span class="'+(j.s1?'ok':'bad')+'">'+(j.s1?t+' ('+j.w1+' µs)':'NO SIGNAL')+'</span><br>'
 +(j.tm?'<span class="bad">GPIO TEST ON</span>':'Nav: '+nav+'<br>Instrument: '+lv(0)+' &nbsp; Cockpit: '+lv(1)+' &nbsp; Front: '+lv(2)+(j.m?' &nbsp; <span class="bad">MASTER OFF</span>':''));
 gLab(j.tm,j.tb);authBox(j);
}).catch(()=>{$('live').textContent='No connection';}).finally(()=>setTimeout(live,500));}
function load(){fetch('/cfg').then(r=>r.json()).then(j=>{C=j;build();}).catch(()=>setTimeout(load,2000));}
load();live();
</script></body></html>)HTML";
