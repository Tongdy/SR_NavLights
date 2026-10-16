// SR Nav Lights - settings web page (kept in a .h file: Arduino does not
// pre-scan .h files, so the page's JavaScript cannot confuse the build)
#pragma once
#include <Arduino.h>

static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>SR Nav Lights</title>
<style>
body{background:#000;color:#fff;font-family:system-ui,Arial,sans-serif;margin:0 auto;max-width:760px;padding:10px 10px 90px;font-size:16px}
h1{color:#ffd000;font-size:22px;margin:4px 0 8px}
h2{color:#ffd000;font-size:18px;border-bottom:2px solid #ffd000;margin:22px 0 8px;padding-bottom:2px}
h3{color:#ffd000;font-size:16px;margin:12px 0 4px}
.live{border:2px solid #fff;border-radius:6px;padding:8px;font-weight:bold;line-height:1.6}
table{border-collapse:collapse;width:100%}td{padding:4px;border-bottom:1px solid #444;vertical-align:middle}
input,select,button{font-size:16px;background:#000;color:#fff;border:1px solid #aaa;border-radius:4px;padding:5px}
input[type=number]{width:64px}input[type=checkbox]{width:22px;height:22px}
input.nm{width:10em;max-width:46vw}
.led{border:1px solid #666;border-radius:6px;padding:6px;margin:6px 0}.led div{margin:3px 0;display:flex;flex-wrap:wrap;gap:6px 14px;align-items:center}
label{white-space:nowrap}
button{color:#ffd000;border:2px solid #ffd000;font-weight:bold;padding:6px 12px}
button.on{background:#ffd000;color:#000}
button.sm{padding:3px 8px;font-size:14px}
button:focus,input:focus,select:focus{outline:3px solid #fff}
option.used{color:#888}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:8px}
.grid button{padding:10px 6px;text-align:left}
details{border:1px solid #666;border-radius:6px;margin:6px 0;padding:6px}summary{font-weight:bold;cursor:pointer}
details.fn{border:2px solid #ffd000}details.fn>summary{color:#ffd000;font-size:17px}
.bar{position:fixed;left:0;right:0;bottom:0;background:#000;border-top:2px solid #ffd000;padding:10px;display:flex;gap:12px;align-items:center;justify-content:center}
.ok{color:#3f3}.bad{color:#f55}.yel{color:#ffd000}.note{color:#ccc;font-size:14px}
.scroll{overflow-x:auto}
.warn{border:3px solid #f55;border-radius:6px;padding:10px;margin-top:10px;line-height:1.6}
.warn textarea{width:100%;font:15px ui-monospace,Menlo,Consolas,monospace;background:#000;color:#fff;border:1px solid #aaa;border-radius:4px;padding:6px;margin:6px 0}
.idbox{font:bold 18px ui-monospace,Menlo,Consolas,monospace;width:11em;color:#ffd000}
</style></head><body>
<h1>SR Nav Lights <span id="ver"></span></h1>
<div class="live" id="live">Connecting...</div>
<div id="auth"></div>
<h2>1. Switches</h2>
<div class="scroll"><table id="modes"></table></div>
<table id="tog"></table>
<p id="pinnote" class="note"></p>
<h2>2. Functions</h2>
<p class="note">Each function has its own LEDs, steps and timing. In its switch position: short press = next step,
hold = function off, press again = back on. Pin lists only offer free pins. Invert ticked = LED on when the pin is LOW.</p>
<div id="fns"></div>
<h2>3. Other</h2>
<table id="oth"></table>
<p><button onclick="bk()">Backup settings</button> <button onclick="$('rf').click()">Restore settings</button>
<input type="file" id="rf" accept=".json" style="display:none" onchange="rs(this)"></p>
<p><button onclick="rb()">Restart board</button> <button onclick="fr()">Factory reset</button></p>
<h2>GPIO test</h2>
<p class="note">Press = LED ON, press again = LED OFF (Invert is followed, the pin level is shown). Normal lights stop while testing.
Ends by itself after 2 minutes or when the phone disconnects. Pins changed on this page are tested after Save.</p>
<div class="grid" id="gt"></div>
<p><button onclick="gEnd()">End test</button></p>
<h2>Firmware update</h2>
<p>Choose the <b>_WiFi.bin</b> file <input type="file" id="fw" accept=".bin"><br><br>
<button onclick="up()">Upload firmware</button> <span id="upm"></span></p>
<div class="bar"><button onclick="save()">Save settings</button><span id="msg"></span></div>
<script>
const PN=["Off","Steady","Single flash","Double flash","Triple flash","Beacon fade","Fast flash","Alternate A","Alternate B","Alt double A","Alt double B"];
const TT=["Momentary, pressed = HIGH","Momentary, pressed = LOW","2-position (every flip)"];
const PINS=[0,1,2,3,4,5,6,7,8,9,10,20,21];
const PWARN={2:'boot pin - check the board still starts',8:'boot pin + onboard status LED (status blink stops)',9:'BOOT button pin - must not be LOW at power-up',
 20:'may flicker at power-up (boot text)',21:'may flicker at power-up (boot text)'};
const NF=5,NL=11,NS=12,NONE=255;
let C=null,L={};const $=i=>document.getElementById(i);
const esc=s=>String(s).replace(/[&<>"|]/g,'');
function opt(v,t,sel,dis){return '<option value="'+v+'"'+(sel?' selected':'')+(dis?' disabled':'')+'>'+t+'</option>';}
function sel(id,o,v,on){let s='<select id="'+id+'"'+(on?' onchange="'+on+'"':'')+'>';o.forEach((t,i)=>s+=opt(i,t,i==v,0));return s+'</select>';}
function num(id,v,a,b){return '<input type="number" id="'+id+'" value="'+v+'" min="'+a+'" max="'+b+'">';}
function chk(id,v,on){return '<input type="checkbox" id="'+id+'"'+(v?' checked':'')+(on?' onchange="'+on+'"':'')+'>';}
function row(a,b){return '<tr><td>'+a+'</td><td>'+b+'</td></tr>';}
function act(i){return C.led[i].f<NF;}
// who uses a pin (other than 'me'): null = free, else {k:'ip0'|'ip1'|'L<i>', t:text}
function pinOwner(p,me){if(p==NONE)return null;if(me!=='ip0'&&C.ip[0]==p)return {k:'ip0',t:'6-pos switch input'};if(me!=='ip1'&&C.ip[1]==p)return {k:'ip1',t:'toggle input'};
 for(let i=0;i<NL;i++)if(me!=='L'+i&&act(i)&&C.led[i].p==p)return {k:'L'+i,t:C.led[i].n+' ('+C.fn[C.led[i].f].n+')'};return null;}
function pinUser(p,me){const o=pinOwner(p,me);return o?o.t:'';}
function pinSel(id,v,me,none){let s='<select id="'+id+'" data-me="'+me+'" data-prev="'+v+'" onchange="pinPick(this)">'+(none?opt(NONE,'None',v==NONE,0):'');
 PINS.forEach(p=>{const u=pinUser(p,me);s+='<option value="'+p+'"'+(p==v?' selected':'')+(u&&p!=v?' class="used"':'')+'>GPIO'+p+(u?' - '+esc(u):'')+'</option>';});return s+'</select>';}
// a used pin can still be picked: confirm, then the old owner gets None
function pinPick(el){const me=el.dataset.me,p=+el.value,prev=+el.dataset.prev,o=pinOwner(p,me);
 const who=me=='ip0'?'the 6-pos switch input':me=='ip1'?'the toggle input':C.led[+me.slice(1)].n;
 readAll();
 if(o){if(o.k.startsWith('ip')&&!me.startsWith('ip')){alert('GPIO'+p+' is the '+o.t+'.\nMove the input to another pin first.');undoPick(me,prev);build();return;}
  if(o.k.startsWith('ip')&&me.startsWith('ip')){if(!confirm('GPIO'+p+' is the '+o.t+'.\nSwap the two inputs?')){undoPick(me,prev);build();return;}
   C.ip[+o.k.slice(2)]=prev;build();msg('Inputs swapped - press Save',1);return;}
  if(!confirm('GPIO'+p+' is used by '+o.t+'.\nGive GPIO'+p+' to '+who+'? '+C.led[+o.k.slice(1)].n+' will be set to None.')){undoPick(me,prev);build();return;}
  C.led[+o.k.slice(1)].p=NONE;build();msg(C.led[+o.k.slice(1)].n+' set to None - press Save',1);return;}
 build();}
function undoPick(me,prev){if(me=='ip0')C.ip[0]=prev;else if(me=='ip1')C.ip[1]=prev;else C.led[+me.slice(1)].p=prev;}
function posNames(){const a=['None'];C.fn.forEach((f,i)=>a.push('F'+(i+1)+' '+esc(f.n)));a.push('WiFi (hold)');return a;}
// ---------- build the page from C ----------
function build(){
 $('ver').textContent='v'+C.ver;
 const open=[...document.querySelectorAll('details[open]')].map(d=>d.id);
 let h='<tr><td><b>Pos</b></td><td><b>Value µs</b></td><td></td><td><b>Does</b></td></tr>';
 const PNm=posNames();
 for(let i=0;i<6;i++)h+='<tr><td>'+(i+1)+'</td><td>'+num('pu'+i,C.pu[i],800,2200)+'</td><td><button class="sm" onclick="learn('+i+')">Learn</button></td><td>'+sel('pf'+i,PNm,C.pf[i])+'</td></tr>';
 $('modes').innerHTML=h;
 $('tog').innerHTML=row('6-pos switch input',pinSel('ip0',C.ip[0],'ip0',0))+row('Toggle input',pinSel('ip1',C.ip[1],'ip1',0))
 +row('Toggle switch type',sel('tt',TT,C.tt))+row('Toggle HIGH above (µs)',num('th',C.th,1000,2000))+row('Toggle LOW below (µs)',num('tl',C.tl,1000,2000))
 +row('New 6-pos position counts after (ms)',num('ps',C.ps,20,2000))
 +row('Function position: hold = function off (s)',num('oh',C.oh,1,10))
 +row('WiFi position: hold = WiFi on/off (s)',num('wh',C.wh,2,30));
 $('pinnote').innerHTML=C.pd?'<span class="bad">The saved pin map was not valid - the board is using the default pins now.</span>':'';
 h='';
 for(let f=0;f<NF;f++){const F=C.fn[f],leds=[];for(let i=0;i<NL;i++)if(C.led[i].f==f)leds.push(i);
  h+='<details class="fn" id="df'+f+'"><summary>F'+(f+1)+': '+esc(F.n)+' <span class="note">('+leds.length+' LED, '+F.s+' steps)</span></summary>';
  h+='<p>Name <input class="nm" id="fn'+f+'" maxlength="16" value="'+esc(F.n)+'" onchange="rebuild()"> &nbsp; Steps '
   +sel('fs'+f,Array.from({length:NS},(_,k)=>String(k+1)),F.s-1,'rebuild()')+'</p>';
  h+='<h3>LEDs</h3>';
  leds.forEach(i=>{const E=C.led[i];h+='<div class="led"><div><input class="nm" id="ln'+i+'" maxlength="16" value="'+esc(E.n)+'" onchange="rebuild()">'+pinSel('lp'+i,E.p,'L'+i,1)+'</div>'
   +'<div><label>'+chk('li'+i,E.i)+' Invert</label><label>'+chk('ld'+i,E.d,'pdim(this)')+' Dimmable</label><label>Offset '+num('lo'+i,E.o,0,10000)+' ms</label>'
   +'<button class="sm" onclick="delLed('+i+')">Remove</button></div></div>';});
  h+='<p><button class="sm" onclick="addLed('+f+')">+ Add LED</button></p>';
  h+='<h3>Steps</h3>';
  if(!leds.length)h+='<p class="note">Add an LED first.</p>';
  else for(let k=0;k<F.s;k++){const S=F.st[k];
   h+='<details id="ds'+f+'_'+k+'"><summary>Step '+(k+1)+': '+esc(S.n)+'</summary><p>Name <input class="nm" id="sn'+f+'_'+k+'" maxlength="12" value="'+esc(S.n)+'"></p><table>';
   leds.forEach(i=>h+='<tr><td>'+esc(C.led[i].n)+'</td><td>'+sel('sp'+f+'_'+k+'_'+i,PN,S.p[i])+'</td><td>'+num('sb'+f+'_'+k+'_'+i,S.b[i],0,100)+' %</td></tr>');
   h+='</table></details>';}
  h+='<h3>Timing (ms)</h3><table><tr><td></td><td><b>On</b></td><td><b>Gap</b></td><td><b>Repeat</b></td></tr>';
  ['Single flash','Double flash','Triple flash'].forEach((n,k)=>{h+='<tr><td>'+n+'</td><td>'+num('ft'+f+'_'+(3*k),F.t[3*k],10,5000)+'</td><td>'
   +(k?num('ft'+f+'_'+(3*k+1),F.t[3*k+1],10,5000):'<input type="hidden" id="ft'+f+'_1" value="'+F.t[1]+'">')+'</td><td>'+num('ft'+f+'_'+(3*k+2),F.t[3*k+2],50,10000)+'</td></tr>';});
  h+='<tr><td>Beacon fade</td><td></td><td></td><td>'+num('ft'+f+'_9',F.t[9],50,10000)+'</td></tr>';
  h+='<tr><td>Fast flash</td><td></td><td></td><td>'+num('ft'+f+'_10',F.t[10],20,10000)+'</td></tr>';
  h+='<tr><td>Wig-wag (Alternate)</td><td></td><td></td><td>'+num('ft'+f+'_11',F.t[11],20,10000)+'</td></tr></table>';
  h+='<p class="note">Alt double A / B use the double flash timing; B flashes half a repeat after A. Offset shifts single/double/triple/beacon of that LED.</p>';
  h+='</details>';}
 $('fns').innerHTML=h;
 $('oth').innerHTML=row('Remember lights after power-off',chk('rm',C.rm))
 +row('WiFi on after power-up if no switch moved (s, 0 = never)',num('wb',C.wb,0,600))
 +row('WiFi off when no phone connected (min, 0 = never)',num('wi',C.wi,0,60))
 +row('WiFi low power (SuperMini fix)',chk('lp',C.lp))
 +row('WiFi name','<input id="ss" maxlength="23" value="'+esc(C.ss)+'">')
 +row('New WiFi password (8+ characters, empty = keep)','<input id="pw" maxlength="23">')
 +row('PWM frequency Hz (used after restart)',num('hz',C.hz,100,20000));
 h='';for(let i=0;i<NL;i++)if(act(i))h+='<button id="gb'+i+'" onclick="gT('+i+')"></button>';$('gt').innerHTML=h||'<p class="note">No LEDs in use.</p>';
 gLab(0,0);
 open.forEach(id=>{const d=$(id);if(d)d.open=true;});
 pwarn();
}
// ---------- read every input back into C ----------
function v(id){const e=$(id);if(!e)return undefined;return e.type=='checkbox'?(e.checked?1:0):e.value;}
function setN(o,k,id){const x=v(id);if(x!==undefined&&x!=='')o[k]=+x;}
function setS(o,k,id,n){const x=v(id);if(x!==undefined)o[k]=String(x).replace(/[|&<>"\\]/g,'').slice(0,n||12)||o[k];}
function readAll(){
 for(let i=0;i<6;i++){setN(C.pu,i,'pu'+i);setN(C.pf,i,'pf'+i);}
 ['th','tl','ps','tt','oh','wh','wb','wi','hz','rm','lp'].forEach(k=>setN(C,k,k));
 setN(C.ip,0,'ip0');setN(C.ip,1,'ip1');
 const ss=v('ss');if(ss!==undefined)C.ss=ss;
 for(let i=0;i<NL;i++){const E=C.led[i];setS(E,'n','ln'+i,16);setN(E,'p','lp'+i);setN(E,'i','li'+i);setN(E,'d','ld'+i);setN(E,'o','lo'+i);}
 for(let f=0;f<NF;f++){const F=C.fn[f];setS(F,'n','fn'+f,16);const s=v('fs'+f);if(s!==undefined)F.s=+s+1;
  for(let t=0;t<12;t++)setN(F.t,t,'ft'+f+'_'+t);
  for(let k=0;k<NS;k++){const S=F.st[k];setS(S,'n','sn'+f+'_'+k);for(let i=0;i<NL;i++){setN(S.p,i,'sp'+f+'_'+k+'_'+i);setN(S.b,i,'sb'+f+'_'+k+'_'+i);}}}
}
function rebuild(){readAll();build();}
function addLed(f){readAll();let i=C.led.findIndex(E=>E.f>=NF);if(i<0){msg('All '+NL+' LED slots are in use',0);return;}
 const E=C.led[i];E.f=f;E.i=1;E.d=0;E.o=0;E.n='LED '+(i+1);E.p=NONE;
 for(const p of PINS)if(!pinUser(p,'L'+i)){E.p=p;break;}
 C.fn[f].st.forEach(S=>{S.p[i]=1;S.b[i]=100;});build();$('df'+f).open=true;msg(E.n+' added'+(E.p==NONE?' (no free pin)':' on GPIO'+E.p)+' - press Save',1);}
function delLed(i){readAll();const n=C.led[i].n;C.led[i].f=NONE;C.led[i].p=NONE;build();msg(n+' removed - press Save',1);}
function dimCount(){let n=0;for(let i=0;i<NL;i++)if(act(i)&&C.led[i].p!=NONE&&C.led[i].d)n++;return n;}
function pdim(el){readAll();if(dimCount()>6){el.checked=false;readAll();msg('Max 6 dimmable LEDs - untick another one first',0);}}
function pwarn(){const w=[];for(let i=0;i<NL;i++)if(act(i)&&C.led[i].p==NONE)w.push(esc(C.led[i].n)+' ('+esc(C.fn[C.led[i].f].n)+') has no pin');for(let i=0;i<NL;i++)if(act(i)&&PWARN[C.led[i].p])w.push(esc(C.led[i].n)+' on GPIO'+C.led[i].p+': '+PWARN[C.led[i].p]);
 [0,1].forEach(k=>{if(PWARN[C.ip[k]])w.push((k?'Toggle':'6-pos')+' input on GPIO'+C.ip[k]+': '+PWARN[C.ip[k]]);});
 $('pinnote').innerHTML=(C.pd?'<span class="bad">The saved pin map was not valid - the board is using the default pins now.</span><br>':'')+w.map(x=>'<span class="yel">⚠ '+x+'</span>').join('<br>');}
function check(){if(C.ip[0]==C.ip[1])return '6-pos and toggle inputs are on the same GPIO';
 const used={};used[C.ip[0]]=1;used[C.ip[1]]=1;
 for(let i=0;i<NL;i++){const E=C.led[i];if(!act(i)||E.p==NONE)continue;if(used[E.p])return esc(E.n)+': GPIO'+E.p+' is already used';used[E.p]=1;}
 if(dimCount()>6)return 'too many dimmable LEDs ('+dimCount()+'), max 6';
 if(+C.tl>=+C.th)return 'toggle LOW must be below HIGH';
 return '';}
// ---------- talk to the board ----------
function msg(t,ok){$('msg').textContent=t;$('msg').className=ok?'ok':'bad';}
function post(u,b){return fetch(u,{method:'POST',body:b}).then(r=>r.text().then(t=>({ok:r.ok,t})));}
function learn(i){if(L.s0){$('pu'+i).value=L.w0;msg('Position '+(i+1)+' = '+L.w0+' µs, press Save',1);}else msg('No signal on the 6-pos input',0);}
function save(){readAll();const e=check();if(e){msg('Not saved: '+e,0);return;}
 const p=new URLSearchParams();
 for(let i=0;i<6;i++){p.append('pu'+i,C.pu[i]);p.append('pf'+i,C.pf[i]);}
 ['th','tl','ps','tt','oh','wh','wb','wi','hz','rm','lp','ss'].forEach(k=>p.append(k,C[k]));
 p.append('ip0',C.ip[0]);p.append('ip1',C.ip[1]);
 const pw=v('pw');if(pw)p.append('pw',pw);
 C.led.forEach((E,i)=>p.append('L'+i,[act(i)?E.f:NONE,act(i)?E.p:NONE,E.i,E.d,E.o].join(',')+'|'+E.n));
 C.fn.forEach((F,f)=>{p.append('F'+f,[F.s].concat(F.t).join(',')+'|'+F.n);
  F.st.forEach((S,k)=>{const a=[];for(let i=0;i<NL;i++)a.push(S.p[i],S.b[i]);p.append('S'+f+'_'+k,a.join(',')+'|'+S.n);});});
 msg('Saving...',1);
 post('/save',p).then(r=>{msg(r.t,r.ok);if(r.ok)setTimeout(load,300);}).catch(()=>msg('Not saved: no connection',0));}
function gT(i){post('/gpio',new URLSearchParams({o:i})).then(r=>{if(!r.ok)msg(r.t,0);}).catch(()=>msg('No connection',0));}
function gEnd(){post('/gpio',new URLSearchParams({end:1})).then(()=>msg('Test ended',1));}
function bk(){readAll();const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(C,null,1)],{type:'application/json'}));a.download='SR_NavLights_settings_v2.json';a.click();msg('Backup saved',1);}
function rs(inp){const f=inp.files[0];if(!f)return;f.text().then(t=>{const j=JSON.parse(t);
 if(!j.led||!j.fn){msg('This backup is from v1.x and cannot be loaded into v2',0);return;}
 delete j.ver;delete j.rp;delete j.ri;delete j.pd;C=Object.assign({},C,j);build();save();}).catch(()=>msg('Not a settings file',0));inp.value='';}
function rb(){if(confirm('Restart the board?'))post('/reboot',new URLSearchParams()).then(()=>msg('Restarting',1));}
function fr(){if(confirm('Reset ALL settings to factory?'))post('/reset',new URLSearchParams()).then(()=>msg('Reset done, restarting',1));}
function up(){const f=$('fw').files[0];if(!f){$('upm').textContent='Choose a file first';return;}
 if(!/_(wifi|ota)\.bin$/i.test(f.name)&&!confirm('The file name does not end with _WiFi.bin. Upload anyway?'))return;
 const d=new FormData();d.append('fw',f,f.name);$('upm').textContent='Uploading, do not power off...';
 fetch('/update',{method:'POST',body:d}).then(r=>r.text()).then(t=>$('upm').textContent=t).catch(()=>$('upm').textContent='Upload failed');}
function gLab(tm,tb){if(!C)return;for(let i=0;i<NL;i++){const b=$('gb'+i);if(!b)continue;const E=C.led[i],rp=C.rp[i],on=!!(tm&&(tb>>i&1)),hi=on!==!!E.i;
 b.className=on?'on':'';b.innerHTML='F'+(E.f+1)+' · '+esc(E.n)+'<br>'+(rp==NONE?'no pin yet (Save)':'GPIO'+rp+' · LED '+(on?'ON':'OFF')+' · pin '+(hi?'HIGH':'LOW'));}}
let AK='';
function authBox(j){const k=j.au+'|'+j.ao+'|'+j.kb+'|'+j.id;if(k===AK)return;AK=k;
 if(j.au){$('auth').innerHTML='<p class="note">'+(j.ao?'Activation not set up in this firmware':'Activated')+' · Board ID '+j.id+'</p>';return;}
 $('auth').innerHTML='<div class="warn"><b class="bad">NOT ACTIVATED — all lights are off</b>'+(j.kb?'<br><span class="bad">Public key in this firmware is not valid</span>':'')
 +'<br>Board ID: <input class="idbox" readonly value="'+j.id+'" onclick="this.select()"> <span class="note">send this to SrRcTech</span>'
 +'<br><textarea id="ac" rows="3" placeholder="Paste the activation code here"></textarea><button onclick="actv()">Activate</button></div>';}
function actv(){const c=$('ac').value;post('/activate',new URLSearchParams({code:c})).then(r=>{msg(r.t,r.ok);if(r.ok)AK='';}).catch(()=>msg('No connection',0));}
function live(){fetch('/live').then(r=>r.json()).then(j=>{L=j;if(!C)return;
 const t=j.tog<0?'-':(j.pr?'PRESSED':'released')+(j.tog?' HIGH':' LOW');
 const pf=j.pos>0?C.pf[j.pos-1]:-1;
 let h='Position: <span class="'+(j.s0?'ok':'bad')+'">'+(j.s0?(j.pos||'-')+' ('+j.w0+' µs)':'NO SIGNAL')+'</span> &nbsp; Toggle: <span class="'+(j.s1?'ok':'bad')+'">'+(j.s1?t+' ('+j.w1+' µs)':'NO SIGNAL')+'</span>';
 if(j.tm)h+='<br><span class="bad">GPIO TEST ON</span>';
 else C.fn.forEach((F,f)=>{const k=j.fs[f],S=F.st[k<F.s?k:0];h+='<br>'+(pf==f+1?'▶ ':'')+'F'+(f+1)+' '+esc(F.n)+': '+(j.fo[f]?'<span class="bad">OFF</span> (step '+(k+1)+')':'Step '+(k+1)+'/'+F.s+' '+esc(S.n));});
 h+='<br>WiFi: '+(j.wf?'on':'off')+(pf==6?' ▶ hold toggle to switch':'');
 $('live').innerHTML=h;gLab(j.tm,j.tb);authBox(j);
}).catch(()=>{$('live').textContent='No connection';}).finally(()=>setTimeout(live,500));}
function load(){fetch('/cfg').then(r=>r.json()).then(j=>{C=j;build();}).catch(()=>setTimeout(load,2000));}
load();live();
</script></body></html>)HTML";
