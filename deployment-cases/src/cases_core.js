/* Deployment cases: scripted ward scenarios run through a model of the MII safety layer (Chapter 5 rules)
   and the battery power supervisor. All people, detections and misses are scripted, nothing here is recorded footage. */
const MII=(()=>{
const C=ShadowCore,DT=.1,CAM=.27,HFOV=40*Math.PI/180,VMAXP=2.0,H_PRED=1.5;
const VERSIONS={
 v40:{id:'v40',name:'v4.0 (first MII)',bgModel:false,bgTrusted:false,gates:false,adaptive:false,immunity:false,reconfirm:true,warmup:0,buffer:1.0},
 v52:{id:'v52',name:'v5.2 (background model)',bgModel:true,bgTrusted:false,gates:true,adaptive:false,immunity:false,reconfirm:false,warmup:.5,buffer:.27},
 v60:{id:'v60',name:'v6.0 (final)',bgModel:true,bgTrusted:true,gates:true,adaptive:true,immunity:true,reconfirm:false,warmup:.5,buffer:.27}};
const RANKS=['SAFE','CAUTION','PREDICTED DANGER','TOO CLOSE','CRITICAL','MOVE AWAY'];
const SPEECH={1:'Caution, fast approach',2:'Predicted danger ahead',3:'Person too close',4:'Critical: person closing fast'};
const CAP=[Infinity,.5,.36,.15,0,0];           /* speed cap per rank when motion supervision is on */
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
function posAt(p,t){const k=p.keys;if(t<=k[0][0])return{x:k[0][1],z:k[0][2]};for(let n=1;n<k.length;n++){if(t<=k[n][0]){const a=k[n-1],b=k[n],f=(t-a[0])/Math.max(1e-6,b[0]-a[0]);return{x:a[1]+(b[1]-a[1])*f,z:a[2]+(b[2]-a[2])*f};}}const e=k[k.length-1];return{x:e[1],z:e[2]};}
const inWin=(w,t)=>(w||[]).some(([a,b])=>t>=a&&t<=b);
const side=x=>x>.15?'LEFT':x<-.15?'RIGHT':'CENTER';
function slope(pts){const n=pts.length;if(n<2)return 0;let mt=0,md=0;pts.forEach(q=>{mt+=q[0];md+=q[1];});mt/=n;md/=n;let a=0,b=0;pts.forEach(q=>{a+=(q[0]-mt)*(q[1]-md);b+=(q[0]-mt)**2;});return b>0?a/b:0;}

function simulate(cs,cfg){
 const V=VERSIONS[cfg.version||'v60'],resp=cfg.response||'supervised',powerSup=cfg.powerSup!==false;
 const prof={...C.DEFAULT,...(cs.profile||{})},lim=C.limits(prof.cells),mass=prof.mass+(cs.payload||0);
 const S={tracks:new Map()},events=[],samples=[],plan=cs.plan||{startT:0,goalZ:0};
 let z=cs.z0||0,v=0,soc=cs.soc??.8,t=0,dead=false,deadT=null,lossS=0,vt=C.ocv(soc,prof.cells),it=0,minV=Infinity;
 const R=prof.resistance,sup=new C.PowerSupervisor(prof,{rOnline:false});
 const alert=(t0,tr,rank,text,type='alert')=>events.push({t:+t0.toFixed(2),key:tr.key,pid:tr.pid,rank,text,type,fake:!tr.person||tr.ghost});
 const fi=cs.frameInterval||0;let nextFrame=0,lastMii=null;const m={minDist:Infinity,stopS:0,ghostStopS:0,unalertedS:0,minV:Infinity,deadT:null,deadZ:null,firstAlert:null,firstClose:null,alerts:0,falseAlerts:0,arrivalT:null,trackFlicker:0};
 const steps=Math.round(cs.duration/DT);
 for(let n=0;n<=steps;n++){t=n*DT;
  const people=cs.people||[],camZ=z+CAM,pos=people.map(p=>posAt(p,t));
  let nYolo=0,rejected=[];
  /* ---------- MII frame ---------- */
  if(!dead&&t>=nextFrame-1e-9){const dtF=lastMii?t-lastMii:DT;lastMii=t;nextFrame=fi?t+fi:t;
   const dets=[],fbs=[],pst={};
   people.forEach((p,k)=>{const X=pos[k].x,Zc=pos[k].z-camZ,d=Math.hypot(X,Zc),inF=Zc>.05&&Math.abs(Math.atan2(X,Zc))<HFOV&&d<7;if(!inF){pst[p.id]='out of camera view';return;}
    const miss=cs.dark||inWin(p.miss,t)||d<.3,k0=p.keys[0],learned=V.bgModel&&!V.bgTrusted&&p.startup&&Math.hypot(pos[k].x-k0[1],pos[k].z-k0[2])<.25;
    pst[p.id]='tracked by YOLO';if(!miss&&d<=6){if(learned){rejected.push({p,X,Zc,d,why:'G2 background match'});pst[p.id]='rejected as background (G2 gate)';}else{dets.push({p,X,Zc,d,pos:pos[k]});nYolo++;}}
    else{pst[p.id]=(cs.dark?'RGB detector blind (dark)':d<.3?'too close for the detector':'detector miss');if(d>=.15&&d<=.9&&!learned){fbs.push({key:'F'+p.id,person:true,id:p.id,X,Zc,d});pst[p.id]+=', depth fallback';}}});
   (cs.statics||[]).forEach(s=>{if(!s.solid)return;const X=s.x,Zc=s.z-camZ,d=Math.hypot(X,Zc);if(Zc>.05&&Math.abs(Math.atan2(X,Zc))<HFOV&&d>=.15&&d<=.9&&(!V.bgModel||t<V.warmup))fbs.push({key:'F'+s.id,person:false,id:s.id,X,Zc,d});});
   const enc=dets.some(q=>q.d<1.5&&q.X>.2)&&dets.some(q=>q.d<1.5&&q.X<-.2),seen=new Set();
   for(const q of dets){const key='T'+q.p.id;seen.add(key);let tr=S.tracks.get(key);
    if(tr&&V.reconfirm&&q.p.ghost&&t>=q.p.ghost[0]&&t<=q.p.ghost[1]){tr.lastSeen=t;tr.ghost=true;tr.raw=tr.rank;pst[q.p.id]='left, ghost track kept alive';continue;}if(!tr){tr={key,pid:q.p.id,kind:'yolo',person:true,firstT:t,hist:[],rank:0,lowSince:null,r5Since:null,lastAlert:-1e9,trig:0,mature:false,lastSeen:t,dLast:null,ghost:false,muted:false,label:''};S.tracks.set(key,tr);}
    const dtp=t-tr.lastSeen,jump=tr.dLast==null?0:Math.abs(q.d-tr.dLast),thr=V.adaptive?Math.max(.4,VMAXP*dtp):.4;
    if(V.gates&&!(V.immunity&&tr.mature)&&jump>thr){if(!V.adaptive){tr.firstT=t;tr.mature=false;tr.hist=[];tr.rank=0;tr.lowSince=null;tr.r5Since=null;m.trackFlicker++;}tr.dLast=q.d;tr.lastSeen=t;tr.gated=true;pst[q.p.id]='track reset by depth-jump gate';continue;}
    tr.gated=false;pst[q.p.id]='tracked by YOLO'+(tr.mature?' (mature)':'');tr.mature=t-tr.firstT>=.15;tr.hist.push({t,x:q.pos.x,z:q.pos.z});while(tr.hist.length>1&&(t-tr.hist[0].t>.6&&tr.hist.length>2||tr.hist.length>8))tr.hist.shift();
    tr.dLast=q.d;tr.lastSeen=t;tr.ghost=false;tr.d=q.d;tr.X=q.X;tr.Zc=q.Zc;
    const vz=-slope(tr.hist.map(h=>[h.t,Math.hypot(h.x,h.z-camZ)])),dp=q.d-vz*H_PRED;tr.vz=vz;
    let raw=0;if(q.d<1)raw=5;else if(q.d<1.5&&(vz>.7||enc))raw=4;else if(q.d<1.5)raw=3;else if(dp<1&&vz>.3)raw=2;else if(vz>.7&&q.d<3)raw=1;
    if(tr.trig>=3&&raw>=2)raw=5;tr.raw=raw;tr.label='ID:'+tr.pid+' '+RANKS[raw]+' '+side(q.X)+' '+q.d.toFixed(2)+'m';}
   /* fallback (depth-change) tracks */
   const fseen=new Set();for(const f of fbs){fseen.add(f.key);let tr=S.tracks.get(f.key);if(!tr){tr={key:f.key,pid:f.id,kind:'fallback',person:f.person,firstT:t,hist:[],rank:0,lowSince:null,r5Since:null,lastAlert:-1e9,trig:0,mature:true,lastSeen:t,ghost:false,muted:false};S.tracks.set(f.key,tr);}
    tr.lastSeen=t;tr.d=f.d;tr.X=f.X;tr.Zc=f.Zc;tr.vz=0;tr.raw=5;tr.label='PERSON '+side(f.X)+' '+f.d.toFixed(2)+'m';}
   /* lost tracks, ghost loop */
   for(const [key,tr] of [...S.tracks]){if(seen.has(key)||fseen.has(key))continue;const p=(cs.people||[]).find(q=>'T'+q.id===key);
    if(tr.kind==='yolo'&&V.reconfirm&&p&&p.ghost&&t>=p.ghost[0]&&t<=p.ghost[1]){tr.lastSeen=t;tr.ghost=true;tr.raw=tr.rank;continue;}
    const lost=t-tr.lastSeen,keep=tr.kind==='fallback'?.5:V.buffer;if(lost>keep){S.tracks.delete(key);}else if(tr.kind==='fallback')tr.raw=0;else tr.raw=tr.rank;}
   /* state machine, alerts */
   for(const tr of S.tracks.values()){if(tr.gated&&!tr.rank)continue;const raw=tr.raw??0;
    if(raw>=tr.rank){if(raw>tr.rank){const was=tr.rank;tr.rank=raw;if(was<2&&raw>=2)tr.trig++;if(raw>=1&&raw<5)alert(t,tr,raw,SPEECH[raw]);if(raw===5){tr.r5Since=t;tr.lastAlert=t;alert(t,tr,5,'Move away immediately');}}tr.lowSince=null;}
    else{if(tr.lowSince==null)tr.lowSince=t;if(t-tr.lowSince>=.5-1e-9){if(tr.rank===5){tr.r5Since=null;tr.muted=false;}tr.rank=raw;tr.lowSince=null;}}
    if(tr.rank===5&&tr.r5Since!=null&&t-tr.lastAlert>=15-1e-9){const ph=t-tr.r5Since;tr.lastAlert=t;if(ph<45)alert(t,tr,5,'Move away immediately');else if(ph<75)alert(t,tr,5,'Please clear the path');else if(!tr.muted){tr.muted=true;alert(t,tr,5,'Audio muted, visual indicator only','visual');}}}
   lastMii={t,tracks:[...S.tracks.values()].map(tr=>({key:tr.key,pid:tr.pid,kind:tr.kind,rank:tr.rank,d:tr.d,vz:tr.vz||0,label:tr.label||'',ghost:tr.ghost,person:tr.person,mature:tr.mature,X:tr.X,Zc:tr.Zc})),nYolo,pst,rejected:rejected.map(r=>({pid:r.p.id,d:r.d,X:r.X,Zc:r.Zc,why:r.why}))};}
  /* ---------- robot ---------- */
  const mii=lastMii||{tracks:[],nYolo:0,pst:{},rejected:[]};let worst=0;for(const tr of mii.tracks)worst=Math.max(worst,tr.rank);
  let vnav=0;if(t>=plan.startT)vnav=Math.min(prof.vmax,Math.sqrt(Math.max(0,2*prof.dec*(plan.goalZ-z))));
  let vlid=Infinity;const obst=[];people.forEach((p,k)=>obst.push([pos[k].x,pos[k].z]));(cs.statics||[]).forEach(s=>{if(s.solid)obst.push([s.x,s.z]);});
  for(const [ox,oz] of obst){const Zc=oz-camZ;if(Math.abs(ox)<.55&&Zc>0)vlid=Math.min(vlid,Math.sqrt(2*1.5*Math.max(0,Zc-.35)));}
  let target=Math.min(vnav,vlid),cap=Infinity;if(resp==='supervised'&&!dead){cap=CAP[worst];target=Math.min(target,cap);}
  const dec=vlid<vnav?1.5:prof.dec;let wanted=clamp(target,v-dec*DT,v+prof.acc*DT),supState='ok',pcap=C.powerCap(C.ocv(soc,prof.cells),R,prof.imax,lim.guard);
  if(!dead&&powerSup){if(n===0){sup.voc=C.ocv(soc,prof.cells);sup.tSample=t;}const d=sup.decide(t,wanted,v,DT,cs.payload||0);supState=d.state;pcap=d.powerCap;if(d.state==='park')wanted=0;else if(d.speedLimit<wanted)wanted=Math.max(0,d.speedLimit);}
  if(dead)wanted=0;
  const gr=(cs.grade||[]).reduce((a,[z0,z1,g])=>z>=z0&&z<=z1?a+g:a,0),P=C.drawPower(wanted,(wanted-v)/DT,mass,prof)+mass*9.81*gr*Math.max(0,wanted)/prof.eta,e=C.supply(P,C.ocv(soc,prof.cells),R);
  if(!dead){vt=e.v;it=e.i;minV=Math.min(minV,vt);soc=Math.max(0,soc-e.i*DT/(prof.capacity_ah*3600));if(vt<lim.vmin||e.collapse){lossS+=DT;if(lossS>=.3-1e-9){dead=true;deadT=t;m.deadT=t;m.deadZ=z;events.push({t:+t.toFixed(2),key:'power',pid:0,rank:0,text:'Supply loss: pack voltage fell below '+lim.vmin.toFixed(1)+' V',type:'power',fake:false});}}else lossS=0;if(powerSup)sup.update(t,vt,it);}
  v=dead?0:wanted;
  samples.push({t,z,v,vnav,vlid:isFinite(vlid)?vlid:null,cap:isFinite(cap)?cap:null,vt,it,soc,pcap,supState,dead,worst,ppl:pos.map((q,k)=>({id:people[k].id,x:q.x,z:q.z})),mii});
  /* metrics */
  people.forEach((p,k)=>{const d=Math.hypot(pos[k].x,pos[k].z-camZ),Zc=pos[k].z-camZ;if(Zc>0&&Math.abs(pos[k].x)<2)m.minDist=Math.min(m.minDist,d);
   if(d<1.5&&Zc>.05&&Math.abs(Math.atan2(pos[k].x,Zc))<HFOV){if(m.firstClose==null)m.firstClose=t;const alerted=mii.tracks.some(tr=>tr.person&&String(tr.pid)===String(p.id)&&tr.rank>=3&&!tr.ghost);if(!alerted)m.unalertedS+=DT;}});
  if(v<.02&&t>=plan.startT&&(plan.goalZ-z)>.3){m.stopS+=DT;if(mii.tracks.some(tr=>tr.rank>=4&&(tr.ghost||!tr.person)))m.ghostStopS+=DT;}
  z+=v*DT;if(m.arrivalT==null&&plan.goalZ>0&&z>=plan.goalZ-.05)m.arrivalT=t;}
 const al=events.filter(e=>e.type==='alert');m.alerts=al.length;m.falseAlerts=al.filter(e=>e.fake).length;const fa=al.find(e=>!e.fake);m.firstAlert=fa?fa.t:null;m.minV=minV;m.finalZ=z;m.dead=dead;
 return{samples,events,metrics:m,cfg,V};}

/* ---------------- the cases ---------------- */
const walk=(id,kind,keys,extra={})=>({id,kind,keys,...extra});
const CASES=[
{id:'A',n:1,tag:'people',title:'Staff and patients standing in the corridor',
 story:'The robot drives 30 m down a ward corridor past a nurse at a door and a patient resting beside an IV pole. Neither person moves toward it. Because the layer corrects for the robot\'s own motion, people who merely stand nearby stay SAFE and cause no alert, so staff are not nagged.',
 chapter:'Section 5.1 and Table 5.1: the layer separates a person about to cross the path from a person just standing nearby.',
 people:[walk(1,'nurse',[[0,1.55,12]]),walk(2,'patient',[[0,-1.5,22]])],statics:[],plan:{startT:1,goalZ:30},duration:60,
 pass:(m)=>({ok:m.alerts===0,why:m.alerts===0?'No alerts: standing people stayed SAFE.':m.alerts+' alert(s) raised for people who never entered the path.'})},
{id:'B',n:2,tag:'people',title:'A visitor steps out of a room and crosses',
 story:'A visitor leaves a room, walks into the lane and stops to read a sign. The layer first sees the closing speed (PREDICTED DANGER), then TOO CLOSE, and the robot slows and yields. Once the visitor moves on, the state drops back to SAFE after 0.5 s of consistent readings.',
 chapter:'Section 5.8: priority ladder, immediate escalation, de-escalation after 0.5 s.',
 people:[walk(1,'visitor',[[0,1.75,11.5],[8,1.75,11.5],[11.5,.1,8.9],[22,.1,8.9],[26,-1.7,9.2],[40,-1.7,9.2]])],statics:[],plan:{startT:1,goalZ:30},duration:80,
 pass:(m)=>({ok:m.unalertedS<=.3&&m.minDist>.45,why:m.unalertedS<=.3?'Visitor was flagged before reaching 1.5 m and the robot kept '+m.minDist.toFixed(2)+' m clear.':'Visitor was inside 1.5 m with no alert for '+m.unalertedS.toFixed(1)+' s.'})},
{id:'C',n:3,tag:'people',title:'Two visitors crowd the robot at the lift lobby',
 story:'Two visitors stand next to the parked robot at about 0.78 m and do not move. Both go straight to MOVE AWAY. The spoken warning repeats every 15 s, changes wording after 45 s, then goes silent at 75 s so it does not nag forever, leaving only the visual indicator.',
 chapter:'Section 5.8 and Figure 5.9a: MOVE AWAY below 1.0 m, three-phase escalation.',
 people:[walk(1,'visitor',[[0,2.4,5],[4,.45,.91],[100,.45,.91]]),walk(2,'visitor',[[0,-2.4,5],[5,-.46,.9],[100,-.46,.9]],{})],statics:[],plan:{startT:999,goalZ:0},duration:95,
 pass:(m,ev)=>{const a=ev.filter(e=>e.type==='alert'&&e.rank===5).length,vis=ev.some(e=>e.type==='visual');return{ok:a>=4&&vis,why:a>=4&&vis?a+' spoken alerts, then muted with a visual indicator.':'Escalation phases did not complete.'};}},
{id:'D',n:4,tag:'failure',title:'Ghost alerts: desk, glass partition and IV pole',
 story:'The robot is docked at the nurses\' station. Nobody is near it, but a desk, a glass partition and an IV pole all sit within 0.9 m. The first MII versions scanned raw depth and fired MOVE AWAY at furniture, over and over. The background model learned the static scene; only a 0.5 s warm-up window remains.',
 chapter:'Section 5.5 and Figure 5.4: ghost detections from the proximity fallback.',
 people:[],statics:[{id:'desk',kind:'desk',x:.15,z:.95,solid:true},{id:'glass',kind:'glass',x:.5,z:.95,solid:true},{id:'ivpole',kind:'ivpole',x:-.45,z:1.0,solid:true}],plan:{startT:999,goalZ:0},duration:40,
 pass:(m,ev)=>{const late=ev.filter(e=>e.type==='alert'&&e.fake&&e.t>.6).length,early=ev.filter(e=>e.type==='alert'&&e.fake&&e.t<=.6).length;return{ok:late===0,why:late===0?(early?early+' alert(s) inside the 0.5 s warm-up window, then quiet.':'No false alerts.'):late+' false MOVE AWAY alerts on furniture after the first second.'};}},
{id:'E',n:5,tag:'failure',title:'A nurse is already standing there when the robot boots',
 story:'The robot powers up with a nurse 0.74 m in front. While the background model warms up it writes her depth into the "static scene", then rejects her as furniture. v5.2 drove off blind to her. The fix marks pixels trusted only after they have been updated without a person in the box.',
 chapter:'Section 5.6 and Figures 5.6, 5.7: false rejection of a real person, bg_trusted pixels.',
 people:[walk(1,'nurse',[[0,.15,1.02],[11,.15,1.02],[13,1.45,1.3],[40,1.45,1.3]],{startup:true})],statics:[],plan:{startT:6,goalZ:12},duration:30,
 pass:(m,ev)=>{const a=ev.find(e=>e.type==='alert'&&!e.fake);return{ok:!!a&&a.t<=2,why:a&&a.t<=2?'Nurse flagged at '+a.t.toFixed(1)+' s.':'Nurse was not flagged while standing at 0.74 m. Only the LiDAR stack stopped the robot.'};}},
{id:'F',n:6,tag:'failure',title:'The person has left but the alert stays',
 story:'A nurse comes up to the robot, then walks away down the corridor. In early versions weak detections on the wall kept re-confirming the old track, so the robot kept warning and, with motion supervision on, stayed stopped for a person who was no longer there. A short track buffer and depth-jump gate now break that loop.',
 chapter:'Section 5.5.2 and Figure 5.5: the ByteTrack re-confirmation loop.',
 people:[walk(1,'nurse',[[0,.2,6],[4,.15,1.12],[12,.15,1.12],[13,.3,2.3],[16,.3,6],[20,1.5,12],[60,1.5,12]],{ghost:[12.5,32]})],statics:[],plan:{startT:14,goalZ:20},duration:60,
 pass:(m)=>({ok:m.ghostStopS<=.5&&m.falseAlerts===0,why:m.ghostStopS<=.5&&m.falseAlerts===0?'Track dropped as soon as the nurse left.':'Robot held for '+m.ghostStopS.toFixed(0)+' s and raised '+m.falseAlerts+' alert(s) for a person who had gone.'})},
{id:'G',n:7,tag:'failure',title:'Night ward: the camera cannot see',
 story:'In a dark ward the RGB detector reports People: 0 even for a person straight ahead. The depth fallback is the only thing left, and it only covers 0.15 to 0.9 m, so the warning comes late. This case shows a real limit: in the dark the layer only warns inside 0.9 m, with no early prediction.',
 chapter:'Section 5.3 and Figure 5.2a: low-light detection failure; the depth fallback was kept because of it.',
 dark:true,people:[walk(1,'visitor',[[0,.1,16],[9,.1,9.5],[60,.1,9.5]])],statics:[],plan:{startT:1,goalZ:14},duration:40,
 pass:(m,ev)=>{const a=ev.find(e=>e.type==='alert'&&!e.fake);return{ok:!!a&&m.minDist>.45,why:a?'Depth fallback raised MOVE AWAY at '+a.t.toFixed(1)+' s, min distance '+m.minDist.toFixed(2)+' m. Late, but it fired.':'No alert in the dark.'};}},
{id:'H',n:8,tag:'failure',title:'Slow computer: the model runs at 0.5 frames per second',
 story:'On a CPU-only test the layer ran at about 0.5 fps. The 0.4 m depth-jump gate was designed for 30 fps, so every walking person looked like a "jump" and their track was thrown away each frame. v6.0 adds mature-track immunity and a gate that scales with the time between frames. States still update only every 2 s, so this is a bench fallback, not a deployment mode.',
 chapter:'Section 5.7 and Figure 5.8: FPS-adaptive ghost gates and track maturity.',
 frameInterval:2,people:[walk(1,'visitor',[[0,.1,9],[16,.1,1.2],[28,.1,1.2],[32,1.5,2.5],[40,1.6,6]])],statics:[],plan:{startT:999,goalZ:0},duration:45,
 pass:(m,ev)=>{const a=ev.find(e=>e.type==='alert'&&!e.fake);return{ok:!!a&&a.t<=16.5,why:a&&a.t<=16.5?'First alert at '+a.t.toFixed(0)+' s despite 2 s frames.':(a?'First alert only at '+a.t.toFixed(0)+' s, after the gate reset the track '+m.trackFlicker+' times.':'Never alerted.')};}},
{id:'I',n:9,tag:'power',title:'Heavy trolley, low battery and three pedestrian stops',
 story:'The robot tows 140 kg on a pack at 35% charge with raised internal resistance. Three visitors cross and pause in the lane, so it must stop and pull away three times. Each pull-away is a power spike. Without the power supervisor the pack sags below its limit and the robot dies in the corridor. With it, acceleration is trimmed to what the battery can give.',
 chapter:'Extension: the battery power supervisor from the Battery & Safety Lab, working beside the MII layer.',
 payload:140,soc:.36,profile:{resistance:.23},grade:[[20,38,.04]],
 people:[walk(1,'visitor',[[0,1.75,8],[4,1.75,8],[6.5,.2,8],[14,.2,8],[17,-1.7,8.2],[40,-1.7,8.2]]),walk(2,'visitor',[[0,-1.75,18],[14,-1.75,18],[17,-.2,18],[25,-.2,18],[28,1.7,18.2],[50,1.7,18.2]]),walk(3,'visitor',[[0,1.75,28],[24,1.75,28],[27,.2,28],[36,.2,28],[39,-1.7,28.2],[60,-1.7,28.2]])],statics:[],plan:{startT:1,goalZ:38},duration:110,
 pass:(m)=>({ok:!m.dead&&m.arrivalT!=null,why:m.dead?'Supply loss at '+m.deadT.toFixed(0)+' s, '+m.deadZ.toFixed(0)+' m along the corridor. The robot stopped for good in the lane.':m.arrivalT!=null?'Arrived at '+m.arrivalT.toFixed(0)+' s with minimum '+m.minV.toFixed(1)+' V.':'Did not arrive.'})}
];
return{simulate,CASES,VERSIONS,RANKS,CAP,posAt,DT};
})();
if(typeof module!=='undefined')module.exports=MII;
