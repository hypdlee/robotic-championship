/* battery-shadow core in JavaScript: port of the Python package (model, supervisor, forecast, fit, scoring).
   Units: m, s, kg, V, A, W, Wh, Ah. Works in Node (for tests) and in the browser. */
const ShadowCore=(()=>{
const G=9.81,FLOOR=.10,DT=.1,STILL=.02,EVENT_MIN_S=.3;
const DEFAULT={name:'generic',mass:22,vmax:.65,acc:.75,dec:.9,eta:.55,crr:.02,base:52,trans:45,imax:25,cells:6,capacity_ah:2.2,resistance:.07};
const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
const kOf=c=>c/6,ocv=(s,c)=>(19.8+5.4*s)*kOf(c),socFromOcv=(v,c)=>clamp((v/kOf(c)-19.8)/5.4,0,1);
const limits=c=>({vmin:19.2*kOf(c),guard:19.7*kOf(c),nom:22.2*kOf(c)});
const usableWh=(s,p)=>p.capacity_ah*(19.8*(s-FLOOR)+2.7*(s*s-FLOOR*FLOOR))*kOf(p.cells);
function supply(power,v,r){if(r<1e-8)return{i:power/v,v,collapse:false};const d=v*v-4*r*power;if(d<0)return{i:v/(2*r),v:v/2,collapse:true};const i=2*power/(v+Math.sqrt(d));return{i,v:v-i*r,collapse:false};}
const drawPower=(v,a,m,p)=>{const ap=Math.max(0,a);return p.base+p.crr*m*G*v/p.eta+m*ap*v/p.eta+p.trans*ap;};
const powerCap=(voc,r,imax,guard)=>{const mi=clamp((voc-guard)/Math.max(r,1e-8),0,imax);return mi*(voc-mi*r);};
function limitSpeed(wanted,v,dt,mass,p,cap){let lo=0,hi=wanted;for(let k=0;k<25;k++){const mid=(lo+hi)/2;if(drawPower(mid,(mid-v)/dt,mass,p)<=cap)lo=mid;else hi=mid;}return lo;}
const sum=a=>{let s=0;for(let k=0;k<a.length;k++)s+=a[k];return s;};
const mean=a=>sum(a)/a.length;
function median(a){if(!a.length)return NaN;const s=[...a].sort((x,y)=>x-y),m=s.length>>1;return s.length%2?s[m]:(s[m-1]+s[m])/2;}
function pct(a,q){a=a.filter(Number.isFinite).sort((x,y)=>x-y);if(!a.length)return NaN;const r=(a.length-1)*q/100,lo=Math.floor(r),hi=Math.ceil(r);return a[lo]+(a[hi]-a[lo])*(r-lo);}

/* ---------- forecast ---------- */
function restSoc(vRest,iRest,p){return socFromOcv(vRest+Math.max(0,iRest)*p.resistance,p.cells);}
function finish(o){o.energyWh=sum(o.power)*DT/3600;o.minV=o.voltage.length?Math.min(...o.voltage):NaN;o.duration=o.t.length?o.t[o.t.length-1]:0;return o;}
function predictPlan(plan,p,vRest,iRest=0,opt={}){
 const dt=DT,powerLimit=!!opt.powerLimit,guard=limits(p.cells).guard,maxSteps=18000;let limitedS=0;
 let soc=restSoc(vRest,iRest,p);const mass=p.mass+plan.payload_kg;let t=0,v=0;const T=[],S=[],V=[],I=[],P=[];let ok=true,note='';
 const out=(o,n)=>finish({t:T,speed:S,voltage:V,current:I,power:P,socEnd:soc,ok:o,note:n,limitedS});
 for(const leg of plan.legs){const vcap=leg.vmax?Math.min(p.vmax,leg.vmax):p.vmax;let s=0,dwell=0,arrived=false;
  for(;;){if(T.length>=maxSteps)return out(false,'horizon exceeded');
   const rem=Math.max(0,leg.distance_m-s),target=arrived?0:Math.min(vcap,Math.sqrt(Math.max(0,2*p.dec*rem)));
   let wanted=clamp(target,v-p.dec*dt,v+p.acc*dt),power=drawPower(wanted,(wanted-v)/dt,mass,p);const voc=ocv(soc,p.cells);
   if(powerLimit){const mi=clamp((voc-guard)/Math.max(p.resistance,1e-8),0,p.imax),cap=mi*(voc-mi*p.resistance);
    if(power>cap){limitedS+=dt;wanted=limitSpeed(wanted,v,dt,mass,p,cap);power=drawPower(wanted,(wanted-v)/dt,mass,p);if(power>cap+1e-4)return out(false,'parked: insufficient power');}}
   const e=supply(power,voc,p.resistance);if(e.collapse){ok=false;note='supply collapse predicted';}
   T.push(t+dt);S.push(wanted);V.push(e.v);I.push(e.i);P.push(power);
   soc=Math.max(0,soc-e.i*dt/(p.capacity_ah*3600));v=wanted;s+=v*dt;t+=dt;
   if(!arrived&&s>=leg.distance_m-.005){s=leg.distance_m;arrived=true;}
   if(arrived&&v<=1e-6){dwell+=dt;if(dwell>=leg.dwell_s)break;}}}
 return out(ok,note);}
function smoothSpeedAccel(t,speed,deadband=.1,smooth='auto'){
 const n=speed.length,dts=[],d=[];for(let k=1;k<n;k++){dts.push(t[k]-t[k-1]);d.push(speed[k]-speed[k-1]);}
 const dtm=dts.length?median(dts):.1;
 if(smooth==='auto'){const st=d.filter(x=>Math.abs(x)<.03).map(Math.abs);const step=st.length?median(st):0;smooth=step/Math.max(dtm,1e-3)>.04;}
 let sm=Array.from(speed);
 if(smooth&&n>=3&&dtm<=.25){sm=new Array(n);for(let k=0;k<n;k++){const lo=Math.max(0,k-1),hi=Math.min(n-1,k+1);let s=0;for(let j=lo;j<=hi;j++)s+=speed[j];sm[k]=s/(hi-lo+1);}}
 const a=new Array(n).fill(0);for(let k=1;k<n;k++){const dd=t[k]-t[k-1];if(dd>0){const x=(sm[k]-sm[k-1])/dd;a[k]=Math.abs(x)<deadband?0:x;}}
 return{sm,a};}
function predictFromSpeed(t,speed,p,vRest,iRest,payload){
 let soc=restSoc(vRest,iRest,p);const mass=p.mass+payload,{a}=smoothSpeedAccel(t,speed),n=t.length,V=new Array(n),I=new Array(n),P=new Array(n);let prev=t[0];
 for(let k=0;k<n;k++){const dt=k?Math.max(0,t[k]-prev):0;prev=t[k];const power=drawPower(Math.max(0,speed[k]),a[k],mass,p),e=supply(power,ocv(soc,p.cells),p.resistance);V[k]=e.v;I[k]=e.i;P[k]=power;soc=Math.max(0,soc-e.i*dt/(p.capacity_ah*3600));}
 const o=finish({t:Array.from(t),speed:Array.from(speed),voltage:V,current:I,power:P,socEnd:soc,ok:true,note:'',limitedS:0});
 let e=0;for(let k=1;k<n;k++)e+=P[k]*(t[k]-t[k-1]);o.energyWh=e/3600;return o;}

/* ---------- calibration ---------- */
function solve(A,b){const n=b.length,M=A.map((r,i)=>[...r,b[i]]);for(let i=0;i<n;i++){let p=i;for(let r=i+1;r<n;r++)if(Math.abs(M[r][i])>Math.abs(M[p][i]))p=r;if(Math.abs(M[p][i])<1e-12)return null;[M[i],M[p]]=[M[p],M[i]];for(let r=i+1;r<n;r++){const f=M[r][i]/M[i][i];for(let c=i;c<=n;c++)M[r][c]-=f*M[i][c];}}const x=new Array(n);for(let i=n-1;i>=0;i--){let s=M[i][n];for(let c=i+1;c<n;c++)s-=M[i][c]*x[c];x[i]=s/M[i][i];}return x;}
function lsq(X,y){const n=X[0].length,A=Array.from({length:n},()=>new Array(n).fill(0)),b=new Array(n).fill(0);for(let k=0;k<X.length;k++)for(let i=0;i<n;i++){b[i]+=X[k][i]*y[k];for(let j=0;j<n;j++)A[i][j]+=X[k][i]*X[k][j];}const w=solve(A,b);if(!w)return null;let ss=0;for(let k=0;k<X.length;k++){let q=0;for(let i=0;i<n;i++)q+=w[i]*X[k][i];ss+=(y[k]-q)**2;}const rms=Math.sqrt(ss/Math.max(1,X.length-n));return{w,rms};}
function detrend(arr,idx,tc){const n=idx.length;let ma=0;for(const k of idx)ma+=arr[k];ma/=n;let sxy=0,sxx=0;for(const k of idx){sxy+=tc[k]*(arr[k]-ma);sxx+=tc[k]*tc[k];}const b=sxx>0?sxy/sxx:0;return idx.map(k=>arr[k]-ma-b*tc[k]);}
function fitLog(t0,v0,i0,speed,mass,trans,groups,smooth='auto'){
 const n=t0.length;if(n<30)return{ok:false,rows:n,notes:['Only '+n+' usable rows. Provide at least 30.']};
 const T=Array.from(t0,x=>x-t0[0]),v=Array.from(v0);let i=Array.from(i0);const notes=[];
 const M=typeof mass==='number'?new Array(n).fill(mass):Array.from(mass);
 if(median(i)<0){i=i.map(x=>-x);notes.push('Current was negative on average, so its sign was flipped (discharge positive).');}
 const out={ok:true,rows:n,R:null,Rse:null,base:null,eta:null,crr:null,powerRms:null,notes};
 const gmap=new Map();{let seg=0;for(let k=0;k<n;k++){if(k&&T[k]-T[k-1]>5)seg++;const g=(groups?groups[k]:'0')+'#'+seg;if(!gmap.has(g))gmap.set(g,[]);gmap.get(g).push(k);}}
 let sii=0,svi=0,nGroups=0;const vr=new Array(n).fill(0),ir=new Array(n).fill(0);
 for(const idx of gmap.values()){if(idx.length<5)continue;nGroups++;const tm=mean(idx.map(k=>T[k])),tc=new Array(n);for(const k of idx)tc[k]=T[k]-tm;
  const a=detrend(v,idx,tc),b=detrend(i,idx,tc);idx.forEach((k,j)=>{vr[k]=a[j];ir[k]=b[j];});}
 for(let k=0;k<n;k++){sii+=ir[k]*ir[k];svi+=vr[k]*ir[k];}
 if(sii>1e-9){const R=-svi/sii;let ss=0;for(let k=0;k<n;k++)ss+=(vr[k]+R*ir[k])**2;const rms=Math.sqrt(ss/Math.max(1,n-2*nGroups-1)),Rse=rms/Math.sqrt(sii);
  if(R<=0||!(Rse<.5*R))notes.push('Pack resistance is not well determined by this log (estimate '+R.toFixed(3)+' ± '+Rse.toFixed(3)+' Ω). Log a run with larger current swings, for example a hard acceleration with a heavy load.');
  else{out.R=R;out.Rse=Rse;}}else notes.push('Current is constant in this log, so pack resistance cannot be fitted.');
 if(speed&&speed.some(Number.isFinite)){const sp=Array.from(speed,x=>Number.isFinite(x)?x:0),{sm,a}=smoothSpeedAccel(T,sp,.1,smooth),X=[],y=[];
  for(let k=0;k<n;k++){const ap=a[k]>.1?a[k]:0;X.push([1,M[k]*sm[k],M[k]*ap*sm[k]]);y.push(v[k]*i[k]-trans*ap);}
  const pf=lsq(X,y);if(pf&&pf.w[2]>1e-6&&pf.w[1]>0){const eta=1/pf.w[2];out.base=pf.w[0];out.eta=eta;out.crr=pf.w[1]*eta/G;out.powerRms=pf.rms;
   if(eta<.2||eta>1)notes.push('Fitted efficiency is outside 0.2 to 1.0, so treat the drivetrain fit as unreliable (check speed units and mass).');}
  else notes.push('Speed and power are too weakly related to separate drivetrain efficiency from rolling resistance. Log a run with acceleration and cruise.');}
 else{const ps=v.map((x,k)=>x*i[k]).sort((a,b)=>a-b);out.base=mean(ps.slice(0,Math.max(3,n>>3)));notes.push('No speed column: base load is estimated from the lowest 12% of power samples.');}
 out.ok=out.R!=null||out.base!=null;return out;}
const applyFit=(f,p)=>{const q={...p};if(f.R!=null)q.resistance=f.R;if(f.base!=null)q.base=f.base;if(f.eta!=null)q.eta=f.eta;if(f.crr!=null)q.crr=f.crr;return q;};

/* ---------- supervisor ---------- */
class PowerSupervisor{
 constructor(p,o={}){this.p=p;this.lim=limits(p.cells);this.staleS=o.staleS??.5;this.vocAlpha=o.vocAlpha??.3;this.releaseMargin=o.releaseMargin??.05;
  this.rOnline=o.rOnline??true;this.rTau=o.rTauS??30;this.rMax=(o.rMaxFactor??3)*p.resistance;this.stat=null;this.rUsed=p.resistance;this.voc=null;this.tSample=null;this.state='ok';}
 updateR(now,v,i){const st=this.stat;if(!st){this.stat={t:now,mv:v,mi:i,vi:0,ii:0};return;}
  const a=1-Math.exp(-Math.max(now-st.t,1e-3)/this.rTau),dv=v-st.mv,di=i-st.mi;st.mv+=a*dv;st.mi+=a*di;st.ii=(1-a)*(st.ii+a*di*di);st.vi=(1-a)*(st.vi+a*dv*di);st.t=now;
  if(st.ii>1&&st.vi<0){this.rUsed=Math.min(this.rMax,Math.max(this.p.resistance,-st.vi/st.ii));}else this.rUsed=Math.max(this.p.resistance,this.rUsed*.999);}
 update(now,v,i){if(!(v===v&&i===i)||v<=0)return;if(this.rOnline)this.updateR(now,v,i);const voc=v+Math.max(0,i)*this.rUsed;this.voc=this.voc==null?voc:this.vocAlpha*voc+(1-this.vocAlpha)*this.voc;this.tSample=now;}
 decide(now,vReq,vNow,dt,payload=0){const p=this.p;
  if(this.voc==null||this.tSample==null||now-this.tSample>this.staleS){this.state='stale';return{state:'stale',speedLimit:0,powerCap:0,vocEst:this.voc||0,limited:true,reason:'battery data missing or older than '+this.staleS.toFixed(1)+' s'};}
  const mass=p.mass+payload,cap=powerCap(this.voc,this.rUsed,p.imax,this.lim.guard),eff=this.state==='limiting'?cap*(1-this.releaseMargin):cap;dt=Math.max(dt,1e-3);
  const wanted=clamp(vReq,vNow-p.dec*dt,vNow+p.acc*dt),need=drawPower(wanted,(wanted-vNow)/dt,mass,p);
  if(need<=eff){this.state='ok';return{state:'ok',speedLimit:wanted,powerCap:cap,vocEst:this.voc,need,limited:false,reason:'demand '+need.toFixed(0)+' W within '+cap.toFixed(0)+' W'};}
  const allowed=limitSpeed(wanted,vNow,dt,mass,p,eff);
  if(drawPower(allowed,(allowed-vNow)/dt,mass,p)>eff+1e-4){this.state='park';return{state:'park',speedLimit:0,powerCap:cap,vocEst:this.voc,need,limited:true,reason:'idle load exceeds '+cap.toFixed(0)+' W cap; charge before continuing'};}
  this.state='limiting';return{state:'limiting',speedLimit:allowed,powerCap:cap,vocEst:this.voc,need,limited:true,reason:'demand '+need.toFixed(0)+' W above '+eff.toFixed(0)+' W cap'};}}

/* ---------- logs ---------- */
function parseLog(text){
 const lines=text.split(/\r?\n/).map(l=>l.trim()).filter(l=>l&&!l.startsWith('#'));if(lines.length<3)throw new Error('Need a header and at least two data rows.');
 const delim=[',',';','\t'].sort((a,b)=>lines[0].split(b).length-lines[0].split(a).length)[0],split=l=>l.split(delim).map(s=>s.trim().replace(/^"|"$/g,''));
 const head=split(lines[0]).map(h=>h.toLowerCase()),rx={t:/^(t|time|time_s|timestamp|sec|seconds)$/,v:/^(v|volt|volts|voltage|vbat|v_bat|battery_v|voltage_v|bus_v)$/,i:/^(i|a|amp|amps|current|ibat|i_bat|battery_a|current_a|bus_a)$/,sp:/^(speed|vel|velocity|v_mps|speed_mps|odom_v|linear_x|speed_m_s)$/,run:/^(run|run_id|trip|trip_id)$/};
 const ci={};for(const k in rx)ci[k]=head.findIndex(h=>rx[k].test(h));
 if(ci.t<0||ci.v<0||ci.i<0)throw new Error('Columns not recognised. Use headers like time_s, voltage_V, current_A, speed_mps, run.');
 const rows=[];for(const l of lines.slice(1)){const p=split(l),r={t:+p[ci.t],v:+p[ci.v],i:+p[ci.i],sp:ci.sp>=0?+p[ci.sp]:NaN,run:ci.run>=0?(p[ci.run]||''):null};if(isFinite(r.t)&&isFinite(r.v)&&isFinite(r.i))rows.push(r);}
 rows.sort((a,b)=>a.t-b.t);
 return{t:rows.map(r=>r.t),v:rows.map(r=>r.v),i:rows.map(r=>r.i),speed:ci.sp>=0?rows.map(r=>r.sp):null,run:ci.run>=0?rows.map(r=>r.run):null};}
function splitRuns(log,o={}){
 const idleGap=o.idleGapS??60,gap=o.gapS??30,still=o.still??STILL,n=log.t.length,bounds=[];
 if(log.run){let st=0;for(let k=1;k<=n;k++){if(k===n||log.run[k]!==log.run[st]){if(log.run[st]!=='')bounds.push([st,k,String(log.run[st])]);st=k;}}}
 else{let st=0,stillSince=null;for(let k=1;k<n;k++){let cut=false;if(log.t[k]-log.t[k-1]>gap)cut=true;else if(log.speed){if(Math.abs(log.speed[k])<still){if(stillSince==null)stillSince=log.t[k];if(log.t[k]-stillSince>=idleGap)cut=true;}else stillSince=null;}if(cut){bounds.push([st,k,String(bounds.length+1)]);st=k;stillSince=null;}}bounds.push([st,n,String(bounds.length+1)]);}
 const runs=[];for(const[a,b,id]of bounds){if(b-a<10)continue;if(log.speed){let mv=false;for(let k=a;k<b;k++)if(Math.abs(log.speed[k]||0)>=still){mv=true;break;}if(!mv)continue;}
  runs.push({id,t:log.t.slice(a,b).map(x=>x-log.t[a]),v:log.v.slice(a,b),i:log.i.slice(a,b),speed:log.speed?log.speed.slice(a,b):null});}
 return runs;}
class RunTracker{constructor(idleGap=60,still=.02){this.idleGap=idleGap;this.still=still;this.no=0;this.active=false;this.stillSince=null;}
 update(now,sp){if(Math.abs(sp)>=this.still){this.stillSince=null;if(!this.active){this.no++;this.active=true;}}else if(this.active){if(this.stillSince==null)this.stillSince=now;else if(now-this.stillSince>=this.idleGap){this.active=false;this.stillSince=null;}}return String(this.active?this.no:this.no+1);}}

/* ---------- shadow replay ---------- */
function events(t,v,th,minS=EVENT_MIN_S){let st=null;for(let k=0;k<t.length;k++){if(v[k]<th){if(st==null)st=t[k];if(t[k]-st>=minS-1e-9)return{hit:true,at:st};}else st=null;}return{hit:false,at:null};}
function trapz(t,v,i){let e=0;for(let k=1;k<t.length;k++)e+=v[k]*i[k]*(t[k]-t[k-1]);return e/3600;}
function scoreRun(run,plan,p,warnMargin=0,restWindow=5){
 const lim=limits(p.cells),s={id:run.id,ok:false,note:'',motionS:0,predS:0,vRest:NaN,vRmsePlan:NaN,vRmseModel:NaN,eMeas:NaN,ePlan:NaN,eModel:NaN,eErrPlan:NaN,eErrModel:NaN,vminMeas:NaN,vminPlan:NaN,event:false,hardEvent:false,warn:false,wouldLimitS:0,lead:null,trace:null};
 if(!run.speed){s.note='no speed column, cannot find the start of motion';return s;}
 let k0=-1,k1=-1,cnt=0;for(let k=0;k<run.t.length;k++)if(Math.abs(run.speed[k])>=STILL){if(k0<0)k0=k;k1=k+1;cnt++;}
 if(cnt<5){s.note='robot did not move in this run';return s;}
 const t0=run.t[k0],t=run.t.slice(k0,k1).map(x=>x-t0),v=run.v.slice(k0,k1),i=run.i.slice(k0,k1),sp=run.speed.slice(k0,k1);
 const pre=[];for(let k=0;k<run.t.length;k++)if(run.t[k]>=t0-restWindow&&run.t[k]<t0&&run.i[k]>-.5)pre.push(k);
 if(!pre.length){s.note='no resting samples before motion (log at least '+restWindow+' s before departure)';return s;}
 const vRest=median(pre.map(k=>run.v[k])),iRest=Math.max(0,median(pre.map(k=>run.i[k])));
 s.ok=true;s.motionS=t[t.length-1];s.vRest=vRest;s.eMeas=trapz(t,v,i);s.vminMeas=Math.min(...v);
 const ev=events(t,v,lim.guard);s.event=ev.hit;s.hardEvent=events(t,v,lim.vmin).hit;
 const mod=predictFromSpeed(t,sp,p,vRest,iRest,plan?plan.payload_kg:0);s.eModel=mod.energyWh;s.vRmseModel=Math.sqrt(mean(mod.voltage.map((x,k)=>(x-v[k])**2)));s.eErrModel=100*(mod.energyWh-s.eMeas)/Math.max(s.eMeas,1e-9);
 let pred=null;if(plan){pred=predictPlan(plan,p,vRest,iRest);s.predS=pred.duration;s.ePlan=pred.energyWh;s.vminPlan=pred.minV;s.warn=pred.minV<lim.guard+warnMargin;
  const vp=[];let q=0;for(let k=0;k<t.length;k++){if(t[k]>pred.duration)break;while(q<pred.t.length-2&&pred.t[q+1]<t[k])q++;const a=pred.t[q],b=pred.t[q+1]??a,f=b>a?clamp((t[k]-a)/(b-a),0,1):0;vp.push(pred.voltage[q]+(pred.voltage[q+1]!==undefined?(pred.voltage[q+1]-pred.voltage[q])*f:0));}
  if(vp.length)s.vRmsePlan=Math.sqrt(mean(vp.map((x,k)=>(x-v[k])**2)));s.eErrPlan=100*(pred.energyWh-s.eMeas)/Math.max(s.eMeas,1e-9);}else s.note='no plan for this run: forecast skipped';
 const sup=new PowerSupervisor(p);let first=null,vPrev=0;const stA=[],limA=[];const SI={ok:0,limiting:1,park:2,stale:3};
 for(let k=0;k<t.length;k++){sup.update(t[k],v[k],i[k]);const dt=k?t[k]-t[k-1]:.1,d=sup.decide(t[k],sp[k],vPrev,Math.max(dt,1e-3),plan?plan.payload_kg:0);
  stA.push(d.limited&&d.state!=='stale'&&d.speedLimit>=sp[k]-1e-3?0:SI[d.state]);limA.push(d.speedLimit);
  if(d.limited&&d.state!=='stale'&&d.speedLimit<sp[k]-1e-3){s.wouldLimitS+=Math.max(dt,0);if(first==null)first=t[k];}vPrev=sp[k];}
 if(first!=null&&ev.at!=null)s.lead=ev.at-first;
 s.trace={t,v,i,sp,st:stA,lim:limA,pt:pred?pred.t:[],pv:pred?pred.voltage:[],guard:lim.guard,vmin:lim.vmin,evAt:ev.at,limAt:first,legs:plan?plan.legs.map(l=>l.distance_m):[],payload:plan?plan.payload_kg:0};
 return s;}
function summarise(scores,nRuns){
 const ok=scores.filter(s=>s.ok),pl=ok.filter(s=>Number.isFinite(s.ePlan));
 const tp=pl.filter(s=>s.event&&s.warn).length,fp=pl.filter(s=>!s.event&&s.warn).length,fn=pl.filter(s=>s.event&&!s.warn).length,tn=pl.filter(s=>!s.event&&!s.warn).length;
 const leads=ok.filter(s=>s.lead!=null&&s.lead>.5).map(s=>s.lead),evRuns=ok.filter(s=>s.event),covered=evRuns.filter(s=>s.lead!=null&&s.lead>=-.5).length,cav=[];
 if(ok.length<20)cav.push('Only '+ok.length+' scored runs: percentiles and warning rates are indicative, not tight.');
 if(tp+fn===0)cav.push('No low-voltage events occurred in these runs, so early-warning recall cannot be assessed.');
 if(tp+fn&&tp+fn<5)cav.push('Fewer than 5 real events: recall is very uncertain.');
 return{nRuns,nScored:ok.length,vRmsePlanMedian:pct(pl.map(s=>s.vRmsePlan),50),vRmsePlanP90:pct(pl.map(s=>s.vRmsePlan),90),vRmseModelMedian:pct(ok.map(s=>s.vRmseModel),50),
  eErrPlanAbsMedian:pct(pl.map(s=>Math.abs(s.eErrPlan)),50),eErrPlanAbsP90:pct(pl.map(s=>Math.abs(s.eErrPlan)),90),eErrModelAbsMedian:pct(ok.map(s=>Math.abs(s.eErrModel)),50),eErrPlanBias:pl.length?mean(pl.map(s=>s.eErrPlan)):NaN,
  tp,fp,fn,tn,precision:tp+fp?tp/(tp+fp):null,recall:tp+fn?tp/(tp+fn):null,meanLead:leads.length?mean(leads):null,supActive:covered,supEvents:evRuns.length,commandsSent:0,caveats:cav};}
function replay(runs,plans,p,defaultPlan=null,warnMargin=0){const scores=runs.map(r=>scoreRun(r,plans[r.id]||defaultPlan,p,warnMargin));return{scores,summary:summarise(scores,runs.length)};}
function massVector(log,plans,baseMass,defaultTotal){if(!log.run||!plans||!Object.keys(plans).length)return{mass:defaultTotal,groups:log.run};const m=log.t.map((_,k)=>{const pl=plans[log.run[k]];return pl?baseMass+pl.payload_kg:defaultTotal;});return{mass:m,groups:log.run};}
function parsePlans(text){const d=JSON.parse(text),runs=d.runs||d,out={};for(const k in runs){const r=runs[k];out[String(k)]={payload_kg:+(r.payload_kg||0),legs:r.legs.map(l=>({distance_m:+l.distance_m,dwell_s:+(l.dwell_s||0),vmax:l.vmax||null}))};}return out;}

/* ---------- synthetic data (NOT hospital data) ---------- */
function rng(seed){let a=seed>>>0;const u=()=>{a|=0;a=a+0x6D2B79F5|0;let t=Math.imul(a^a>>>15,1|a);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};
 return{u,uni:(lo,hi)=>lo+(hi-lo)*u(),int:(lo,hi)=>lo+Math.floor(u()*(hi-lo+1)),norm:(m,s)=>m+s*Math.sqrt(-2*Math.log(u()+1e-12))*Math.cos(2*Math.PI*u()),pick:a=>a[Math.floor(u()*a.length)]};}
const TRUTH={eta:.62,crr:.028,base:58,trans:38,capAh:2.15,rBase:.12,cells:6};
function speedTrace(r,legs,vmax,acc,dec){const sp=[];let v=0;for(const[dist,dwell]of legs){const stops=Array.from({length:r.pick([0,0,1,2])},()=>r.uni(.15,.85)*dist).sort((a,b)=>a-b),targets=[...stops.map(s=>[s,r.uni(3,9)]),[dist,dwell]];let pos=0;
  for(const[goal,wait]of targets){const cruise=vmax*r.uni(.92,1);while(pos<goal-.01){const rem=goal-pos,tgt=Math.min(cruise,Math.sqrt(2*dec*rem));v=Math.min(Math.max(tgt,v-dec*DT),v+acc*DT);v=Math.max(v,Math.min(.05,rem/DT));pos+=v*DT;sp.push(v);}v=0;sp.push(0);for(let k=0;k<Math.floor(wait/DT);k++)sp.push(0);}}return sp;}
function makeDay(seed,nRuns,o={}){const r=rng(seed),payR=o.payload||[30,150],socR=o.soc||[.22,.9],legs=o.legs||[[60,12],[60,0]],preS=10,postS=75,rScale=o.rScale||[.9,1.8],tr=new RunTracker(60);
 const log={t:[],v:[],i:[],speed:[],run:[]},plans={};let t=0,soc=r.uni(...socR);const kc=1;
 for(let run=1;run<=nRuns;run++){const payload=r.int(payR[0],payR[1]);if(run%3===1)soc=r.uni(...socR);const R=TRUTH.rBase*r.uni(...rScale),mass=22+payload,trace=speedTrace(r,legs,.65,.72,.88);let aux=0,vprev=0;
  const seq=[...new Array(Math.floor(preS/DT)).fill(0),...trace,...new Array(Math.floor(postS/DT)).fill(0)];
  for(const spd of seq){const a=(spd-vprev)/DT,ap=Math.max(0,a);aux=.98*aux+r.norm(0,.8);const pl=TRUTH.base+aux+TRUTH.crr*mass*G*spd/TRUTH.eta+mass*ap*spd/TRUTH.eta+TRUTH.trans*ap,voc=(19.8+5.4*soc+.4*Math.sin(Math.PI*soc))*kc;
   const d=voc*voc-4*R*pl;let i,vt;if(d<0){i=voc/(2*R);vt=voc/2;}else{i=2*pl/(voc+Math.sqrt(d));vt=voc-i*R;}
   soc=Math.max(0,soc-i*DT/(TRUTH.capAh*3600));const rid=tr.update(t,spd);log.t.push(t);log.v.push(vt+r.norm(0,.02));log.i.push(i+r.norm(0,.05));log.speed.push(Math.max(0,spd+r.norm(0,.004)));log.run.push(rid);vprev=spd;t+=DT;}
  t+=run%3===0?600:5;plans[String(run)]={payload_kg:payload,legs:legs.map(([d,w])=>({distance_m:d,dwell_s:w,vmax:null}))};}
 return{log,plans};}

/* ---------- report ---------- */
const f=(x,nd=2,u='')=>x==null||!Number.isFinite(x)?'n/a':x.toFixed(nd)+u;
function reportMarkdown(scores,s,p,title,synthetic,targets={}){const lim=limits(p.cells),L=[];
 const verdict=(k,val)=>{const t=targets[k];if(t==null||!Number.isFinite(t)||!Number.isFinite(val))return'target to agree';return val<=t?'within target':'outside target ('+t+')';};
 L.push('# '+title,'');if(synthetic)L.push('> **Synthetic data.** These runs were generated by software, not recorded on a robot. They show the pipeline works. They say nothing about a real robot or a real hospital.','');
 L.push('Runs in log: '+s.nRuns+'. Runs scored: '+s.nScored+'. Robot commands sent: '+s.commandsSent+' (shadow mode is read-only).','');
 L.push('Profile: '+p.cells+' cells, guard '+lim.guard.toFixed(2)+' V, hard limit '+lim.vmin.toFixed(2)+' V, resistance '+p.resistance.toFixed(3)+' ohm, efficiency '+p.eta.toFixed(2)+', rolling resistance '+p.crr.toFixed(3)+', base load '+p.base.toFixed(0)+' W.','');
 L.push('## Results against the four success criteria','','| Measure | Result | Status |','| --- | --- | --- |',
  '| Voltage prediction error (forecast from plan) | median '+f(s.vRmsePlanMedian,3,' V')+', 90th percentile '+f(s.vRmsePlanP90,3,' V')+' RMS | '+verdict('vRmsePlanMedian',s.vRmsePlanMedian)+' |',
  '| Voltage error (model driven by the real speed) | median '+f(s.vRmseModelMedian,3,' V')+' RMS | diagnostic |',
  '| Energy prediction error (forecast from plan) | median '+f(s.eErrPlanAbsMedian,1,'%')+' absolute, 90th percentile '+f(s.eErrPlanAbsP90,1,'%')+', mean bias '+f(s.eErrPlanBias,1,'%')+' | '+verdict('eErrPlanAbsMedian',s.eErrPlanAbsMedian)+' |',
  '| Energy error (model driven by the real speed) | median '+f(s.eErrModelAbsMedian,1,'%')+' absolute | diagnostic |',
  '| Early warning | caught '+s.tp+' of '+(s.tp+s.fn)+' real events; false alarms: '+s.fp+'; quiet runs correctly quiet: '+s.tn+' | target to agree |',
  '| Precision / recall | '+f(s.precision,2)+' / '+f(s.recall,2)+' | target to agree |',
  '| Supervisor coverage | would already be limiting at the start of '+s.supActive+' of '+s.supEvents+' real events; mean advance warning '+f(s.meanLead,1,' s')+' where it was more than 0.5 s | diagnostic |',
  '| No operational impact | no commands sent; the replay only reads the log | met by design |','',
  'Forecast means the prediction made from the route plan and the resting voltage alone, before the run. The model-driven rows replay the same battery model using the speed the robot really drove, so a gap between the two shows how much error comes from the plan (waits, detours) rather than the battery model.','');
 if(s.caveats.length){L.push('## Caveats','');s.caveats.forEach(c=>L.push('- '+c));L.push('');}
 const bad=scores.filter(x=>!x.ok);if(bad.length){L.push('## Runs that could not be scored','');bad.forEach(x=>L.push('- Run '+x.id+': '+x.note));L.push('');}
 return L.join('\n');}
return{DEFAULT,limits,ocv,socFromOcv,usableWh,supply,drawPower,powerCap,limitSpeed,predictPlan,predictFromSpeed,smoothSpeedAccel,fitLog,applyFit,PowerSupervisor,parseLog,splitRuns,RunTracker,scoreRun,summarise,replay,massVector,parsePlans,makeDay,reportMarkdown,median,pct};
})();
if(typeof module!=='undefined')module.exports=ShadowCore;
