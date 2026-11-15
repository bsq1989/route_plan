"""Render standalone HTML from the same record exported as JSON."""

import json


def write_replay(path, record):
    encoded = json.dumps(record, ensure_ascii=False, allow_nan=False).replace("<", "\\u003c")
    path.write_text(PAGE.replace("__RECORD__", encoded), encoding="utf-8")


PAGE = """<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>定向接近与分区拾取</title>
<style>
:root{font-family:"PingFang SC","Microsoft YaHei",system-ui,sans-serif;color:#162c2a;background:#edf1ed}
body{max-width:1100px;margin:0 auto;padding:clamp(16px,4vw,44px)}
header{display:flex;align-items:baseline;justify-content:space-between;gap:18px;margin-bottom:20px}
h1{font-size:clamp(22px,4vw,39px);letter-spacing:-.035em;margin:0}
small,.sub{color:#526661} main{display:grid;grid-template-columns:minmax(0, 1.8fr) minmax(240px,1fr);gap:14px}
.card{background:#fff;border:1px solid #dce5df;border-radius:16px;padding:16px}
canvas{display:block;width:100%;height:auto;background:#f8faf8;border-radius:10px}
.legend{display:flex;flex-wrap:wrap;gap:8px 14px;font-size:13px;color:#415752;margin-top:12px}
i{display:inline-block;width:.7em;height:.7em;margin-right:4px;border-radius:50%;background:var(--color)}
.controls{display:flex;align-items:center;flex-wrap:wrap;gap:8px;margin-top:12px}
button{border:1px solid #baccbf;border-radius:7px;background:#f3f7f3;padding:7px 11px;color:inherit;cursor:pointer}
#play{background:#174c3e;color:white} input{min-width:80px;flex:1;accent-color:#226957}
aside h2{font-size:19px;margin:0 0 8px} aside p{font-size:14px;line-height:1.55;margin:8px 0}
dt{font-size:12px;color:#51625d}dd{margin:3px 0 13px;font-variant-numeric:tabular-nums}
@media(max-width:720px){main{grid-template-columns:1fr}header{display:block}}
</style>
<header><h1>定向接近与分区拾取</h1><small id="scenario"></small></header>
<main>
<section class="card">
<canvas width="640" height="560" aria-label="map 坐标轨迹、发现扇区和拾取圆环"></canvas>
<div class="legend">
<span><i style="--color:#4b91ba"></i>发现扇区</span><span><i style="--color:#48a383"></i>拾取圆环</span>
<span><i style="--color:#885ab9"></i>auto_fence</span><span><i style="--color:#c17319"></i>方位线索</span>
<span><i style="--color:#116659"></i>精测</span><span><i style="--color:#db4f41"></i>当批</span>
<span><i style="--color:#889589"></i>已拾取</span><span><i style="--color:#c7d2c9"></i>场景真值</span>
</div>
<div class="controls"><button id="play">播放</button><button id="prev" aria-label="上一帧">←</button><button id="next" aria-label="下一帧">→</button><input id="position" type="range" min="0" value="0" aria-label="回放进度"></div>
</section>
<aside class="card">
<h2 id="stage"></h2><p class="sub" id="clock"></p>
<dl><dt>实测位姿 x / y / yaw</dt><dd id="pose"></dd><dt>选向理由</dt><dd id="choice"></dd>
<dt>本轮精测批次</dt><dd id="batch"></dd><dt>观测与拾取</dt><dd id="progress"></dd><dt>停止结果</dt><dd id="result"></dd></dl>
</aside>
</main>
<script id="record" type="application/json">__RECORD__</script>
<script>
const data = JSON.parse(document.querySelector("#record").textContent);
const get = id => document.getElementById(id);
const canvas = document.querySelector("canvas"), ctx = canvas.getContext("2d");
const slider = get("position"), frames = data.trace, regions = data.regions;
const custom = data.scene_config.custom, world = data.scene_config.world;
const [ox, oy] = world.offset || [0,0];
const fence = custom.fence_enabled !== false && custom.fence
  ? custom.fence : [[ox,oy],[ox+world.width,oy],[ox+world.width,oy+world.height],[ox,oy+world.height]];
const xmin = Math.min(...fence.map(p=>p[0])), xmax = Math.max(...fence.map(p=>p[0]));
const ymin = Math.min(...fence.map(p=>p[1])), ymax = Math.max(...fence.map(p=>p[1]));
const scale = Math.min(530/(xmax-xmin), 470/(ymax-ymin));
const xy = (x,y) => [320+(x-(xmin+xmax)/2)*scale, 280-(y-(ymin+ymax)/2)*scale];
const dims = data.initial.dimensions, targets = data.initial.remaining;
const labels = {initial:"初始发现", turn:"原地转向", approach:"定向渐近", pickup:"执行局部计划", pickup_stop:"停靠拾取"};
const reasons = {all_collected:"已全清", no_observed_targets:"没有观测到剩余物品",
  unresolved_cluster_hints:"仍有未确认的簇线索", incomplete_final_scan:"末尾环视受阻",
  unresolved_precise_targets:"仍有未拾取的精测目标",
  no_precise_batch:"当前没有可拾取的精测批次", no_safe_heading:"没有可安全执行的候选朝向",
  round_limit:"达到轮数上限", pickup_conflict:"停靠点的实际覆盖与批次不符",
  "Map boundary":"场地触边", "Motion circle boundary":"超出 auto_fence"};
get("scenario").textContent = data.case + " · 种子 " + data.seed;
slider.max = frames.length-1;
function path(points, fill, stroke, close=true){
  ctx.beginPath();
  points.forEach((point,i) => i ? ctx.lineTo(...xy(...point)) : ctx.moveTo(...xy(...point)));
  if(close) ctx.closePath();
  if(fill){ctx.fillStyle=fill;ctx.fill()}
  if(stroke){ctx.strokeStyle=stroke;ctx.stroke()}
}
function disk(x,y,r,fill,stroke){
  ctx.beginPath();ctx.arc(...xy(x,y),r*scale,0,2*Math.PI);
  if(fill){ctx.fillStyle=fill;ctx.fill()}
  if(stroke){ctx.strokeStyle=stroke;ctx.stroke()}
}
function wedge(x,y,yaw,angle,inner,outer,color){
  const a=angle*Math.PI/360, [px,py]=xy(x,y);
  ctx.beginPath();ctx.arc(px,py,outer*scale,-yaw-a,-yaw+a);
  if(inner)ctx.arc(px,py,inner*scale,-yaw+a,-yaw-a,true);
  else ctx.lineTo(px,py);
  ctx.closePath();ctx.fillStyle=color;ctx.fill();
}
function rect(pose,x0,y0,x1,y1,fill,stroke){
  const [x,y,a]=pose,c=Math.cos(a),s=Math.sin(a);
  path([[x0,y0],[x1,y0],[x1,y1],[x0,y1]].map(([u,v])=>[x+c*u-s*v,y+s*u+c*v]),fill,stroke);
}
function render(){
  const index=+slider.value, frame=frames[index], [x,y,yaw]=frame.pose;
  const used=new Set(frame.collected_ids.map(String));
  const observation=data.observations.findLast(item=>item.trace_index<=index);
  const decision=data.decisions.findLast(item=>observation && item.frame_index<=observation.index);
  const candidate=decision?.selected;
  const plan=data.plans.findLast(item=>item.trace_index<=index);
  const batchIds=new Set(frame.auto_fence ? (plan?.batch_ids||[]).map(String) : []);
  const batchPoints=new Set(frame.auto_fence ? (plan?.request?.targets||[]).map(p=>`${p.x},${p.y}`) : []);
  const pickedPoints=new Set([...used].map(id=>String(targets[id])));
  const pickedSinceObservation=observation && used.size!==frames[observation.trace_index].collected_ids.length;
  ctx.clearRect(0,0,640,560);ctx.lineWidth=1;
  path(fence,"#fff",null);ctx.save();ctx.clip();
  for(let px=Math.ceil(xmin*2)/2;px<xmax;px+=.5)path([[px,ymin],[px,ymax]],null,"#e8efea",false);
  for(let py=Math.ceil(ymin*2)/2;py<ymax;py+=.5)path([[xmin,py],[xmax,py]],null,"#e8efea",false);
  wedge(x,y,yaw,regions.view_angle_deg,0,regions.view_range_m,"#75adc359");
  wedge(x,y,yaw,regions.pickup_angle_deg,regions.pickup_inner_radius_m,regions.pickup_outer_radius_m,"#57b5958a");
  if(regions.cluster_radius_m){
    const active=decision?.rule==="cluster_density"?decision.selected:null;
    for(const cluster of observation?.clusters||[]){
      if(!cluster.pending && cluster.id!==active)continue;
      ctx.setLineDash(cluster.id===active?[6,4]:[2,4]);
      disk(...cluster.anchor_xy_m,regions.cluster_radius_m,null,cluster.id===active?"#ca751b":"#64748b");
    }
    ctx.setLineDash([]);
  }
  if(frame.auto_fence){ctx.setLineDash([7,4]);disk(...frame.auto_fence,null,"#885ab9");ctx.setLineDash([])}
  ctx.lineWidth=2;path(frames.slice(0,index+1).map(item=>item.pose.slice(0,2)),null,"#627a71",false);ctx.lineWidth=1;
  for(const [bx,by,r] of data.initial.obstacles||[]) disk(bx,by,r,"#44544d");
  for(const [id,point] of Object.entries(targets)){
    disk(...point,data.initial.target_diameter/2,used.has(id)?"#889589":"#c7d2c9");
  }
  for(const item of (observation?.known||[])){
    const id=String(item.id);
    if(item.quality==="bearing" ? pickedSinceObservation : pickedPoints.has(String(item.xy_m)))continue;
    const color=batchPoints.has(String(item.xy_m))?"#db4f41":item.quality==="precise"?"#116659":"#c17319";
    ctx.setLineDash(item.quality==="precise"?[]:[4,4]);
    if(item.quality==="bearing"){
      const [ox,oy]=item.origin_xy_m, angle=item.bearing_map_rad, span=regions.view_range_m;
      path([[ox,oy],[ox+span*Math.cos(angle),oy+span*Math.sin(angle)]],null,color,false);
      ctx.setLineDash([]);
      continue;
    }
    path([[x,y],item.xy_m],null,color,false);ctx.setLineDash([]);
    disk(...item.xy_m,.041,color,"#fff");ctx.fillStyle=color;
    const [px,py]=xy(...item.xy_m);ctx.font="13px system-ui";ctx.fillText("#"+id,px+6,py-6);
  }
  const l=dims.body_length/2,w=dims.body_width/2,m=dims.body_margin,col=custom.collector;
  const front=custom.body_front_x??l, rear=front-2*l;
  ctx.setLineDash([5,4]);rect(frame.pose,rear-m,-w-m,front+m,w+m,null,"#b3582d");ctx.setLineDash([]);
  rect(frame.pose,rear,-w,front,w,"#1b4f50","#fff");
  if(col.polygon){
    const c=Math.cos(yaw),s=Math.sin(yaw);
    path(col.polygon.map(([u,v])=>[x+c*u-s*v,y+s*u+c*v]),"#167e66","#fff");
  }else{
    rect(frame.pose,front+col.gap,col.lateral_offset-col.width/2,front+col.gap+col.depth,col.lateral_offset+col.width/2,"#167e66","#fff");
  }
  path([[x,y],[x+.26*Math.cos(yaw),y+.26*Math.sin(yaw)]],null,"#fff",false);
  ctx.restore();ctx.lineWidth=2;path(fence,null,"#72877c");
  ctx.fillStyle="#586f65";ctx.font="12px system-ui";ctx.fillText("map / m",24,30);
  get("stage").textContent=labels[frame.stage]||frame.stage;
  get("clock").textContent=frame.time_s.toFixed(2)+" s · "+(index+1)+" / "+frames.length+" 帧";
  get("pose").textContent=x.toFixed(3)+" m / "+y.toFixed(3)+" m / "+(yaw*180/Math.PI).toFixed(1)+"°";
  get("choice").textContent=decision?.rule==="cluster_final_scan" ? "原地环视检查" :
    decision?.rule==="cluster_density" ? "选中簇 C"+candidate :
    candidate ? "覆盖 "+candidate.ids.length+" 条连线 · 参考 #"+candidate.reference_id : "等待选向";
  get("batch").textContent=batchIds.size ? [...batchIds].map(id=>"#"+id).join("、") :
    plan ? "最近一批 "+plan.batch_ids.map(id=>"#"+id).join("、")+(plan.reason?" · "+(reasons[plan.reason]||plan.reason):"") : "等待停稳重测";
  get("progress").textContent="观测 "+(observation?observation.index+1:0)+" 帧 · 已拾取 "+used.size+" 件";
  get("result").textContent=index===frames.length-1
    ? (reasons[data.stop_reason]||data.stop_reason)+" · 剩余 "+Object.keys(data.remaining).length+" 件" : "回放进行中";
}
let timer=null;
function pause(){clearInterval(timer);timer=null;get("play").textContent="播放"}
function seek(index){slider.value=Math.min(frames.length-1,Math.max(0,index));render()}
get("play").onclick=()=>{
  if(timer!==null){pause();return}
  if(+slider.value===frames.length-1)seek(0);
  get("play").textContent="暂停";
  timer=setInterval(()=>{seek(+slider.value+1);if(+slider.value===frames.length-1)pause()},40);
};
get("prev").onclick=()=>{pause();seek(+slider.value-1)};
get("next").onclick=()=>{pause();seek(+slider.value+1)};
slider.oninput=()=>{pause();render()};
render();
</script>
</html>
"""
