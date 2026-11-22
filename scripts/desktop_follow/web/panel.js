'use strict';
const $ = id => document.getElementById(id);
let state = {}, shown = null, frozen = false, uv = null, rect = null, drag = null, busy = false, imgVersion = 0;
let deviceKey = '', loadedProfile = '', currentView = '';
let loadedAlertText;
let editingAlertText = false;
const msg = (text, error=false) => {
  $('message').textContent = text;
  $('message').className = error ? 'error' : '';
};
async function send(op, data={}) {
  const response = await fetch('/api/'+op, {method:'POST', headers:{'Content-Type':'application/json','X-Desk-Control':'1'},body:JSON.stringify(data)});
  const result = await response.json();
  if (!response.ok) throw Error(result.error || response.statusText);
  return result;
}
function handle(id, callback) {
  $(id).onclick = async () => {
    try { await callback(); await refresh(); }
    catch (e) { msg(e.message, true); }
  };
}
function frameData(needsPoint=true) {
  if (!shown || (needsPoint && !uv)) throw Error('请先在相机画面中取点');
  return {frame:shown, uv:uv && [1-uv[0], uv[1]]};
}
function selection() {
  if (!rect) throw Error('请先在画面内拖出矩形');
  return [1-rect[2], rect[1], 1-rect[0], rect[3]];
}
function drawRect(r) {
  $('rubber').hidden = !r;
  if (r) Object.assign($('rubber').style,{left:r[0]*100+'%',top:r[1]*100+'%',width:(r[2]-r[0])*100+'%',height:(r[3]-r[1])*100+'%'});
}
function clearPick() {
  uv = rect = null;
  $('pick').hidden = true;
  drawRect(null);
}
function live() {
  imgVersion++;
  frozen = false;
  shown = null;
  currentView = '';
  clearPick();
  $('freeze').textContent = '定格';
}
async function pin() {
  if (!shown) throw Error('请等待相机画面');
  imgVersion++;
  frozen = true;
  try { await send('freeze', {frame:shown}); }
  catch (e) { live(); throw e; }
  $('freeze').textContent = '恢复实时';
}
async function showFrame(id) {
  if (!id) { $('plane').hidden = true; $('empty').hidden = false; shown = null; return; }
  const view = $('view').value;
  if (shown === id && currentView === view) return;
  const version = ++imgVersion;
  const src = '/api/frame/'+id+'?mirror=1&depth='+view;
  const next = new Image();
  next.src = src;
  try { await next.decode(); } catch { return; }
  if (version !== imgVersion || drag) return;
  const old = $('image');
  next.id='image'; next.alt=old.alt; next.draggable=false;
  old.replaceWith(next);
  shown = id; currentView = view;
  $('empty').hidden = true; $('plane').hidden = false;
}
let savingSpeed = false;
let editingSpeed = false;
let pendingSpeed = null;
const speed = $('manual-speed');
let queuedGridTarget = null;
let sendingGridTarget = false;
let servoRevision = 0;
function applyServoReply(robot) {
  ++servoRevision;
  state.robot = robot;
  renderServos(robot);
}
function renderServos(robot = {}) {
  const online = [robot.x, robot.y, robot.targetX, robot.targetY, robot.minX, robot.maxX,
    robot.minY, robot.maxY].every(Number.isFinite) && robot.maxX > robot.minX && robot.maxY > robot.minY;
  const head = $('servo-head');
  const goal = $('servo-goal');
  const centerX = $('servo-center-x'), centerY = $('servo-center-y');
  centerX.hidden = centerY.hidden = !online;
  head.hidden = !online;
  goal.hidden = !online || !robot.moving;
  $('servo-wait').hidden = online;
  $('servo-map').disabled = !online;
  if (!online) queuedGridTarget = null;
  for (const button of document.querySelectorAll('.pad button')) button.disabled = !online;
  const supported = online && Number.isFinite(robot.manualSpeed);
  speed.disabled = !supported;
  if (!supported) {
    editingSpeed = false;
    pendingSpeed = null;
    $('speed-value').textContent = online ? '需更新固件' : '等待读取';
  } else if (!editingSpeed && !savingSpeed) {
    const hasRange = Number.isInteger(robot.manualSpeedMin) && Number.isInteger(robot.manualSpeedMax);
    speed.min = hasRange ? robot.manualSpeedMin : 5;
    speed.max = hasRange ? robot.manualSpeedMax : 20;
    speed.step = hasRange ? 1 : 5;
    speed.value = robot.manualSpeed;
    $('speed-value').textContent = speed.value + '°/秒';
  }
  $('motion-state').textContent = online
    ? ({manual:'手动', follow:'跟随', auto:'自动'}[robot.mode] + (robot.moving ? ' · 移动中' : ' · 已到位'))
    : '待连接';
  const angles = (x,y) => x + '° / ' + y + '°';
  const hasCenter = online && Number.isFinite(robot.centerOffsetX) && Number.isFinite(robot.centerOffsetY);
  const canSaveCenter = hasCenter && robot.mode === 'manual' && !robot.moving;
  $('save-center').disabled = !canSaveCenter;
  $('reset-center').disabled = !canSaveCenter || !(robot.centerOffsetX || robot.centerOffsetY);
  $('center-offsets').textContent = hasCenter ? '已保存偏移（水平 / 俯仰） ' + angles(robot.centerOffsetX, robot.centerOffsetY)
    : online ? '当前固件需升级后才能保存中位' : '等待读取机器人中位';
  $('servo-current').textContent = online ? angles(robot.x, robot.y) : '—';
  $('servo-target').textContent = online ? angles(robot.targetX, robot.targetY) : '—';
  $('position').textContent = $('servo-current').textContent;
  $('servo-range').hidden = !online;
  if (!online) return;
  $('servo-range').textContent = '从正中可转：左 ' + (90-robot.minX) + '°，右 ' + (robot.maxX-90)
    + '°，上 ' + (90-robot.minY) + '°，下 ' + (robot.maxY-90) + '°';
  const left = x => 100 * (x - robot.minX) / (robot.maxX - robot.minX) + '%';
  const top = y => 100 * (y - robot.minY) / (robot.maxY - robot.minY) + '%';
  centerX.style.left = left(90);
  centerY.style.top = top(90);
  Object.assign(head.style, {left:left(robot.x), top:top(robot.y)});
  Object.assign(goal.style, {left:left(robot.targetX), top:top(robot.targetY)});
}

function render(s) {
  if (typeof s.alert_text === 'string' && s.alert_text !== loadedAlertText) {
    loadedAlertText = s.alert_text;
    if (!editingAlertText) $('alert-text').value = loadedAlertText;
  }
  state = s;
  $('connection').textContent = [s.camera, s.model_error, s.reason].filter(Boolean).join(' · ');
  $('follow').textContent = s.active ? '停止跟随' : '开启跟随';
  $('follow').disabled = !s.active && (!s.frame || !s.profile || !!s.model_error);
  $('subject').textContent = s.alert_track != null ? '来人警戒 · #' + s.alert_track : s.status;
  $('subject').classList.toggle('alert', s.alert_track != null);
  $('method').textContent = s.method + (s.limit ? ' · 已到限位' : '');
  const r = s.robot;
  renderServos(r);
  if (!$('address').value && s.address) $('address').value = s.address;
  const key=JSON.stringify([s.devices,s.serial]);
  if (key !== deviceKey) {
    deviceKey=key;
    $('serial').replaceChildren(new Option('自动选择 RealSense D405',''), ...s.devices.map(d=>new Option(d.model+' · '+d.serial,d.serial)));
    if (s.serial && !s.devices.some(d=>d.serial===s.serial)) $('serial').add(new Option(s.serial,s.serial));
    $('serial').value=s.serial;
  }
  const profile=JSON.stringify([s.stream?.serial,s.stream?.modes,r.robot,s.trim,s.range]);
  if (profile!==loadedProfile) {
    loadedProfile=profile;
    [$('tx').value,$('ty').value]=s.trim;
    [$('near').value,$('far').value]=s.range || ['',''];
  }
  $('stream').textContent=s.stream ? s.stream.model+' · '+s.stream.modes[0]+' × '+s.stream.modes[1]+' · 深度单位 '+(s.stream.unit*1000).toFixed(3)+' mm' : '';
  $('owner').textContent=(s.owner ? '主人样本 '+s.owner+' 个' : '未录入主人 · 区域内任何人均可触发跟随') + (s.frame && s.people_count != null
    ? ' · 实时画面 '+s.people_count+' 人，'+s.ready_faces+' 张脸可录入' : '');
  $('direction-hint').hidden = !!s.owner || s.pose != null || Object.keys(s.regions).length > 0;
  $('gate').textContent=(s.roi?'自定义提醒区域':'全画面提醒')+' · '+(s.range?s.range.join(' – ')+' m':'不限距离');
  $('quality').textContent=s.pose==null?'等待底座标定':'重投影误差 '+s.pose.toFixed(2)+' px · R 右 / F 前 / U 上';
  $('regions').textContent=Object.entries(s.regions).map(([k,v])=>k+' '+v.count+' 点'+(v.rms==null?'':' · 误差 '+v.rms.toFixed(1)+'°')).join('，');
  $('names').replaceChildren(...Object.keys(s.regions).map(k=>new Option(k,k)));
  $('checks').textContent=s.checks.map(c=>c.truth+' m · '+(c.error==null?'无有效深度':'偏差 '+c.error+' m')+' · 有效 '+Math.round(c.valid*100)+'%'+(c.jitter==null?'':' · '+c.frames+' 帧波动 '+c.jitter.toFixed(3)+' m')).join('\n');
  $('freeze').disabled=!s.frame && !frozen;
}
async function refresh() {
  if (busy) return;
  busy=true;
  const revision = servoRevision;
  try {
    const response=await fetch('/api/state'); const s=await response.json();
    if (revision !== servoRevision) return;
    render(s);
    if (!frozen && !drag) await showFrame(s.frame);
  } catch(e) {
    if (revision !== servoRevision) return;
    $('connection').textContent='本机后台连接中断';
    renderServos();
  }
  finally {busy=false;}
}
$('servo-map').onclick = async event => {
  const map = event.currentTarget;
  if (map.disabled) return;
  const bounds = $('servo-travel').getBoundingClientRect();
  const robot = state.robot;
  function angle(pixel, origin, length, min, max) {
    const fraction = Math.max(0, Math.min(1, (pixel - origin) / length));
    return Math.round(min + fraction * (max - min));
  }
  queuedGridTarget = event.detail === 0 ? {center:true} : {
    x: angle(event.clientX, bounds.left, bounds.width, robot.minX, robot.maxX),
    y: angle(event.clientY, bounds.top, bounds.height, robot.minY, robot.maxY)
  };
  if (sendingGridTarget) return;
  sendingGridTarget = true;
  const error = $('servo-error');
  error.hidden = true;
  try {
    // Keep only the newest click while the previous command is awaiting acknowledgement.
    while (queuedGridTarget) {
      const target = queuedGridTarget;
      queuedGridTarget = null;
      const result = await send('manual', target);
      applyServoReply(result.robot);
    }
  } catch (e) {
    queuedGridTarget = null;
    error.textContent = e.message;
    error.hidden = false;
  } finally {
    sendingGridTarget = false;
    await refresh();
  }
};

const plane=$('plane');
function coordinates(e) {
  const b=plane.getBoundingClientRect();
  return [Math.max(0,Math.min(1,(e.clientX-b.left)/b.width)),Math.max(0,Math.min(1,(e.clientY-b.top)/b.height))];
}
function corners(a,b) {return [Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.max(a[0],b[0]),Math.max(a[1],b[1])];}
plane.onpointerdown=e=>{drag=coordinates(e); imgVersion++; plane.setPointerCapture(e.pointerId);};
plane.onpointermove=e=>{if(drag) drawRect(corners(drag,coordinates(e)));};
plane.onpointercancel=()=>{drag=null;drawRect(rect);};
plane.onpointerup=async e=>{
  if(!drag) return;
  const a=drag, b=coordinates(e); drag=null;
  try {
    if(!frozen) await pin();
    if(Math.hypot(a[0]-b[0],a[1]-b[1])>.025) {
      rect=corners(a,b);drawRect(rect);msg('已框选区域');return;
    }
    drawRect(rect);uv=b;$('pick').hidden=false;
    Object.assign($('pick').style,{left:b[0]*100+'%',top:b[1]*100+'%'});
    const m=await send('measure',frameData());
    $('measure').textContent=(m.m==null?'此点深度无效':m.m.toFixed(3)+' m')+' · 有效 '+Math.round(m.valid*100)+'%'+(m.mad==null?'':' · 波动 '+m.mad.toFixed(3)+' m');
  } catch(e){msg(e.message,true);}
};
document.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('[data-tab]').forEach(x=>{x.classList.toggle('active',x===b);$(x.dataset.tab).hidden=x!==b;});
});
if (location.hash === '#cal') document.querySelector('[data-tab="cal"]').click();
handle('freeze',async()=>{if(frozen)live();else await pin();});
$('view').onchange=()=>showFrame(shown);
handle('connect',async()=>{await send('connect',{address:$('address').value});live();msg('已连接机器人');});
handle('camera',async()=>{await send('camera',{serial:$('serial').value});live();msg('正在重新连接相机');});
handle('follow',async()=>{await send(state.active?'stop':'start');live();msg('跟随状态已更新');});
function action(id,op,data,label) {handle(id,async()=>{await send(op,data());msg(label);});}
handle('enroll',async()=>{
  await send('enroll',frameData(false));
  live();
  msg('主人样本已保存，可稍微转头再录入一次');
});
action('delete_owner','delete_owner',()=>({}),'主人特征已清除');
action('roi','roi',()=>({rect:selection()}),'提醒区域已保存');
action('whole','roi',()=>({rect:[0,0,1,1]}),'已恢复全画面');
action('range','range',()=>({range:($('near').value || $('far').value)?[Number($('near').value),Number($('far').value)]:null}),'距离设置已保存');
speed.oninput = () => {
  editingSpeed = true;
  $('speed-value').textContent = speed.value + '°/秒';
};
speed.onpointercancel = speed.onblur = () => {
  if (!editingSpeed) return;
  editingSpeed = false;
  if (!speed.disabled) renderServos(state.robot);
};
speed.onchange = async () => {
  editingSpeed = false;
  pendingSpeed = Number(speed.value);
  if (savingSpeed) return;
  savingSpeed = true;
  try {
    while (pendingSpeed !== null) {
      const value = pendingSpeed;
      pendingSpeed = null;
      const result = await send('speed', {value});
      if (result.robot) applyServoReply(result.robot);
    }
  } catch (e) {
    pendingSpeed = null;
    msg(e.message, true);
  } finally {
    savingSpeed = false;
    if (!speed.disabled) renderServos(state.robot);
    await refresh();
  }
};
action('center','manual',()=>({center:true}),'正在回中');
for (const [id,op] of [['save-center','save_center'], ['reset-center','reset_center']]) {
  $(id).onclick = async () => {
    try {
      const {boot,generation} = state.robot;
      const result = await send(op,{boot,generation});
      applyServoReply(result.robot);
      $('center-result').textContent = op === 'save_center' ? '已保存，试着转开后再回中复核。' : '已恢复出厂中位，点回中复核。';
    } catch (e) { $('center-result').textContent = e.message; }
    await refresh();
  };
}
for(const b of document.querySelectorAll('[data-dx],[data-dy]')) b.onclick=async()=>{try{await send('manual',{dx:Number(b.dataset.dx||0),dy:Number(b.dataset.dy||0)});await refresh();}catch(e){msg(e.message,true);}};
action('test','test',frameData,'已提交试转');
$('marker').oninput=()=>{$('print').href='/marker/'+Number($('marker').value)+'.png';};
action('pose','marker',()=>({...frameData(false),options:Object.fromEntries(['marker','size','yaw','right','forward','up'].map(k=>[k,Number($(k).value)]))}),'已计算底座姿态，恢复实时可查看坐标轴');
action('clear_pose','clear_pose',()=>({}),'三维标定已清除');
action('trim','trim',()=>({trim:[Number($('tx').value),Number($('ty').value)]}),'零偏已保存');
action('region','region',()=>({name:$('name').value,rect:selection()}),'已建立二维区域');
action('clear_region','clear_region',()=>({name:$('name').value}),'区域已清除');
action('sample','sample',()=>({...frameData(),name:$('name').value}),'采样点已保存');
action('check','check',()=>({...frameData(),truth:Number($('truth').value)}),'核验结果已记录');
$('alert-text').oninput = () => { editingAlertText = true; };
for (const [formId, inputId, resultId, op] of [
  ['alert-form', 'alert-text', 'alert-result', 'alert_text'],
  ['speak-form', 'speak-text', 'speak-result', 'speak'],
]) {
  $(formId).onsubmit = async event => {
    event.preventDefault();
    const button = event.currentTarget.querySelector('button');
    if (button.disabled) return;
    const input = $(inputId), result = $(resultId), text = input.value.trim();
    button.disabled = true;
    result.className = '';
    result.textContent = op === 'speak' ? '正在准备语音…' : '正在保存…';
    try {
      const reply = await send(op, {text});
      if (op === 'alert_text' && input.value.trim() === text) {
        loadedAlertText = input.value = reply.text;
        editingAlertText = false;
      }
      result.textContent = op === 'speak' ? '机器人已接收语音' : '提醒词已保存';
    } catch (error) {
      result.className = 'error';
      result.textContent = error.message;
    } finally {
      button.disabled = false;
    }
  };
}
refresh();setInterval(refresh,350);

// Older running consoles already serve the updated static files, but lack /debug.
// Use the standalone chat listener until the visual console is restarted.
const chatLink = document.querySelector('a[href="/debug"]');
if (chatLink) {
  fetch('/debug', {method: 'HEAD'}).then(response => {
    if (response.status === 404) {
      const standalone = new URL('/debug', window.location.href);
      standalone.port = '8766';
      chatLink.href = standalone.href;
    }
  }).catch(() => {});
}
