const form = document.querySelector('#composer');
const input = document.querySelector('#prompt');
const send = document.querySelector('#send');
const reset = document.querySelector('#reset');
const log = document.querySelector('.transcript');
const empty = document.querySelector('#empty-chat');
const status = document.querySelector('#status');
const model = document.querySelector('#model');
let history = [];
let busy = false;
let turns = 0;

function controls() {
  send.disabled = busy || !input.value.trim();
  reset.disabled = busy || (!turns && !input.value);
  input.readOnly = busy;
}

function notice(text, error = false) {
  status.textContent = text;
  status.classList.toggle('error', error);
}

async function requestChat(options) {
  const response = await fetch('/api/debug/chat', options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || `请求失败（HTTP ${response.status}）。`);
  return data;
}

function appendMessage(message) {
  const bubble = document.createElement('div');
  bubble.className = `turn ${message.role}`;
  const name = document.createElement('strong');
  name.textContent = message.role === 'user' ? '你' : '小智';
  const text = document.createElement('p');
  text.textContent = message.content;
  bubble.append(name, text);
  empty.hidden = true;
  log.append(bubble);
  log.scrollTop = log.scrollHeight;
  return bubble;
}

input.addEventListener('input', controls);
input.addEventListener('keydown', event => {
  if (event.key === 'Enter' && !event.shiftKey && !event.isComposing && event.keyCode !== 229) {
    event.preventDefault();
    if (!send.disabled) form.requestSubmit();
  }
});

form.addEventListener('submit', async event => {
  event.preventDefault();
  if (busy || !input.value.trim()) return;
  const question = {role: 'user', content: input.value.trim()};
  const messages = [...history, question];
  let size = messages.reduce((n, m) => n + m.content.length, 0);
  while (size > 16000 && messages.length > 1) {
    size -= messages.splice(0, 2).reduce((n, m) => n + m.content.length, 0);
  }
  const pending = appendMessage(question);
  busy = true;
  controls();
  notice('等待小智回复…');
  try {
    const data = await requestChat({method: 'POST', headers: {'Content-Type': 'application/json', 'X-Desk-Control': '1'}, body: JSON.stringify({messages})});
    const answer = {role: 'assistant', content: data.reply};
    appendMessage(answer);
    model.textContent = data.model;
    turns++;
    history = [...messages, answer].slice(-16);
    notice(`第 ${turns} 轮完成 · 本次包含 ${Math.floor(messages.length / 2)} 轮历史`);
    input.value = '';
  } catch (error) {
    pending.remove();
    empty.hidden = turns > 0;
    notice(error.message || '发送失败，请重试。', true);
  } finally {
    busy = false;
    controls();
    input.focus();
  }
});

reset.addEventListener('click', () => {
  history = [];
  turns = 0;
  input.value = '';
  log.replaceChildren(empty);
  empty.hidden = false;
  notice('新对话已开始。');
  controls();
  input.focus();
});

requestChat().then(data => { if (!turns) model.textContent = data.model; })
  .catch(error => { if (!turns) model.textContent = error.message; });
