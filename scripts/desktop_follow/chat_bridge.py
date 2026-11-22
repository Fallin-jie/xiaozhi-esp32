"""Text-only chat bridge, run alongside the local Xiaozhi server in Compose."""
import asyncio
import os
from pathlib import Path

from aiohttp import web
import httpx
import yaml


class ChatError(Exception):
    def __init__(self, status, message):
        self.status = status
        super().__init__(message)


def validate_messages(data):
    messages = data.get("messages") if isinstance(data, dict) else None
    if not isinstance(messages, list) or not 1 <= len(messages) <= 17 or len(messages) % 2 == 0:
        raise ChatError(400, "对话需以提问开始和结束，最多携带前 8 轮。")
    total = 0
    for i, message in enumerate(messages):
        role = "user" if i % 2 == 0 else "assistant"
        if not isinstance(message, dict) or set(message) != {"role", "content"} or message["role"] != role:
            raise ChatError(400, "对话消息的格式或顺序有误。")
        content = message["content"]
        if not isinstance(content, str) or not content.strip() or len(content) > (2000 if role == "user" else 12000):
            raise ChatError(400, "消息为空或超长，单次提问最多 2000 字。")
        total += len(content)
    if total > 16000:
        raise ChatError(400, "上下文过长，请开启新对话。")
    return messages


async def agent_config(client, manager, device):
    try:
        r = await client.post(manager["url"].rstrip("/") + "/config/agent-models",
                              headers={"Authorization": "Bearer " + manager["secret"]},
                              json={"macAddress": device, "clientId": "desktop-debug", "selectedModule": {}}, timeout=8)
        r.raise_for_status()
        result = r.json()
        if result.get("code") != 0:
            raise ChatError(503, "无法读取智能体，请检查智控台的设备绑定与服务配置。")
        agent = result["data"]
        model = agent["LLM"][agent["selected_module"]["LLM"]]
        if not isinstance(model, dict) or not model.get("model_name"):
            raise ChatError(422, "智能体缺少模型名称，请检查智控台配置。")
        if model.get("type") != "openai":
            raise ChatError(422, "此调试页支持 OpenAI 兼容模型，请检查智能体的模型类型。")
        return model, agent.get("prompt") or ""
    except (httpx.HTTPError, KeyError, TypeError, ValueError) as exc:
        raise ChatError(503, "读取智能体配置失败，请检查本机智控台服务。") from exc


async def completion(client, model, prompt, messages):
    endpoint = model.get("base_url") or model.get("url")
    name = model.get("model_name")
    if not endpoint or not name:
        raise ChatError(422, "模型配置缺少接口地址或模型名称。")
    dialogue = ([{"role": "system", "content": prompt}] if prompt else []) + messages
    params = {"model": name, "messages": dialogue, "stream": False}
    try:
        params["max_tokens"] = min(int(model.get("max_tokens") or 1024), 1024)
        for key in ("temperature", "top_p", "frequency_penalty"):
            if model.get(key) not in (None, ""):
                params[key] = round(float(model[key]), 1)
    except (TypeError, ValueError) as exc:
        raise ChatError(422, "模型生成参数有误，请在智控台检查。") from exc
    headers = {"Authorization": "Bearer " + model["api_key"]} if model.get("api_key") else {}
    r = await client.post(endpoint.rstrip("/") + "/chat/completions", json=params, headers=headers, timeout=65)
    if r.status_code in (401, 403):
        raise ChatError(502, "模型服务拒绝鉴权，请在智控台检查模型密钥和权限。")
    if r.status_code in (402, 429):
        raise ChatError(429, "模型额度或请求频率受限，请检查额度或稍后重试。")
    if not r.is_success:
        raise ChatError(502, f"模型服务返回 HTTP {r.status_code}，请检查模型配置或稍后重试。")
    try:
        reply = r.json()["choices"][0]["message"]["content"]
        if not isinstance(reply, str) or not reply.strip() or len(reply) > 12000:
            raise ValueError
    except (KeyError, TypeError, IndexError, ValueError) as exc:
        raise ChatError(502, "模型未返回有效的文字回复，请重试。") from exc
    return reply.strip()


def build_app(manager, device):
    app = web.Application(client_max_size=96 * 1024)
    slots = asyncio.Semaphore(2)

    async def handle(request):
        try:
            messages = None
            if request.method == "POST":
                if request.headers.get("X-Desk-Control") != "1":
                    raise ChatError(403, "请求缺少控制台标识。")
                if request.content_type != "application/json":
                    raise ChatError(415, "请发送 JSON 格式的对话。")
                try:
                    messages = validate_messages(await request.json())
                except (ValueError, RecursionError) as exc:
                    raise ChatError(400, "请求的 JSON 格式有误。") from exc
            if slots.locked():
                raise ChatError(429, "调试服务忙，请稍后重试。")
            async with slots, httpx.AsyncClient(trust_env=False) as client:
                model, prompt = await agent_config(client, manager, device)
                result = {"model": model["model_name"]}
                if messages is not None:
                    result["reply"] = await asyncio.wait_for(completion(client, model, prompt, messages), timeout=70)
                return web.json_response(result)
        except ChatError as exc:
            return web.json_response({"error": str(exc)}, status=exc.status)
        except (asyncio.TimeoutError, httpx.TimeoutException):
            return web.json_response({"error": "模型响应超时，请重试。"}, status=504)
        except httpx.RequestError:
            return web.json_response({"error": "无法连接智能体配置的模型服务。"}, status=502)

    app.router.add_get("/chat", handle, allow_head=False)
    app.router.add_post("/chat", handle)
    return app


if __name__ == "__main__":
    manager = yaml.safe_load(Path("/run/xiaozhi-config.yaml").read_text(encoding="utf-8"))["manager-api"]
    web.run_app(build_app(manager, os.environ["DEBUG_DEVICE_MAC"]), port=18765, access_log=None)
