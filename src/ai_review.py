import os
import sys
import json
import time
import hmac
import base64
import urllib.request
import urllib.error
import urllib.parse

import ssl
ssl._create_default_https_context = ssl._create_unverified_context



# ================= 版本管理 =================
SCRIPT_VERSION = "1.0.0"
SCRIPT_NAME = "ai_review.py"

def print_version():
    """打印脚本版本信息"""
    print(f"📄 {SCRIPT_NAME} v{SCRIPT_VERSION}")


# ================= 配置区 =================
GITLAB_API_URL = os.environ.get("CI_API_V4_URL")
PROJECT_ID = os.environ.get("CI_PROJECT_ID")
COMMIT_SHA = os.environ.get("CI_COMMIT_SHA")
GITLAB_TOKEN = os.environ.get("GITLAB_API_TOKEN")
AI_API_KEY = os.environ.get("DEEPSEEK_API_KEY")
DINGTALK_WEBHOOK = os.environ.get("DINGTALK_WEBHOOK")
DINGTALK_SECRET = os.environ.get("DINGTALK_SECRET")



# ================= 核心功能函数 =================

def get_commit_changes():
    """从 GitLab 获取 Commit 的代码变更 (Diff)"""
    url = f"{GITLAB_API_URL}/projects/{PROJECT_ID}/repository/commits/{COMMIT_SHA}/diff"
    req = urllib.request.Request(url, headers={"PRIVATE-TOKEN": GITLAB_TOKEN})
    try:
        with urllib.request.urlopen(req, timeout=20) as response:
            changes = json.loads(response.read().decode())
            return "\n".join([change['diff'] for change in changes])
    except Exception as e:
        print(f"❌ 获取 Commit 变更失败: {e}")
        return None


def get_commit_info():
    """从 GitLab 获取当前 Commit 的详细信息（提交者、提交信息等）"""
    url = f"{GITLAB_API_URL}/projects/{PROJECT_ID}/repository/commits/{COMMIT_SHA}"
    req = urllib.request.Request(url, headers={"PRIVATE-TOKEN": GITLAB_TOKEN})
    try:
        with urllib.request.urlopen(req, timeout=20) as response:
            return json.loads(response.read().decode())
    except Exception as e:
        print(f"❌ 获取 Commit 信息失败: {e}")
        return None


def get_project_info():
    """从 GitLab 获取当前项目的信息"""
    url = f"{GITLAB_API_URL}/projects/{PROJECT_ID}"
    req = urllib.request.Request(url, headers={"PRIVATE-TOKEN": GITLAB_TOKEN})
    try:
        with urllib.request.urlopen(req, timeout=20) as response:
            return json.loads(response.read().decode())
    except Exception as e:
        print(f"❌ 获取项目信息失败: {e}")
        return None


def review_code_with_ai(code_diff, commit_info, project_info):
    """调用 DeepSeek API 进行代码审查"""
    # 提取提交者信息
    author_name = commit_info.get('author_name', 'Unknown') if commit_info else 'Unknown'
    author_email = commit_info.get('author_email', 'Unknown') if commit_info else 'Unknown'
    commit_title = commit_info.get('title', 'Unknown') if commit_info else 'Unknown'
    commit_message = commit_info.get('message', 'Unknown') if commit_info else 'Unknown'

    # 提取项目信息
    project_name = project_info.get('name_with_namespace', 'Unknown') if project_info else 'Unknown'
    project_url = project_info.get('web_url', 'Unknown') if project_info else 'Unknown'

    prompt = f"""你是一个资深的后端开发工程师。请审查以下 GitLab 推送的代码变更，只评估变更的代码部分。

## 审查上下文

| 项目 | 值 |
|------|-----|
| **项目名称** | {project_name} |
| **项目地址** | {project_url} |
| **提交者** | {author_name} ({author_email}) |
| **Commit 标题** | {commit_title} |
| **Commit 信息** | {commit_message} |

请重点关注：逻辑正确性、潜在的 Bug、性能问题和代码规范。
以列表形式给出你的审查意见，如果没有问题，请回复"未发现明显问题"。
请对变更的代码进行打分，范围为 1 到 100 分。打分依据：
- 代码逻辑正确性：20 分
- 潜在的 Bug：20 分
- 代码质量：20 分
- 代码规范：20 分
- 代码量：20 分,给出代码量的统计信息

最后请给出一个**整体评价**，用一两句话总结这段代码的质量水平，例如"代码质量优秀，逻辑清晰，开发者水平较高"或"代码存在较多问题，质量一般，建议改进"等类似评价。

代码差异（Diff）：
{code_diff}"""

    payload = json.dumps({
        "model": "deepseek-v4-flash",
        "messages": [
            {"role": "system", "content": "你是一个严格的代码审查员"},
            {"role": "user", "content": prompt}
        ]
    }).encode('utf-8')


    req = urllib.request.Request(
        "https://api.deepseek.com/chat/completions",
        data=payload,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {AI_API_KEY}"
        }
    )
    try:
        with urllib.request.urlopen(req, timeout=60) as response:
            result = json.loads(response.read().decode())
            return result['choices'][0]['message']['content']
    except Exception as e:
        print(f"❌ AI 审查失败: {e}")
        return "AI 审查服务暂时不可用，请人工审查。"


def post_review_to_gitlab(comment, commit_info, project_info):
    """将审查结果评论到 GitLab Commit 上"""
    # 提取提交者信息
    author_name = commit_info.get('author_name', 'Unknown') if commit_info else 'Unknown'
    author_email = commit_info.get('author_email', 'Unknown') if commit_info else 'Unknown'
    commit_title = commit_info.get('title', 'Unknown') if commit_info else 'Unknown'
    commit_message = commit_info.get('message', 'Unknown') if commit_info else 'Unknown'

    # 提取项目信息
    project_name = project_info.get('name_with_namespace', 'Unknown') if project_info else 'Unknown'
    project_url = project_info.get('web_url', 'Unknown') if project_info else 'Unknown'

    full_comment = (
        f"🤖 **AI 代码审查报告**\n\n"
        f"---\n\n"
        f"**项目**: [{project_name}]({project_url})\n\n"
        f"**提交者**: {author_name}（{author_email}）\n\n"
        f"**Commit 标题**: {commit_title}\n\n"
        f"**Commit 信息**: {commit_message}\n\n"
        f"---\n\n"
        f"{comment}"
    )

    url = f"{GITLAB_API_URL}/projects/{PROJECT_ID}/repository/commits/{COMMIT_SHA}/comments"
    payload = json.dumps({"note": full_comment}).encode('utf-8')
    req = urllib.request.Request(url, data=payload, headers={
        "PRIVATE-TOKEN": GITLAB_TOKEN,
        "Content-Type": "application/json"
    })
    try:
        urllib.request.urlopen(req, timeout=20)
        print("✅ AI 审查意见已成功发布到 GitLab Commit！")
    except Exception as e:
        print(f"❌ 发布 GitLab 评论失败: {e}")


def send_dingtalk_notification(review_result, commit_info, project_info):
    """将审查结果发送到钉钉群"""
    if not DINGTALK_WEBHOOK or not DINGTALK_SECRET:
        print("⚠️ 未配置 DINGTALK_WEBHOOK 或 DINGTALK_SECRET 环境变量，跳过发送钉钉消息。")
        return

    # 提取提交者信息
    author_name = commit_info.get('author_name', 'Unknown') if commit_info else 'Unknown'
    author_email = commit_info.get('author_email', 'Unknown') if commit_info else 'Unknown'
    commit_title = commit_info.get('title', 'Unknown') if commit_info else 'Unknown'
    commit_message = commit_info.get('message', 'Unknown') if commit_info else 'Unknown'

    # 提取项目信息
    project_name = project_info.get('name_with_namespace', 'Unknown') if project_info else 'Unknown'
    project_url = project_info.get('web_url', 'Unknown') if project_info else 'Unknown'

    # 1. 钉钉加签逻辑
    timestamp = str(round(time.time() * 1000))
    secret_enc = DINGTALK_SECRET.encode('utf-8')
    string_to_sign = f'{timestamp}\n{DINGTALK_SECRET}'
    string_to_sign_enc = string_to_sign.encode('utf-8')
    hmac_code = hmac.new(secret_enc, string_to_sign_enc, digestmod='sha256').digest()
    sign = urllib.parse.quote_plus(base64.b64encode(hmac_code))

    # 2. 构造最终请求 URL
    final_webhook = f"{DINGTALK_WEBHOOK}&timestamp={timestamp}&sign={sign}"

    # 3. 构造 Markdown 消息体
    commit_short_sha = COMMIT_SHA[:8] if COMMIT_SHA else "Unknown"
    commit_url = f"{GITLAB_API_URL}/projects/{PROJECT_ID}/repository/commits/{COMMIT_SHA}"
    
    data = {
        "msgtype": "markdown",
        "markdown": {
            "title": "🤖 AI 代码审查报告",
            "text": (
                f"### 🤖 AI 代码审查报告\n\n"
                f"**项目**: [{project_name}]({project_url})\n\n"
                f"**提交者**: {author_name}（{author_email}）\n\n"
                f"**Commit**: [{commit_short_sha}]({commit_url})\n\n"
                f"**Commit 标题**: {commit_title}\n\n"
                f"**Commit 信息**: {commit_message}\n\n"
                f"---\n\n"
                f"**审查结果**:\n\n{review_result}"
            )
        },
        "at": {
            "atMobiles": [],
            "isAtAll": False
        }
    }

    # 4. 发送请求
    payload = json.dumps(data).encode('utf-8')
    req = urllib.request.Request(final_webhook, data=payload, headers={
        "Content-Type": "application/json"
    })

    try:
        with urllib.request.urlopen(req, timeout=20) as response:
            resp_body = response.read().decode('utf-8')
            resp_json = json.loads(resp_body)
            if resp_json.get("errcode") == 0:
                print("✅ AI 审查报告已成功发送到钉钉！")
            else:
                print(f"❌ 钉钉发送失败: {resp_json.get('errmsg')}")
    except Exception as e:
        print(f"❌ 发送钉钉消息时发生异常: {e}")


# ================= 主程序入口 =================

if __name__ == "__main__":
    print_version()
    print("🚀 开始执行 AI Code Review 流程...")
    
    # 检查必要的环境变量
    required_vars = ["CI_API_V4_URL", "CI_PROJECT_ID", "CI_COMMIT_SHA", "GITLAB_API_TOKEN", "DEEPSEEK_API_KEY"]
    missing_vars = [var for var in required_vars if not os.environ.get(var)]
    if missing_vars:
        print(f"❌ 缺少必要的环境变量: {', '.join(missing_vars)}")
        sys.exit(1)

    # 1. 获取代码变更
    diff = get_commit_changes()
    if not diff:
        print("⚠️ 未获取到代码变更，流程终止。")
        sys.exit(0)

    # 2. 获取 Commit 详情和项目信息
    commit_info = get_commit_info()
    project_info = get_project_info()

    if commit_info:
        print(f"👤 提交者: {commit_info.get('author_name', 'Unknown')} ({commit_info.get('author_email', 'Unknown')})")
        print(f"📋 Commit 标题: {commit_info.get('title', 'Unknown')}")
    if project_info:
        print(f"📦 项目: {project_info.get('name_with_namespace', 'Unknown')}")

    # 3. AI 审查
    review = review_code_with_ai(diff, commit_info, project_info)
    print(f"📝 AI 审查结果:\n{review}")

    # 4. 发布到 GitLab
    post_review_to_gitlab(review, commit_info, project_info)

    # 5. 发送钉钉通知
    send_dingtalk_notification(review, commit_info, project_info)

    print("🎉 AI Code Review 流程执行完毕！")