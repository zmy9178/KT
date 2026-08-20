# GitHub 上传规则

本文档用于规范本项目向 GitHub 上传代码、数据和文档的流程。

## 1. 上传前检查

上传前必须确认：

- 项目中没有密码、Token、私钥、许可证密钥或其他凭据。
- 文件中没有个人隐私、内部资料或未经授权的数据。
- 代码可以正常运行，至少完成一次基本测试。
- 文件名、目录名和文档标题清晰，避免使用无意义的临时名称。
- 不上传与项目无关的个人文件、软件安装包或系统生成文件。

## 2. 建议上传的内容

以下内容通常可以上传：

- MATLAB 源代码：`.m`
- Python 源代码：`.py`
- 项目说明文档：`.md`
- 必要的配置模板，例如 `.example` 文件
- 可公开使用的小型示例数据
- 测试代码和测试说明

## 3. 默认不上传的内容

以下文件默认不应上传，除非项目明确需要并确认其中没有敏感信息：

- Python 缓存：`__pycache__/`、`*.pyc`
- MATLAB 调试或临时数据：`*.mat`
- 大型数据文件：`*.csv`、`*.xlsx`、`*.npz`
- 临时图片、日志和运行结果：`*.log`、`tmp/`、`debug/`
- 编辑器和系统文件：`.vscode/`、`.idea/`、`.DS_Store`
- 本地虚拟环境：`.venv/`、`venv/`
- 编译产物、压缩包和安装包

如果数据对复现实验很重要，应优先提供：

1. 小型、脱敏的示例数据；
2. 数据格式说明；
3. 生成或获取数据的脚本；
4. 数据文件的下载地址和使用许可。

## 4. 敏感信息处理

禁止直接上传以下内容：

```text
.env
*.key
*.pem
*.p12
credentials.json
config.local.*
```

应使用模板文件代替，例如：

```text
.env.example
config.example.json
```

模板中只能保留变量名，不能填写真实值：

```text
API_KEY=请填写你的密钥
DATABASE_URL=请填写数据库地址
```

如果敏感信息已经上传，应立即撤销或更换对应凭据。仅删除本地文件并重新提交，不能保证敏感信息从 Git 历史中消失。

## 5. 提交信息规范

提交信息应简短、明确，建议使用以下格式：

```text
类型: 简要说明
```

常用类型：

- `feat`: 新增功能
- `fix`: 修复问题
- `refactor`: 重构代码
- `test`: 增加或修改测试
- `docs`: 修改文档
- `data`: 修改实验数据或样例数据
- `chore`: 修改项目配置或维护文件

示例：

```text
feat: 增加 Python 伞降仿真入口
fix: 修正主伞阶段的状态切换
docs: 补充 GitHub 上传规则
```

## 6. 日常上传流程

本项目采用以下流程：

```text
dev 开发和测试
        ↓
Pull Request
        ↓
main 稳定版本
```

在项目根目录执行：

```powershell
# 创建并切换到 dev 分支
git switch -c dev

# 只添加本次需要上传的文件
git add <需要上传的文件>
git diff --cached
git commit -m "类型: 简要说明"

# 首次推送 dev 分支
git push -u origin dev
```

如果本地已经存在 `dev` 分支，使用：

```powershell
git switch dev
git pull origin dev
```

不要在未检查的情况下使用 `git add .`，避免把调试数据、缓存文件或其他无关改动一起提交。

## 7. 上传前重点确认

执行 `git diff --cached` 时重点检查：

- 是否误加入了密钥或隐私文件；
- 是否误加入了 `__pycache__` 和 `*.pyc`；
- 是否误加入大型调试数据；
- 是否包含不应公开的输入、输出或实验数据；
- 是否只提交了本次修改相关的文件。

## 8. 分支规则

- `main`：只保存已经验证、可以使用的稳定版本。
- `dev`：日常开发、测试和阶段性提交的分支。
- 新功能或问题修复应从 `dev` 分支继续创建独立分支。
- 分支名称建议使用：

```text
feature/功能名称
fix/问题名称
docs/文档名称
```

示例：

```powershell
git switch dev
git switch -c feature/python-simulation
```

完成开发后：

1. 在功能分支中完成测试。
2. 将功能分支推送到 GitHub。
3. 创建 Pull Request，目标分支选择 `dev`。
4. 检查通过后合并到 `dev`。
5. `dev` 完成阶段性验证后，再创建 Pull Request 合并到 `main`。

也可以在本地将 `dev` 合并到 `main`：

```powershell
git switch main
git pull origin main
git merge --no-ff dev
git push origin main
```

合并完成后，继续开发前应同步分支：

```powershell
git switch dev
git pull origin dev
```

## 9. 仓库可见性

本项目应保持为 **Private（私密）**，除非已经确认所有代码、数据和文档都可以公开。

修改仓库可见性前，应检查：

- 是否包含个人信息；
- 是否包含未公开的研究资料；
- 是否包含第三方受限制的数据；
- 是否包含真实配置和访问凭据；
- 是否有公开仓库或公开 Fork 会暴露相关内容。

## 10. 推荐维护的文件

项目逐步完善后，建议维护以下文件：

```text
README.md          # 项目介绍、安装和运行方法
GITHUB_UPLOAD_RULES.md  # GitHub 上传规则
.gitignore         # Git 忽略规则
LICENSE            # 项目许可证
requirements.txt   # Python 依赖
```

每次上传前先阅读本规则，并确保提交内容符合项目的隐私和可复现性要求。
