# Git LFS 配置完成

## ✅ 已完成配置

Git LFS 已成功配置并推送到远程仓库。

## 📋 配置内容

### 1. `.gitattributes` - LFS 追踪规则

已配置 LFS 追踪以下文件类型：

| 类别 | 扩展名 |
|------|--------|
| ROS bag 文件 | `*.bag`, `*.db3` |
| 模型权重 | `*.pth`, `*.pth.tar`, `*.pt`, `*.onnx` |
| 点云数据 | `*.ply`, `*.pcd`, `*.las`, `*.laz` |
| 二进制数据 | `*.bin`, `data/**/*.bin` |
| 文档 | `*.pdf` |
| 媒体文件 | `*.gif`, `*.mp4`, `*.avi`, `*.mov` |
| 压缩包 | `*.tar`, `*.tar.gz`, `*.zip` |
| 其他 | `*.pkl`, `*.h5`, `*.hdf5`, `*.ckpt`, `*.safetensors` |

### 2. `.gitignore` - 忽略规则

已更新忽略规则，只排除：
- Docker 依赖和构建产物
- 临时文件和缓存
- IDE 配置文件

### 3. `git-lfs-manager.sh` - 管理脚本

提供了自动化管理脚本：
- `./git-lfs-manager.sh help` - 显示帮助
- `./git-lfs-manager.sh status` - 查看状态
- `./git-lfs-manager.sh info` - 使用情况
- `./git-lfs-manager.sh pull` - 拉取 LFS 文件
- `./git-lfs-manager.sh push` - 推送 LFS 文件

### 4. `docs/GIT_LFS_GUIDE.md` - 完整文档

详细的使用指南，包括：
- 安装和初始化
- 文件管理
- 常用命令
- 最佳实践
- 故障排查
- 团队协作

## 📤 上传整个工程到 GitHub（不删除任何本地文件）

**承诺：以下流程仅做 LFS 标记与推送，不会删除本地任何代码或文件。**

```bash
# 1. 确保 LFS 就绪（仅配置，不删文件）
bash scripts/upload_to_github_with_lfs.sh

# 2. 添加要提交的内容（大文件会按 .gitattributes 自动走 LFS）
git add .

# 3. 查看将用 LFS 的文件（可选）
git lfs status

# 4. 提交并推送
git commit -m "feat: sync project with Git LFS"
git push -u origin main
```

若只需提交 `.gitattributes` 的修改：
```bash
bash scripts/upload_to_github_with_lfs.sh --commit-and-push
```

## 🚀 快速开始

### 新成员入职

```bash
# 1. 克隆仓库
git clone git@github.com:xinruozhishui201314/automap_pro.git
cd automap_pro

# 2. 安装 Git LFS（如果未安装）
sudo apt install git-lfs  # Ubuntu/Debian
# 或
brew install git-lfs      # macOS

# 3. 初始化 LFS
git lfs install

# 4. 拉取 LFS 文件
git lfs pull

# 5. 验证
git lfs ls-files
```

### 添加新的大文件

```bash
# 1. 检查文件类型是否在 .gitattributes 中
git lfs track

# 2. 添加文件（.gitattributes 会自动处理）
git add new_file.bag
git commit -m "add new data file"
git push

# 3. 验证文件已由 LFS 追踪
git lfs ls-files | grep new_file.bag
```

### 查看状态

```bash
# 查看所有 LFS 文件
git lfs ls-files

# 查看使用情况
./git-lfs-manager.sh info

# 查看状态
./git-lfs-manager.sh status
```

## 📊 当前状态

```bash
# 仓库大小
du -sh .git
# 预期: ~100MB（已清理大文件）

# LFS 追踪规则
git lfs track
# 显示: 26 种文件类型

# LFS 文件数量
git lfs ls-files | wc -l
# 预期: 0（还没有 LFS 文件）
```

## ⚠️ 重要提示

### 文件大小策略

| 文件大小 | 处理方式 |
|----------|----------|
| < 1MB | 普通 Git |
| 1MB - 10MB | 普通 Git（考虑使用 LFS） |
| **> 10MB** | **必须使用 LFS** |
| > 100MB | **必须使用 LFS + 压缩** |

### GitHub LFS 配额

| 计划 | 存储 | 带宽/月 | 价格 |
|------|------|---------|------|
| Free | 1 GB | 1 GB | 免费 |
| Pro | 2 GB | 10 GB | $4/月 |
| Team | 2 GB | 50 GB | $4/用户/月 |

**当前使用**: 查看配额
- https://github.com/settings/billing

### 团队协作

**推送前**:
```bash
git lfs push --all origin main
git push origin main
```

**拉取后**:
```bash
git lfs pull
```

## 🔍 常见问题

### Q: 文件显示为指针文件（文本）？

**A**: LFS 文件未下载，运行：
```bash
git lfs pull
```

### Q: 推送失败 - LFS 配额不足？

**A**: 检查配额并升级计划：
- https://github.com/settings/billing

### Q: 克隆后 LFS 文件缺失？

**A**: 运行：
```bash
git lfs install
git lfs pull
```

### Q: .gitattributes 不生效？

**A**: 检查格式并重新扫描：
```bash
cat .gitattributes
git add --renormalize .
```

## 📚 详细文档

请查看完整文档：[docs/GIT_LFS_GUIDE.md](docs/GIT_LFS_GUIDE.md)

## 📞 支持

如有问题，请联系：
- Automap Pro Team
- 查看文档：docs/GIT_LFS_GUIDE.md
- Git LFS 官方文档：https://git-lfs.github.com/

---

**配置日期**: 2026-03-01
**版本**: 1.0
**状态**: ✅ 已完成
