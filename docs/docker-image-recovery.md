# Docker 镜像恢复信息（DG-202606）

> 记录时间：2026-09-10
> 背景：`环境/` 目录下两个镜像 tar 存档（共约 30 GB）已删除，用于释放 E 盘空间。
> 本文档保留镜像的精确标识，供灾后核对与重新获取。

## 一、已删除的本地存档

| 原路径 | 大小 | 对应镜像 |
|---|---|---|
| `环境/supermarket_sorting_server.tar` | 18,727,134,244 B（约 17.4 GiB） | `supermarket_sorting_final:server` |
| `环境/supermarket_sorting_client.tar` | 12,112,086,744 B（约 11.3 GiB） | `supermarket_sorting_final:client` |

两文件与下述镜像的字节数逐一吻合，可确认其为镜像的 `docker save` 原样导出。

## 二、镜像精确标识（灾后核对依据）

仓库前缀：`crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/`

### server

```
tag      : supermarket_sorting_final:server
image id : sha256:eb0b58a600b85910c2e5392852e9268ca6a5aaca02385ab8a0f69981d833c4b2
digest   : supermarket_sorting_final@sha256:eb0b58a600b85910c2e5392852e9268ca6a5aaca02385ab8a0f69981d833c4b2
created  : 2026-07-27T20:58:14.805715714+08:00
size     : 18727134244 B（约 17.4 GiB，磁盘占用 37.7 GB）
layers   : 17
```

### client

```
tag      : supermarket_sorting_final:client
image id : sha256:dbe0bfd2c75e34af430e2e79f07c0ab9528d843e6fc6b8ce067216fd28168f0e
digest   : supermarket_sorting_final@sha256:dbe0bfd2c75e34af430e2e79f07c0ab9528d843e6fc6b8ce067216fd28168f0e
created  : 2026-07-27T17:44:18.566404816+08:00
size     : 12112086744 B（约 11.3 GiB，磁盘占用 24.4 GB）
```

## 三、重建路径（按优先级）

1. **本机 Docker 仍在**
   镜像当前完好存在于本机 Docker 中，日常运行不受存档删除影响。只需确保 `E:\workspace\WSL\ext4.vhdx` 不损坏、不被误清。
   ```bash
   wsl -d Ubuntu-22.04 -u root -- docker images
   ```

2. **重新导出存档（重建灾备）**
   ⚠️ Docker 中**未保存**阿里云私有仓库登录凭据（`~/.docker/config.json` 不存在），本机当前无法 `docker pull` 重新拉取。若需重建 tar 存档，须先 `docker login` 取得仓库访问权限：
   ```bash
   wsl -d Ubuntu-22.04 -u root -- bash -c '
     docker login crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com
     docker save -o /mnt/e/<目标路径>/supermarket_sorting_server.tar \
       crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:server
     docker save -o /mnt/e/<目标路径>/supermarket_sorting_client.tar \
       crpi-1pzq998p9m7w0auy.cn-hangzhou.personal.cr.aliyuncs.com/challengecup/supermarket_sorting_final:client
   '
   ```
   注：重新导出前请确认目标盘有 ≥30 GB 可用空间；导出后校验镜像 digest 是否与本文档一致。

3. **向组委会重新索取**
   若本机 Docker 数据完全丢失且无仓库权限，向锐捷网络（DG-202606 发榜方）申请重新下发镜像，并提供本文档中的 digest 用于版本核对。

## 四、运行环境关键路径

- WSL 发行版：`Ubuntu-22.04`
- WSL 虚拟磁盘：`E:\workspace\WSL\ext4.vhdx`（约 77 GB）
- 自研代码挂载（**不在镜像内**，随宿主机代码走）：
  - `/opt/jbgz/baseline` → 容器 `/workspace/baseline`（只读）
  - `/opt/jbgz/baseline/patches/discoverse/envs/simulator.py` → 容器 `/workspace/supermarket_sorting_task/discoverse/envs/simulator.py`
  - `/opt/jbgz/baseline/patches/examples/ros2/mmk2_ros2.py` → 容器 `/workspace/supermarket_sorting_task/examples/ros2/mmk2_ros2.py`
- 共享缓存卷：`supermarket_sorting_cache` → 容器 `/root/.cache`
