# Ubuntu 直连海康摄像头抓流步骤

本文档用于记录：**电脑重启后**，如何在 `Ubuntu 22.04` 下重新通过网线直连海康摄像头，并通过命令行预览、抓图、录像。

当前已验证的设备信息：

- 摄像头 IP：`192.168.1.64`
- 电脑有线网口：`eno1`
- 电脑有线口静态 IP：`192.168.1.10/24`
- 连接方式：`12V` 电源给摄像头供电，网线直连电脑 `eno1`

---

## 1. 物理连接

每次开机后，先确认硬件连接正确：

1. 给摄像头接上 `12V` 电源。
2. 用网线将摄像头直连到电脑有线网口 `eno1`。
3. 等待 `30` 到 `60` 秒，让摄像头启动完成。

注意事项：

- `12V` 电源只负责供电，不负责传输数据。
- 电脑的 `Wi-Fi` 可以继续上网，但摄像头通信走的是 `eno1`。
- 如果网线重插过，`eno1` 的 IP 配置可能会被系统改回自动分配，需要重新设置。

---

## 2. 检查网口状态

先检查有线网卡是否已经连通：

```bash
ip addr show eno1
```

正常时应看到类似：

```text
eno1: <BROADCAST,MULTICAST,UP,LOWER_UP>
```

说明：

- `UP` 表示网卡已启用。
- `LOWER_UP` 表示物理链路已建立，说明网线和对端设备连接正常。

如果没有 `LOWER_UP`：

- 检查摄像头是否已上电。
- 检查网线是否插紧。
- 检查摄像头网口灯是否亮或闪烁。

---

## 3. 重新设置电脑有线口 IP

电脑重启或网线重插后，`eno1` 可能被系统自动改成其他地址，例如 `192.168.158.15/24`。这时必须改回和摄像头同网段的固定地址。

执行：

```bash
sudo ip addr flush dev eno1
sudo ip addr add 192.168.1.10/24 dev eno1
sudo ip link set eno1 up
ip addr show eno1
```

确认输出里包含：

```text
inet 192.168.1.10/24
```

说明：

- `flush` 用于清除旧地址，避免和新的静态地址冲突。
- 摄像头当前地址是 `192.168.1.64`，所以电脑必须放在 `192.168.1.x` 网段。

---

## 4. 测试和摄像头的连通性

执行：

```bash
ping -c 4 192.168.1.64
```

正常时会看到类似：

```text
64 bytes from 192.168.1.64
```

如果不通：

- 再次确认 `eno1` 仍然是 `192.168.1.10/24`
- 再次确认摄像头仍然使用 `192.168.1.64`
- 检查摄像头是否已完成启动
- 检查是否重新插拔过网线，导致地址被覆盖

---

## 5. 安装抓流工具

首次使用时安装：

```bash
sudo apt update
sudo apt install ffmpeg vlc
```

如果系统中自带 `ffplay`，可用于直接弹窗预览。

检查：

```bash
ffplay -version
```

---

## 6. RTSP 地址格式

海康摄像头常用 RTSP 地址格式如下：

```text
rtsp://用户名:密码@192.168.1.64:554/Streaming/Channels/101
```

常见通道说明：

- `101`：主码流，清晰度高
- `102`：子码流，清晰度低，通常更稳

示例：

```text
rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101
```

注意事项：

- 如果密码里包含 `@`、`:`、`#` 等特殊字符，URL 可能解析失败。
- 初次测试建议使用不含 URL 特殊字符的强密码。

---

## 7. 直接预览摄像头画面

使用 `ffplay` 预览主码流：

```bash
ffplay -rtsp_transport tcp "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101"
```

如果主码流太卡，可改试子码流：

```bash
ffplay -rtsp_transport tcp "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/102"
```

说明：

- `-rtsp_transport tcp` 通常比 `udp` 更稳定。
- 这条命令会直接弹出实时画面窗口。

---

## 8. 不弹窗，只测试是否能拉到流

如果只想确认流是否可用：

```bash
ffmpeg -rtsp_transport tcp -i "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101" -f null -
```

如果命令持续输出视频信息，说明已经成功连接摄像头并开始接收视频流。

---

## 9. 抓拍一张图片

执行：

```bash
ffmpeg -rtsp_transport tcp -i "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101" -frames:v 1 snapshot.jpg
```

生成文件：

- `snapshot.jpg`

---

## 10. 录制一段测试视频

先录制 `30` 秒：

```bash
ffmpeg -rtsp_transport tcp -i "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101" -t 30 -c copy test.mp4
```

说明：

- `-t 30` 表示录制 `30` 秒
- `-c copy` 表示不转码，直接保存原始码流，CPU 占用低

录完后可播放检查：

```bash
vlc test.mp4
```

---

## 11. 长时间连续录像

建议按时间切片保存，而不是写成一个超大文件：

```bash
mkdir -p ~/camera_record
ffmpeg -rtsp_transport tcp -i "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101" -c copy -f segment -segment_time 300 -reset_timestamps 1 -strftime 1 ~/camera_record/record_%Y%m%d_%H%M%S.mp4
```

说明：

- 每 `300` 秒切一个文件
- 录像文件保存在 `~/camera_record`
- 长时间采集更稳定，也方便后续查找和删除

---

## 12. 每次重启后的最短操作清单

如果只想快速恢复抓流，按下面顺序执行即可：

```bash
ip addr show eno1
sudo ip addr flush dev eno1
sudo ip addr add 192.168.1.10/24 dev eno1
sudo ip link set eno1 up
ping -c 4 192.168.1.64
ffplay -rtsp_transport tcp "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101"
```

如果需要直接录像：

```bash
ffmpeg -rtsp_transport tcp -i "rtsp://admin:你的密码@192.168.1.64:554/Streaming/Channels/101" -t 30 -c copy test.mp4
```

---

## 13. 常见问题

### 1. 为什么重启后又 ping 不通了？

因为 `eno1` 的静态 IP 没有永久保存，系统可能重新给它分配了别的地址。重新执行第 3 节的命令即可恢复。

### 2. 为什么 `ping 192.168.1.64` 不通？

常见原因：

- 摄像头未上电
- 网线未插好
- `eno1` 不在 `192.168.1.10/24`
- 摄像头启动未完成

### 3. 为什么能 ping 通，但拉流失败？

常见原因：

- 用户名或密码错误
- RTSP 地址写错
- 密码中有特殊字符
- 摄像头未启用对应码流

### 4. 为什么网页可以不用，仍然能抓流？

因为网页主要用于配置和预览，真正的视频数据可以直接通过 `RTSP` 获取，`ffplay`、`ffmpeg`、`VLC` 都可以直接拉流。

---

## 14. 建议

- 把 `eno1` 永久设置为静态 IP：`192.168.1.10/24`
- 摄像头编码优先使用 `H.264`
- 先测试 `101` 主码流，不稳定时再试 `102`
- 长时间录像时优先用 `ffmpeg` 分段保存
- 记录好摄像头用户名、密码和 RTSP 地址，避免每次重新排查
