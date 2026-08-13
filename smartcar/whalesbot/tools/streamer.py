#!/usr/bin/python3
# -*- coding: utf-8 -*-
# streamer.py - 页面右上角带按键反馈的智能车双路监控系统

import cv2
import threading
import socket
from flask import Flask, Response, request, jsonify
import time
import numpy as np
import atexit
import os, sys

# 添加当前目录到路径，支持直接运行
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# 导入你提供的固定 Camera 类
try:
    from .camera import Camera
except ImportError:
    from camera import Camera

class Streamer:
    """
    MJPEG 视频流媒体类（支持多路摄像头 + 键盘事件捕获）
    """
    
    _instances = {}  # 记录已启动的实例，避免端口冲突

    def __init__(self, port=5000, fps=30, quality=80):
        self.port = port
        self.fps = fps
        self.quality = quality
        self.frames = {}  # 存储多路摄像头帧：{cam_id: frame}
        self.frame_lock = threading.Lock()
        self.app = Flask(__name__)
        # 关闭 Flask 访问日志（不显示 GET /video_feed 等信息）
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.ERROR)
        
        self.server_thread = None
        self.running = False
        self._server = None

        # 按键相关
        self.last_key = None
        self.key_lock = threading.Lock()

        self._setup_routes()
        atexit.register(self.stop)

        self.start()  # 实例化时自动启动服务

    def _setup_routes(self):
        """设置 Flask 路由"""
        @self.app.route('/')
        def index():
            # 现代化界面设计 + 右上角固定按键反馈面板
            svg = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32' fill='none'%3E%3Crect width='32' height='32' rx='4' fill='%231a1a2e'/%3E%3Cpath fill='%2300d4ff' d='M9 10h14l-1.5 6H10.5L9 10z'/%3E%3Crect x='7' y='16' width='18' height='8' rx='1.5' fill='%2330cfd0'/%3E%3Ccircle cx='10' cy='22' r='2.5' fill='%2300ff88'/%3E%3Ccircle cx='22' cy='22' r='2.5' fill='%2300ff88'/%3E%3Ccircle cx='16' cy='13' r='1.5' fill='%2300ff88'/%3E%3Cpath stroke='%2300d4ff' stroke-width='1.5' d='M16 8V6'/%3E%3Cpath stroke='%2300d4ff' stroke-width='1.5' d='M19 11.5l1.5-1.5'/%3E%3Cpath stroke='%2300d4ff' stroke-width='1.5' d='M13 11.5l-1.5-1.5'/%3E%3C/svg%3E"

            return f'''
            <html lang="zh-CN">
                <head>
                    <meta charset="UTF-8">
                    <meta name="viewport" content="width=device-width, initial-scale=1.0">
                    <title>智能车监控系统</title>
                    <link rel="icon" type="image/svg+xml" href="{svg}" />
                    <style>
                        * {{
                            margin: 0;
                            padding: 0;
                            box-sizing: border-box;
                        }}

                        body {{
                            font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
                            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
                            min-height: 100vh;
                            color: #e0e0e0;
                            padding: 20px;
                        }}

                        .container {{
                            max-width: 1400px;
                            margin: 0 auto;
                        }}



                        /* 头部样式 */
                        header {{
                            display: flex;
                            justify-content: space-between;
                            align-items: center;
                            margin-bottom: 20px;
                            padding: 15px 20px;
                            background: rgba(255, 255, 255, 0.05);
                            border-radius: 15px;
                            border: 1px solid rgba(255, 255, 255, 0.1);
                            backdrop-filter: blur(10px);
                        }}

                        h1 {{
                            font-size: 1.5em;
                            background: linear-gradient(90deg, #00d4ff, #7b2cbf, #ff006e);
                            -webkit-background-clip: text;
                            -webkit-text-fill-color: transparent;
                            background-clip: text;
                            margin: 0;
                            text-shadow: 0 0 30px rgba(0, 212, 255, 0.3);
                        }}

                        .key-panel-title {{
                            font-size: 0.9em;
                            color: #8892b0;
                            margin: 0;
                            font-weight: normal;
                            display: flex;
                            align-items: center;
                            gap: 10px;
                        }}

                        .key-panel-title .key-icon {{
                            font-size: 2.2em;
                        }}

                        .key-panel-title #floatKeyDisplay {{
                            font-size: 2.2em;
                            font-weight: bold;
                            background: linear-gradient(90deg, #00d4ff, #00ff88);
                            -webkit-background-clip: text;
                            -webkit-text-fill-color: transparent;
                            background-clip: text;
                            transition: all 0.2s ease;
                        }}

                        .key-panel-title #floatKeyDisplay.active {{
                            transform: scale(1.2);
                            text-shadow: 0 0 20px rgba(0, 212, 255, 0.6);
                        }}

                        /* 视频流容器 */
                        .stream-container {{
                            display: flex;
                            justify-content: space-between;
                            gap: 20px;
                            margin-top: 20px;
                            width: 100%;
                        }}

                        .stream-box {{
                            background: rgba(255, 255, 255, 0.03);
                            border: 2px solid rgba(255, 255, 255, 0.1);
                            border-radius: 15px;
                            padding: 15px;
                            flex: 1;
                            min-width: 300px;
                            backdrop-filter: blur(5px);
                            position: relative;
                        }}

                        .stream-box:not(:last-child) {{
                            border-right: 2px solid rgba(0, 212, 255, 0.2);
                            padding-right: 30px;
                        }}

                        .stream-box h3 {{
                            text-align: center;
                            margin-bottom: 15px;
                            color: #ccd6f6;
                            font-size: 1.3em;
                            display: flex;
                            align-items: center;
                            justify-content: center;
                            gap: 10px;
                        }}

                        .stream-box h3::before {{
                            content: '📹';
                        }}

                        .stream-box img {{
                            max-width: 100%;
                            height: auto;
                            border-radius: 10px;
                            border: 2px solid rgba(0, 0, 0, 0.3);
                            box-shadow: 0 5px 20px rgba(0, 0, 0, 0.5);
                        }}

                        /* 底部键盘提示区域 */
                        .keyboard-hint {{
                            margin-top: 40px;
                            text-align: center;
                            color: #667085;
                            font-size: 0.95em;
                        }}

                        .keyboard-hint code {{
                            background: rgba(255, 255, 255, 0.1);
                            padding: 3px 8px;
                            border-radius: 5px;
                            color: #00d4ff;
                        }}

                        /* 页脚 */
                        footer {{
                            text-align: center;
                            margin-top: 30px;
                            color: #495670;
                            font-size: 0.9em;
                        }}

                        /* ✅ 响应式适配：小屏幕自动调整按键面板位置 */
                        @media (max-width: 1100px) {{
                            .key-float-panel {{
                                position: static;
                                margin: 0 auto 20px auto;
                                max-width: 300px;
                            }}
                            
                            .stream-container {{
                                flex-direction: column;
                                align-items: center;
                            }}
                            
                            .stream-box {{
                                width: 100%;
                                max-width: 600px;
                            }}
                            
                            .stream-box:not(:last-child) {{
                                border-right: none;
                                border-bottom: 2px solid rgba(0, 212, 255, 0.2);
                                padding-right: 15px;
                                padding-bottom: 30px;
                                margin-bottom: 20px;
                            }}
                            
                            .stream-box img {{
                                max-width: 100%;
                            }}
                        }}
                    </style>
                </head>
                <body>
                    <div class="container">
                        <header>
                            <h1>🚗 智能车监控系统</h1>
                            <h4 class="key-panel-title"> <span id="floatKeyDisplay"> </span> <span class="key-icon">⌨️</span> </h4>
                        </header>

                        <div class="stream-container">
                            <div class="stream-box">
                                <h3>画面一 (cam1)</h3>
                                <img src="/video_feed/cam1" alt="Camera 1">
                            </div>
                            <div class="stream-box">
                                <h3>画面二 (cam2)</h3>
                                <img src="/video_feed/cam2" alt="Camera 2">
                            </div>
                        </div>



                        <footer>
                            <p>Powered by Flask & OpenCV | Designed for Jetson Nano</p>
                        </footer>
                    </div>

                    <script>
                        let isPageActive = false;
                        
                        // 点击页面激活键盘监听
                        document.body.addEventListener('click', function() {{
                            if (!isPageActive) {{
                                isPageActive = true;
                                console.log('页面已激活键盘监听');
                            }}
                        }});

                        // 键盘按下事件处理
                        document.addEventListener('keydown', function(event) {{
                            if (!isPageActive) return;
                            
                            const key = event.key;
                            console.log('按下了:', key);

                            // 发送按键到服务器
                            fetch('/keypress', {{
                                method: 'POST',
                                headers: {{ 'Content-Type': 'application/json' }},
                                body: JSON.stringify({{ key: key }})
                            }})
                            .then(response => response.json())
                            .then(data => {{
                                // 更新按键显示
                                const keyElement = document.getElementById('floatKeyDisplay');
                                keyElement.innerText = data.received;
                                // 触发按下动画
                                keyElement.classList.remove('active');
                                void keyElement.offsetWidth; // 强制重排触发动画
                                keyElement.classList.add('active');
                            }})
                            .catch(err => console.error('发送失败:', err));

                            // 阻止浏览器默认刷新/调试快捷键
                            if (['F5', 'F12'].includes(event.key)) {{
                                event.preventDefault();
                            }}
                        }});
                    </script>
                </body>
            </html>
            '''

        # 支持通过 cam_id 区分不同摄像头的视频流
        @self.app.route('/video_feed/<cam_id>')
        def video_feed(cam_id):
            return Response(self._generate_frames(cam_id), 
                          mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/health')
        def health():
            return {'status': 'running' if self.running else 'stopped', 
                    'active_cams': list(self.frames.keys()),
                    'port': self.port}

        # 支持清空指定摄像头或所有摄像头的帧
        @self.app.route('/clear')
        def clear():
            cam_id = request.args.get('cam_id')
            with self.frame_lock:
                if cam_id:
                    if cam_id in self.frames:
                        del self.frames[cam_id]
                else:
                    self.frames.clear()
            return {'status': 'cleared', 'cam_id': cam_id or 'all'}

        # 接收按键的接口
        @self.app.route('/keypress', methods=['POST'])
        def keypress():
            data = request.get_json()
            if data and 'key' in data:
                key = data['key']
                with self.key_lock:
                    self.last_key = key
                return jsonify({'status': 'ok', 'received': key})
            return jsonify({'status': 'error', 'message': 'Invalid data'}), 400

    def _generate_frames(self, cam_id):
        """生成器：根据 cam_id 输出对应摄像头的 JPEG 帧"""
        last_frame_time = 0
        while self.running:
            current_time = time.time()
            if current_time - last_frame_time < 1.0 / self.fps:
                time.sleep(0.01)
                continue

            with self.frame_lock:
                frame = self.frames.get(cam_id)  # 从字典获取指定摄像头的帧
                if frame is not None:
                    ret, buffer = cv2.imencode('.jpg', frame, 
                                              [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
                    if ret:
                        frame_bytes = buffer.tobytes()
                        last_frame_time = current_time
                        yield (b'--frame\r\n'
                              b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                else:
                    # 无信号时显示等待画面（标注摄像头 ID）
                    blank = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(blank, f'Waiting for {cam_id}...', (180, 240), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    ret, buffer = cv2.imencode('.jpg', blank)
                    if ret:
                        yield (b'--frame\r\n'
                              b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(0.01)

    def _run_server(self):
        from werkzeug.serving import make_server
        self._server = make_server('0.0.0.0', self.port, self.app, threaded=True)
        self._server.serve_forever()

    def start(self):
        """启动流媒体服务"""
        if self.port in Streamer._instances:
            print(f"⚠️  端口 {self.port} 已被占用，先停止旧服务...")
            Streamer._instances[self.port].stop()

        if self.running:
            print("⚠️  流媒体服务已在运行")
            return

        self.running = True
        self.server_thread = threading.Thread(target=self._run_server, daemon=True)
        self.server_thread.start()

        time.sleep(0.5)
        Streamer._instances[self.port] = self
        self.show_local_info()
        
    def show_local_info(self):
        ip = self._get_local_ip()
        print(f"\n📡 双路流媒体服务已启动-打开链接访问:\n\t http://{ip}:{self.port}/ \n")

    def stop(self):
        """停止流媒体服务"""
        if not self.running:
            return
        self.running = False
        if self._server:
            self._server.shutdown()
        with self.frame_lock:
            self.frames.clear()
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=2)

        if self.port in Streamer._instances:
            del Streamer._instances[self.port]

        try:
            atexit.unregister(self.stop)
        except:
            pass
        print(f"🛑 流媒体服务已停止")

    def update_frame(self, image, cam_id="cam1"):
        """更新指定摄像头的视频帧"""
        if image is None:
            return
        with self.frame_lock:
            self.frames[cam_id] = image.copy()

    def clear_frame(self, cam_id=None):
        """清空指定摄像头或所有帧"""
        with self.frame_lock:
            if cam_id:
                if cam_id in self.frames:
                    del self.frames[cam_id]
            else:
                self.frames.clear()

    def get_key(self, clear=True):
        """获取最后一次按下的键值"""
        with self.key_lock:
            key = self.last_key
            if clear:
                self.last_key = None
            return key

    def _get_local_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


if __name__ == '__main__':
    print("🧪 测试双路 MJPEG 流媒体（页面右上角按键反馈版）...")
    
    # 1. 初始化两个摄像头（index=1对应/dev/cam1，index=2对应/dev/cam2）
    try:
        print("正在初始化摄像头 1 (/dev/cam1)...")
        cap1 = Camera(index=1, width=640, height=480)
        print("正在初始化摄像头 2 (/dev/cam2)...")
        cap2 = Camera(index=2, width=640, height=480)
    except Exception as e:
        print(f"⚠️  摄像头初始化失败: {e}")
        print("   将使用测试图像代替...")
        # 生成测试图像备用
        test_img1 = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(test_img1, 'Camera 1 (/dev/cam1)', (120, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        test_img2 = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(test_img2, 'Camera 2 (/dev/cam2)', (120, 240), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        class DummyCamera:
            def __init__(self, img):
                self.img = img
            def read(self):
                return self.img
            def close(self):
                pass

        cap1 = DummyCamera(test_img1)
        cap2 = DummyCamera(test_img2)
    
    # 2. 初始化单端口流媒体服务
    streamer = Streamer(port=5000, fps=30)
    
    try:
        frame_count = 0
        while True:
            # 3. 处理按键逻辑
            key = streamer.get_key()
            if key:
                print(f"用户按下了: {key}")
                if key == 'q':
                    print("收到退出指令...")
                    break

            # 4. 读取两个摄像头的纯净画面（无额外绘制）
            frame1 = cap1.read()
            frame2 = cap2.read()
            
            # 仅保留画面左上角的帧计数，不添加其他内容
            if frame1 is not None:
                cv2.putText(frame1, f'/dev/cam1 | Frame: {frame_count}', (20, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                streamer.update_frame( frame1,'cam1')
            
            if frame2 is not None:
                cv2.putText(frame2, f'/dev/cam2 | Frame: {frame_count}', (20, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                streamer.update_frame( frame2,'cam2')
            
            frame_count += 1
            time.sleep(0.033)  # 约 30 FPS
    except KeyboardInterrupt:
        print("\n⚠️  收到中断信号 (Ctrl+C)")
    finally:
        streamer.stop()
        # 关闭摄像头
        try:
            cap1.close()
            cap2.close()
        except:
            pass
        print("✅ 测试完成")