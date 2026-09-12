import os
import sys
import math
import threading
import time
import traceback
from abc import abstractmethod

import cv2
import glfw
from PIL import Image
import OpenGL.GL as gl

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from discoverse import DISCOVERSE_ASSETS_DIR
from discoverse.utils import BaseConfig, get_screen_scale

SIMULATION_WINDOW_TITLE = "Shentoon RobotStudio"

if sys.platform == "linux":
    try:
        import torch
        from gaussian_renderer.gs_renderer_mujoco import GSRendererMuJoCo
        DISCOVERSE_GAUSSIAN_RENDERER = True

    except ImportError:
        print("Warning: gaussian_splatting renderer not found. Please install the required packages to use it.")
        DISCOVERSE_GAUSSIAN_RENDERER = False
else:
    DISCOVERSE_GAUSSIAN_RENDERER = False

def setRenderOptions(options):
    options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True
    options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    # options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    # options.flags[mujoco.mjtVisFlag.mjVIS_COM] = True
    # options.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True
    # options.flags[mujoco.mjtVisFlag.mjVIS_PERTOBJ] = True
    options.frame = mujoco.mjtFrame.mjFRAME_BODY.value
    pass

class SimulatorBase:
    running = True
    obs = None
    img_rgb_obs_s = {}
    img_depth_obs_s = {}
    img_rgb_native_aruco_s = {}
    free_body_qpos_ids = {}

    cam_id = -1  # -1表示自由视角
    last_cam_id = -1
    render_cnt = 0
    camera_names = []
    camera_pose_changed = False
    camera_rmat = np.array([
        [ 0,  0, -1],
        [-1,  0,  0],
        [ 0,  1,  0],
    ])

    use_default_window_size = False
    mouse_pressed = {
        'left': False,
        'right': False,
        'middle': False
    }
    mouse_pos = {
        'x': 0,
        'y': 0
    }

    options = mujoco.MjvOption()

    def __init__(self, config:BaseConfig):
        self.config = config

        if self.config.mjcf_file_path.startswith("/"):
            self.mjcf_file = self.config.mjcf_file_path
        elif os.path.exists(self.config.mjcf_file_path):
            self.mjcf_file = self.config.mjcf_file_path
        else:
            self.mjcf_file = os.path.join(DISCOVERSE_ASSETS_DIR, self.config.mjcf_file_path)
        if os.path.exists(self.mjcf_file):
            print("mjcf found: {}".format(self.mjcf_file))
        else:
            print("\033[0;31;40mFailed to load mjcf: {}\033[0m".format(self.mjcf_file))
            raise FileNotFoundError("Failed to load mjcf: {}".format(self.mjcf_file))
        self.load_mjcf()
        self.decimation = self.config.decimation
        self.delta_t = self.mj_model.opt.timestep * self.decimation
        self.render_fps = self.config.render_set["fps"]
        # 3DGS is expensive on the X11 path. Keep ROS camera data/physics
        # responsive by limiting render() calls independently of physics.
        render_limit = os.getenv("SUPERMARKET_RENDER_FPS")
        try:
            self.render_fps = max(1.0, float(render_limit)) if render_limit else float(self.render_fps)
        except ValueError:
            print(f"Invalid SUPERMARKET_RENDER_FPS={render_limit!r}; using configured fps", flush=True)
        self._next_render_sim_time = 0.0
        self.img_rgb_native_aruco_s = {}
        self.batch_render_results = {}
        self._gs_async_enabled = (
            os.getenv("SUPERMARKET_GS_ASYNC", "1").strip().lower()
            in {"1", "true", "yes", "on"}
        )
        # 手眼相机（left=1/right=2）走原生 MuJoCo 渲染进 ROS 话题，不进 GS
        # 批量：伺服与三视角审查需要手眼图像，但 GS 批量渲三路会把异步渲染
        # 线程压满（实测 RTF 掉到 0.3）。原生小图开销极低（GUI 手眼切换同款
        # 渲染路径，主人实测不卡）。SUPERMARKET_HAND_EYE_NATIVE=0 可关闭。
        self._native_obs_cam_ids = (
            {1, 2}
            if os.getenv("SUPERMARKET_HAND_EYE_NATIVE", "1").strip().lower()
            in {"1", "true", "yes", "on"}
            else set()
        )
        self._gs_render_lock = threading.Lock()
        self._gs_render_thread = None
        # 手眼渲染阶段开关：executor 在 creep 前写标志文件、retreat 后删除。
        # 常开原生手眼会把 RTF 拖到 0.22（2026-09-12 实测），阶段开关让
        # 扫描/导航阶段零开销。文件在 baseline 挂载卷上，两端容器都可见。
        self._hand_eye_flag_path = "/workspace/baseline/debug_data/handeye_render.flag"

        if self.config.enable_render:
            self.free_camera = mujoco.MjvCamera()
            self.free_camera.fixedcamid = -1
            self.free_camera.type = mujoco._enums.mjtCamera.mjCAMERA_FREE
            mujoco.mjv_defaultFreeCamera(self.mj_model, self.free_camera)

            self.config.use_gaussian_renderer = self.config.use_gaussian_renderer and DISCOVERSE_GAUSSIAN_RENDERER
            if self.config.use_gaussian_renderer:
                from discoverse.utils.download_from_huggingface import download_from_huggingface
                hf_repo_id = getattr(self.config, 'hf_repo_id', 'tatp/DISCOVERSE-models')
                for name, path in self.config.gs_model_dict.items():
                    if not os.path.isabs(path):
                        abs_path = os.path.join(DISCOVERSE_ASSETS_DIR, "3dgs", path)
                        if os.path.exists(abs_path):
                            self.config.gs_model_dict[name] = abs_path
                        else:
                            self.config.gs_model_dict[name] = download_from_huggingface(path, hf_repo_id)
                    elif not os.path.exists(path):
                        print(f"Warning: Model {name} path {path} is absolute and not found locally.")

                self.gs_renderer = GSRendererMuJoCo(self.config.gs_model_dict, self.mj_model)
                self.last_cam_id = self.cam_id
                self.show_gaussian_img = True

        self.window = None
        self.glfw_initialized = False
        
        if not hasattr(self.config.render_set, "window_title"):
            self.config.render_set["window_title"] = SIMULATION_WINDOW_TITLE
        
        if self.config.enable_render and not self.config.headless:
            try:
                if not glfw.init():
                    raise RuntimeError("无法初始化GLFW")
                self.glfw_initialized = True
                
                # 设置OpenGL版本和窗口属性
                glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 2)
                glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
                glfw.window_hint(glfw.VISIBLE, True)

                # 如果设置了use_default_window_size，禁用窗口最大化功能
                if self.use_default_window_size:
                    # 禁用窗口最大化
                    glfw.window_hint(glfw.MAXIMIZED, False)
                    # 确保窗口有装饰（标题栏等）
                    glfw.window_hint(glfw.DECORATED, True)
                    # 允许用户手动调整窗口大小
                    glfw.window_hint(glfw.RESIZABLE, True)
                    print("已禁用窗口最大化功能，但允许调整窗口大小")

                # 创建窗口
                self.window = glfw.create_window(
                    self.config.render_set["width"],
                    self.config.render_set["height"],
                    self.config.render_set.get("window_title", SIMULATION_WINDOW_TITLE),
                    None, None
                )
                
                if not self.window:
                    glfw.terminate()
                    raise RuntimeError("无法创建GLFW窗口")
                
                # WSLg/GLFW 可能恢复到上一次的屏幕外坐标；显式放回主屏左上角。
                # 这不会改变仿真尺寸，只修复窗口可见性。
                glfw.set_window_pos(self.window, 40, 40)
                glfw.show_window(self.window)
                glfw.focus_window(self.window)
                glfw.make_context_current(self.window)
                glfw.swap_interval(1)

                # 设置窗口最大尺寸
                glfw.set_window_size_limits(self.window, 320, 240, self.mj_model.vis.global_.offwidth, self.mj_model.vis.global_.offheight)

                # 初始化OpenGL设置
                gl.glClearColor(0.0, 0.0, 0.0, 1.0)
                gl.glShadeModel(gl.GL_SMOOTH)
                gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
                
                # 设置回调
                glfw.set_key_callback(self.window, self.on_key)
                glfw.set_cursor_pos_callback(self.window, self.on_mouse_move)
                glfw.set_mouse_button_callback(self.window, self.on_mouse_button)
                glfw.set_scroll_callback(self.window, self.on_mouse_scroll)
                
                # 如果设置了use_default_window_size，添加窗口大小变化回调
                if self.use_default_window_size:
                    glfw.set_window_maximize_callback(self.window, self.maximize_callback)

                if sys.platform == "darwin":
                    self.screen_scale = get_screen_scale(0)
                    gl.glPixelZoom(self.screen_scale, self.screen_scale)
                else:
                    self.screen_scale = 1

                # 注册清理函数
                import atexit
                atexit.register(self._cleanup_before_exit)

            except Exception as e:
                print(f"GLFW初始化失败: {e}")
                if self.glfw_initialized:
                    glfw.terminate()
                self.config.headless = True
                self.window = None

        self.last_render_time = time.time()
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        mujoco.mj_forward(self.mj_model, self.mj_data)

    def maximize_callback(self, window, maximized):
        # WSLg may not expose a primary monitor through screeninfo.
        # Do not immediately restore a maximized GLFW window; let the user resize it.
        return

    def object_pose(self, body_name):
        """获取物体的位姿（位置xyz和朝向wxyz）"""
        try:
            qid = self.mj_model.jnt_qposadr[self.free_body_qpos_ids[body_name]]
            return self.mj_data.qpos[qid:qid+7][...]
        except KeyError:
            raise KeyError(f"Body name '{body_name}' not found in free_body_qpos_ids. Available bodies: {list(self.free_body_qpos_ids.keys())}")
    
    def get_joint_position(self, joint_name):
        return self.mj_data.qpos[self.mj_model.joint(joint_name).qposadr]
    
    def set_joint_position(self, joint_name, value):
        self.mj_data.qpos[self.mj_model.joint(joint_name).qposadr] = value

    def load_mjcf(self):
        if self.mjcf_file.endswith(".xml"):
            self.mj_model = mujoco.MjModel.from_xml_path(self.mjcf_file)
        elif self.mjcf_file.endswith(".mjb"):
            self.mj_model = mujoco.MjModel.from_binary_path(self.mjcf_file)
        self.mj_model.opt.timestep = self.config.timestep
        # self.mj_model.vis.quality.shadowsize = 4096 * 8
        self.mj_data = mujoco.MjData(self.mj_model)
        if self.config.enable_render:
            for i in range(self.mj_model.ncam):
                self.camera_names.append(self.mj_model.camera(i).name)

            if type(self.config.obs_rgb_cam_id) is int:
                assert -2 < self.config.obs_rgb_cam_id < len(self.camera_names), "Invalid obs_rgb_cam_id {}".format(self.config.obs_rgb_cam_id)
                tmp_id = self.config.obs_rgb_cam_id
                self.config.obs_rgb_cam_id = [tmp_id]
            elif type(self.config.obs_rgb_cam_id) is list:
                for cam_id in self.config.obs_rgb_cam_id:
                    assert -2 < cam_id < len(self.camera_names), "Invalid obs_rgb_cam_id {}".format(cam_id)
            elif self.config.obs_rgb_cam_id is None:
                self.config.obs_rgb_cam_id = []
            
            if type(self.config.obs_depth_cam_id) is int:
                assert -2 < self.config.obs_depth_cam_id < len(self.camera_names), "Invalid obs_depth_cam_id {}".format(self.config.obs_depth_cam_id)
            elif type(self.config.obs_depth_cam_id) is list:
                for cam_id in self.config.obs_depth_cam_id:
                    assert -2 < cam_id < len(self.camera_names), "Invalid obs_depth_cam_id {}".format(cam_id)
            elif self.config.obs_depth_cam_id is None:
                self.config.obs_depth_cam_id = []
        
            # WSLg/screeninfo may return an empty monitor list without raising.
            # Set a safe default before enumeration so GUI mode never references
            # an uninitialized screen_width/screen_height pair.
            screen_width, screen_height = 1920, 1080
            self.use_default_window_size = True
            try:
                import screeninfo
                monitors = screeninfo.get_monitors()
                for m in monitors:
                    if m.is_primary:
                        screen_width, screen_height = m.width, m.height
                        self.use_default_window_size = False
                        break
                if not monitors:
                    print(f"screeninfo returned no monitors, using default screen size: {screen_width}x{screen_height}")
                elif self.use_default_window_size:
                    print(f"screeninfo found no primary monitor, using default screen size: {screen_width}x{screen_height}")
            except Exception as e:
                print(f"screeninfo error: {e}, using default screen size: {screen_width}x{screen_height}")

            self.mj_model.vis.global_.offwidth = max(self.mj_model.vis.global_.offwidth, screen_width)
            self.mj_model.vis.global_.offheight = max(self.mj_model.vis.global_.offheight, screen_height)

            # GUI 观察模式可以使用更高的内部渲染分辨率，避免大窗口把 640x480 放大后变糊。
            # 不设置环境变量时保持官方/比赛默认分辨率，不影响主线传感器链路。
            if not self.config.headless:
                gui_width = os.getenv("SUPERMARKET_GUI_RENDER_WIDTH")
                gui_height = os.getenv("SUPERMARKET_GUI_RENDER_HEIGHT")
                if gui_width and gui_height:
                    try:
                        requested_width = max(320, int(gui_width))
                        requested_height = max(240, int(gui_height))
                        self.config.render_set["width"] = requested_width
                        self.config.render_set["height"] = requested_height
                        print(
                            f"GUI render override: {requested_width}x{requested_height}",
                            flush=True,
                        )
                    except ValueError:
                        print(
                            "Invalid GUI render override; using configured render size",
                            flush=True,
                        )

            # The X server's visible screen may be smaller than the requested
            # internal render size. MuJoCo still needs an offscreen framebuffer
            # at least as large as the image passed to Renderer.
            requested_width = int(self.config.render_set["width"])
            requested_height = int(self.config.render_set["height"])
            self.mj_model.vis.global_.offwidth = max(
                self.mj_model.vis.global_.offwidth, requested_width
            )
            self.mj_model.vis.global_.offheight = max(
                self.mj_model.vis.global_.offheight, requested_height
            )
            print(
                f"GUI framebuffer: {self.mj_model.vis.global_.offwidth}x"
                f"{self.mj_model.vis.global_.offheight}; render={requested_width}x{requested_height}",
                flush=True,
            )

            self.renderer = mujoco.Renderer(self.mj_model, requested_height, requested_width)

        for i in range(self.mj_model.nbody):
            if len(self.mj_model.body(i).name) and self.mj_model.body(i).dofnum == 6:
                jq_id = np.where(self.mj_model.jnt_bodyid == self.mj_model.body(i).id)[0]
                if jq_id.size:
                    self.free_body_qpos_ids[self.mj_model.body(i).name] = int(jq_id[0])

        self.post_load_mjcf()

    def post_load_mjcf(self):
        pass

    def update_renderer_window_size(self, width, height):
        self.renderer._width = width
        self.renderer._height = height
        self.renderer._rect.width = width
        self.renderer._rect.height = height

    def get_current_window_size(self):
        width = int(self.config.render_set["width"])
        height = int(self.config.render_set["height"])

        if not self.config.headless and self.window is not None:
            fb_width, fb_height = glfw.get_framebuffer_size(self.window)
            if fb_width > 0 and fb_height > 0:
                screen_scale = getattr(self, "screen_scale", 1)
                width = max(1, int(fb_width / screen_scale))
                height = max(1, int(fb_height / screen_scale))

        return width, height

    @staticmethod
    def _get_camera_frame(collection, camera_id):
        """Safely read a camera frame from dict/list/tuple containers.

        The async 3DGS path may not have produced the first camera-0 frame yet;
        a missing cache entry must not terminate the physics loop.
        """
        if collection is None:
            return None
        if isinstance(collection, dict):
            frame = collection.get(camera_id)
            if frame is None:
                frame = collection.get(str(camera_id))
            return frame
        try:
            return collection[camera_id]
        except (IndexError, KeyError, TypeError):
            return None

    def _convert_gs_render_results(self, results_tensor):
        """Normalize GS renderer results to ``{camera_id: (rgb, depth)}``.

        Installed gsplat versions have returned either a camera-keyed dict or
        a batched ``(rgb, depth)`` pair. Treat an empty/unsupported result as
        an empty frame set so the caller can use its safe fallback image.
        """
        if results_tensor is None:
            return {}
        if isinstance(results_tensor, dict):
            items = results_tensor.items()
        elif isinstance(results_tensor, (tuple, list)) and len(results_tensor) == 2:
            rgb_batch, depth_batch = results_tensor
            try:
                items = ((index, (rgb_batch[index], depth_batch[index])) for index in range(len(rgb_batch)))
            except (TypeError, AttributeError):
                return {}
        else:
            # ``render([])`` in some package versions returns three empty
            # dictionaries; no camera frame is available in that case.
            return {}

        results = {}
        for raw_cid, value in items:
            try:
                cid = int(raw_cid)
                rgb_tensor, depth_tensor = value[:2]
            except (TypeError, ValueError, IndexError, KeyError):
                continue
            if torch.is_tensor(rgb_tensor):
                rgb = (255. * torch.clamp(rgb_tensor, 0.0, 1.0)).to(torch.uint8).cpu().numpy()
            else:
                rgb = np.clip(np.asarray(rgb_tensor), 0.0, 1.0)
                rgb = np.asarray(255. * rgb, dtype=np.uint8)
            depth = depth_tensor.cpu().numpy() if torch.is_tensor(depth_tensor) else np.asarray(depth_tensor)
            results[cid] = (rgb, depth)
        return results

    def update_texture(self, texture_name, mtl_img_pil, no_render=False):
        """更新纹理"""
        if not hasattr(self, 'renderer') or self.renderer is None:
            print(f"Renderer not initialized, cannot update texture: {texture_name}")
            return False

        try:
            tex_id = self.renderer.model.tex(texture_name).id
        except Exception as e:
            print(f"Texture '{texture_name}' not found: {e}")
            return False
        
        if not no_render:
            self.renderer.update_scene(self.mj_data, self.free_camera, self.options)
            self.renderer.render()
        
        tex_bind_id = self.renderer._mjr_context.texture[tex_id]
        gl.glBindTexture(gl.GL_TEXTURE_2D, tex_bind_id)
        
        try:
            width = gl.glGetTexLevelParameteriv(gl.GL_TEXTURE_2D, 0, gl.GL_TEXTURE_WIDTH)
            height = gl.glGetTexLevelParameteriv(gl.GL_TEXTURE_2D, 0, gl.GL_TEXTURE_HEIGHT)
        except Exception as e:
            print(f"Error getting texture dimensions: {e}")
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
            return False

        try:
            if mtl_img_pil.mode != 'RGB':
                mtl_img_pil = mtl_img_pil.convert('RGB')

            if mtl_img_pil.size != (width, height):
                mtl_img_pil = mtl_img_pil.resize((width, height), Image.Resampling.LANCZOS)
            
            mtl_img = np.array(mtl_img_pil)
            mtl_img = np.flipud(mtl_img)
            mtl_img = np.ascontiguousarray(mtl_img, dtype=np.uint8)

            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)

            gl.glTexSubImage2D(gl.GL_TEXTURE_2D, 0, 0, 0, width, height, 
                              gl.GL_RGB, gl.GL_UNSIGNED_BYTE, mtl_img.tobytes())
            
        except Exception as e:
            print(f"Error processing image for texture '{texture_name}': {e}")
            return False
        finally:
            gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
            
        return True

    def _schedule_async_gs_render(self, render_width: int, render_height: int) -> None:
        """Snapshot MuJoCo state and render 3DGS without blocking physics."""
        native_ids = getattr(self, "_native_obs_cam_ids", ())
        obs_cam_ids = sorted(set(self.config.obs_rgb_cam_id + self.config.obs_depth_cam_id))
        # 原生手眼不进 GS 批量：它们的图像由 render() 用 getRgbImg/getDepthImg 填充
        obs_cam_ids = [cid for cid in obs_cam_ids if cid not in native_ids]
        if not obs_cam_ids or not hasattr(self, "gs_renderer"):
            return
        with self._gs_render_lock:
            if self._gs_render_thread is not None and self._gs_render_thread.is_alive():
                return
            body_ids = np.asarray(self.gs_renderer.gs_body_ids, dtype=np.int32)
            body_pos = self.mj_data.xpos[body_ids].copy()
            body_quat = self.mj_data.xquat[body_ids].copy()
            cam_ids = np.asarray(obs_cam_ids, dtype=np.int32)
            cam_pos = self.mj_data.cam_xpos[cam_ids].copy()
            cam_xmat = self.mj_data.cam_xmat[cam_ids].copy()
            fovy = self.mj_model.cam_fovy[cam_ids].copy()
            self._gs_render_thread = threading.Thread(
                target=self._async_gs_render_worker,
                args=(body_pos, body_quat, cam_pos, cam_xmat, fovy, render_width, render_height, obs_cam_ids),
                name="gs-render",
                daemon=True,
            )
            self._gs_render_thread.start()

    def _async_gs_render_worker(
        self,
        body_pos: np.ndarray,
        body_quat: np.ndarray,
        cam_pos: np.ndarray,
        cam_xmat: np.ndarray,
        fovy: np.ndarray,
        render_width: int,
        render_height: int,
        cam_ids: list[int],
    ) -> None:
        try:
            self.gs_renderer.update_gaussian_properties(body_pos, body_quat)
            rgb_tensor, depth_tensor = self.gs_renderer.render_batch(
                cam_pos, cam_xmat, render_height, render_width, fovy
            )
            tensor_results = {
                cid: (rgb_tensor[index], depth_tensor[index])
                for index, cid in enumerate(cam_ids)
            }
            results = self._convert_gs_render_results(tensor_results)
            with self._gs_render_lock:
                self.batch_render_results = results
                for cid, (rgb, depth) in results.items():
                    self.img_rgb_obs_s[cid] = rgb
                    self.img_depth_obs_s[cid] = depth
        except Exception as exc:
            print(f"[simulator] async 3DGS render failed: {type(exc).__name__}: {exc}", flush=True)

    def render(self):
        self.render_cnt += 1

        render_width = int(self.config.render_set["width"])
        render_height = int(self.config.render_set["height"])
        window_width, window_height = self.get_current_window_size()
        display_result = None

        self.update_renderer_window_size(render_width, render_height)
        display_cam_id = self.cam_id if not self.config.headless and self.window is not None else None
        if self.config.use_gaussian_renderer and self.show_gaussian_img:
            if self._gs_async_enabled:
                self._schedule_async_gs_render(render_width, render_height)
                with self._gs_render_lock:
                    latest_results = dict(self.batch_render_results)
                if display_cam_id is not None:
                    display_result = latest_results.get(display_cam_id)
            else:
                obs_cam_ids = list(set(self.config.obs_rgb_cam_id + self.config.obs_depth_cam_id))
                render_cam_ids = obs_cam_ids.copy()
                if display_cam_id is not None and (window_width, window_height) == (render_width, render_height):
                    if display_cam_id not in render_cam_ids:
                        render_cam_ids.append(display_cam_id)

                if len(render_cam_ids) > 0 or display_cam_id is not None:
                    if -1 in render_cam_ids or display_cam_id == -1:
                        self.renderer.update_scene(self.mj_data, self.free_camera, self.options)

                    self.gs_renderer.update_gaussians(self.mj_data)
                    self.batch_render_results = {}

                    if len(render_cam_ids) > 0:
                        if getattr(self.config, "gs_render_sequential", False):
                            for render_cam_id in render_cam_ids:
                                results_tensor = self.gs_renderer.render(
                                    self.mj_model,
                                    self.mj_data,
                                    [render_cam_id],
                                    render_width,
                                    render_height,
                                    self.free_camera,
                                )
                                self.batch_render_results.update(
                                    self._convert_gs_render_results(results_tensor)
                                )
                        else:
                            results_tensor = self.gs_renderer.render(
                                self.mj_model,
                                self.mj_data,
                                render_cam_ids,
                                render_width,
                                render_height,
                                self.free_camera,
                            )
                            self.batch_render_results = self._convert_gs_render_results(results_tensor)

                    if display_cam_id is not None:
                        if (window_width, window_height) == (render_width, render_height) and display_cam_id in self.batch_render_results:
                            display_result = self.batch_render_results[display_cam_id]
                        else:
                            window_results_tensor = self.gs_renderer.render(
                                self.mj_model,
                                self.mj_data,
                                [display_cam_id],
                                window_width,
                                window_height,
                                self.free_camera,
                            )
                            display_result = self._convert_gs_render_results(window_results_tensor).get(display_cam_id)

                    for cid, (rgb, depth) in self.batch_render_results.items():
                        self.img_rgb_obs_s[cid] = rgb
                        self.img_depth_obs_s[cid] = depth

            # 原生手眼填充：left/right 相机每 tick 用 getRgbImg/getDepthImg
            # 渲染进 ROS 话题（伺服与审查用）。原生小图开销极低，不走 GS 批量。
            # ⚠️ 必须每 tick 无条件填充：obs 列表含 cam 1/2 时，下游 obs 字典
            # 需要这些键常驻——曾经用 handeye_render.flag 门控，标志关闭期间
            # 缓冲不填充导致 obs 管线异常（地图只收到 4/45 槽位，实测）。
            native_obs = sorted(getattr(self, "_native_obs_cam_ids", ()) or ())
            if native_obs:
                rgb_ids = [nid for nid in native_obs if nid in self.config.obs_rgb_cam_id]
                depth_ids = [nid for nid in native_obs if nid in self.config.obs_depth_cam_id]
                depth_rendering = self.renderer._depth_rendering
                if rgb_ids:
                    self.renderer.disable_depth_rendering()
                    try:
                        for nid in rgb_ids:
                            self.img_rgb_obs_s[nid] = self.getRgbImg(nid)
                    finally:
                        self.renderer._depth_rendering = depth_rendering
                if depth_ids:
                    self.renderer.enable_depth_rendering()
                    try:
                        for nid in depth_ids:
                            self.img_depth_obs_s[nid] = self.getDepthImg(nid)
                    finally:
                        self.renderer._depth_rendering = depth_rendering

            # 3DGS RGB does not contain the MJCF ArUco tiles. If requested,
            # render native ArUco frames on the physics/GL thread only.
            if os.getenv("SUPERMARKET_PUBLISH_ARUCO_NATIVE", "0").strip().lower() in {"1", "true", "yes", "on"}:
                native_names = {
                    item.strip().lower()
                    for item in os.getenv("SUPERMARKET_ARUCO_NATIVE_CAMERAS", "head").split(",")
                    if item.strip()
                }
                native_ids = [
                    index
                    for index, name in enumerate(("head", "left", "right"))
                    if name in native_names and index in self.config.obs_rgb_cam_id
                ]
                depth_rendering = self.renderer._depth_rendering
                self.renderer.disable_depth_rendering()
                try:
                    for native_id in native_ids:
                        self.img_rgb_native_aruco_s[native_id] = self.getRgbImg(native_id)
                finally:
                    self.renderer._depth_rendering = depth_rendering

        else:
            depth_rendering = self.renderer._depth_rendering
            self.renderer.disable_depth_rendering()
            for id in self.config.obs_rgb_cam_id:
                img = self.getRgbImg(id)
                self.img_rgb_obs_s[id] = img
                self.img_rgb_native_aruco_s[id] = img
            
            self.renderer.enable_depth_rendering()
            for id in self.config.obs_depth_cam_id:
                img = self.getDepthImg(id)
                self.img_depth_obs_s[id] = img
            self.renderer._depth_rendering = depth_rendering
        
        if not self.config.headless and self.window is not None:
            # 保持 MuJoCo 内部渲染器使用固定的传感器分辨率；窗口放大只在最终显示阶段缩放。
            # 不能在这里直接改 Renderer 的私有尺寸字段，否则 WSLg 放大窗口时会产生花屏。
            if not self.renderer._depth_rendering:
                if self.config.use_gaussian_renderer and self.show_gaussian_img and display_result is not None:
                    img_vis = display_result[0]
                else:
                    cached_rgb = self._get_camera_frame(self.img_rgb_obs_s, self.cam_id)
                    if cached_rgb is not None and self.cam_id in self.config.obs_rgb_cam_id and (window_width, window_height) == (render_width, render_height):
                        img_vis = cached_rgb
                    else:
                        # Async GS can miss the first frame; use the native
                        # MuJoCo camera until a GS frame is available.
                        img_vis = self.getRgbImg(self.cam_id)
            else:
                if self.config.use_gaussian_renderer and self.show_gaussian_img and display_result is not None:
                    img_depth = display_result[1]
                else:
                    cached_depth = self._get_camera_frame(self.img_depth_obs_s, self.cam_id)
                    if cached_depth is not None and self.cam_id in self.config.obs_depth_cam_id and (window_width, window_height) == (render_width, render_height):
                        img_depth = cached_depth
                    else:
                        # Async GS can miss the first depth frame; use the
                        # native MuJoCo depth camera until a GS frame exists.
                        img_depth = self.getDepthImg(self.cam_id)
                
                if img_depth is not None:
                    img_vis = cv2.applyColorMap(cv2.convertScaleAbs(img_depth, alpha=255./self.config.max_render_depth), cv2.COLORMAP_JET)
                else:
                    img_vis = None

            try:
                if glfw.window_should_close(self.window):
                    # XLaunch/XWayland 偶发把窗口表面报告为“已关闭”，但窗口实际仍可继续显示。
                    # 默认把它视为瞬态事件：记录、清除关闭标志并恢复窗口；只有显式
                    # SUPERMARKET_GUI_EXIT_ON_CLOSE=1 才允许窗口关闭终止仿真。
                    self.gui_close_events = getattr(self, "gui_close_events", 0) + 1
                    print(
                        f"[simulator] GUI close event #{self.gui_close_events}; "
                        f"exit_on_close={os.getenv('SUPERMARKET_GUI_EXIT_ON_CLOSE', '0')}",
                        flush=True,
                    )
                    # A transient X11/XWayland close flag is a display event,
                    # not a simulation shutdown request.  The Server entry
                    # point owns shutdown through stop_event/ROS Context.
                    if os.getenv("SUPERMARKET_GUI_EXIT_ON_CLOSE", "0") == "1":
                        print("[simulator] GUI exit request ignored; use Server stop_event to shut down", flush=True)
                    glfw.set_window_should_close(self.window, False)
                    glfw.set_window_pos(self.window, 40, 40)
                    glfw.show_window(self.window)
                    glfw.focus_window(self.window)
                    
                glfw.make_context_current(self.window)
                fb_width, fb_height = glfw.get_framebuffer_size(self.window)
                gl.glViewport(0, 0, fb_width, fb_height)
                gl.glClear(gl.GL_COLOR_BUFFER_BIT)

                if img_vis is not None:
                    # WSLg 对 glPixelZoom + glDrawPixels 的放大兼容性不稳定，改用标准纹理四边形显示。
                    # MuJoCo 始终按固定传感器分辨率渲染，窗口大小只影响最终贴图区域。
                    img_vis = img_vis[::-1]
                    img_vis = np.ascontiguousarray(img_vis)
                    src_height, src_width = img_vis.shape[:2]
                    if src_width > 0 and src_height > 0:
                        if not hasattr(self, "display_texture") or self.display_texture is None:
                            self.display_texture = gl.glGenTextures(1)

                        pixel_format = gl.GL_RGBA if img_vis.shape[2] == 4 else gl.GL_RGB
                        internal_format = gl.GL_RGBA8 if img_vis.shape[2] == 4 else gl.GL_RGB8
                        gl.glBindTexture(gl.GL_TEXTURE_2D, self.display_texture)
                        gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT, 1)
                        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
                        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
                        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_CLAMP_TO_EDGE)
                        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_CLAMP_TO_EDGE)
                        gl.glTexImage2D(
                            gl.GL_TEXTURE_2D,
                            0,
                            internal_format,
                            src_width,
                            src_height,
                            0,
                            pixel_format,
                            gl.GL_UNSIGNED_BYTE,
                            img_vis.tobytes(),
                        )

                        scale = min(fb_width / src_width, fb_height / src_height)
                        draw_width = max(1, int(src_width * scale))
                        draw_height = max(1, int(src_height * scale))
                        draw_left = max(0, (fb_width - draw_width) // 2)
                        draw_bottom = max(0, (fb_height - draw_height) // 2)

                        gl.glViewport(0, 0, fb_width, fb_height)
                        gl.glDisable(gl.GL_DEPTH_TEST)
                        gl.glEnable(gl.GL_TEXTURE_2D)

                        gl.glMatrixMode(gl.GL_PROJECTION)
                        gl.glPushMatrix()
                        gl.glLoadIdentity()
                        gl.glOrtho(0, fb_width, 0, fb_height, -1, 1)
                        gl.glMatrixMode(gl.GL_MODELVIEW)
                        gl.glPushMatrix()
                        gl.glLoadIdentity()

                        gl.glBegin(gl.GL_QUADS)
                        gl.glTexCoord2f(0.0, 0.0)
                        gl.glVertex2f(draw_left, draw_bottom)
                        gl.glTexCoord2f(1.0, 0.0)
                        gl.glVertex2f(draw_left + draw_width, draw_bottom)
                        gl.glTexCoord2f(1.0, 1.0)
                        gl.glVertex2f(draw_left + draw_width, draw_bottom + draw_height)
                        gl.glTexCoord2f(0.0, 1.0)
                        gl.glVertex2f(draw_left, draw_bottom + draw_height)
                        gl.glEnd()

                        gl.glMatrixMode(gl.GL_MODELVIEW)
                        gl.glPopMatrix()
                        gl.glMatrixMode(gl.GL_PROJECTION)
                        gl.glPopMatrix()
                        gl.glMatrixMode(gl.GL_MODELVIEW)
                        gl.glDisable(gl.GL_TEXTURE_2D)
                        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
                
                glfw.swap_buffers(self.window)
                glfw.poll_events()
                
                if self.config.sync:
                    current_time = time.time()
                    wait_time = max(1.0/self.render_fps - (current_time - self.last_render_time), 0)
                    if wait_time > 0:
                        time.sleep(wait_time)
                    self.last_render_time = time.time()
                    
            except Exception as e:
                print(f"渲染错误: {e}")

    def getRgbImg(self, cam_id):
        if cam_id == -1:
            self.renderer.update_scene(self.mj_data, self.free_camera, self.options)
        elif cam_id > -1:
            self.renderer.update_scene(self.mj_data, self.camera_names[cam_id], self.options)
        else:
            return None
        rgb_img = self.renderer.render()
        return rgb_img

    def getDepthImg(self, cam_id):
        if cam_id == -1:
            self.renderer.update_scene(self.mj_data, self.free_camera, self.options)
        elif cam_id > -1:
            self.renderer.update_scene(self.mj_data, self.camera_names[cam_id], self.options)
        else:
            return None
        depth_img = self.renderer.render()
        return depth_img

    def on_mouse_move(self, window, xpos, ypos):
        if self.cam_id == -1:
            dx = xpos - self.mouse_pos['x']
            dy = ypos - self.mouse_pos['y']
            _, height = self.get_current_window_size()
            
            action = None
            if self.mouse_pressed['left']:
                action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
            elif self.mouse_pressed['right']:
                action = mujoco.mjtMouse.mjMOUSE_MOVE_V
            elif self.mouse_pressed['middle']:
                action = mujoco.mjtMouse.mjMOUSE_ZOOM

            if action is not None:
                self.camera_pose_changed = True
                mujoco.mjv_moveCamera(self.mj_model,  action,  dx/height,  dy/height, self.renderer.scene, self.free_camera)

        self.mouse_pos['x'] = xpos
        self.mouse_pos['y'] = ypos

    def on_mouse_button(self, window, button, action, mods):
        is_pressed = action == glfw.PRESS
        
        if button == glfw.MOUSE_BUTTON_LEFT:
            self.mouse_pressed['left'] = is_pressed
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            self.mouse_pressed['right'] = is_pressed
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            self.mouse_pressed['middle'] = is_pressed

    def on_mouse_scroll(self, window, xoffset, yoffset):
        self.free_camera.distance -= yoffset * 0.1
        if self.free_camera.distance < 0.1:
            self.free_camera.distance = 0.1

    def on_key(self, window, key, scancode, action, mods):
        if action == glfw.PRESS:
            is_ctrl_pressed = (mods & glfw.MOD_CONTROL)
            
            if is_ctrl_pressed:
                if key == glfw.KEY_G:  # Ctrl + G
                    if self.config.use_gaussian_renderer:
                        self.show_gaussian_img = not self.show_gaussian_img
                        self.gs_renderer.need_rerender = True
                elif key == glfw.KEY_D:  # Ctrl + D
                    if self.config.use_gaussian_renderer:
                        self.gs_renderer.need_rerender = True
                    if self.renderer._depth_rendering:
                        self.renderer.disable_depth_rendering()
                    else:
                        self.renderer.enable_depth_rendering()
            else:
                if key == glfw.KEY_H:  # 'h': 显示帮助
                    self.printHelp()
                elif key == glfw.KEY_P:  # 'p': 打印信息
                    self.printMessage()
                elif key == glfw.KEY_R:  # 'r': 重置状态
                    self.reset()
                elif key == glfw.KEY_ESCAPE:  # ESC: 切换到自由视角
                    self.cam_id = -1
                    self.camera_pose_changed = True
                elif key == glfw.KEY_RIGHT_BRACKET:  # ']': 下一个相机
                    if self.mj_model.ncam:
                        self.cam_id += 1
                        self.cam_id = self.cam_id % self.mj_model.ncam
                elif key == glfw.KEY_LEFT_BRACKET:  # '[': 上一个相机
                    if self.mj_model.ncam:
                        self.cam_id += self.mj_model.ncam - 1
                        self.cam_id = self.cam_id % self.mj_model.ncam

    def printHelp(self):
        """打印帮助信息"""
        print("\n=== 键盘控制说明 ===")
        print("H: 显示此帮助信息")
        print("P: 打印当前状态信息")
        print("R: 重置模拟器状态")
        print("G: 切换高斯渲染（如果可用）")
        print("D: 切换深度渲染")
        print("Ctrl+G: 组合键切换高斯模式")
        print("Ctrl+D: 组合键切换深度图模式")
        print("ESC: 切换到自由视角")
        print("[: 切换到上一个相机")
        print("]: 切换到下一个相机")
        print("\n=== 鼠标控制说明 ===")
        print("左键拖动: 旋转视角")
        print("右键拖动: 平移视角")
        print("中键拖动: 缩放视角")
        print("================\n")

    def printMessage(self):
        """打印当前状态信息"""
        print("\n=== 当前状态 ===")
        print(f"当前相机ID: {self.cam_id}")
        if self.cam_id >= 0:
            print(f"相机名称: {self.camera_names[self.cam_id]}")
        print(f"高斯渲染: {'开启' if self.show_gaussian_img else '关闭'}")
        print(f"深度渲染: {'开启' if self.renderer._depth_rendering else '关闭'}")
        print("==============\n")

    def resetState(self):
        mujoco.mj_resetData(self.mj_model, self.mj_data)
        mujoco.mj_forward(self.mj_model, self.mj_data)
        self.camera_pose_changed = True

    def getCameraPose(self, cam_id):
        if cam_id == -1:
            rotation_matrix = self.camera_rmat @ Rotation.from_euler('xyz', [self.free_camera.elevation * np.pi / 180.0, self.free_camera.azimuth * np.pi / 180.0, 0.0]).as_matrix()
            camera_position = self.free_camera.lookat + self.free_camera.distance * rotation_matrix[:3,2]
        else:
            rotation_matrix = np.array(self.mj_data.camera(self.camera_names[cam_id]).xmat).reshape((3,3))
            camera_position = self.mj_data.camera(self.camera_names[cam_id]).xpos

        return camera_position, Rotation.from_matrix(rotation_matrix).as_quat()[[3,0,1,2]]

    def _cleanup_before_exit(self):
        """在Python退出前执行的清理函数"""
        try:
            gs_thread = getattr(self, "_gs_render_thread", None)
            if gs_thread is not None and gs_thread.is_alive():
                gs_thread.join(timeout=10.0)
            # 如果GLFW上下文有效，先清理Mujoco渲染器
            if hasattr(self, 'renderer'):
                try:
                    del self.renderer
                except Exception:
                    pass

            # 然后清理GLFW资源
            if hasattr(self, 'window') and self.window is not None:
                try:
                    glfw.destroy_window(self.window)
                except Exception:
                    pass
                self.window = None
            
            # 最后终止GLFW
            if hasattr(self, 'glfw_initialized') and self.glfw_initialized:
                try:
                    glfw.terminate()
                except Exception:
                    pass
                self.glfw_initialized = False
            
        except Exception:
            pass

    # ------------------------------------------------------------------------------
    # ---------------------------------- Override ----------------------------------
    def reset(self):
        self.resetState()
        if self.config.enable_render:
            self.render()
        self.render_cnt = 0
        return self.getObservation()

    def updateControl(self, action):
        pass

    # 包含了一些需要子类实现的抽象方法
    @abstractmethod
    def post_physics_step(self):
        pass

    @abstractmethod
    def getChangedObjectPose(self):
        raise NotImplementedError("pubObjectPose is not implemented")

    @abstractmethod
    def checkTerminated(self):
        raise NotImplementedError("checkTerminated is not implemented")    

    @abstractmethod
    def getObservation(self):
        raise NotImplementedError("getObservation is not implemented")

    @abstractmethod
    def getPrivilegedObservation(self):
        raise NotImplementedError("getPrivilegedObservation is not implemented")

    @abstractmethod
    def getReward(self):
        raise NotImplementedError("getReward is not implemented")
    
    # ---------------------------------- Override ----------------------------------
    # ------------------------------------------------------------------------------

    def step(self, action=None): # 主要的仿真步进函数
        for _ in range(self.decimation):
            self.updateControl(action)
            mujoco.mj_step(self.mj_model, self.mj_data)

        terminated = self.checkTerminated()
        if terminated:
            self.resetState()
        
        self.post_physics_step()
        if (
            self.config.enable_render
            and self.mj_data.time >= self._next_render_sim_time
        ):
            self.render()
            self._next_render_sim_time = self.mj_data.time + (1.0 / self.render_fps)

        return self.getObservation(), self.getPrivilegedObservation(), self.getReward(), terminated, {}

    def view(self):
        self.mj_data.time += self.delta_t
        self.mj_data.qvel[:] = 0
        mujoco.mj_forward(self.mj_model, self.mj_data)
        if self.mj_data.time >= self._next_render_sim_time:
            self.render()
            self._next_render_sim_time = self.mj_data.time + (1.0 / self.render_fps)


