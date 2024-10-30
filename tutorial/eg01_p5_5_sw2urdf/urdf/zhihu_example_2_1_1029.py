import mujoco as mj
from mujoco.glfw import glfw
import mujoco.viewer
import numpy as np
import time
import os


class GraspControl:
    def __init__(self, filename, is_show): #对应伪代码 initVisualData();
        # 1. model and data 加载模型与数据
        self.model = mj.MjModel.from_xml_path(filename)
        self.data = mj.MjData(self.model)
        self.is_show = is_show
        if self.is_show:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data, key_callback=self.keyboard_cb, show_left_ui = True, show_right_ui = False)
            self.viewer.opt.frame = mj.mjtFrame.mjFRAME_CAMERA
            # self.viewer.cam.type = 'mjCAMERA_FIXED'
            # self.viewer.cam.fixedcamid = 0
        self.grasp_ongoing = False
        self.start_time = None

    def init_controller(self): #对应伪代码 initControlData();
        mj.set_mjcb_control(self.controller_Rst_UpOpen)

    def controller_Rst_UpOpen(self, model, data): #对应伪代码 callback function (此处按照controller与visualization所规定的callback函数进行补充)
        data.ctrl[0] = 40 #constant actuator signal, tendon length, +: open gripper, -: close gripper
        data.ctrl[1] = 0.0 #constant actuator signal
        data.ctrl[[2, 3, 4]] = 0, 0, 0
        self.grasp_ongoing = False
        self.start_time = None

    def controller_Grasp(self, model, data):
       # Initialize the sequence if not already started
        if not self.grasp_ongoing:
            self.start_time = time.time()
            self.grasp_ongoing = True
            print("Starting grasp sequence")
        
        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        # Check for reset condition first if you want the task to be repeat
        # if elapsed_time > 5:
        #     self.grasp_ongoing = False
        #     self.start_time = None
        #     print("Sequence complete, resetting")
        #     return
        
        # Execute sequence based on elapsed time
        if elapsed_time < 2:
            # Phase 1: First 2 seconds
            data.ctrl[0] = 40
            data.ctrl[1] = -0.27
            data.ctrl[[2, 3, 4]] = 0, 0, 0
            print(f"Grasp-open&reach: time < 2s (elapsed: {elapsed_time:.2f}s)")
            
        elif 2 <= elapsed_time < 3:
            # Phase 2: Between 2 and 4 seconds
            data.ctrl[0] = -40
            data.ctrl[1] = -0.27  # Maintaining previous value
            data.ctrl[[2, 3, 4]] = 0, 0, 0
            print(f"Grasp-closing: time 2-3s (elapsed: {elapsed_time:.2f}s)")
            
        elif  elapsed_time >= 3:
            # Phase 3: After 3 seconds
            data.ctrl[0] = -40
            data.ctrl[1] = 0.2
            data.ctrl[[2, 3, 4]] = 0, 0, 0
            # if elapsed_time <= 5:
            print(f"Grasp-lifting: time > 3s (elapsed: {elapsed_time:.2f}s)")
        #     elif elapsed_time > 5:
        #         self.grasp_ongoing = False
        #         self.start_time = None   

        # # Reset the sequence if you want it to be repeatable
        # elif elapsed_time > 5:
        #     self.grasp_ongoing = False
        #     self.start_time = None   

    def controller_Release(self, model, data):
        data.ctrl[0] = 40 #constant actuator signal, tendon length, +: open gripper, -: close gripper
        # data.ctrl[1] = -0.27 #constant actuator signal
        data.ctrl[[2, 3, 4]] = 0, 0, 0
        self.grasp_ongoing = False
        self.start_time = None

    def controller_Mani_swing_Y(self, model, data):
        data.ctrl[[2, 3, 4]] = 0, 1.0, 0
        self.grasp_ongoing = False
        self.start_time = None

    def controller_Mani_swing_Y_(self, model, data):
        data.ctrl[[2, 3, 4]] = 0, -1.0, 0
        self.grasp_ongoing = False
        self.start_time = None

    def controller_Mani_swing_X(self, model, data):
        data.ctrl[[2, 3, 4]] = 1.0, 0, 0
        self.grasp_ongoing = False
        self.start_time = None

    def controller_Mani_swing_X_(self, model, data):
        data.ctrl[[2, 3, 4]] = -1.0, 0, 0
        self.grasp_ongoing = False
        self.start_time = None

    def main(self): 
        """
        对应伪代码 realtime simulation,  
        while(...) {
        updateControlData();
        updateVisualData(); }
        """
        sim_start, sim_end = time.time(), 300.0
        while time.time() - sim_start < sim_end:
            step_start = time.time()
            loop_num, loop_count = 50, 0
            # 1. running for 0.002*50 = 0.1s
            while loop_count < loop_num:
                loop_count = loop_count + 1
                mj.mj_step(self.model, self.data)
            # 2. GUI show
            if self.is_show:
                if self.viewer.is_running():
                    self.viewer.sync()
                else:
                    break
            # 3. sleep for next period
            step_next_delta = self.model.opt.timestep * loop_count - (time.time() - step_start)
            if step_next_delta > 0:
                time.sleep(step_next_delta)
        if self.is_show: 
            self.viewer.close()

    def keyboard_cb(self, keycode):
        # Print keycode for debugging
        print(f"Received keycode: {keycode}")
        
        # Handle space bar
        if keycode == 32:  # Space bar
            mj.mj_forward(self.model, self.data)
            mj.set_mjcb_control(self.controller_Rst_UpOpen)
            # self.init_controller()
        
        # Arrow keys using direct ASCII values
        elif keycode == 265:  # Up arrow
            print("Up pressed")
            mj.mj_forward(self.model, self.data)
            mj.set_mjcb_control(self.controller_Mani_swing_X_)  
            
        elif keycode == 264:  # Down arrow
            print("Down pressed")
            mj.mj_forward(self.model, self.data)
            mj.set_mjcb_control(self.controller_Mani_swing_X)            
            
        elif keycode == 263:  # Left arrow
            print("Left pressed")
            mj.mj_forward(self.model, self.data)              
            mj.set_mjcb_control(self.controller_Mani_swing_Y_)        
           
        elif keycode == 262:  # Right arrow
            print("Right pressed")
            mj.mj_forward(self.model, self.data)
            mj.set_mjcb_control(self.controller_Mani_swing_Y)

        elif keycode == 257:  # Enter key
            print("Enter pressed")
            mj.mj_forward(self.model, self.data)
            mj.set_mjcb_control(self.controller_Grasp)

        elif keycode == 267:  # Page-down key
            print("Page-down pressed")
            mj.mj_forward(self.model, self.data)
            mj.set_mjcb_control(self.controller_Release)            

if __name__ == "__main__":
    rel_path = "eg01p5_5_scene_with_manipulator.xml"
    # dir_name = os.path.dirname(__file__)
    dir_name = os.path.dirname(__file__) if '__file__' in globals() else os.getcwd()
    xml_path = os.path.join(dir_name + "\\" + rel_path)
    print(xml_path)
    is_show = True
    GraspControl = GraspControl(xml_path, is_show)
    GraspControl.main()