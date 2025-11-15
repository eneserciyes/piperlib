from piperlib import PiperController, ControllerConfig
import time

class TestPythonBindings:
    def test_1(self):
        controller_config = ControllerConfig()
        piper_controller = PiperController(controller_config)
        
        if not piper_controller.start():
            raise RuntimeError("Failed to start Piper controller")
        
        piper_controller.set_target(
            new_target_pos=[0.0] * 6,
            new_target_vel=[0.0] * 6,
            new_target_acc=[0.0] * 6,
        )

        time.sleep(1)

        piper_controller.stop()


if __name__ == "__main__":
    test = TestPythonBindings()
    test.test_1()
        
